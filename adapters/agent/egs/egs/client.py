"""HTTP client for the AgentService (adapters/agent/API.md). Standard library only."""

import http.client
import json
import socket
import urllib.error
import urllib.parse
import urllib.request

DEFAULT_URL = "http://127.0.0.1:8643"
DEFAULT_TIMEOUT_S = 30.0

EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_USAGE = 2
EXIT_TRANSPORT = 3

HINTS = {
    "no_keyframe": "no keyframe ingested yet this boot: let the robot move, then retry",
    "node_gone": "the keyframe was trimmed meanwhile: retry",
    "not_persisted": "the map directory could not be written (full or read-only?): nothing changed, retry",
    "unknown_anchor": "place.yaml names an anchor the process does not know (a cp -r?)",
    "unresolvable": "the binding is orphaned: go there and `egs place save` it again",
    "no_binding": "the target has no place.yaml of its own",
    "no_robot_pose": "the process has no robot pose yet",
    "fs_error": "the process could not write the file",
    "unknown_session": "no such session: see `egs session ls`",
    "freezing": "a freeze is in progress: retry in a few seconds",
    "fed_session": "the session being fed cannot be removed",
    "frozen_session": "frozen sessions are permanent",
    "plan_changed": "the graph changed between plan and apply; nothing was done: re-run",
    "exists": "the destination already exists",
    "not_empty": "directory not empty: use -r",
    "not anchored": "the fed session has no link to the frozen base yet: drive where they overlap",
    "not_started": "the SLAM backend has not started yet (or is stopping): retry shortly",
    "bad_param": "malformed or missing parameter",
    "path_escape": "path leaves memory/ (absolute, '..' or outside after realpath)",
    "symlink": "symbolic links are refused",
    "not_slug": "directory names are [a-z0-9][a-z0-9_-]*; other names go into node.yaml aliases",
    "reserved_name": "reserved name (skills is never a node)",
    "owned_by_process": "place.yaml, index.tsv and README.md belong to the SLAM process",
    "too_many_layers": "at most 4 layers: split into two views",
    "unknown_layer": "unknown layer",
    "too_large": "file over 1 MiB",
    "not_found": "no such file",
    "unknown_map": "no such map: see `egs map ls`",
    "not_supported": "this host cannot switch maps: restart it with map_root/map",
    "switching": "a map switch is in progress: wait for it, then `egs map ls`",
}


class EgsError(Exception):
    def __init__(self, message, code=EXIT_USAGE):
        super().__init__(message)
        self.code = code


def explain(reason, detail=None):
    text = str(reason)
    hint = HINTS.get(reason)
    if hint:
        text += " (%s)" % hint
    if detail:
        text += ": %s" % detail
    return text


class Response:
    def __init__(self, status, headers, body):
        self.status = status
        self.headers = headers
        self.body = body

    @property
    def is_json(self):
        return (self.headers.get("Content-Type") or "").startswith("application/json")

    def json(self):
        try:
            return json.loads(self.body.decode("utf-8"))
        except ValueError:
            raise EgsError("service sent malformed JSON", EXIT_TRANSPORT)


def _encode(params):
    out = {}
    for k, v in (params or {}).items():
        if v is None:
            continue
        out[k] = ("true" if v else "false") if isinstance(v, bool) else str(v)
    return urllib.parse.urlencode(out)


class Client:
    def __init__(self, base_url=DEFAULT_URL, timeout=DEFAULT_TIMEOUT_S):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def request(self, method, path, params=None, body=None, mutating=False):
        """GET params go in the query; POST params go in a form body unless raw `body` is given."""
        url = self.base_url + path
        data, headers = None, {}
        if method == "GET" or body is not None:
            q = _encode(params)
            if q:
                url += "?" + q
            if body is not None:
                data, headers["Content-Type"] = body, "application/octet-stream"
        else:
            data = _encode(params).encode("ascii")
            headers["Content-Type"] = "application/x-www-form-urlencoded"
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return Response(r.status, r.headers, r.read())
        except urllib.error.HTTPError as e:
            with e:
                return Response(e.code, e.headers, e.read())
        except (socket.timeout, TimeoutError):
            raise self._timeout(mutating)
        except (http.client.HTTPException, ConnectionError) as e:
            if mutating:
                raise EgsError(
                    "outcome unknown: the connection dropped (%s); check with `egs status` "
                    "before retrying" % e,
                    EXIT_TRANSPORT,
                )
            raise EgsError("connection to %s dropped (%s)" % (self.base_url, e), EXIT_TRANSPORT)
        except urllib.error.URLError as e:
            if isinstance(e.reason, (socket.timeout, TimeoutError)):
                raise self._timeout(mutating)
            raise EgsError(
                "cannot reach the SLAM process at %s (%s); is it running with --agent_port? "
                "set EGS_URL otherwise" % (self.base_url, e.reason),
                EXIT_TRANSPORT,
            )

    def _timeout(self, mutating):
        if mutating:
            return EgsError(
                "outcome unknown: no answer within %gs; the request may still run. "
                "Check with `egs status` / `egs here` / `egs session ls` before retrying"
                % self.timeout,
                EXIT_TRANSPORT,
            )
        return EgsError("no answer within %gs" % self.timeout, EXIT_TRANSPORT)

    def call(self, method, path, params=None, mutating=False):
        """JSON endpoints. 200 returns the receipt (maybe ok=false); anything else raises."""
        return self.check(self.request(method, path, params, mutating=mutating))

    @staticmethod
    def check(resp):
        if resp.status == 200 and resp.is_json:
            return resp.json()
        if resp.is_json:
            j = resp.json()
            code = EXIT_TRANSPORT if resp.status == 503 else EXIT_USAGE
            raise EgsError(
                "HTTP %d %s" % (resp.status, explain(j.get("reason"), j.get("detail"))), code
            )
        raise EgsError("HTTP %d from the service" % resp.status, EXIT_TRANSPORT)

    def get(self, path, params=None):
        return self.call("GET", path, params)

    def post(self, path, params=None):
        return self.call("POST", path, params, mutating=True)
