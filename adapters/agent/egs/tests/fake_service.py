"""An in-process stand-in for the AgentService speaking the JSON of adapters/agent/API.md."""

import json
import os
import re
import shutil
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PNG = b"\x89PNG\r\n\x1a\n" + b"\0" * 16


def pose(x, y, theta):
    return {"x": x, "y": y, "theta": theta}


def fnv1a64(text):
    h = 0xCBF29CE484222325
    for b in text.encode("utf-8"):
        h = ((h ^ b) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return "%016x" % h


class FakeState:
    def __init__(self, memory_dir, map_dir):
        self.memory_dir = memory_dir  # what /fs/* serves
        self.map_dir = map_dir  # where snapshots and views land
        self.root_memory_dir = memory_dir  # what /root reports
        self.root_map_dir = map_dir  # what /root and /snapshot report
        self.map_root = os.path.dirname(map_dir)  # None: the host has no map root
        self.map_name = os.path.basename(map_dir)
        self.maps = [self.map_name]
        self.can_switch = True
        # After a switch request, /root polls first see a dropped connection, then 503, then the
        # new map: switch_down_polls dropped, the rest up to switch_after_polls answer 503.
        self.switch_down_polls = 0
        self.switch_after_polls = 0
        self.switching = None  # name being switched to
        self.switch_polls = 0
        self.has_frozen_base = True
        self.aligned_to_base = True
        self.view_dropped = None  # e.g. "robot,scan": legend's dropped= item
        self.solves = 100
        self.robot = pose(0.0, 0.0, 0.0)
        self.anchors = {}
        self.next_id = 50
        self.keyframe = True
        self.fed = 2
        self.sessions = [
            {
                "id": 0,
                "role": "frozen",
                "nodes": 400,
                "submaps": 12,
                "anchors": 3,
                "phase": None,
                "anchored": None,
                "start_ns": 1,
                "last_node_ns": 2,
            },
            {
                "id": 1,
                "role": "floating",
                "nodes": 40,
                "submaps": 3,
                "anchors": 1,
                "phase": "BOOTSTRAP",
                "anchored": False,
                "start_ns": 3,
                "last_node_ns": 4,
            },
            {
                "id": 2,
                "role": "fed",
                "nodes": 90,
                "submaps": 4,
                "anchors": 0,
                "phase": "LOCKED",
                "anchored": True,
                "start_ns": 5,
                "last_node_ns": time.time_ns(),
            },
        ]
        self.freeze_rejection = None
        self.freeze_visible_after = 1  # /sessions polls before the frozen role shows
        self.mutate_before_apply = False  # makes apply see a different plan
        self.delays = {}
        self.requests = []
        self.lock = threading.Lock()

    def anchor(self, aid, p, state="frozen", orphan_reason=None, saved_at_ns=None):
        self.anchors[aid] = {
            "anchor": aid,
            "state": state,
            "orphan_reason": orphan_reason,
            "pose": p,
            "submap_id": [0, aid],
            "saved_at_ns": saved_at_ns or time.time_ns() - 6 * 86400 * 10**9,
        }

    def paths_of(self, method, path):
        return [r for r in self.requests if r[0] == method and r[1] == path]


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    @property
    def st(self):
        return self.server.state

    def _send(self, status, body, ctype="application/json", headers=None):
        data = json.dumps(body).encode() if ctype == "application/json" else body
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def ok(self, **fields):
        body = {"ok": True, "reason": None, "at_num_solves": self.st.solves}
        body.update(fields)
        self._send(200, body)

    def refuse(self, reason, status=200, **fields):
        body = {"ok": False, "reason": reason, "at_num_solves": self.st.solves}
        body.update(fields)
        self._send(status, body)

    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")

    def _dispatch(self, method):
        url = urllib.parse.urlparse(self.path)
        q = dict(urllib.parse.parse_qsl(url.query, keep_blank_values=True))
        body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        raw = url.path in ("/fs/write", "/fs/append")
        if method == "POST" and not raw:
            q.update(dict(urllib.parse.parse_qsl(body.decode(), keep_blank_values=True)))
        with self.st.lock:
            self.st.requests.append((method, url.path, q))
        delay = self.st.delays.get(url.path)
        if delay:
            time.sleep(delay)
        name = method + " " + url.path
        if url.path.startswith("/anchors/") and method == "GET":
            return self.anchor_one(url.path.rsplit("/", 1)[1], q)
        handler = ROUTES.get(name)
        if handler is None:
            return self.refuse("bad_param", 404, detail="no route " + name)
        handler(self, q, body)

    # --- process endpoints -------------------------------------------------------------------

    def root(self, q, body):
        st = self.st
        with st.lock:
            polls = None
            if st.switching is not None:
                st.switch_polls += 1
                polls = st.switch_polls
                if polls > st.switch_after_polls:
                    st.map_name, st.switching = st.switching, None
                    st.root_map_dir = os.path.join(st.map_root, st.map_name)
                    st.root_memory_dir = os.path.join(st.root_map_dir, "memory")
        if polls is not None and polls <= st.switch_down_polls:
            self.close_connection = True
            return
        if polls is not None and polls <= st.switch_after_polls:
            return self.refuse("not_started", 503, detail="switching")
        self._send(
            200,
            {
                "ok": True,
                "reason": None,
                "map_dir": st.root_map_dir,
                "memory_dir": st.root_memory_dir,
                "map_root": st.map_root,
                "map": st.map_name if st.map_root else None,
            },
        )

    def maps_ls(self, q, body):
        st = self.st
        if not st.map_root:
            return self.refuse("not_supported")
        self.ok(
            map_root=st.map_root,
            current=st.map_name,
            maps=[{"name": n, "current": n == st.map_name} for n in sorted(st.maps)],
        )

    def _map_name(self, q):
        name = q.get("name", "")
        if not re.fullmatch(r"[a-z0-9][a-z0-9_-]*", name):
            self.refuse("not_slug", 400, detail=name)
            return None
        if not self.st.map_root or not self.st.can_switch:
            self.refuse("not_supported")
            return None
        if self.st.switching is not None:
            self.refuse("switching")
            return None
        return name

    def _begin_switch(self, name):
        with self.st.lock:
            self.st.switching = name
            self.st.switch_polls = 0
        self.ok(map=name, pending=True)

    def maps_new(self, q, body):
        name = self._map_name(q)
        if name is None:
            return
        if name in self.st.maps:
            return self.refuse("exists")
        self.st.maps.append(name)
        self._begin_switch(name)

    def maps_open(self, q, body):
        name = self._map_name(q)
        if name is None:
            return
        if name not in self.st.maps:
            return self.refuse("unknown_map")
        if name == self.st.map_name:
            return self.ok(map=name, pending=False)
        self._begin_switch(name)

    def status(self, q, body):
        self.ok(
            boot_count=3,
            map=self.st.map_name if self.st.map_root else None,
            fed_session=self.st.fed,
            has_frozen_base=self.st.has_frozen_base,
            aligned_to_base=self.st.aligned_to_base,
            phase="LOCKED",
            closures_delta=2,
            num_anchors=len(self.st.anchors),
            num_orphans=sum(1 for a in self.st.anchors.values() if a["state"] == "orphan"),
            num_place_files=5,
            duplicate_anchor_paths=[],
        )

    def here(self, q, body):
        self.ok(
            robot=self.st.robot,
            keyframe_age_s=1.5,
            has_frozen_base=self.st.has_frozen_base,
            aligned_to_base=self.st.aligned_to_base,
            current={"path": "places/dock", "anchor": 1, "dist_m": 0.4},
            closest=[
                {"path": "places/dock", "anchor": 1, "dist_m": 0.4, "state": "frozen"},
                {"path": "places/kitchen", "anchor": 2, "dist_m": 5.0, "state": "frozen"},
            ],
        )

    def place_save(self, q, body):
        path = q["path"]
        if not self.st.keyframe:
            return self.refuse("no_keyframe")
        pf = os.path.join(self.st.memory_dir, path, "place.yaml")
        existing = None
        if os.path.exists(pf):
            with open(pf) as f:
                existing = int(f.read().split(":")[1])
        aid = existing if existing in self.st.anchors else None
        if aid is None:
            aid = self.st.next_id
            self.st.next_id += 1
            os.makedirs(os.path.dirname(pf), exist_ok=True)
            with open(pf, "w") as f:
                f.write("anchor: %d\n" % aid)
        self.st.anchor(aid, dict(self.st.robot), state="pending", saved_at_ns=time.time_ns())
        self.ok(
            anchor=aid,
            submap_id=[2, 0],
            state="pending",
            keyframe_age_s=0.5,
            rebound_existing=existing is not None and aid == existing,
        )

    def anchor_one(self, aid, q):
        a = self.st.anchors.get(int(aid))
        if a is None:
            return self.refuse("unknown_anchor")
        fields = dict(a)
        if q.get("robot") == "1":
            fields["robot"] = self.st.robot
        self.ok(**fields)

    def anchors_all(self, q, body):
        fields = {
            "anchors": [
                {k: a[k] for k in ("anchor", "state", "orphan_reason", "pose")}
                for a in self.st.anchors.values()
            ]
        }
        if q.get("robot") == "1":
            fields["robot"] = self.st.robot
        self.ok(**fields)

    def snapshot(self, q, body):
        d = os.path.join(self.st.map_dir, "snapshots", "000001")
        os.makedirs(d, exist_ok=True)
        files = ["map.png", "summary.txt"]
        for f in files:
            with open(os.path.join(d, f), "wb") as fh:
                fh.write(b"x")
        self.ok(seq=1, dir=os.path.join(self.st.root_map_dir, "snapshots", "000001"), files=files)

    def view(self, q, body):
        layers = {"map": "map,places", "here": "map,robot,scan", "trail": "map,trail,robot"}
        preset = q.get("preset")
        if preset == "route":
            target = q.get("target", "")
            if not os.path.exists(os.path.join(self.st.memory_dir, target, "place.yaml")):
                return self.refuse("no_binding")
            eq = "map,robot,target=%s,places" % target
        elif preset == "custom":
            eq = q.get("layers", "")
            if len(eq.split(",")) > 4:
                return self.refuse("too_many_layers", 400, detail="")
        else:
            eq = layers.get(preset, "session=%s,trail" % q.get("session", self.st.fed))
        d = os.path.join(self.st.map_dir, "views")
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "000007_%s.png" % preset)
        with open(path, "wb") as f:
            f.write(PNG)
        self._send(
            200,
            PNG,
            "image/png",
            {
                "X-EGS-Layers": eq,
                "X-EGS-Legend": "; ".join(
                    ["layers=" + ",".join(layer.split("=")[0] for layer in eq.split(","))]
                    + (["dropped=%s" % self.st.view_dropped] if self.st.view_dropped else [])
                    + ["1 places/dock 0.4m", "2 places/kitchen 5.0m"]
                ),
                "X-EGS-View": os.path.relpath(path, self.st.map_dir),
                "X-EGS-Solve": str(self.st.solves),
            },
        )

    def init_pose(self, q, body):
        if "anchor" in q:
            a = self.st.anchors.get(int(q["anchor"]))
            if a is None:
                return self.refuse("unknown_anchor")
            if a["pose"] is None:
                return self.refuse("unresolvable")
            return self.ok(pose=a["pose"])
        self.ok(pose=pose(float(q["x"]), float(q["y"]), float(q["theta"])))

    def relocalize(self, q, body):
        self.ok()

    def checkpoint(self, q, body):
        self.ok(num_checkpoints_written=4)

    # --- sessions --------------------------------------------------------------------------

    def sessions(self, q, body):
        with self.st.lock:
            for s in self.st.sessions:
                if s.get("_freezing") is not None:
                    s["_freezing"] -= 1
                    if s["_freezing"] <= 0:
                        s["role"] = "frozen"
                        del s["_freezing"]
            listed = [
                {k: v for k, v in s.items() if not k.startswith("_")} for s in self.st.sessions
            ]
        self.ok(fed=self.st.fed, sessions=listed)

    def _token(self, op, affected, deletes, orphaned):
        return fnv1a64("%s|%s|%s|%s" % (op, sorted(affected), sorted(deletes), sorted(orphaned)))

    def _freeze_plan(self):
        rej = self.st.freeze_rejection
        affected = [self.st.fed]
        report = {
            "rejection": rej,
            "would_delete": [],
            "sessions_affected": affected,
            "anchors_orphaned": [],
            "plan_token": self._token("freeze", affected, [], []),
            "verdict": {"eligible": rej is None, "bootstrap": False, "rejection": rej},
        }
        return report

    def freeze_plan(self, q, body):
        self.st.solves += 1
        self.ok(**self._freeze_plan())

    def freeze_apply(self, q, body):
        self.st.solves += 1
        if self.st.mutate_before_apply:
            self.st.fed += 10
        plan = self._freeze_plan()
        if plan["rejection"] or plan["plan_token"] != q.get("plan_token"):
            return self.refuse("plan_changed")
        fed = self.st.fed
        for s in self.st.sessions:
            if s["id"] == fed:
                s["_freezing"] = self.st.freeze_visible_after
        self.st.sessions.append(
            {
                "id": fed + 1,
                "role": "fed",
                "nodes": 0,
                "submaps": 0,
                "anchors": 0,
                "phase": "BOOTSTRAP",
                "anchored": False,
                "start_ns": time.time_ns(),
                "last_node_ns": time.time_ns(),
            }
        )
        self.st.fed = fed + 1
        self.ok(frozen_session=fed, pending=True)

    def _rm_plan(self, sid):
        s = next((x for x in self.st.sessions if x["id"] == sid), None)
        if s is None:
            rej = "unknown_session"
        elif sid == self.st.fed:
            rej = "fed_session"
        elif s["role"] == "frozen":
            rej = "frozen_session"
        else:
            rej = None
        deletes = [[sid, i] for i in range(s["submaps"])] if rej is None else []
        orphaned = (
            [
                a["anchor"]
                for a in self.st.anchors.values()
                if a["submap_id"][0] == sid and a["state"] != "orphan"
            ]
            if rej is None
            else []
        )
        return {
            "rejection": rej,
            "would_delete": deletes,
            "sessions_affected": [sid],
            "anchors_orphaned": orphaned,
            "plan_token": self._token("rm", [sid], deletes, orphaned),
        }

    def rm_plan(self, q, body):
        self.ok(**self._rm_plan(int(q["id"])))

    def rm_apply(self, q, body):
        sid = int(q["id"])
        plan = self._rm_plan(sid)
        if plan["rejection"] or plan["plan_token"] != q.get("plan_token"):
            return self.refuse("plan_changed")
        self.st.sessions = [s for s in self.st.sessions if s["id"] != sid]
        for aid in plan["anchors_orphaned"]:
            self.st.anchors[aid].update(state="orphan", orphan_reason="session_removed", pose=None)
        self.ok(removed=sid, anchors_orphaned=plan["anchors_orphaned"])

    # --- fs passthrough -------------------------------------------------------------------

    def _fs(self, rel):
        if not rel or rel.startswith("/") or ".." in rel.split("/"):
            self.refuse("path_escape", 400, detail=rel)
            return None
        return os.path.join(self.st.memory_dir, rel)

    @staticmethod
    def _owned(rel):
        return os.path.basename(rel).lower() == "place.yaml" or rel.lower() in (
            "index.tsv",
            "readme.md",
        )

    def fs_ls(self, q, body):
        p = self._fs(q.get("path"))
        if p is None:
            return
        if not os.path.isdir(p):
            return self.refuse("fs_error", detail="not a directory")
        entries = []
        for n in sorted(os.listdir(p)):
            fp = os.path.join(p, n)
            if os.path.islink(fp):
                continue
            isdir = os.path.isdir(fp)
            entries.append(
                {
                    "name": n,
                    "type": "dir" if isdir else "file",
                    "size": 0 if isdir else os.path.getsize(fp),
                }
            )
        self._send(200, {"ok": True, "reason": None, "entries": entries})

    def fs_tree(self, q, body):
        p = self._fs(q.get("path"))
        if p is not None:
            self._send(200, ("\n".join(sorted(os.listdir(p))) + "\n").encode(), "text/plain")

    def fs_cat(self, q, body):
        p = self._fs(q.get("path"))
        if p is None:
            return
        if not os.path.exists(p):
            return self.refuse("not_found", 404, detail="no such file")
        if not os.path.isfile(p):
            return self.refuse("fs_error", detail="not a file")
        with open(p, "rb") as f:
            self._send(200, f.read(), "application/octet-stream")

    def _fs_done(self):
        self._send(200, {"ok": True, "reason": None})

    def fs_write(self, q, body):
        rel = q.get("path")
        p = self._fs(rel)
        if p is None:
            return
        if self._owned(rel):
            return self.refuse("owned_by_process", 400)
        if not os.path.isdir(os.path.dirname(p)):
            return self.refuse("fs_error", detail="no parent")
        with open(p + ".tmp", "wb") as f:
            f.write(body)
        os.replace(p + ".tmp", p)
        self._fs_done()

    def fs_append(self, q, body):
        rel = q.get("path")
        p = self._fs(rel)
        if p is None:
            return
        if self._owned(rel):
            return self.refuse("owned_by_process", 400)
        with open(p, "ab") as f:
            f.write(body)
        self._fs_done()

    def fs_mkdir(self, q, body):
        p = self._fs(q.get("path"))
        if p is not None:
            os.makedirs(p, exist_ok=True)
            self._fs_done()

    def fs_mv(self, q, body):
        s, d = self._fs(q.get("from")), self._fs(q.get("to"))
        if s is None or d is None:
            return
        if os.path.isfile(s) and self._owned(q["from"]):
            return self.refuse("owned_by_process", 400)
        if "skills" in q["to"].split("/") and any(
            n in ("place.yaml", "node.yaml") for _, _, names in os.walk(s) for n in names
        ):
            return self.refuse("reserved_name", 400)
        if os.path.exists(d):
            return self.refuse("exists")
        os.rename(s, d)
        self._fs_done()

    def fs_rm(self, q, body):
        rel = q.get("path")
        p = self._fs(rel)
        if p is None:
            return
        if os.path.isdir(p):
            if os.listdir(p) and q.get("recursive") != "1":
                return self.refuse("not_empty")
            shutil.rmtree(p)
        else:
            if self._owned(rel):
                return self.refuse("owned_by_process", 400)
            os.unlink(p)
        self._fs_done()


ROUTES = {
    "GET /root": _Handler.root,
    "GET /status": _Handler.status,
    "GET /maps": _Handler.maps_ls,
    "POST /maps/new": _Handler.maps_new,
    "POST /maps/open": _Handler.maps_open,
    "GET /here": _Handler.here,
    "POST /place/save": _Handler.place_save,
    "GET /anchors": _Handler.anchors_all,
    "POST /snapshot": _Handler.snapshot,
    "GET /view": _Handler.view,
    "POST /init-pose": _Handler.init_pose,
    "POST /relocalize": _Handler.relocalize,
    "POST /checkpoint": _Handler.checkpoint,
    "GET /sessions": _Handler.sessions,
    "POST /sessions/freeze/plan": _Handler.freeze_plan,
    "POST /sessions/freeze/apply": _Handler.freeze_apply,
    "POST /sessions/rm/plan": _Handler.rm_plan,
    "POST /sessions/rm/apply": _Handler.rm_apply,
    "GET /fs/ls": _Handler.fs_ls,
    "GET /fs/tree": _Handler.fs_tree,
    "GET /fs/cat": _Handler.fs_cat,
    "POST /fs/write": _Handler.fs_write,
    "POST /fs/append": _Handler.fs_append,
    "POST /fs/mkdir": _Handler.fs_mkdir,
    "POST /fs/mv": _Handler.fs_mv,
    "POST /fs/rm": _Handler.fs_rm,
}


class FakeService:
    def __init__(self, state):
        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        self.httpd.daemon_threads = True
        self.httpd.state = state
        self.url = "http://127.0.0.1:%d" % self.httpd.server_address[1]
        self.thread = threading.Thread(
            target=self.httpd.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True
        )
        self.thread.start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()
