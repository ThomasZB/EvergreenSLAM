"""memory/ access: the local directory when it is readable here, the /fs endpoints otherwise.

Both stores take paths relative to memory/ ('.' is memory/ itself) and apply the same rules as the
service's fs passthrough (API.md "fs passthrough"), so `egs fs` behaves the same either way.
"""

import os
import posixpath
import re
import stat
import tempfile

from .client import EXIT_REFUSED, EXIT_USAGE, EgsError, explain

SLUG = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
PROCESS_OWNED_ROOT = ("index.tsv", "readme.md")
NODE_FILES = ("place.yaml", "node.yaml")
RESERVED_NAMES = ("skills", "attachments")


def norm(path, memory_dir=None):
    """Normalises a user path to memory-relative form; raises on escapes."""
    if path is None or path.strip() == "" or "\0" in path:
        raise EgsError(explain("path_escape", "empty path or NUL"), EXIT_USAGE, "path_escape")
    p = path.strip()
    if os.path.isabs(p) and memory_dir:
        real_mem = os.path.realpath(memory_dir)
        real_p = os.path.realpath(p)
        if real_p == real_mem or real_p.startswith(real_mem + os.sep):
            p = os.path.relpath(real_p, real_mem)
    if os.path.isabs(p):
        raise EgsError(explain("path_escape", path), EXIT_USAGE, "path_escape")
    p = posixpath.normpath(p.replace(os.sep, "/"))
    if p == ".." or p.startswith("../"):
        raise EgsError(explain("path_escape", path), EXIT_USAGE, "path_escape")
    return p


def node_path(path, memory_dir=None):
    p = norm(path, memory_dir)
    parts = p.split("/")
    if parts[0] != "places":
        raise EgsError(
            "paths are relative to memory/ and start with places/ (got %r; try places/%s)"
            % (path, p),
            EXIT_USAGE,
            "bad_path",
        )
    if any(part in RESERVED_NAMES for part in parts):
        raise EgsError(explain("reserved_name", p), EXIT_USAGE, "reserved_name")
    return p


def is_process_owned(rel):
    # Case-folded like the service: on a case-insensitive filesystem Place.yaml is place.yaml.
    return posixpath.basename(rel).lower() == "place.yaml" or rel.lower() in PROCESS_OWNED_ROOT


def _holds_node_files(path):
    """path is, or holds anywhere below it, a place.yaml or node.yaml; symlinks never count."""
    if os.path.islink(path) or not os.path.isdir(path):
        return os.path.basename(path) in NODE_FILES and not os.path.islink(path)
    for dirpath, _, filenames in os.walk(path):
        for n in filenames:
            if n in NODE_FILES and not os.path.islink(os.path.join(dirpath, n)):
                return True
    return False


def _refuse(reason, detail):
    raise EgsError(explain(reason, detail), EXIT_REFUSED, reason)


def _bad(reason, detail):
    raise EgsError(explain(reason, detail), EXIT_USAGE, reason)


class LocalStore:
    remote = False

    def __init__(self, root):
        self.root = os.path.abspath(root)

    def _abs(self, rel):
        return self.root if rel == "." else os.path.join(self.root, *rel.split("/"))

    def _check(self, rel):
        """Refuses symlink components and anything outside memory/ after realpath."""
        if rel == ".":
            return self.root
        cur = self.root
        for part in rel.split("/"):
            cur = os.path.join(cur, part)
            try:
                if stat.S_ISLNK(os.lstat(cur).st_mode):
                    _bad("symlink", rel)
            except FileNotFoundError:
                break
        real = os.path.realpath(self._abs(rel))
        real_root = os.path.realpath(self.root)
        if real != real_root and not real.startswith(real_root + os.sep):
            _bad("path_escape", rel)
        return self._abs(rel)

    def kind(self, rel):
        try:
            mode = os.lstat(self._abs(rel)).st_mode
        except OSError:
            return None
        if stat.S_ISLNK(mode):
            return "link"
        return "dir" if stat.S_ISDIR(mode) else "file"

    def listdir(self, rel):
        """[(name, type, size)] sorted, or None when rel is not a readable directory."""
        base = self._check(rel)
        try:
            names = sorted(os.listdir(base))
        except OSError:
            return None
        out = []
        for n in names:
            try:
                st = os.lstat(os.path.join(base, n))
            except OSError:
                continue
            t = (
                "link"
                if stat.S_ISLNK(st.st_mode)
                else "dir" if stat.S_ISDIR(st.st_mode) else "file"
            )
            out.append((n, t, st.st_size if t == "file" else 0))
        return out

    def read(self, rel):
        path = self._check(rel)
        try:
            with open(path, "rb") as f:
                return f.read()
        except OSError:
            return None

    def write(self, rel, data):
        path = self._check(rel)
        if is_process_owned(rel):
            _bad("owned_by_process", rel)
        parent = os.path.dirname(path)
        if not os.path.isdir(parent):
            _refuse("fs_error", "parent directory does not exist: " + posixpath.dirname(rel))
        fd, tmp = tempfile.mkstemp(dir=parent, prefix=".egs-", suffix=".tmp")
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
        except OSError as e:
            if os.path.exists(tmp):
                os.unlink(tmp)
            _refuse("fs_error", str(e))

    def append(self, rel, data):
        path = self._check(rel)
        if is_process_owned(rel):
            _bad("owned_by_process", rel)
        try:
            fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
            try:
                os.write(fd, data)
            finally:
                os.close(fd)
        except OSError as e:
            _refuse("fs_error", str(e))

    def mkdir(self, rel):
        self._check(rel)
        cur = self.root
        for part in rel.split("/"):
            cur = os.path.join(cur, part)
            if not os.path.isdir(cur) and not SLUG.match(part):
                _bad("not_slug", part)
        try:
            os.makedirs(self._abs(rel), exist_ok=True)
        except OSError as e:
            _refuse("fs_error", str(e))

    def mv(self, src, dst):
        s, d = self._check(src), self._check(dst)
        if not os.path.lexists(s):
            _refuse("fs_error", "no such file or directory: " + src)
        if os.path.isfile(s) and (is_process_owned(src) or is_process_owned(dst)):
            _bad("owned_by_process", src)
        if os.path.isdir(s) and not SLUG.match(posixpath.basename(dst)):
            _bad("not_slug", posixpath.basename(dst))
        # Scans skip skills/ and attachments/ subtrees: a place or thing moved there would vanish.
        if any(c in RESERVED_NAMES for c in dst.split("/")) and _holds_node_files(s):
            _bad(
                "reserved_name",
                "skills and attachments never hold place.yaml or node.yaml: %s -> %s" % (src, dst),
            )
        if os.path.lexists(d):
            _refuse("exists", dst)
        try:
            os.rename(s, d)
        except OSError as e:
            _refuse("fs_error", str(e))

    def rm(self, rel, recursive=False):
        path = self._check(rel)
        if rel == ".":
            _bad("path_escape", "refusing to remove memory/")
        if not os.path.lexists(path):
            _refuse("fs_error", "no such file or directory: " + rel)
        try:
            if os.path.isdir(path):
                if os.listdir(path) and not recursive:
                    _refuse("not_empty", rel)
                _rmtree(path)
            else:
                if is_process_owned(rel):
                    _bad("owned_by_process", rel)
                os.unlink(path)
        except OSError as e:
            _refuse("fs_error", str(e))

    def tree(self, rel, depth):
        path = self._check(rel)
        if not os.path.isdir(path):
            _refuse("fs_error", "not a directory: " + rel)
        lines = [rel]
        counts = [0, 0]

        def walk(r, prefix, level):
            entries = self.listdir(r) or []
            for i, (name, t, _) in enumerate(entries):
                last = i == len(entries) - 1
                lines.append(prefix + ("└── " if last else "├── ") + name)
                counts[0 if t == "dir" else 1] += 1
                if t != "dir":
                    continue
                child, child_prefix = posixpath.join(r, name), prefix + ("    " if last else "│   ")
                if level < depth:
                    walk(child, child_prefix, level + 1)
                elif self.listdir(child):
                    lines.append(child_prefix + "└── …")

        walk(rel, "", 1)
        lines.append("")
        lines.append("%d directories, %d files" % (counts[0], counts[1]))
        return "\n".join(lines) + "\n"


def _rmtree(path):
    for dirpath, dirnames, filenames in os.walk(path, topdown=False):
        for n in filenames:
            os.unlink(os.path.join(dirpath, n))
        for n in dirnames:
            p = os.path.join(dirpath, n)
            if os.path.islink(p):
                os.unlink(p)
            else:
                os.rmdir(p)
    os.rmdir(path)


class RemoteStore:
    """The same operations over GET/POST /fs/*. Listings are cached for one command."""

    remote = True

    def __init__(self, client):
        self.client = client
        self._ls = {}

    def _fail(self, resp):
        if resp.is_json:
            j = resp.json()
            code = EXIT_USAGE if resp.status == 400 else EXIT_REFUSED
            if resp.status == 200 or resp.status == 400:
                raise EgsError(explain(j.get("reason"), j.get("detail")), code, j.get("reason"))
        self.client.check(resp)
        raise EgsError("unexpected response from /fs", EXIT_USAGE)

    def listdir(self, rel):
        if rel in self._ls:
            return self._ls[rel]
        resp = self.client.request("GET", "/fs/ls", {"path": rel})
        out = None
        if resp.status == 200 and resp.is_json:
            j = resp.json()
            if j.get("ok"):
                out = [(e["name"], e["type"], e.get("size", 0)) for e in j.get("entries", [])]
        elif resp.status != 400:
            self.client.check(resp)
        self._ls[rel] = out
        return out

    def kind(self, rel):
        if rel == ".":
            return "dir"
        entries = self.listdir(posixpath.dirname(rel) or ".")
        name = posixpath.basename(rel)
        for n, t, _ in entries or []:
            if n == name:
                return t
        return None

    def read(self, rel):
        resp = self.client.request("GET", "/fs/cat", {"path": rel})
        if resp.status == 200 and not resp.is_json:
            return resp.body
        if resp.status in (200, 400, 404):
            return None
        self.client.check(resp)
        return None

    def _post(self, path, params, body=None):
        self._ls.clear()
        resp = self.client.request("POST", path, params, body=body, mutating=True)
        if resp.status == 200 and resp.is_json and resp.json().get("ok"):
            return
        self._fail(resp)

    def write(self, rel, data):
        self._post("/fs/write", {"path": rel}, body=data)

    def append(self, rel, data):
        self._post("/fs/append", {"path": rel}, body=data)

    def mkdir(self, rel):
        self._post("/fs/mkdir", {"path": rel})

    def mv(self, src, dst):
        self._post("/fs/mv", {"from": src, "to": dst})

    def rm(self, rel, recursive=False):
        self._post("/fs/rm", {"path": rel, "recursive": 1 if recursive else 0})

    def tree(self, rel, depth):
        resp = self.client.request("GET", "/fs/tree", {"path": rel, "depth": depth})
        if resp.status == 200 and not resp.is_json:
            return resp.body.decode("utf-8", "replace")
        self._fail(resp)
