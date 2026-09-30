"""Tree semantics of memory/places (memory/README.md): bindings, walk-up, offsets, observations.

The service resolves anchor ids only; everything about names and paths happens here.
"""

import difflib
import json
import posixpath
import re

from . import se2
from .client import EXIT_USAGE, EgsError
from .yamlmini import YamlSubsetError, loads

RANK_NAME, RANK_ALIAS, RANK_KIND, RANK_WORD, RANK_FUZZY = range(5)
ATTACHMENTS = "attachments"
FUZZY_CUTOFF = 0.75


class UnreadablePlace(EgsError):
    """A place.yaml that does not parse to an anchor id; `place` is its directory."""

    def __init__(self, place, message):
        super().__init__(message, EXIT_USAGE, "unreadable_place")
        self.place = place


class Resolution:
    """What `egs where` answers for one path."""

    def __init__(self, path):
        self.path = path
        self.precision = "none"
        self.place = None  # A: nearest directory with a place.yaml, path included
        self.anchor = None
        self.state = None
        self.orphan_reason = None
        self.saved_at_ns = None
        self.approach = None  # A's pose
        self.pose = None  # position of path; theta only meaningful for own / offset
        self.resolved_via = None
        self.notes = []
        self.bound_children = []

    @property
    def resolved(self):
        return self.pose is not None


class Memory:
    def __init__(self, store):
        self.store = store

    def _names(self, rel):
        return {n: t for n, t, _ in (self.store.listdir(rel) or [])}

    def is_dir(self, rel):
        return self.store.kind(rel) == "dir"

    def place_anchor(self, rel):
        """Anchor id from rel/place.yaml, None without one."""
        if self._names(rel).get("place.yaml") != "file":
            return None
        data = self.store.read(posixpath.join(rel, "place.yaml"))
        if data is None:
            return None
        try:
            anchor = loads(data.decode("utf-8")).get("anchor")
        except (YamlSubsetError, UnicodeDecodeError) as e:
            raise UnreadablePlace(rel, "%s/place.yaml is unreadable: %s" % (rel, e))
        if not isinstance(anchor, int) or isinstance(anchor, bool) or anchor <= 0:
            raise UnreadablePlace(rel, "%s/place.yaml has no valid anchor id" % rel)
        return anchor

    def node_yaml(self, rel):
        """(dict or None when absent, error string or None)."""
        if self._names(rel).get("node.yaml") != "file":
            return None, None
        data = self.store.read(posixpath.join(rel, "node.yaml"))
        if data is None:
            return None, None
        try:
            return loads(data.decode("utf-8")), None
        except (YamlSubsetError, UnicodeDecodeError) as e:
            return None, "%s/node.yaml: %s" % (rel, e)

    @staticmethod
    def chain(rel):
        """rel, its parent, ..., places."""
        out = [rel]
        while out[-1] != "places" and "/" in out[-1]:
            out.append(posixpath.dirname(out[-1]))
        return out

    def nearest_place(self, rel):
        """(A, anchor id, directories strictly below A down to rel, deepest first) or None."""
        chain = self.chain(rel)
        for i, d in enumerate(chain):
            aid = self.place_anchor(d)
            if aid is not None:
                return d, aid, chain[:i]
        return None

    def children(self, rel):
        return [
            posixpath.join(rel, n)
            for n, t in sorted(self._names(rel).items())
            if t == "dir" and n not in ("skills", ATTACHMENTS)
        ]

    def nodes(self, rel="places"):
        """Every node directory below rel: no skills/ or attachments/ subtrees, no symlinks."""
        out = []
        for c in self.children(rel):
            out.append(c)
            out.extend(self.nodes(c))
        return out

    def resolve(self, rel, lookup):
        """lookup(anchor_id) -> {state, pose|None, orphan_reason, saved_at_ns?} or None."""
        r = Resolution(rel)
        found = self.nearest_place(rel)
        if found is None:
            r.bound_children = [c for c in self.children(rel) if self.place_anchor(c) is not None]
            return r
        a_path, aid, below = found
        r.place, r.anchor = a_path, aid
        entry = lookup(aid)
        if entry is None:
            r.state = "unknown"
            r.notes.append("anchor %d from %s/place.yaml is unknown to the process" % (aid, a_path))
            return r
        r.state = entry.get("state")
        r.orphan_reason = entry.get("orphan_reason")
        r.saved_at_ns = entry.get("saved_at_ns")
        r.approach = se2.from_json(entry.get("pose"))
        if r.approach is None:
            return r
        if rel == a_path:
            r.precision, r.pose, r.resolved_via = "own", r.approach, a_path
            return r
        for d in below:
            ny, err = self.node_yaml(d)
            if err:
                r.notes.append("ignored " + err)
                continue
            if not ny or "offset" not in ny:
                continue
            off, src = ny.get("offset"), ny.get("offset_from")
            if src != aid:
                r.notes.append(
                    "offset in %s/node.yaml ignored: offset_from %s, nearest place %s is anchor %d"
                    % (d, src if src is not None else "missing", a_path, aid)
                )
                continue
            if not _is_pose(off):
                r.notes.append("offset in %s/node.yaml ignored: not [dx, dy, dtheta]" % d)
                continue
            r.pose = se2.compose(r.approach, tuple(float(v) for v in off))
            r.resolved_via = d
            r.precision = "offset" if d == rel else "inherited"
            return r
        r.precision, r.pose, r.resolved_via = "inherited", r.approach, a_path
        return r

    def observations(self, rel):
        data = self.store.read(posixpath.join(rel, "observations.jsonl"))
        out = []
        for line in (data or b"").decode("utf-8", "replace").splitlines():
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if isinstance(obj, dict) and isinstance(obj.get("recorded_at"), int):
                out.append(obj)
        return out

    def last_observation(self, rel):
        """Newest line carrying a result that no later line invalidates."""
        lines = self.observations(rel)
        dead = {o["invalidates"] for o in lines if isinstance(o.get("invalidates"), int)}
        live = [o for o in lines if "result" in o and o["recorded_at"] not in dead]
        return max(live, key=lambda o: o["recorded_at"]) if live else None


def _is_pose(v):
    return (
        isinstance(v, list)
        and len(v) == 3
        and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)
    )


def _words(s):
    return re.sub(r"[\s_-]+", " ", str(s).strip().lower())


def match_rank(query, name, node):
    """name > alias > kind (or label) > word-boundary substring > difflib; None for no match."""
    q = _words(query)
    if not q:
        return None
    node = node or {}
    aliases = [_words(a) for a in _as_list(node.get("aliases"))]
    kind = _words(node.get("kind")) if node.get("kind") is not None else None
    labels = [_words(a) for a in _as_list(node.get("labels"))]
    n = _words(name)
    if q == n:
        return RANK_NAME
    if q in aliases:
        return RANK_ALIAS
    if q == kind or q in labels:
        return RANK_KIND
    fields = [n] + aliases + ([kind] if kind else [])
    word = re.compile(r"(?<![^\W_])" + re.escape(q) + r"(?![^\W_])")
    if any(word.search(f) for f in fields):
        return RANK_WORD
    candidates = fields + [w for f in fields for w in f.split(" ")]
    if any(difflib.SequenceMatcher(None, q, c).ratio() >= FUZZY_CUTOFF for c in candidates):
        return RANK_FUZZY
    return None


def _as_list(v):
    if v is None:
        return []
    return v if isinstance(v, list) else [v]
