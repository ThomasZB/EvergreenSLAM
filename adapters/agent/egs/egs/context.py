"""Per-invocation state: configuration, the HTTP client, lazy memory/ access."""

import os
import sys

from .client import (
    DEFAULT_TIMEOUT_S,
    DEFAULT_URL,
    EXIT_REFUSED,
    EXIT_USAGE,
    Client,
    EgsError,
    explain,
)
from .memory import Memory
from .store import LocalStore, RemoteStore


class Ctx:
    def __init__(self, env=None, out=None, err=None):
        self.env = os.environ if env is None else env
        self.out = out or sys.stdout
        self.err = err or sys.stderr
        timeout = float(self.env.get("EGS_TIMEOUT") or DEFAULT_TIMEOUT_S)
        self.client = Client(self.env.get("EGS_URL") or DEFAULT_URL, timeout)
        self.poll_s = float(self.env.get("EGS_POLL_S") or 10.0)
        self._root = None
        self._memory_dir = None
        self._store = None

    def print(self, *lines):
        for line in lines:
            self.out.write(line + "\n")

    def warn(self, line):
        self.err.write(line + "\n")

    def root(self):
        if self._root is None:
            j = self.client.get("/root")
            if not j.get("ok"):
                raise EgsError(explain(j.get("reason")), EXIT_REFUSED)
            self._root = j
        return self._root

    def memory_dir(self):
        if self._memory_dir is None:
            self._memory_dir = self.env.get("EGS_MEMORY") or self.root()["memory_dir"]
        return self._memory_dir

    def map_dir(self):
        return self.local_map_dir() or self.root().get("map_dir")

    def local_map_dir(self):
        """memory/ is always <map_dir>/memory, so a readable EGS_MEMORY names map_dir here."""
        mem = self.env.get("EGS_MEMORY")
        if not mem or not self.local_ok():
            return None
        return os.path.dirname(os.path.normpath(os.path.abspath(mem)))

    def refuse_inside_memory(self, path):
        """Exports carry map-frame XY, which memory/ never holds (memory/README.md dead rule 1)."""
        real = os.path.realpath(path)
        mem = os.path.realpath(self.memory_dir())
        if real == mem or real.startswith(mem + os.sep):
            raise EgsError(
                "refused: %s is inside memory/, which holds no map-frame XY; "
                "give -o a directory outside it (e.g. under /tmp)" % path,
                EXIT_USAGE,
            )

    def local_ok(self):
        d = self.memory_dir()
        return os.path.isdir(d) and os.access(d, os.R_OK | os.X_OK)

    def store(self):
        if self._store is None:
            self._store = (
                LocalStore(self.memory_dir()) if self.local_ok() else RemoteStore(self.client)
            )
        return self._store

    def memory(self):
        return Memory(self.store())

    def refused(self, j, what="refused"):
        self.print("%s: %s" % (what, explain(j.get("reason"), j.get("detail"))))
        return EXIT_REFUSED
