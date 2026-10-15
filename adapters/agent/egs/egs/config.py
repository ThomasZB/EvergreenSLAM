"""Where egs connects: --robot > EGS_* > ~/.config/egs/config > defaults (README "Config")."""

import configparser
import os

from .client import DEFAULT_TIMEOUT_S, DEFAULT_URL, EXIT_USAGE, EgsError

KEYS = ("url", "timeout", "memory")
ENV_KEYS = {"url": "EGS_URL", "timeout": "EGS_TIMEOUT", "memory": "EGS_MEMORY"}


class Settings:
    def __init__(self, url, timeout, memory, robot, path):
        self.url = url
        self.timeout = timeout
        self.memory = memory
        self.robot = robot
        self.path = path


def config_path(env):
    base = env.get("XDG_CONFIG_HOME") or os.path.join(
        env.get("HOME") or os.path.expanduser("~"), ".config"
    )
    return os.path.join(base, "egs", "config")


def load(env, robot_arg=None):
    path = config_path(env)
    parser = configparser.ConfigParser(interpolation=None)
    try:
        parser.read(path, encoding="utf-8")
    except (configparser.Error, UnicodeDecodeError) as e:
        raise EgsError("cannot parse %s: %s" % (path, e), EXIT_USAGE, "bad_config")
    robot = robot_arg or env.get("EGS_ROBOT") or None
    section = {}
    if robot:
        name = "robot." + robot
        if not parser.has_section(name):
            raise EgsError("no [%s] in %s" % (name, path), EXIT_USAGE, "unknown_robot")
        section = dict(parser.items(name))
    default = dict(parser.items("default")) if parser.has_section("default") else {}
    from_env = {k: env.get(v) for k, v in ENV_KEYS.items()}
    # A robot named on the command line beats ambient EGS_* variables; one from EGS_ROBOT does not.
    layers = (section, from_env, default) if robot_arg else (from_env, section, default)
    value = {k: next((layer[k] for layer in layers if layer.get(k)), None) for k in KEYS}
    try:
        timeout = float(value["timeout"] or DEFAULT_TIMEOUT_S)
    except ValueError:
        raise EgsError("timeout %r is not a number of seconds" % value["timeout"], EXIT_USAGE)
    memory = os.path.expanduser(value["memory"]) if value["memory"] else None
    return Settings(value["url"] or DEFAULT_URL, timeout, memory, robot, path)
