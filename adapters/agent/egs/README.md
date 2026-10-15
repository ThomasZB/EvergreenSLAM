# egs

Command-line bridge from an agent's shell to the running EvergreenSLAM process (contract:
`../API.md`, tree rules: `../memory_README.md`). Python 3.9+, standard library only.

## Install

On the computer the agent runs on (no ROS needed there):

```bash
pipx install ./adapters/agent/egs          # from a clone; gives the `egs` command
egs skill install ~/.claude/skills         # writes ~/.claude/skills/evergreenslam/SKILL.md
```

`egs skill show` prints the same SKILL.md; reinstall it after upgrading egs. Without pipx:
`export PATH="$PATH:/path/to/EvergreenSLAM/adapters/agent/egs/bin"`, or
`cd adapters/agent/egs && python3 -m egs --help`.

## Reaching the robot

The SLAM process serves the agent API on the robot at `127.0.0.1:8643`, without authentication,
so it is never exposed on the network. From another computer, forward the port over SSH and keep
the tunnel open while the agent works:

```bash
ssh -NT -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
    -L 127.0.0.1:8643:127.0.0.1:8643 user@robot
```

Then `egs root` checks the connection. It prints `remote http://127.0.0.1:8643  map <name>` when
`memory/` lives on the robot (all file work goes through `egs fs`), or the local `memory/` path
when this machine runs the SLAM process itself. On failure it prints the URL it tried.

egs ignores HTTP proxies (a system proxy would otherwise intercept the tunnel and answer 502);
`EGS_USE_PROXY=1` restores them.

## Config

Settings are taken from, highest first: `--robot NAME` (its config section), the environment,
the config file, the defaults. `EGS_ROBOT=NAME` selects a section too, but below the environment.

| key | environment | default | meaning |
|---|---|---|---|
| `url` | `EGS_URL` | `http://127.0.0.1:8643` | the service, usually the local end of the tunnel |
| `timeout` | `EGS_TIMEOUT` | `30` | seconds per request |
| `memory` | `EGS_MEMORY` | unset | a `memory/` directory on this machine to use directly |

The file is `~/.config/egs/config` (`$XDG_CONFIG_HOME/egs/config`):

```ini
[default]
timeout = 30

[robot.lab]
url = http://127.0.0.1:8643

[robot.warehouse]
url = http://127.0.0.1:18643
```

With several robots, give each tunnel its own local port
(`-L 127.0.0.1:18643:127.0.0.1:8643 user@warehouse`) and run `egs --robot warehouse ...`.

egs reads and writes `memory/` directly only when `memory` is set, or when the SLAM process runs
on this machine (it proves that with the instance id it writes to `<map_dir>/agent_instance`).
Otherwise everything goes through the service, so a directory of the same name on the agent's
computer is never touched.

Humans only: `egs session freeze --force` waives `not anchored`.
Tests: `python3 -m unittest discover -s adapters/agent/egs/tests`.
