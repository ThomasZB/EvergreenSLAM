# egs

Command-line bridge from an agent's shell to the running EvergreenSLAM process (contract:
`../API.md`, tree rules: `../memory_README.md`). Python 3.9+, standard library only.

Install: `export PATH="$PATH:/path/to/EvergreenSLAM/adapters/agent/egs/bin"`, then `egs --help`.
Without the launcher: `cd adapters/agent/egs && python3 -m egs --help`. For the agent, copy the
`../skills/evergreenslam/` directory into its skills directory.

Config: `EGS_URL` (default `http://127.0.0.1:8643`), `EGS_MEMORY`, `EGS_TIMEOUT`. Humans only:
`egs session freeze --force` waives `not anchored`. Tests: `python3 -m unittest discover -s adapters/agent/egs/tests`.
