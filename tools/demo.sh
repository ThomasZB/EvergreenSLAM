#!/usr/bin/env bash
# pixi run -e ros demo: Cartographer's public revo_lds bag (a Neato Revo LDS, 3 MB) replayed with
# the browser view and the agent service. Each run starts a fresh map in runs/demo.
set -euo pipefail
cd "$(dirname "$0")/.."

bag=bags/revo_lds
if [ ! -f "$bag/metadata.yaml" ]; then
  mkdir -p bags/download
  curl -fL --retry 3 -o bags/download/revo_lds.bag \
    https://storage.googleapis.com/cartographer-public-data/bags/revo_lds/cartographer_paper_revo_lds.bag
  python tools/convert_ros1_bag.py bags/download/revo_lds.bag "$bag"
fi

rm -rf runs/demo
mkdir -p runs/demo
port=${WEBUI_PORT:-8642}
cat <<EOF

  browser view   http://localhost:$port
  agent service  127.0.0.1:8643, e.g. in another shell:
                   export PATH="\$PWD/adapters/agent/egs/bin:\$PATH"
                   egs here
                   egs place save places/start
                   egs where places/start

  16 min of driving replayed at ${SPEED:-8}x (SPEED changes it); Ctrl-C quits.

EOF
set +u
source build-pixi/install/setup.bash
set -u
exec ros2 run evergreenslam_ros evergreenslam_bag --bag "$bag" --config configs/evergreenslam.yaml \
  --scan_topic /horizontal_laser_2d --base_from_laser 0,0,0 --out_prefix runs/demo/replay \
  --webui_port "$port" --speed "${SPEED:-8}" --map_root runs/demo --agent_port 8643
