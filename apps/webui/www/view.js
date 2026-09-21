// World metres in, screen pixels out: the only file that knows the two are related.

const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');

const view = { x: 0, y: 0, scale: 20, ready: false, follow: false };  // scale is px per metre

// Fetches and the SSE handler only write here; rendering on arrival would couple the frame
// rate to the network.
const state = {
  frame: null, frames: 0, trajectory: [],
  map: null, mapSeq: -1,                // the matching submap, in the frontend's local frame
  globalMap: null, globalMapSeq: -1,    // every session composed at optimized poses, global
  graph: null, graphSeq: -1,
  textures: new Map(),                  // 'session/index' -> raster, one per finished submap
  hover: null,                          // submap key the pointer is over in the tree
  selected: null,                       // submap key clicked on the map, or null
  dirty: false,
};

function sx(x) { return view.x + x * view.scale; }
function sy(y) { return view.y - y * view.scale; }

// The frontend publishes in its own local frame, the backend in the global one. They coincide
// only until a map is loaded or a session rotates. Null before the first graph arrives.
function alignment() {
  return (state.graph && state.graph.to_global) || null;
}

function toGlobal(pose) {
  const a = alignment();
  if (!a) return pose;
  const cos = Math.cos(a[2]), sin = Math.sin(a[2]);
  return [cos * pose[0] - sin * pose[1] + a[0], sin * pose[0] + cos * pose[1] + a[1],
          pose[2] + a[2]];
}

function resize() {
  const dpr = devicePixelRatio || 1;
  canvas.width = innerWidth * dpr;
  canvas.height = innerHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  state.dirty = true;
}

function centreOn(pose) {
  view.x = innerWidth / 2 - pose[0] * view.scale;
  view.y = innerHeight / 2 + pose[1] * view.scale;
  view.ready = true;
  state.dirty = true;
}

// The rotation cannot be folded into the destination rectangle: the raster is axis aligned in
// `frame`, not in the world.
function drawGrid(m, frame) {
  const [ax, ay, atheta] = frame || [0, 0, 0];
  ctx.imageSmoothingEnabled = false;
  ctx.save();
  ctx.translate(sx(ax), sy(ay));
  ctx.rotate(-atheta);  // screen y points down, so a positive yaw turns the other way
  ctx.drawImage(m.canvas,
                m.originX * view.scale,
                -(m.originY + m.height * m.resolution) * view.scale,
                m.width * m.resolution * view.scale,
                m.height * m.resolution * view.scale);
  ctx.restore();
}

function drawScaleBar() {
  const candidates = [0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
  const metres = candidates.find(m => m * view.scale > 60) || candidates[candidates.length - 1];
  document.querySelector('#scale .bar').style.width = `${metres * view.scale}px`;
  document.getElementById('scale-text').textContent = `${metres} m`;
}
