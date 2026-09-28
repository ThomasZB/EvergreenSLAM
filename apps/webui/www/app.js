// Everything here is published per scan by the DebugSink; graph.js owns the backend's view.

const hud = document.getElementById('hud');

function drawScan(points, pose, color) {
  const [px, py, theta] = toGlobal(pose);
  const cos = Math.cos(theta), sin = Math.sin(theta);
  ctx.fillStyle = color;
  const size = Math.max(1.5, Math.min(3, view.scale / 12));
  for (let i = 0; i < points.length; i += 2) {
    const x = points[i], y = points[i + 1];
    const wx = cos * x - sin * y + px, wy = sin * x + cos * y + py;
    ctx.fillRect(sx(wx) - size / 2, sy(wy) - size / 2, size, size);
  }
}

function drawRobot(pose, color) {
  const [px, py, theta] = toGlobal(pose);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(sx(px), sy(py), 7, 0, Math.PI * 2);
  ctx.moveTo(sx(px), sy(py));
  ctx.lineTo(sx(px) + Math.cos(theta) * 16, sy(py) - Math.sin(theta) * 16);
  ctx.stroke();
}

function drawTrajectory(color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let i = 0; i < state.trajectory.length; i += 2) {
    // The whole track rides the newest alignment, so older stretches are only roughly placed.
    const [x, y] = toGlobal([state.trajectory[i], state.trajectory[i + 1], 0]);
    i === 0 ? ctx.moveTo(sx(x), sy(y)) : ctx.lineTo(sx(x), sy(y));
  }
  ctx.stroke();
}

function draw() {
  // The unknown colour: everywhere outside a map has not been seen either.
  ctx.fillStyle = '#2b3440';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Two frames: textures and the composed map are global, the matching submap is local via
  // map_pose. Unknown stays transparent so they compose instead of blanking each other.
  if (state.globalMap && (layerOn('composed') || (layerOn('submaps') && !haveTextures()))) {
    drawGrid(state.globalMap);
  }
  if (state.graph && layerOn('submaps')) drawTextures(state.graph);
  if (state.map && layerOn('active')) {
    drawGrid(state.map, toGlobal((state.frame && state.frame.map_pose) || [0, 0, 0]));
  }

  drawGraph();

  if (layerOn('traj') && state.trajectory.length > 1) drawTrajectory(byKey.get('traj').color);

  if (state.frame) {
    // Reversed so the tree can lead with the answer while the answer still paints on top.
    for (const layer of [...LAYERS].reverse()) {
      const pose = state.frame.poses[layer.key];
      if (layer.on && pose) drawScan(state.frame.scan, pose, layer.color);
    }
    if (layerOn('robot') && state.frame.poses.matched) {
      drawRobot(state.frame.poses.matched, byKey.get('robot').color);
    }
  }
  drawScaleBar();
}

function tick() {
  if (state.dirty) {
    state.dirty = false;
    draw();
  }
  requestAnimationFrame(tick);
}

// --- trajectory ------------------------------------------------------------
// Dropped frames leave gaps, so resync whenever our length falls behind what the server reports.
let fetchingTrajectory = false;
async function fetchTrajectory() {
  if (fetchingTrajectory) return;
  fetchingTrajectory = true;
  try {
    const response = await fetch('trajectory');
    if (response.status !== 200) return;
    state.trajectory = Array.from(new Float64Array(await response.arrayBuffer()));
    const n = state.trajectory.length;
    if (!view.ready && n >= 2) {
      centreOn(toGlobal([state.trajectory[n - 2], state.trajectory[n - 1], 0]));
    }
    state.dirty = true;
  } finally {
    fetchingTrajectory = false;
  }
}

// --- selection -------------------------------------------------------------
// The corners come round the rectangle, so the point is inside when every edge turns the same way.
function insideQuad(box, x, y) {
  let positive = false, negative = false;
  for (let i = 0; i < 8; i += 2) {
    const j = (i + 2) % 8;
    const cross = (box[j] - box[i]) * (y - box[i + 1]) - (box[j + 1] - box[i + 1]) * (x - box[i]);
    if (cross > 0) positive = true;
    if (cross < 0) negative = true;
  }
  return !(positive && negative);
}

const kOpenSubmapRadius = 0.5;  // metres; an unfinished submap has no box to hit

// Submaps overlap a dozen deep in a revisited room, so among the boxes containing the point the
// one whose pose marker is nearest wins: that is the marker the click was aimed at.
function submapAt(x, y) {
  const graph = state.graph;
  if (!graph) return null;
  let best = null, bestDistance = Infinity;
  for (const submap of graph.submaps) {
    if (!submapVisible(submap.session, submap.index)) continue;
    const distance = Math.hypot(submap.pose[0] - x, submap.pose[1] - y);
    const hit = submap.box ? insideQuad(submap.box, x, y) : distance <= kOpenSubmapRadius;
    if (!hit || distance >= bestDistance) continue;
    bestDistance = distance;
    best = submap;
  }
  return best;
}

function selectSubmap(submap) {
  state.selected = submap && `${submap.session}/${submap.index}`;
  if (submap) expanded.add(submap.session);
  renderTree();
  const row = document.querySelector('#tree .row.selected');
  if (row) row.scrollIntoView({ block: 'nearest' });
  renderBackendPanel();
  state.dirty = true;
}

// --- input -----------------------------------------------------------------
let dragging = null;
let pressed = null;
const kClickSlop = 4;  // px; further than this and the press was a pan, not a click
canvas.onmousedown = event => {
  dragging = { x: event.clientX, y: event.clientY };
  pressed = event.button === 0 ? { x: event.clientX, y: event.clientY } : null;
};
canvas.onmouseup = event => {
  if (!pressed) return;
  const moved = Math.hypot(event.clientX - pressed.x, event.clientY - pressed.y);
  pressed = null;
  if (moved >= kClickSlop) return;
  selectSubmap(submapAt((event.clientX - view.x) / view.scale,
                        (view.y - event.clientY) / view.scale));
};
addEventListener('mouseup', () => { dragging = null; pressed = null; });
addEventListener('mousemove', event => {
  if (!dragging) return;
  view.x += event.clientX - dragging.x;
  view.y += event.clientY - dragging.y;
  dragging = { x: event.clientX, y: event.clientY };
  view.follow = false;
  state.dirty = true;
});
canvas.onwheel = event => {
  event.preventDefault();
  const factor = Math.exp(-event.deltaY * 0.0015);
  // Keep the world point under the cursor fixed, otherwise zooming walks the map away.
  view.x = event.clientX - (event.clientX - view.x) * factor;
  view.y = event.clientY - (event.clientY - view.y) * factor;
  view.scale *= factor;
  state.dirty = true;
};
addEventListener('resize', resize);
addEventListener('keydown', event => {
  if (event.key === 'f') {
    view.follow = !view.follow;
  } else if (event.key === 'c' && state.frame && state.frame.poses.matched) {
    centreOn(toGlobal(state.frame.poses.matched));
  } else if (event.key === 'Escape') {
    selectSubmap(null);
  } else {
    return;
  }
  state.dirty = true;
});

// --- stream ----------------------------------------------------------------
const events = new EventSource('events');
events.onmessage = message => {
  const frame = JSON.parse(message.data);
  state.frame = frame;
  // A new epoch is another map: its submaps reuse the cached textures' keys.
  if (frame.epoch !== undefined) {
    if (state.epoch !== undefined && frame.epoch !== state.epoch) {
      location.reload();
      return;
    }
    state.epoch = frame.epoch;
  }
  state.frames++;
  const matched = frame.poses.matched;
  if (matched) {
    if (!view.ready || view.follow) centreOn(toGlobal(matched));
    state.trajectory.push(matched[0], matched[1]);
    if (state.trajectory.length / 2 !== frame.traj_len + 1) fetchTrajectory();
  }
  if (frame.map_seq !== state.mapSeq) {
    state.mapSeq = frame.map_seq;
    fetchGrid('map').then(map => { if (map) { state.map = map; state.dirty = true; } });
  }
  if (frame.global_map_seq !== undefined && frame.global_map_seq !== state.globalMapSeq) {
    state.globalMapSeq = frame.global_map_seq;
    fetchGrid('global_map').then(map => {
      if (map) { state.globalMap = map; state.dirty = true; }
    });
  }
  if (frame.graph_seq !== undefined && frame.graph_seq !== state.graphSeq) {
    state.graphSeq = frame.graph_seq;
    fetchGraph();
  }
  const delta = matched && frame.poses.predicted
      ? Math.hypot(matched[0] - frame.poses.predicted[0], matched[1] - frame.poses.predicted[1])
      : 0;
  const a = alignment();
  hud.innerHTML =
      // The stream is latest wins, so the received count alone understates the pipeline.
      `<b>${frame.traj_len + 1}</b> scans, <b>${state.frames}</b> drawn` +
      ` &nbsp; t <b>${frame.time.toFixed(2)}</b><br>` +
      `score <b>${frame.score.toFixed(3)}</b> &nbsp; ` +
      `match &minus; predict <b>${(delta * 100).toFixed(1)}</b> cm &nbsp; ` +
      `<b>${frame.scan.length / 2}</b> points` +
      (a ? `<br>local &rarr; global <b>${a[0].toFixed(2)}, ${a[1].toFixed(2)}, ` +
           `${a[2].toFixed(3)}</b> rad`
         : `<br><span style="opacity:.6">frontend only, no pose graph</span>`);
  state.dirty = true;
};
events.onerror = () => { hud.innerHTML += '<br><span class="bad">stream closed</span>'; };

restore();
renderTree();
resize();
// A page opened after the run ended gets no frame, so ask once. The graph first: its alignment
// is what everything local is drawn through.
fetchGraph().then(fetchTrajectory);
fetchGrid('map').then(map => { if (map) { state.map = map; state.dirty = true; } });
fetchGrid('global_map').then(map => { if (map) { state.globalMap = map; state.dirty = true; } });
tick();
