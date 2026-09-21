// The lifelong backend's view, all in the global frame; pulled whenever graph_seq moves.

// Green at zero, red past kBadResidual: what a bad loop closure looks like before it drags
// the map with it.
const kBadResidual = 0.20;

function residualColor(metres) {
  const hue = 140 * Math.max(0, 1 - metres / kBadResidual);
  return `hsl(${hue.toFixed(0)},70%,55%)`;
}

let fetchingGraph = false;
async function fetchGraph() {
  if (fetchingGraph) return;
  fetchingGraph = true;
  try {
    const response = await fetch('graph');
    if (response.status !== 200) return;
    state.graph = await response.json();
    syncTextures(state.graph);
    state.dirty = true;
    renderTree();
    renderBackendPanel();
  } finally {
    fetchingGraph = false;
  }
}

// --- drawing ---------------------------------------------------------------
function drawEdges(graph, wanted, color, width, dash) {
  const { type, ends, session, residual } = graph.constraints;
  ctx.strokeStyle = color || '#000';
  ctx.lineWidth = width;
  ctx.setLineDash(dash || []);
  ctx.beginPath();
  for (let i = 0; i < type.length; i++) {
    if (type[i] !== wanted) continue;
    if (session && !edgeVisible(session[i * 2], session[i * 2 + 1])) continue;
    // The odometry chain is thousands of edges of one colour and batches into a single path.
    if (color === null) {
      ctx.stroke();
      ctx.strokeStyle = residualColor(residual[i * 2]);
      ctx.beginPath();
    }
    ctx.moveTo(sx(ends[i * 4]), sy(ends[i * 4 + 1]));
    ctx.lineTo(sx(ends[i * 4 + 2]), sy(ends[i * 4 + 3]));
  }
  ctx.stroke();
  ctx.setLineDash([]);
}

function highlighted(submap) {
  const key = `${submap.session}/${submap.index}`;
  return state.hover === key || state.selected === key;
}

function drawFootprints(graph, onlyHighlighted, expandedOnly) {
  for (const submap of graph.submaps) {
    if (onlyHighlighted !== highlighted(submap)) continue;
    if (expandedOnly && !expanded.has(submap.session)) continue;
    if (!submap.box || !submapVisible(submap.session, submap.index)) continue;
    ctx.strokeStyle = sessionColor(submap.session);
    ctx.globalAlpha = onlyHighlighted ? 1 : (submap.finished ? 0.35 : 0.6);
    ctx.lineWidth = onlyHighlighted ? 2.5 : 1;
    ctx.setLineDash(submap.finished ? [] : [4, 3]);
    ctx.beginPath();
    for (let i = 0; i < 8; i += 2) {
      const x = sx(submap.box[i]), y = sy(submap.box[i + 1]);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
  ctx.setLineDash([]);
}

function drawNodes(graph) {
  const frozen = new Set(graph.sessions.filter(s => s.frozen).map(s => s.id));
  const { session, pose } = graph.nodes;
  const size = Math.max(1.5, Math.min(4, view.scale / 10));
  for (let i = 0; i < session.length; i++) {
    if (!sessionVisible(session[i])) continue;
    ctx.fillStyle = sessionColor(session[i]);
    // Dimmed so a glance tells the frozen base map from the session still being optimized.
    ctx.globalAlpha = frozen.has(session[i]) ? 0.45 : 1;
    ctx.fillRect(sx(pose[i * 3]) - size / 2, sy(pose[i * 3 + 1]) - size / 2, size, size);
  }
  ctx.globalAlpha = 1;
}

// A session that will not freeze says so here: the ellipse failing the gate is holding it open.
function drawFreeze(graph) {
  const poses = new Map();
  for (const submap of graph.submaps) {
    poses.set(`${submap.session}/${submap.index}`, submap.pose);
  }
  ctx.lineWidth = 1.5;
  for (const entry of graph.freeze.submaps) {
    if (!entry.available || !submapVisible(entry.session, entry.index)) continue;
    const pose = poses.get(`${entry.session}/${entry.index}`);
    if (!pose) continue;
    ctx.strokeStyle = entry.passed ? '#38bdf8' : '#f87171';
    ctx.beginPath();
    ctx.ellipse(sx(pose[0]), sy(pose[1]), Math.max(2, 3 * entry.stddev[0] * view.scale),
                Math.max(2, 3 * entry.stddev[1] * view.scale), 0, 0, Math.PI * 2);
    ctx.stroke();
  }
}

// Circle with a heading tick, so a submap origin never reads as one more node square.
function drawSubmapPoses(graph) {
  const radius = Math.max(3.5, Math.min(5, view.scale / 5));
  ctx.font = '10px ui-monospace, Menlo, Consolas, monospace';
  ctx.textBaseline = 'middle';
  ctx.lineJoin = 'round';
  for (const submap of graph.submaps) {
    if (!submapVisible(submap.session, submap.index)) continue;
    const [px, py, yaw] = submap.pose;
    const x = sx(px), y = sy(py);
    const color = sessionColor(submap.session);
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = highlighted(submap) ? 2.5 : 1.5;
    ctx.setLineDash(submap.finished ? [] : [3, 2]);
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    submap.finished ? ctx.fill() : ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + Math.cos(yaw) * 12, y - Math.sin(yaw) * 12);
    ctx.stroke();
    if (view.scale < 15 && !highlighted(submap)) continue;
    const label = `${submap.session}/${submap.index}`;
    ctx.lineWidth = 3;
    ctx.strokeStyle = '#2b3440';
    ctx.strokeText(label, x + radius + 4, y);
    ctx.fillText(label, x + radius + 4, y);
  }
  ctx.lineWidth = 1;
  ctx.lineJoin = 'miter';
  ctx.setLineDash([]);
}

function drawGraph() {
  const graph = state.graph;
  if (!graph) return;
  // Without the layer the outlines still follow the tree: an expanded session shows its parts.
  drawFootprints(graph, false, !layerOn('g.boxes'));
  if (layerOn('g.edges')) drawEdges(graph, 0, '#3f4a58', 1);
  if (layerOn('g.priors')) drawEdges(graph, 2, '#a78bfa', 1.5, [3, 3]);
  if (layerOn('g.recov')) drawEdges(graph, 3, '#fb923c', 1.5, [5, 3]);
  if (layerOn('g.loops')) drawEdges(graph, 1, null, 1.5);
  if (layerOn('g.freeze')) drawFreeze(graph);
  if (layerOn('g.submaps')) drawSubmapPoses(graph);
  if (layerOn('g.nodes')) drawNodes(graph);
  // Regardless of the outline layer: a hovered or selected row must show its submap either way.
  if (state.hover || state.selected) drawFootprints(graph, true);
}

// --- panel -----------------------------------------------------------------
function row(label, value, className, title) {
  const hint = title ? ` title="${title}"` : '';
  return `<tr${hint}><td>${label}</td><td class="${className || ''}">${value}</td></tr>`;
}

function renderBackendPanel() {
  const box = document.getElementById('backend');
  const graph = state.graph;
  if (!graph) return;
  box.style.display = 'block';
  const s = graph.stats;
  const accepted = s.matches_attempted
      ? (100 * s.matches_accepted / s.matches_attempted).toFixed(0) : '0';
  let html = '<h4>graph</h4><table>';
  html += row('nodes', s.nodes) + row('submaps', s.submaps) + row('constraints', s.constraints);
  html += row('variables', s.variables);
  // Fewer residual blocks than constraints is normal: a frozen session's constraints never
  // reach the solver.
  html += row('residual blocks', s.residual_blocks, '',
              'constraints handed to the solver; frozen sessions&#39; are not among them');
  html += '</table><h4>loop closure</h4><table>';
  html += row('attempted', s.matches_attempted);
  html += row('accepted', `${s.matches_accepted} (${accepted}%)`);
  html += '</table><h4>trimming</h4><table>';
  html += row('submaps trimmed', s.submaps_trimmed) + row('priors added', s.priors_added);
  html += '</table><h4>freeze verdict</h4><table>';
  html += row('eligible', graph.freeze.eligible ? 'yes' : 'no',
              graph.freeze.eligible ? 'good' : '');
  if (!graph.freeze.eligible) html += row('blocked by', graph.freeze.rejection, 'warn');
  const failing = graph.freeze.submaps.filter(u => u.available && !u.passed);
  if (failing.length) {
    const worst = Math.max(...failing.map(u => Math.hypot(u.stddev[0], u.stddev[1])));
    html += row('over threshold', `${failing.length} submaps`, 'bad');
    html += row('worst σ', `${(100 * worst).toFixed(1)} cm`, 'bad');
  }
  html += '</table><h4>persistence</h4><table>';
  if (s.map_dir === undefined) {
    html += row('map dir', 'none — nothing persists', 'warn');
  } else {
    html += row('checkpoints', s.checkpoints) + row('frozen files', s.frozen_files);
  }
  if (s.insertions_dropped) html += row('dropped keyframes', s.insertions_dropped, 'bad');
  html += '</table><h4>textures</h4><table>';
  html += row('loaded', `${state.textures.size} / ${graph.submaps.filter(s => s.finished).length}`,
              '', 'one raster per finished submap, dropped when the graph stops listing it');
  html += '</table>';
  // Skipped rather than shown stale when the trimmer dropped the selected submap.
  const chosen = state.selected &&
      graph.submaps.find(s => `${s.session}/${s.index}` === state.selected);
  if (chosen) {
    html += '<h4>selected submap</h4><table>';
    html += row('id', `${chosen.session}/${chosen.index}`);
    html += row('scans', chosen.scans);
    html += row('state', chosen.finished ? 'finished' : 'open');
    html += row('x, y', `${chosen.pose[0].toFixed(2)}, ${chosen.pose[1].toFixed(2)} m`);
    html += row('yaw', `${chosen.pose[2].toFixed(3)} rad`);
    html += '</table>';
  }
  box.innerHTML = html;
}
