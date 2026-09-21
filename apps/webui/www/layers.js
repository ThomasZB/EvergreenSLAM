// What is drawn and what is not. Every draw path asks this file rather than keeping its own flag.

const LAYERS = [
  { group: 'maps', key: 'submaps',  label: 'submap textures', color: '#dfe4ea', on: true,
    hint: 'each finished submap at its optimized pose' },
  { group: 'maps', key: 'composed', label: 'composed map',    color: '#94a3b8', on: false,
    hint: '/global_map: one raster, cannot be taken apart' },
  { group: 'maps', key: 'active',   label: 'matching submap', color: '#cbd5e1', on: true,
    hint: 'what the matcher matched against, live' },
  { group: 'maps', key: 'tint',     label: 'tint submaps',    color: '#f0abfc', on: false,
    hint: 'one hue per submap so overlaps show which two disagree' },

  { group: 'frontend', key: 'matched',   label: 'scan @ matched',   color: '#4ade80', on: true },
  { group: 'frontend', key: 'predicted', label: 'scan @ predicted', color: '#f87171', on: true },
  { group: 'frontend', key: 'coarse',    label: 'scan @ coarse',    color: '#fbbf24', on: false },
  { group: 'frontend', key: 'robot',     label: 'robot',            color: '#eef2f7', on: true },
  { group: 'frontend', key: 'traj',      label: 'odometry track',   color: '#60a5fa', on: true },

  { group: 'backend', key: 'g.nodes',  label: 'optimized nodes',   color: '#e879f9', on: true },
  { group: 'backend', key: 'g.submaps', label: 'submap poses',     color: '#cbd5e1', on: true },
  { group: 'backend', key: 'g.loops',  label: 'loop closures',     color: '#4ade80', on: true },
  { group: 'backend', key: 'g.edges',  label: 'odometry edges',    color: '#3f4a58', on: false },
  { group: 'backend', key: 'g.recov',  label: 'trim summaries',    color: '#fb923c', on: true },
  { group: 'backend', key: 'g.priors', label: 'priors',            color: '#a78bfa', on: true },
  { group: 'backend', key: 'g.boxes',  label: 'submap outlines',   color: '#64748b', on: false },
  { group: 'backend', key: 'g.freeze', label: 'freeze 3σ',         color: '#38bdf8', on: true },
];

const GROUPS = [['maps', 'maps'], ['frontend', 'frontend'], ['backend', 'pose graph']];

// One hue per session, cycled; everything else borrows a fixed colour.
const SESSION_COLORS = ['#e879f9', '#38bdf8', '#facc15', '#fb7185', '#34d399', '#c084fc'];
function sessionColor(id) { return SESSION_COLORS[id % SESSION_COLORS.length]; }

const hidden = { sessions: new Set(), submaps: new Set() };
const expanded = new Set();

const byKey = new Map(LAYERS.map(layer => [layer.key, layer]));
function layerOn(key) { const layer = byKey.get(key); return !!layer && layer.on; }
function allLayers() { return LAYERS; }

function sessionVisible(id) { return !hidden.sessions.has(id); }
function submapVisible(session, index) {
  return sessionVisible(session) && !hidden.submaps.has(`${session}/${index}`);
}
// A cross-session edge belongs to both ends, so hiding either end does not hide it.
function edgeVisible(from, to) { return sessionVisible(from) || sessionVisible(to); }

// Layer choices survive a reload; session and submap ids mean nothing in the next run.
const kStore = 'evergreenslam.layers';
function restore() {
  try {
    const saved = JSON.parse(localStorage.getItem(kStore) || '{}');
    for (const layer of LAYERS) {
      if (typeof saved[layer.key] === 'boolean') layer.on = saved[layer.key];
    }
  } catch (error) { /* a corrupt entry is not worth a blank page */ }
}
function persist() {
  try {
    localStorage.setItem(kStore, JSON.stringify(
        Object.fromEntries(LAYERS.map(layer => [layer.key, layer.on]))));
  } catch (error) { /* private mode */ }
}

// --- the tree ---------------------------------------------------------------
function makeRow({ label, color, on, note, hint, twist, onToggle, onName, onHover }) {
  const row = document.createElement('div');
  row.className = on ? 'row' : 'row off';
  if (hint) row.title = hint;
  row.innerHTML = (twist !== undefined ? `<span class="twist">${twist}</span>` : '') +
                  `<span class="box" style="color:${color}"></span>` +
                  `<span class="name">${label}</span>` +
                  (note ? `<span class="note">${note}</span>` : '');
  row.onclick = event => {
    // Alt-click solos.
    if (onName && event && event.target && event.target.className === 'name' && !event.altKey) {
      onName();
      return;
    }
    onToggle(!!(event && event.altKey));
    state.dirty = true;
  };
  if (onHover) {
    row.onmouseenter = () => { state.hover = onHover; state.dirty = true; };
    row.onmouseleave = () => { state.hover = null; state.dirty = true; };
  }
  return row;
}

function renderTree() {
  const box = document.getElementById('tree');
  const parts = [];
  for (const [group, title] of GROUPS) {
    const heading = document.createElement('h4');
    heading.textContent = title;
    parts.push(heading);
    for (const layer of LAYERS.filter(l => l.group === group)) {
      parts.push(makeRow({
        label: layer.label, color: layer.color, on: layer.on, hint: layer.hint,
        onToggle: solo => {
          if (solo) {
            for (const other of LAYERS) {
              if (other.group === group) other.on = other === layer;
            }
          } else {
            layer.on = !layer.on;
          }
          persist();
          renderTree();
        },
      }));
    }
  }
  const graph = state.graph;
  if (graph) {
    const heading = document.createElement('h4');
    heading.textContent = 'sessions';
    parts.push(heading);
    for (const session of graph.sessions) {
      const open = expanded.has(session.id);
      parts.push(makeRow({
        label: `#${session.id}${session.fed ? ' fed' : ''}` +
               `${session.frozen ? ' frozen' : ''}`,
        color: sessionColor(session.id), on: sessionVisible(session.id),
        note: `${session.nodes}n ${session.submaps}s`, twist: open ? '▾' : '▸',
        hint: session.frozen ? 'frozen: constant in the solver, never optimized again'
                             : 'active: still being optimized',
        onToggle: solo => {
          if (solo) {
            hidden.sessions =
                new Set(graph.sessions.map(s => s.id).filter(id => id !== session.id));
          } else if (!hidden.sessions.delete(session.id)) {
            hidden.sessions.add(session.id);
          }
          renderTree();
        },
        onName: () => {
          expanded.has(session.id) ? expanded.delete(session.id) : expanded.add(session.id);
          renderTree();
        },
      }));
      if (!open) continue;
      const kids = document.createElement('div');
      kids.className = 'kids';
      for (const submap of graph.submaps.filter(s => s.session === session.id)) {
        const key = `${submap.session}/${submap.index}`;
        const row = makeRow({
          label: `submap ${submap.index}`, color: sessionColor(session.id),
          on: !hidden.submaps.has(key),
          note: `${submap.scans}·${submap.finished ? 'fin' : 'open'}`,
          hint: submap.finished ? 'finished: immutable, has a texture'
                                : 'still being written by the frontend, no texture yet',
          onHover: key,
          onToggle: () => {
            if (!hidden.submaps.delete(key)) hidden.submaps.add(key);
            renderTree();
          },
          onName: () => centreOn(submap.pose),
        });
        if (state.selected === key) row.classList.add('selected');
        kids.appendChild(row);
      }
      parts.push(kids);
    }
  }
  const hint = document.createElement('p');
  hint.className = 'hint';
  hint.innerHTML = 'click toggles, alt-click solos<br>' +
                   'a name expands, or centres on the submap<br>' +
                   'click a submap on the map selects it, <b>esc</b> clears<br>' +
                   'drag pans, wheel zooms, <b>f</b> follows, <b>c</b> centres';
  parts.push(hint);
  box.replaceChildren(...parts);
}
