// EGM1 grids: one byte per cell, 0..100 occupancy and 255 unknown, so the raw cell encoding
// never leaves C++. Same wire format on /map, /global_map and /submap.

const kUnknownCell = 255;
// Submap textures only: they overlap two deep, and fading uncertain cells is what makes the
// overlap compose by confidence rather than by draw order.
const kConfident = 0.7;

function decodeGrid(buffer, fade) {
  const head = new DataView(buffer);
  if (String.fromCharCode(head.getUint8(0), head.getUint8(1), head.getUint8(2),
                          head.getUint8(3)) !== 'EGM1') return null;
  const width = head.getInt32(4, true), height = head.getInt32(8, true);
  // drawImage on a zero sized canvas throws and would take the page down.
  if (width === 0 || height === 0) return null;
  const map = {
    width, height,
    resolution: head.getFloat64(12, true),
    originX: head.getFloat64(20, true),
    originY: head.getFloat64(28, true),
    canvas: document.createElement('canvas'),
    tintKey: null,  // which tint the pixels currently carry
  };
  map.cells = new Uint8Array(buffer, 36, width * height);
  map.fade = !!fade;
  map.canvas.width = width;
  map.canvas.height = height;
  recolorGrid(map, null);
  return map;
}

// Textures are decoded once and cached, so a per-submap tint cannot be a draw-time filter: the
// pixels are rewritten in place instead. tint is [r, g, b] or null for plain grey.
function recolorGrid(map, tint) {
  const { width, height, cells } = map;
  const context = map.canvas.getContext('2d');
  const image = context.createImageData(width, height);
  for (let y = 0; y < height; y++) {
    // Cell row 0 is the lowest y in world space, image row 0 is the top, so flip.
    const src = y * width, dst = (height - 1 - y) * width;
    for (let x = 0; x < width; x++) {
      const value = cells[src + x];
      // Transparent so two rasters compose instead of the upper one blanking the lower; the
      // unknown colour lives on the page background.
      if (value === kUnknownCell) continue;
      const i = (dst + x) * 4;
      const grey = 235 - Math.round(value * 2.2);
      if (tint) {
        image.data[i] = Math.round(0.7 * grey + 0.3 * tint[0]);
        image.data[i + 1] = Math.round(0.7 * grey + 0.3 * tint[1]);
        image.data[i + 2] = Math.round(0.7 * grey + 0.3 * tint[2]);
      } else {
        image.data[i] = image.data[i + 1] = image.data[i + 2] = grey;
      }
      image.data[i + 3] =
          map.fade ? Math.round(255 * Math.min(1, Math.abs(value - 50) / 50 / kConfident)) : 255;
    }
  }
  context.putImageData(image, 0, 0);
}

function hslToRgb(h, s, l) {
  const c = (1 - Math.abs(2 * l - 1)) * s, m = l - c / 2;
  const x = c * (1 - Math.abs((h / 60) % 2 - 1));
  const sector = Math.floor(h / 60) % 6;
  const rgb = [[c, x, 0], [x, c, 0], [0, c, x], [0, x, c], [x, 0, c], [c, 0, x]][sector];
  return rgb.map(v => Math.round(255 * (v + m)));
}

async function fetchGrid(url, fade) {
  const response = await fetch(url);
  if (response.status !== 200) return null;
  return decodeGrid(await response.arrayBuffer(), fade);
}

// --- textures --------------------------------------------------------------
// Dropped once the graph stops listing it: the trimmer removes submaps, and a texture nobody
// drops keeps drawing over whatever replaced it.
const asked = new Set();  // every key ever requested; a trimmed submap never comes back
const queue = [];
let running = 0;
// Two at a time: the server's thread pool is small and the SSE stream holds one thread for the
// whole run, so a wide fan-out parks the rest on keep-alive.
const kParallelFetches = 2;

function pump() {
  while (running < kParallelFetches && queue.length > 0) {
    const key = queue.shift();
    const [session, index] = key.split('/');
    running++;
    fetchGrid(`submap?s=${session}&i=${index}`, true).then(raster => {
      // null is cached deliberately: a finished submap with no blob has an empty grid and
      // always will, so refetching it would be a request per graph per submap forever.
      state.textures.set(key, raster);
      state.dirty = true;
    }).catch(() => { asked.delete(key); }).finally(() => {
      running--;
      if (queue.length === 0 && running === 0) renderBackendPanel();
      pump();
    });
  }
}

function syncTextures(graph) {
  const live = new Set();
  for (const submap of graph.submaps) {
    const key = `${submap.session}/${submap.index}`;
    live.add(key);
    if (!submap.finished || state.textures.has(key) || asked.has(key)) continue;
    asked.add(key);
    queue.push(key);
  }
  for (const key of state.textures.keys()) {
    if (!live.has(key)) state.textures.delete(key);
  }
  pump();
}

// A 404 is cached as null, so this is also false for a sink built with serve_submaps off.
function haveTextures() {
  for (const raster of state.textures.values()) {
    if (raster) return true;
  }
  return false;
}

function drawTextures(graph) {
  const tinting = layerOn('tint');
  // Id order, as global_map.cc composes, so the newer of two disagreeing submaps is on top.
  for (const submap of graph.submaps) {
    const key = `${submap.session}/${submap.index}`;
    const raster = state.textures.get(key);
    if (!raster || !submapVisible(submap.session, submap.index)) continue;
    const tintKey = tinting ? String(submap.index) : null;
    if (raster.tintKey !== tintKey) {
      // Golden angle: neighbouring submaps never land on neighbouring hues.
      recolorGrid(raster, tinting ? hslToRgb((submap.index * 137.508) % 360, 0.7, 0.55) : null);
      raster.tintKey = tintKey;
    }
    drawGrid(raster, submap.to_world);
  }
}
