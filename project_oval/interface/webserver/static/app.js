// --- State ---
let graphData   = null;
let selection   = [];    // up to 2 vertex ids (int)
let currentPath = [];    // ordered list of int ids
let currentDist = 0;

// Leaflet layers
let markerLayers = {};   // id -> L.circleMarker
let edgeLayers   = [];   // L.polyline[]
let pathLayer    = null;
let selLayers    = {};   // id -> highlight circleMarker

// Label lookup built once after graph loads
let labelById = {};

// --- Map init ---
const map = L.map('map');
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© OpenStreetMap contributors',
  maxZoom: 20,
}).addTo(map);

// --- UI refs ---
const hState    = document.getElementById('h-state');
const status    = document.getElementById('status');
const pathList  = document.getElementById('path-list');
const pathCount = document.getElementById('path-count');
const btnSend   = document.getElementById('btn-send');
const distBadge = document.getElementById('dist-badge');
const distVal   = document.getElementById('dist-val');

// --- Bootstrap ---
async function init() {
  setStatus('Loading map data...', '');
  try {
    const res = await fetch('/graph');
    if (!res.ok) throw new Error('Server returned ' + res.status);
    graphData = await res.json();
    buildLabelIndex();
    buildMap();
    map.setView(graphData.origin, 18);
    hState.textContent = 'READY';
    setStatus('Map loaded — click a node to begin', 'ok');
  } catch (e) {
    hState.textContent = 'ERROR';
    setStatus('Failed to load map: ' + e.message, 'err');
  }
}

function buildLabelIndex() {
  graphData.nodes.forEach(n  => { labelById[n.id]  = n.node_name || ('Node ' + n.id); });
  graphData.points.forEach(p => { labelById[p.id]  = 'Pt ' + p.id; });
}

// --- Map rendering ---
function buildMap() {
  drawEdges();
  drawNodes();
}

function drawEdges() {
  const coordsById = {};
  graphData.nodes.forEach(n  => { coordsById[n.id]  = n.latlon; });
  graphData.points.forEach(p => { coordsById[p.id]  = p.latlon; });

  const drawn = new Set();
  graphData.points.forEach(p => {
    p.linked_points.forEach(qid => {
      const key = [Math.min(p.id, qid), Math.max(p.id, qid)].join('_');
      if (drawn.has(key) || !(qid in coordsById)) return;
      drawn.add(key);
      const line = L.polyline([coordsById[p.id], coordsById[qid]], {
        color: '#BDBDBD', weight: 1.5, opacity: 0.6,
      }).addTo(map);
      edgeLayers.push(line);
    });
  });
}

function drawNodes() {
  // Named landmark nodes
  graphData.nodes.forEach(n => {
    const m = L.circleMarker(n.latlon, {
      radius: 10, fillColor: '#CC0000', color: '#990000', weight: 2, fillOpacity: 1,
    }).addTo(map);
    m.bindTooltip(n.node_name, { permanent: false, direction: 'top' });
    m.on('click', () => handleClick(n.id));
    markerLayers[n.id] = m;
  });

  // Path waypoints
  graphData.points.forEach(p => {
    const m = L.circleMarker(p.latlon, {
      radius: 4, fillColor: '#FFFFFF', color: '#CC0000', weight: 1.2, fillOpacity: 0.9,
    }).addTo(map);
    m.bindTooltip('Point ' + p.id, { permanent: false, direction: 'top' });
    m.on('click', () => handleClick(p.id));
    markerLayers[p.id] = m;
  });
}

// --- Interaction ---
function handleClick(id) {
  if (selection.includes(id)) {
    setStatus('Already selected — pick a different point', 'err');
    return;
  }
  if (selection.length >= 2) {
    clearPath();
  }

  selection.push(id);
  highlightSelected(id, selection.length === 1 ? '#212121' : '#CC0000');

  if (selection.length === 1) {
    setStatus('Start: ' + labelById[id] + ' — now click destination', '');
    hState.textContent = 'START SET';
  } else {
    computePath();
  }
}

function highlightSelected(id, color) {
  const coords = getCoords(id);
  if (!coords) return;
  if (selLayers[id]) selLayers[id].remove();
  selLayers[id] = L.circleMarker(coords, {
    radius: 13, fillColor: color, color: '#fff', weight: 2, fillOpacity: 0.9,
  }).addTo(map);
}

function getCoords(id) {
  const n = graphData.nodes.find(x => x.id === id);
  if (n) return n.latlon;
  const p = graphData.points.find(x => x.id === id);
  if (p) return p.latlon;
  return null;
}

// --- Pathfinding ---
async function computePath() {
  const [start, goal] = selection;
  setStatus('Computing path...', '');
  hState.textContent = 'COMPUTING';

  try {
    const res = await fetch('/find_path', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ start, goal }),
    });

    if (!res.ok) {
      const err = await res.json();
      setStatus('No path: ' + err.detail, 'err');
      hState.textContent = 'NO PATH';
      return;
    }

    const data = await res.json();
    currentPath = data.path;
    currentDist = data.distance_m;
    drawPath(data.coords);
    updateSidebar();
    hState.textContent = 'PATH READY';
    setStatus('Path found — ' + data.path.length + ' points, ' + data.distance_m + ' m', 'ok');

  } catch (e) {
    setStatus('Error: ' + e.message, 'err');
    hState.textContent = 'ERROR';
  }
}

function drawPath(coords) {
  if (pathLayer) { pathLayer.remove(); pathLayer = null; }
  pathLayer = L.polyline(coords.map(c => c.latlon), {
    color: '#CC0000', weight: 4, opacity: 0.9, dashArray: '8 5',
  }).addTo(map);
  map.fitBounds(pathLayer.getBounds(), { padding: [40, 40] });
}

// --- Sidebar ---
function updateSidebar() {
  pathCount.textContent = currentPath.length;
  btnSend.disabled = currentPath.length < 2;
  distBadge.style.display = 'block';
  distVal.textContent = currentDist + ' m';

  pathList.innerHTML = currentPath.map((id, i) =>
    `<div class="pi">
      <span class="pi-idx">${i + 1}</span>
      <span class="pi-id">${id}</span>
      <span class="pi-name">${labelById[id] || ''}</span>
    </div>`
  ).join('');
}

// --- Clear ---
function clearPath() {
  selection   = [];
  currentPath = [];
  currentDist = 0;

  Object.values(selLayers).forEach(l => l.remove());
  selLayers = {};
  if (pathLayer) { pathLayer.remove(); pathLayer = null; }

  pathCount.textContent    = '0';
  btnSend.disabled         = true;
  distBadge.style.display  = 'none';
  pathList.innerHTML       = '<div class="no-path">No path defined</div>';
  hState.textContent       = 'READY';
  setStatus('Path cleared — click a node to begin', '');
}

// --- Send to car ---
async function sendPath() {
  if (currentPath.length < 2) return;
  btnSend.disabled   = true;
  hState.textContent = 'TRANSMITTING';
  setStatus('Sending path to car...', '');

  try {
    const res = await fetch('/send_path', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ path: currentPath }),
    });

    if (res.ok) {
      setStatus('Path sent successfully', 'ok');
      hState.textContent = 'PATH SENT';
    } else {
      const err = await res.json();
      setStatus('Error: ' + (err.detail || 'unknown'), 'err');
      hState.textContent = 'ERROR';
    }
  } catch (e) {
    setStatus('Connection error: ' + e.message, 'err');
    hState.textContent = 'DISCONNECTED';
  }

  btnSend.disabled = false;
}

// --- Utility ---
function setStatus(msg, type) {
  status.textContent = msg;
  status.className   = type || '';
}

// --- Start ---
init();