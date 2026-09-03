const state = {
  commands: [],
  processes: {},
  telemetry: {},
  map: null,
  logOffsets: {},
  activeLog: "sensors",
  poseSource: "odometry",
};

const $ = (id) => document.getElementById(id);

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "content-type": "application/json" },
    ...options,
  });
  const data = await response.json();
  if (!response.ok || data.error) throw new Error(data.error || response.statusText);
  return data;
}

async function post(path, payload = {}) {
  return api(path, { method: "POST", body: JSON.stringify(payload) });
}

function renderProcesses() {
  const grid = $("processGrid");
  grid.innerHTML = "";
  for (const command of state.commands) {
    const proc = state.processes[command.id] || {};
    const row = document.createElement("div");
    row.className = "process";

    const name = document.createElement("div");
    name.className = "name";
    name.textContent = command.label;

    const dot = document.createElement("div");
    dot.className = `dot ${proc.running ? "running" : ""}`;
    dot.title = proc.running ? `pid ${proc.pid}` : "stopped";

    const button = document.createElement("button");
    button.textContent = proc.running ? "Stop" : "Start";
    button.className = command.danger && !proc.running ? "danger" : "";
    button.onclick = async () => {
      await post(proc.running ? "/api/process/stop" : "/api/process/start", { id: command.id });
      await refreshStatus();
    };

    row.append(name, dot, button);
    grid.append(row);
  }

  for (const select of [$("logProcess"), $("stdinProcess")]) {
    const previous = select.value;
    select.innerHTML = "";
    for (const command of state.commands) {
      const option = document.createElement("option");
      option.value = command.id;
      option.textContent = command.label;
      select.append(option);
    }
    select.value = previous || state.activeLog;
  }
}

function ageText(item) {
  if (!item || !item.received_at) return "no data";
  const age = Math.max(0, Date.now() / 1000 - item.received_at);
  return `${age.toFixed(1)}s ago`;
}

function topicRow(name, item, detail) {
  const alive = item && item.received_at && Date.now() / 1000 - item.received_at < 2.5;
  return `
    <div class="topic">
      <div class="dot ${alive ? "running" : ""}"></div>
      <div>${name}<small>${detail || ageText(item)}</small></div>
    </div>
  `;
}

function renderTopics() {
  const t = state.telemetry || {};
  const steering = t.steering ? `value ${t.steering.value}, ${ageText(t.steering)}` : "";
  const throttle = t.throttle ? `value ${t.throttle.value}, ${ageText(t.throttle)}` : "";
  const gps = t.gps ? `status ${t.gps.status}, ${ageText(t.gps)}` : "";
  $("bridgeStatus").textContent = t.bridge_started ? "ROS bridge active" : (t.bridge_error || "ROS bridge waiting");
  $("topicStatus").innerHTML = [
    topicRow("/odometry/filtered", t.odometry),
    topicRow("/zed/zed_node/pose", t.zed_pose),
    topicRow("/navsatfix", t.gps, gps),
    topicRow("/gemini/steering", t.steering, steering),
    topicRow("/gemini/throttle", t.throttle, throttle),
    topicRow("/motors/rpm", t.rpm),
  ].join("");
}

function boundsForMap() {
  const pts = [];
  if (state.map) {
    pts.push(...state.map.nodes);
    pts.push(...state.map.route);
  }
  const pose = state.telemetry[state.poseSource];
  if (pose) pts.push(pose);
  if (!pts.length) return null;
  const xs = pts.map((p) => p.x);
  const ys = pts.map((p) => p.y);
  return {
    minX: Math.min(...xs),
    maxX: Math.max(...xs),
    minY: Math.min(...ys),
    maxY: Math.max(...ys),
  };
}

function drawMap() {
  const canvas = $("mapCanvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);

  if (!state.map) return;
  const bounds = boundsForMap();
  if (!bounds) return;
  const pad = 32;
  const width = Math.max(1, bounds.maxX - bounds.minX);
  const height = Math.max(1, bounds.maxY - bounds.minY);
  const scale = Math.min((rect.width - pad * 2) / width, (rect.height - pad * 2) / height);
  const nodeById = new Map(state.map.nodes.map((n) => [n.id, n]));
  const project = (p) => ({
    x: pad + (p.x - bounds.minX) * scale,
    y: rect.height - pad - (p.y - bounds.minY) * scale,
  });

  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  ctx.strokeStyle = "#3a463d";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (const edge of state.map.edges) {
    const a = nodeById.get(edge.from);
    const b = nodeById.get(edge.to);
    if (!a || !b) continue;
    const pa = project(a);
    const pb = project(b);
    ctx.moveTo(pa.x, pa.y);
    ctx.lineTo(pb.x, pb.y);
  }
  ctx.stroke();

  ctx.fillStyle = "#b8c3b6";
  for (const node of state.map.nodes) {
    const p = project(node);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 2.2, 0, Math.PI * 2);
    ctx.fill();
  }

  if (state.map.route.length > 1) {
    ctx.strokeStyle = "#5aa9e6";
    ctx.lineWidth = 4;
    ctx.beginPath();
    for (const [index, point] of state.map.route.entries()) {
      const p = project(point);
      if (index === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
  }

  const pose = state.telemetry[state.poseSource];
  if (pose && Number.isFinite(pose.x) && Number.isFinite(pose.y)) {
    const p = project(pose);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-(pose.yaw || 0));
    ctx.fillStyle = "#e0a841";
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(-8, -7);
    ctx.lineTo(-4, 0);
    ctx.lineTo(-8, 7);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

async function refreshStatus() {
  const data = await api("/api/status");
  state.commands = data.commands;
  state.processes = data.processes;
  state.telemetry = data.telemetry;
  renderProcesses();
  renderTopics();
  drawMap();
}

async function refreshMap() {
  state.map = await api("/api/map");
  $("mapMeta").textContent = `${state.map.nodes.length} nodes, ${state.map.edges.length} edges, ${state.map.route.length} route points`;
  drawMap();
}

async function refreshLogs() {
  const id = state.activeLog;
  const offset = state.logOffsets[id] || 0;
  const data = await api(`/api/logs?id=${encodeURIComponent(id)}&offset=${offset}`);
  state.logOffsets[id] = data.next_offset;
  if (!data.lines.length) return;
  const logs = $("logs");
  logs.textContent += data.lines.map((line) => line.text).join("\n") + "\n";
  logs.scrollTop = logs.scrollHeight;
}

$("stopAll").onclick = async () => {
  await post("/api/process/stop_all");
  await refreshStatus();
};

$("sendInput").onclick = async () => {
  const text = $("stdinText").value;
  if (!text) return;
  await post("/api/process/input", { id: $("stdinProcess").value, text });
  $("stdinText").value = "";
};

$("stdinText").addEventListener("keydown", (event) => {
  if (event.key === "Enter") $("sendInput").click();
});

$("logProcess").onchange = () => {
  state.activeLog = $("logProcess").value;
  $("logs").textContent = "";
  state.logOffsets[state.activeLog] = 0;
  refreshLogs().catch(console.error);
};

$("poseSource").onchange = () => {
  state.poseSource = $("poseSource").value;
  drawMap();
};

window.addEventListener("resize", drawMap);

refreshMap().catch(console.error);
refreshStatus().catch(console.error);
setInterval(() => refreshStatus().catch(console.error), 1000);
setInterval(() => refreshLogs().catch(console.error), 700);
