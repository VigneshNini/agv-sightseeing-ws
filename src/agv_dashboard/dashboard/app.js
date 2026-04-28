// AGV Dashboard JavaScript - WebSocket client with real-time visualization

const WS_URL = `ws://${window.location.hostname}:9090`;
const RECONNECT_INTERVAL = 3000;
const MAP_SCALE = 5;  // pixels per meter
const MAP_OFFSET = { x: 250, y: 225 };

let ws = null;
let reconnectTimer = null;
let emergencyActive = false;
let mapCanvas, mapCtx, speedCanvas, speedCtx;
let agvState = {
  battery_percent: 100, speed_mps: 0, heading_deg: 0,
  mode: 'IDLE', current_stop: '', emergency_stop: false,
  latitude: 0, longitude: 0, system_status: 'OK',
  obstacles: [], pose_x: 0, pose_y: 0,
  behavior_state: 'IDLE', safety_level: 'SAFE'
};

const TOUR_STOPS = [
  { name: 'Entrance Gate', x: 0, y: 0, id: 0 },
  { name: 'Fountain', x: 15, y: 20, id: 1 },
  { name: 'Rose Garden', x: 10, y: 40, id: 2 },
  { name: 'Art Museum', x: 30, y: 50, id: 3 },
  { name: 'Viewpoint', x: 50, y: 30, id: 4 },
];

// ========================== WebSocket ==========================
function connect() {
  if (ws) { ws.close(); ws = null; }
  setConnectionStatus(false);
  try {
    ws = new WebSocket(WS_URL);
    ws.onopen = () => {
      setConnectionStatus(true);
      addLog('Connected to AGV WebSocket bridge', 'info');
      if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
    };
    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === 'state') updateState(msg.data);
      } catch (e) { console.error('Parse error:', e); }
    };
    ws.onclose = () => {
      setConnectionStatus(false);
      addLog('Disconnected from AGV. Reconnecting...', 'warn');
      scheduleReconnect();
    };
    ws.onerror = () => {
      addLog('WebSocket error', 'error');
    };
  } catch (e) {
    addLog(`Connection failed: ${e.message}`, 'error');
    scheduleReconnect();
  }
}

function scheduleReconnect() {
  if (!reconnectTimer) {
    reconnectTimer = setInterval(connect, RECONNECT_INTERVAL);
  }
}

function sendCommand(type, value) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type, value }));
  } else {
    addLog('Cannot send command: not connected', 'warn');
  }
}

// ========================== UI Updates ==========================
function setConnectionStatus(connected) {
  const el = document.getElementById('connection-status');
  el.textContent = connected ? '● CONNECTED' : '● DISCONNECTED';
  el.className = `status-badge ${connected ? 'connected' : 'disconnected'}`;
}

function updateState(data) {
  agvState = { ...agvState, ...data };
  updateModeDisplay();
  updateSafetyDisplay();
  updateBattery();
  updateSpeed();
  updateHeading();
  updateCurrentStop();
  updatePosition();
  updateObstacles();
  updateEmergency();
  drawMap();
}

function updateModeDisplay() {
  const el = document.getElementById('mode-display');
  el.textContent = agvState.behavior_state;
  const modeMap = {
    'IDLE': 'mode-idle', 'NAVIGATE': 'mode-navigate',
    'DWELL': 'mode-dwell', 'EMERGENCY': 'mode-emergency',
    'TOUR_COMPLETE': 'mode-dwell'
  };
  el.className = 'card-value ' + (modeMap[agvState.behavior_state] || 'mode-idle');
}

function updateSafetyDisplay() {
  const el = document.getElementById('safety-display');
  el.textContent = agvState.safety_level;
  const safetyMap = {
    'SAFE': 'safety-safe', 'CAUTION': 'safety-caution',
    'WARNING': 'safety-warning', 'EMERGENCY': 'safety-emergency'
  };
  el.className = 'card-value ' + (safetyMap[agvState.safety_level] || 'safety-safe');
}

function updateBattery() {
  const pct = Math.max(0, Math.min(100, agvState.battery_percent));
  const fill = document.getElementById('battery-fill');
  const text = document.getElementById('battery-text');
  fill.style.width = `${pct}%`;
  text.textContent = `${pct.toFixed(1)}%`;
  fill.className = 'battery-fill' + (pct < 20 ? ' low' : pct < 40 ? ' medium' : '');
}

function updateSpeed() {
  const speed = agvState.speed_mps;
  document.getElementById('speed-text').textContent = `${speed.toFixed(2)} m/s`;
  drawSpeedGauge(speed);
}

function updateHeading() {
  document.getElementById('heading-display').textContent = `${agvState.heading_deg.toFixed(1)}°`;
}

function updateCurrentStop() {
  const el = document.getElementById('current-stop');
  el.textContent = agvState.current_stop || 'None';
  document.getElementById('system-status').textContent = agvState.system_status || 'OK';
}

function updatePosition() {
  document.getElementById('gps-display').textContent =
    `${agvState.latitude.toFixed(6)}, ${agvState.longitude.toFixed(6)}`;
  document.getElementById('position-display').textContent =
    `X: ${agvState.pose_x.toFixed(2)}m, Y: ${agvState.pose_y.toFixed(2)}m`;
}

function updateObstacles() {
  const obs = agvState.obstacles || [];
  const alertDiv = document.getElementById('obstacle-alerts');
  const listDiv = document.getElementById('obstacle-list');
  if (obs.length === 0) {
    alertDiv.classList.add('hidden');
    return;
  }
  alertDiv.classList.remove('hidden');
  listDiv.replaceChildren(...obs.slice(0, 5).map(o => {
    const div = document.createElement('div');
    div.textContent = `ID:${o.id} ${o.type} @ (${o.x.toFixed(1)}, ${o.y.toFixed(1)})m`;
    return div;
  }));
}

function updateEmergency() {
  const em = agvState.emergency_stop;
  const indicator = document.getElementById('emergency-indicator');
  if (em) {
    indicator.classList.remove('hidden');
    document.getElementById('estop-btn').textContent = '✅ Release E-Stop';
  } else {
    indicator.classList.add('hidden');
    document.getElementById('estop-btn').textContent = '🛑 EMERGENCY STOP';
  }
}

// ========================== Controls ==========================
function startTour() { sendCommand('start_tour', true); addLog('Tour started', 'info'); }
function stopTour() { sendCommand('stop_tour', false); addLog('Tour stopped', 'warn'); }
function toggleEStop() {
  const activate = !agvState.emergency_stop;
  sendCommand('emergency_stop', activate);
  addLog(`Emergency stop ${activate ? 'ACTIVATED' : 'released'}`, activate ? 'error' : 'info');
}

// ========================== Speed Gauge ==========================
function drawSpeedGauge(speed) {
  if (!speedCtx) return;
  const MAX_SPEED = 3.0;
  const W = 120, H = 80;
  speedCtx.clearRect(0, 0, W, H);
  const cx = W / 2, cy = H * 0.85;
  const r = 50;
  const startAngle = Math.PI;
  const endAngle = 2 * Math.PI;
  speedCtx.beginPath();
  speedCtx.arc(cx, cy, r, startAngle, endAngle);
  speedCtx.strokeStyle = '#30363d';
  speedCtx.lineWidth = 8;
  speedCtx.stroke();
  const frac = Math.min(speed / MAX_SPEED, 1.0);
  if (frac > 0) {
    speedCtx.beginPath();
    speedCtx.arc(cx, cy, r, startAngle, startAngle + frac * Math.PI);
    speedCtx.strokeStyle = frac > 0.8 ? '#f85149' : frac > 0.5 ? '#d29922' : '#3fb950';
    speedCtx.lineWidth = 8;
    speedCtx.stroke();
  }
  const angle = startAngle + frac * Math.PI;
  speedCtx.beginPath();
  speedCtx.moveTo(cx, cy);
  speedCtx.lineTo(cx + (r - 12) * Math.cos(angle), cy + (r - 12) * Math.sin(angle));
  speedCtx.strokeStyle = '#e6edf3';
  speedCtx.lineWidth = 2;
  speedCtx.stroke();
}

// ========================== Map Drawing ==========================
function worldToCanvas(wx, wy) {
  return {
    x: MAP_OFFSET.x + wx * MAP_SCALE,
    y: MAP_OFFSET.y - wy * MAP_SCALE
  };
}

function drawMap() {
  if (!mapCtx) return;
  const W = mapCanvas.width, H = mapCanvas.height;
  mapCtx.fillStyle = '#0a1a0a';
  mapCtx.fillRect(0, 0, W, H);

  // Grid
  mapCtx.strokeStyle = 'rgba(48, 54, 61, 0.4)';
  mapCtx.lineWidth = 0.5;
  for (let x = 0; x < W; x += MAP_SCALE * 10) {
    mapCtx.beginPath(); mapCtx.moveTo(x, 0); mapCtx.lineTo(x, H); mapCtx.stroke();
  }
  for (let y = 0; y < H; y += MAP_SCALE * 10) {
    mapCtx.beginPath(); mapCtx.moveTo(0, y); mapCtx.lineTo(W, y); mapCtx.stroke();
  }

  // Draw simulated paths between tour stops
  if (TOUR_STOPS.length > 1) {
    mapCtx.setLineDash([4, 4]);
    mapCtx.strokeStyle = 'rgba(88, 166, 255, 0.3)';
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    const p0 = worldToCanvas(TOUR_STOPS[0].x, TOUR_STOPS[0].y);
    mapCtx.moveTo(p0.x, p0.y);
    for (let i = 1; i < TOUR_STOPS.length; i++) {
      const p = worldToCanvas(TOUR_STOPS[i].x, TOUR_STOPS[i].y);
      mapCtx.lineTo(p.x, p.y);
    }
    mapCtx.stroke();
    mapCtx.setLineDash([]);
  }

  // Tour stops
  TOUR_STOPS.forEach((stop) => {
    const p = worldToCanvas(stop.x, stop.y);
    const isCurrentStop = agvState.current_stop === stop.name;
    mapCtx.beginPath();
    mapCtx.arc(p.x, p.y, isCurrentStop ? 10 : 7, 0, Math.PI * 2);
    mapCtx.fillStyle = isCurrentStop ? '#3fb950' : 'rgba(63, 185, 80, 0.5)';
    mapCtx.fill();
    mapCtx.strokeStyle = '#3fb950';
    mapCtx.lineWidth = 1.5;
    mapCtx.stroke();
    mapCtx.fillStyle = '#e6edf3';
    mapCtx.font = '10px sans-serif';
    mapCtx.fillText(stop.name, p.x + 11, p.y + 4);
  });

  // Obstacles
  (agvState.obstacles || []).forEach(obs => {
    const ox = agvState.pose_x + obs.x;
    const oy = agvState.pose_y + obs.y;
    const p = worldToCanvas(ox, oy);
    mapCtx.beginPath();
    mapCtx.arc(p.x, p.y, 5, 0, Math.PI * 2);
    mapCtx.fillStyle = 'rgba(248, 81, 73, 0.7)';
    mapCtx.fill();
    mapCtx.strokeStyle = '#f85149';
    mapCtx.lineWidth = 1;
    mapCtx.stroke();
  });

  // AGV (robot)
  const agvP = worldToCanvas(agvState.pose_x, agvState.pose_y);
  const headRad = (agvState.heading_deg * Math.PI) / 180;
  mapCtx.save();
  mapCtx.translate(agvP.x, agvP.y);
  mapCtx.rotate(-headRad);
  mapCtx.fillStyle = agvState.emergency_stop ? '#f85149' : '#58a6ff';
  mapCtx.strokeStyle = '#fff';
  mapCtx.lineWidth = 1.5;
  mapCtx.beginPath();
  mapCtx.moveTo(0, -10);
  mapCtx.lineTo(7, 8);
  mapCtx.lineTo(0, 4);
  mapCtx.lineTo(-7, 8);
  mapCtx.closePath();
  mapCtx.fill();
  mapCtx.stroke();
  mapCtx.restore();

  // Heading line
  mapCtx.beginPath();
  mapCtx.moveTo(agvP.x, agvP.y);
  const hx = agvP.x + 30 * Math.cos(-headRad + Math.PI / 2);
  const hy = agvP.y + 30 * Math.sin(-headRad + Math.PI / 2);
  mapCtx.lineTo(hx, hy);
  mapCtx.strokeStyle = 'rgba(88, 166, 255, 0.6)';
  mapCtx.lineWidth = 1.5;
  mapCtx.stroke();
}

// ========================== Log ==========================
function addLog(message, level = '') {
  const container = document.getElementById('log-container');
  const entry = document.createElement('div');
  entry.className = `log-entry ${level}`;
  const time = new Date().toLocaleTimeString();
  entry.textContent = `[${time}] ${message}`;
  container.appendChild(entry);
  container.scrollTop = container.scrollHeight;
  while (container.children.length > 100) {
    container.removeChild(container.firstChild);
  }
}

// ========================== Init ==========================
window.addEventListener('DOMContentLoaded', () => {
  mapCanvas = document.getElementById('map-canvas');
  mapCtx = mapCanvas.getContext('2d');
  speedCanvas = document.getElementById('speed-gauge');
  speedCtx = speedCanvas.getContext('2d');

  drawMap();
  drawSpeedGauge(0);

  setInterval(drawMap, 200);
  setInterval(drawSpeedGauge.bind(null, agvState.speed_mps), 200);

  connect();
  addLog('AGV Dashboard initialized', 'info');
});
