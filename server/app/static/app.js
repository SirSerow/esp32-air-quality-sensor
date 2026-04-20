// ── Metric definitions ─────────────────────────────────────────────────────
const METRICS = {
  temperature_c: { label: "Temperature", unit: "°C",  color: "#ef4444",
    thresholds: [{ value: 26, color: "#f97316" }, { value: 30, color: "#ef4444" }] },
  humidity_pct:  { label: "Humidity",    unit: "%",   color: "#3b82f6", yMin: 0, yMax: 100 },
  eco2_ppm:      { label: "eCO₂",        unit: "ppm", color: "#8b5cf6",
    thresholds: [{ value: 1000, color: "#f59e0b" }, { value: 2000, color: "#ef4444" }] },
  aqi:           { label: "AQI",         unit: "",    color: "#10b981", yMin: 0, yMax: 5,
    thresholds: [{ value: 2.5, color: "#f59e0b" }, { value: 3.5, color: "#ef4444" }] },
  tvoc_ppb:      { label: "TVOC",        unit: "ppb", color: "#f59e0b",
    thresholds: [{ value: 220, color: "#f59e0b" }, { value: 660, color: "#ef4444" }] },
  ens_validity:  { label: "ENS validity", unit: "",   color: "#06b6d4", yMin: 0, yMax: 3 },
};

// ── App state ──────────────────────────────────────────────────────────────
const state = { readings: [], summary: null, status: null };
const chartMeta = {}; // canvasId → chart snapshot for hover
const chartCache = {}; // canvasId → ImageData (no crosshair)

const $ = (id) => document.getElementById(id);
const tooltip = $("chartTooltip");

// ── Theme ──────────────────────────────────────────────────────────────────
function applyTheme(dark) {
  document.documentElement.dataset.theme = dark ? "dark" : "light";
  localStorage.setItem("theme", dark ? "dark" : "light");
}

(function initTheme() {
  const saved = localStorage.getItem("theme");
  const prefersDark = window.matchMedia("(prefers-color-scheme: dark)").matches;
  applyTheme(saved ? saved === "dark" : prefersDark);
})();

$("themeButton").addEventListener("click", () => {
  applyTheme(document.documentElement.dataset.theme !== "dark");
  renderCharts(); // redraw with new theme colors
});

// ── Settings panel ─────────────────────────────────────────────────────────
$("settingsButton").addEventListener("click", async () => {
  const panel = $("settingsPanel");
  const open = panel.classList.toggle("open");
  if (open) {
    try {
      const cfg = await fetchJson("/api/config");
      $("espUrlInput").value = cfg.sensor_base_url || "";
    } catch (_) {}
  }
});

$("espUrlSave").addEventListener("click", async () => {
  const url = $("espUrlInput").value.trim();
  if (!url) return;
  const status = $("espUrlStatus");
  status.textContent = "Saving…";
  try {
    await fetchJson("/api/config", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sensor_base_url: url }),
    });
    status.textContent = "✓ Applied";
    setTimeout(() => (status.textContent = ""), 2500);
    await refreshAll();
  } catch (err) {
    status.textContent = "Error: " + err.message;
  }
});

// ── Formatting helpers ─────────────────────────────────────────────────────
function fmtTime(unix) {
  if (!unix) return "—";
  return new Date(unix * 1000).toLocaleString();
}

function fmtVal(v, digits = 1) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "—";
  return Number(v).toFixed(digits);
}

function fmtAxisTime(unix, spanSec) {
  const d = new Date(unix * 1000);
  if (spanSec <= 3 * 3600)
    return d.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
  if (spanSec <= 3 * 86400)
    return d.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  if (spanSec <= 14 * 86400) {
    const mo = d.toLocaleDateString([], { month: "short", day: "numeric" });
    const hr = d.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
    return mo + "\n" + hr;
  }
  return d.toLocaleDateString([], { month: "short", day: "numeric" });
}

// ── Range / query helpers ──────────────────────────────────────────────────
function selectedRange() {
  const range = $("rangeSelect").value;
  const now = Math.floor(Date.now() / 1000);
  if (range === "day")   return { start: now - 86400, end: null };
  if (range === "week")  return { start: now - 7 * 86400, end: null };
  if (range === "month") return { start: now - 30 * 86400, end: null };
  if (range === "year")  return { start: now - 365 * 86400, end: null };
  if (range === "custom") {
    const s = $("startInput").value;
    const e = $("endInput").value;
    return {
      start: s ? Math.floor(new Date(s).getTime() / 1000) : null,
      end:   e ? Math.floor(new Date(e).getTime() / 1000) : null,
    };
  }
  return { start: null, end: null };
}

function queryString(extra = {}) {
  const params = new URLSearchParams();
  const { start, end } = selectedRange();
  if (start) params.set("start", start);
  if (end)   params.set("end", end);
  for (const [k, v] of Object.entries(extra))
    if (v != null) params.set(k, v);
  return params.toString();
}

async function fetchJson(url, options) {
  const res = await fetch(url, options);
  if (!res.ok) throw new Error(`${res.status} ${await res.text()}`);
  return res.json();
}

// ── Main refresh ───────────────────────────────────────────────────────────
async function refreshAll() {
  const view = $("viewSelect").value;
  const dataUrl = view === "raw"
    ? `/api/readings?${queryString({ limit: 10000 })}`
    : `/api/rollup?${queryString({ bucket: view })}`;

  const [status, summary, data] = await Promise.all([
    fetchJson("/api/status"),
    fetchJson(`/api/summary?${queryString()}`),
    fetchJson(dataUrl),
  ]);

  state.status   = status;
  state.summary  = summary;
  state.readings = view === "raw"
    ? data.readings
    : data.points.map((r) => ({ unix_time: r.bucket_start, ...r }));

  renderStatus();
  renderSummary();
  renderCharts();
  renderTable();
  updateCsvLink();
}

// ── Status ─────────────────────────────────────────────────────────────────
function renderStatus() {
  const s = state.status;
  const ok = Boolean(s.state?.connected);
  $("connectionDot").className = "dot " + (ok ? "ok" : "bad");
  $("connectionText").textContent = ok ? "Connected" : "Disconnected";
  $("sourceLabel").textContent = s.sensor_base_url || "—";
  $("totalReadings").textContent = s.totals?.reading_count ?? "0";
  $("latestSample").textContent  = fmtTime(s.latest?.unix_time);
  $("storageSource").textContent = s.state?.device_status?.storage_source || s.latest?.source || "—";
  $("lastSync").textContent      = fmtTime(s.state?.last_success_at);
}

// ── Summary cards ──────────────────────────────────────────────────────────
const SUMMARY_ORDER = ["temperature_c", "humidity_pct", "eco2_ppm", "aqi", "tvoc_ppb", "ens_validity"];

function renderSummary() {
  const grid = $("summaryGrid");
  grid.innerHTML = "";
  for (const key of SUMMARY_ORDER) {
    const m   = METRICS[key];
    const row = state.summary.metrics[key];
    const dig = key === "ens_validity" ? 0 : key === "aqi" ? 1 : 1;
    const card = document.createElement("div");
    card.className = "summary-card";
    card.innerHTML = `
      <div class="color-bar" style="background:${m.color}"></div>
      <div class="label">${m.label}</div>
      <div class="value" style="color:${m.color}">${fmtVal(row.avg, dig)}<small style="font-size:.65em;font-weight:400;margin-left:3px">${m.unit}</small></div>
      <div class="range">↓ ${fmtVal(row.min, dig)} · ↑ ${fmtVal(row.max, dig)} ${m.unit}</div>
    `;
    grid.appendChild(card);
  }
}

// ── Charts ─────────────────────────────────────────────────────────────────
const CHART_DEFS = [
  { id: "tempChart",     key: "temperature_c" },
  { id: "humidityChart", key: "humidity_pct"  },
  { id: "eco2Chart",     key: "eco2_ppm"      },
  { id: "aqiChart",      key: "aqi"           },
  { id: "tvocChart",     key: "tvoc_ppb"      },
  { id: "validityChart", key: "ens_validity"  },
];

// ── Chart expand / collapse ────────────────────────────────────────────────
const expandState = JSON.parse(localStorage.getItem("chartExpand") || "{}");

function saveExpandState() {
  localStorage.setItem("chartExpand", JSON.stringify(expandState));
}

function applyExpandState() {
  document.querySelectorAll(".charts article[data-chart-id]").forEach((article) => {
    const id = article.dataset.chartId;
    const expanded = Boolean(expandState[id]);
    article.dataset.expanded = expanded ? "true" : "false";
    const btn = article.querySelector(".expand-btn");
    if (btn) btn.textContent = expanded ? "⤡" : "⤢";
  });
}

document.querySelectorAll(".expand-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    const article = btn.closest("article[data-chart-id]");
    const id = article.dataset.chartId;
    const nowExpanded = article.dataset.expanded !== "true";
    expandState[id] = nowExpanded;
    saveExpandState();
    applyExpandState();
    // Redraw after layout settles
    requestAnimationFrame(() => {
      const def = CHART_DEFS.find((d) => d.id === id);
      if (def) drawChart(def.id, def.key);
    });
  });
});

applyExpandState();

function renderCharts() {
  for (const def of CHART_DEFS) drawChart(def.id, def.key);
}

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function drawChart(canvasId, metricKey) {
  const canvas = $(canvasId);
  const metric = METRICS[metricKey];
  const points = state.readings;

  // Size canvas to CSS pixels × DPR for crispness
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const W = rect.width * dpr;
  const H = rect.height * dpr;
  if (canvas.width !== W || canvas.height !== H) {
    canvas.width  = W;
    canvas.height = H;
  }
  const w = rect.width;
  const h = rect.height;

  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const bgColor   = cssVar("--chart-bg");
  const gridColor = cssVar("--chart-grid");
  const grid2     = cssVar("--chart-grid-2");
  const textColor = cssVar("--chart-text");

  ctx.fillStyle = bgColor;
  ctx.fillRect(0, 0, w, h);

  const compactChart = w < 520;
  const axisFontSize = compactChart ? 12 : 11;
  const pad = {
    left: compactChart ? 62 : 56,
    right: compactChart ? 12 : 18,
    top: 14,
    bottom: compactChart ? 42 : 46,
  };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

  if (!points.length) {
    ctx.fillStyle = textColor;
    ctx.font = `13px system-ui`;
    ctx.fillText("No data in selected range", pad.left + 12, h / 2);
    return;
  }

  const vals = points.map((p) => p[metricKey]).filter((v) => v != null).map(Number);
  if (!vals.length) return;

  const xs   = points.map((p) => p.unix_time);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const spanX = Math.max(1, maxX - minX);

  let dataMin = Math.min(...vals);
  let dataMax = Math.max(...vals);
  if (dataMin === dataMax) { dataMin -= 1; dataMax += 1; }

  // Expand to defined min/max if provided
  if (metric.yMin != null) dataMin = Math.min(dataMin, metric.yMin);
  if (metric.yMax != null) dataMax = Math.max(dataMax, metric.yMax);

  // Nice Y axis ticks. Keep labels readable on narrow portrait layouts.
  const yTickTarget = Math.max(3, Math.min(6, Math.floor(plotH / (axisFontSize * 3.2))));
  const { ticks: yTicks, min: yMin, max: yMax } = niceRange(dataMin, dataMax, yTickTarget);

  const xScale = (x) => pad.left + ((x - minX) / spanX) * plotW;
  const yScale = (y) => pad.top + (1 - (y - yMin) / Math.max(1e-9, yMax - yMin)) * plotH;

  // ── Threshold bands ────────────────────────────────────────────────────
  if (metric.thresholds?.length) {
    const sorted = [...metric.thresholds].sort((a, b) => a.value - b.value);
    for (let i = 0; i < sorted.length; i++) {
      const yTop = yScale(Math.min(sorted[i + 1]?.value ?? yMax, yMax));
      const yBot = yScale(sorted[i].value);
      if (yBot > yTop) {
        ctx.fillStyle = hexAlpha(sorted[i].color, 0.06);
        ctx.fillRect(pad.left, yTop, plotW, yBot - yTop);
      }
    }
  }

  // ── Y grid lines ───────────────────────────────────────────────────────
  ctx.font = `${axisFontSize}px system-ui`;
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (const tick of yTicks) {
    const y = yScale(tick);
    if (y < pad.top - 1 || y > pad.top + plotH + 1) continue;
    ctx.strokeStyle = tick === 0 ? grid2 : gridColor;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(pad.left + plotW, y);
    ctx.stroke();
    ctx.fillStyle = textColor;
    ctx.fillText(fmtAxisVal(tick), pad.left - 6, y);
  }

  // ── X axis ticks ───────────────────────────────────────────────────────
  const xTicks = niceTimeTicks(minX, maxX, Math.max(3, Math.min(8, Math.floor(plotW / (compactChart ? 96 : 80)))));
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (const tick of xTicks) {
    const x = xScale(tick);
    if (x < pad.left || x > pad.left + plotW) continue;
    ctx.strokeStyle = grid2;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, pad.top);
    ctx.lineTo(x, pad.top + plotH);
    ctx.stroke();
    ctx.fillStyle = textColor;
    const label = fmtAxisTime(tick, spanX);
    const lines = label.split("\n");
    lines.forEach((line, i) => ctx.fillText(line, x, pad.top + plotH + 6 + i * 14));
  }

  // ── Area fill ──────────────────────────────────────────────────────────
  ctx.save();
  ctx.beginPath();
  let started = false;
  for (const pt of points) {
    const v = pt[metricKey];
    if (v == null) { started = false; continue; }
    const x = xScale(pt.unix_time);
    const y = yScale(Number(v));
    if (!started) { ctx.moveTo(x, y); started = true; }
    else          ctx.lineTo(x, y);
  }
  if (started) {
    const lastPt = points.findLast((p) => p[metricKey] != null);
    const firstPt = points.find((p) => p[metricKey] != null);
    ctx.lineTo(xScale(lastPt.unix_time), pad.top + plotH);
    ctx.lineTo(xScale(firstPt.unix_time), pad.top + plotH);
    ctx.closePath();
  }
  const grad = ctx.createLinearGradient(0, pad.top, 0, pad.top + plotH);
  grad.addColorStop(0,   hexAlpha(metric.color, 0.22));
  grad.addColorStop(0.6, hexAlpha(metric.color, 0.06));
  grad.addColorStop(1,   hexAlpha(metric.color, 0.0));
  ctx.fillStyle = grad;
  ctx.fill();
  ctx.restore();

  // ── Line ───────────────────────────────────────────────────────────────
  ctx.save();
  ctx.strokeStyle = metric.color;
  ctx.lineWidth = 1.8;
  ctx.lineJoin = "round";
  ctx.beginPath();
  started = false;
  for (const pt of points) {
    const v = pt[metricKey];
    if (v == null) { started = false; continue; }
    const x = xScale(pt.unix_time);
    const y = yScale(Number(v));
    if (!started) { ctx.moveTo(x, y); started = true; }
    else          ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.restore();

  // ── Threshold lines ────────────────────────────────────────────────────
  if (metric.thresholds) {
    ctx.save();
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 1;
    for (const th of metric.thresholds) {
      const y = yScale(th.value);
      if (y < pad.top || y > pad.top + plotH) continue;
      ctx.strokeStyle = hexAlpha(th.color, 0.65);
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(pad.left + plotW, y);
      ctx.stroke();
    }
    ctx.restore();
  }

  // ── Store metadata for hover ───────────────────────────────────────────
  chartMeta[canvasId] = { metricKey, metric, points, xScale, yScale, pad, w, h, spanX };
  chartCache[canvasId] = ctx.getImageData(0, 0, canvas.width, canvas.height);

  if (!canvas._hoverReady) {
    canvas._hoverReady = true;
    canvas.addEventListener("mousemove", onChartMouseMove);
    canvas.addEventListener("mouseleave", onChartMouseLeave);
  }
}

// ── Hover / crosshair ──────────────────────────────────────────────────────
function onChartMouseMove(e) {
  const canvas = e.currentTarget;
  const id = canvas.id;
  const meta = chartMeta[id];
  if (!meta) return;

  const rect = canvas.getBoundingClientRect();
  const dpr = canvas.width / rect.width;
  const cssX = e.clientX - rect.left;
  const canvasX = cssX * dpr;
  const { points, xScale, pad, w, h, metricKey, metric } = meta;

  if (cssX < pad.left || cssX > w - pad.right) {
    tooltip.style.display = "none";
    if (chartCache[id]) canvas.getContext("2d").putImageData(chartCache[id], 0, 0);
    return;
  }

  // Find nearest point
  let nearestIdx = 0, nearestDist = Infinity;
  for (let i = 0; i < points.length; i++) {
    const d = Math.abs(xScale(points[i].unix_time) * dpr - canvasX);
    if (d < nearestDist) { nearestDist = d; nearestIdx = i; }
  }
  const pt = points[nearestIdx];
  const cx = xScale(pt.unix_time); // CSS pixels

  // Restore cached image, draw crosshair
  const ctx = canvas.getContext("2d");
  if (chartCache[id]) ctx.putImageData(chartCache[id], 0, 0);

  ctx.save();
  ctx.strokeStyle = cssVar("--muted");
  ctx.globalAlpha = 0.4;
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.moveTo(cx, pad.top);
  ctx.lineTo(cx, pad.top + (h - pad.top - pad.bottom));
  ctx.stroke();
  // Dot on the line
  const v = pt[metricKey];
  if (v != null) {
    const cy = meta.yScale(Number(v));
    ctx.beginPath();
    ctx.globalAlpha = 1;
    ctx.setLineDash([]);
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fillStyle = metric.color;
    ctx.fill();
    ctx.strokeStyle = cssVar("--chart-bg");
    ctx.lineWidth = 2;
    ctx.stroke();
  }
  ctx.restore();

  // Populate tooltip
  $("ttTime").textContent = fmtTime(pt.unix_time);
  $("ttRows").innerHTML = "";
  const row = document.createElement("div");
  row.className = "tt-row";
  row.innerHTML = `<span class="tt-label">${metric.label}</span><span class="tt-val" style="color:${metric.color}">${fmtVal(v, metricKey === "ens_validity" ? 0 : 1)} ${metric.unit}</span>`;
  $("ttRows").appendChild(row);

  // Position tooltip
  const tx = e.clientX + 14;
  const ty = e.clientY - 20;
  tooltip.style.left  = Math.min(tx, window.innerWidth - 220) + "px";
  tooltip.style.top   = Math.max(4, ty) + "px";
  tooltip.style.display = "block";
}

function onChartMouseLeave(e) {
  const id = e.currentTarget.id;
  tooltip.style.display = "none";
  if (chartCache[id]) {
    e.currentTarget.getContext("2d").putImageData(chartCache[id], 0, 0);
  }
}

// ── Table ──────────────────────────────────────────────────────────────────
function renderTable() {
  const rows = state.readings.slice(-80).reverse();
  $("tableNote").textContent = `${rows.length} newest rows`;
  $("rowsBody").innerHTML = rows.map((r) => `
    <tr>
      <td>${fmtTime(r.unix_time)}</td>
      <td>${fmtVal(r.temperature_c)}</td>
      <td>${fmtVal(r.humidity_pct)}</td>
      <td class="${qClass("aqi",  r.aqi)}">${fmtVal(r.aqi, 0)}</td>
      <td class="${qClass("tvoc", r.tvoc_ppb)}">${fmtVal(r.tvoc_ppb, 0)}</td>
      <td class="${qClass("eco2", r.eco2_ppm)}">${fmtVal(r.eco2_ppm, 0)}</td>
      <td>${fmtVal(r.ens_validity, 0)}</td>
    </tr>`).join("");
}

function qClass(m, v) {
  if (v == null) return "";
  if (m === "aqi")  return v <= 2 ? "quality-ok" : v === 3 ? "quality-warn" : "quality-bad";
  if (m === "tvoc") return v <= 220 ? "quality-ok" : v <= 660 ? "quality-warn" : "quality-bad";
  if (m === "eco2") return v <= 1000 ? "quality-ok" : v <= 2000 ? "quality-warn" : "quality-bad";
  return "";
}

function updateCsvLink() {
  $("csvLink").href = `/api/export.csv?${queryString()}`;
}

// ── Sync buttons ───────────────────────────────────────────────────────────
async function runSync(full) {
  $("syncButton").disabled = true;
  $("fullSyncButton").disabled = true;
  try {
    await fetchJson(`/api/sync?full=${full}`, { method: "POST" });
    await refreshAll();
  } finally {
    $("syncButton").disabled = false;
    $("fullSyncButton").disabled = false;
  }
}

$("syncButton").addEventListener("click", () => runSync(false).catch(alert));
$("fullSyncButton").addEventListener("click", () => runSync(true).catch(alert));
for (const id of ["rangeSelect", "startInput", "endInput", "viewSelect"])
  $(id).addEventListener("change", () => refreshAll().catch(console.error));

// Redraw charts on window resize (canvas CSS size changed)
let resizeTimer;
window.addEventListener("resize", () => {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(renderCharts, 120);
});

// ── Axis math helpers ──────────────────────────────────────────────────────
function niceRange(minV, maxV, targetTicks = 5) {
  const range = maxV - minV || 1;
  const raw   = range / targetTicks;
  const mag   = Math.pow(10, Math.floor(Math.log10(raw)));
  const steps = [1, 2, 2.5, 5, 10];
  const step  = mag * (steps.find((s) => s * mag >= raw) ?? 10);
  const nMin  = Math.floor(minV / step) * step;
  const nMax  = Math.ceil(maxV / step) * step;
  const ticks = [];
  for (let v = nMin; v <= nMax + step * 0.001; v = Math.round((v + step) * 1e9) / 1e9)
    ticks.push(v);
  return { min: nMin, max: nMax, ticks };
}

function niceTimeTicks(minX, maxX, count) {
  const span = maxX - minX || 1;
  const raw  = span / count;
  const nice = [60, 300, 600, 900, 1800, 3600, 2 * 3600, 3 * 3600, 6 * 3600,
                12 * 3600, 86400, 2 * 86400, 7 * 86400, 14 * 86400, 30 * 86400];
  const interval = nice.find((i) => i >= raw) ?? nice[nice.length - 1];
  const start = Math.ceil(minX / interval) * interval;
  const ticks = [];
  for (let t = start; t <= maxX; t += interval) ticks.push(t);
  return ticks;
}

function fmtAxisVal(v) {
  if (Math.abs(v) >= 10000) return (v / 1000).toFixed(1) + "k";
  if (Number.isInteger(v) || Math.abs(v) >= 100) return String(Math.round(v));
  return v.toFixed(1);
}

function hexAlpha(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r},${g},${b},${alpha})`;
}

// ── Boot ───────────────────────────────────────────────────────────────────
refreshAll().catch((err) => {
  console.error(err);
  $("connectionText").textContent = "Error";
});
setInterval(() => refreshAll().catch(console.error), 30000);
