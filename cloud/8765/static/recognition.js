const state = {
  cloud: null,
  recognition: null,
  commandBusy: false,
};

const commandToken = document.body.dataset.commandToken || "";
const LIVE_BOARD_MS = 15000;

function $(id) {
  return document.getElementById(id);
}

function ageMs(iso) {
  if (!iso) return Number.POSITIVE_INFINITY;
  const value = Date.now() - new Date(iso).getTime();
  return Number.isFinite(value) ? value : Number.POSITIVE_INFINITY;
}

function fmtAge(iso) {
  if (!iso) return "-";
  const diff = ageMs(iso);
  if (!Number.isFinite(diff)) return "-";
  if (diff < 1000) return "刚刚";
  if (diff < 60000) return `${Math.floor(diff / 1000)} 秒前`;
  if (diff < 3600000) return `${Math.floor(diff / 60000)} 分钟前`;
  return `${Math.floor(diff / 3600000)} 小时前`;
}

function fmtNumber(value, digits = 1) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "-";
}

function actionLabel(action) {
  const labels = {
    capture_recognition: "拍照识别",
    start_mapping: "开始建图",
    stop_mapping: "停止建图",
    save_map: "保存地图",
    start_navigation: "开始导航",
    stop_navigation: "停止导航",
  };
  return labels[action] || action || "-";
}

function statusLabel(status) {
  const labels = {
    pending: "等待中",
    claimed: "已接收",
    running: "执行中",
    completed: "完成",
    failed: "失败",
    rejected: "拒绝",
  };
  return labels[status] || status || "-";
}

function latestRecognition() {
  const payload = state.recognition || {};
  return payload.latest && typeof payload.latest === "object" ? payload.latest : {};
}

function recognitionHistory() {
  const payload = state.recognition || {};
  return Array.isArray(payload.history) ? payload.history : [];
}

function detectionSummary(item) {
  const meterReadings = Array.isArray(item.meter_readings) ? item.meter_readings : [];
  const meterParts = meterReadings
    .filter((reading) => reading && reading.value !== null && reading.value !== undefined)
    .map((reading) => {
      const label = reading.meter_type === "voltage" ? "电压表" : reading.meter_type === "current" ? "电流表" : "仪表";
      const digits = reading.meter_type === "current" ? 2 : 1;
      const value = fmtNumber(reading.value, digits);
      return `${label} ${value}${reading.unit || ""}`;
    });
  const digitReadings = item.digit_readings && typeof item.digit_readings === "object" ? item.digit_readings : {};
  if (digitReadings.global_text) {
    meterParts.push(`数字 ${digitReadings.global_text}`);
  }
  const detections = Array.isArray(item.detections) ? item.detections : [];
  const targetParts = detections
    .slice(0, 6)
    .map((det) => {
      const name = det.class_name || `类别 ${det.class_id}`;
      const confidence = Number(det.confidence);
      return Number.isFinite(confidence) ? `${name} ${(confidence * 100).toFixed(1)}%` : name;
    });
  const parts = [...meterParts, ...targetParts];
  if (!parts.length) return "未识别到目标";
  return parts.join("，");
}

function renderStatus() {
  const cloud = state.cloud || {};
  const board = cloud.board || {};
  const latest = latestRecognition();
  const history = recognitionHistory();
  const online = ageMs(board.last_seen_at) <= LIVE_BOARD_MS;

  $("board-online").textContent = online ? "在线" : "离线";
  $("last-seen").textContent = board.last_seen_at ? `最近上传 ${fmtAge(board.last_seen_at)}` : "-";
  $("recognition-count").textContent = `${Number(latest.detection_count || 0)} 个目标`;
  $("recognition-time").textContent = latest.uploaded_at ? fmtAge(latest.uploaded_at) : "-";
  $("recognition-infer").textContent = latest.inference_ms !== undefined && latest.inference_ms !== ""
    ? `${fmtNumber(latest.inference_ms, 1)} ms`
    : "-";
  $("recognition-backend").textContent = latest.backend || "-";
  $("history-count").textContent = `${history.length} 张`;
}

function renderLatest() {
  const latest = latestRecognition();
  const image = $("recognition-image");
  const placeholder = $("recognition-placeholder");
  const summary = $("recognition-summary");
  const pill = $("recognition-pill");
  const annotated = latest.annotated_image || {};

  if (!annotated.url) {
    image.removeAttribute("src");
    image.style.display = "none";
    placeholder.style.display = "grid";
    summary.textContent = "等待识别结果";
    pill.textContent = "等待照片";
    return;
  }

  image.src = `${annotated.url}?ts=${encodeURIComponent(latest.uploaded_at || "")}`;
  image.style.display = "block";
  placeholder.style.display = "none";
  summary.textContent = detectionSummary(latest);
  pill.textContent = latest.detection_count > 0 ? `识别到 ${latest.detection_count} 个目标` : "未识别到目标";
}

function renderHistory() {
  const target = $("recognition-history");
  const history = recognitionHistory();
  target.innerHTML = "";
  if (!history.length) {
    const empty = document.createElement("div");
    empty.className = "placeholder-inline";
    empty.textContent = "暂无历史照片";
    target.appendChild(empty);
    return;
  }

  for (const item of history) {
    const annotated = item.annotated_image || {};
    const card = document.createElement("article");
    card.className = "recognition-history-card";
    const link = document.createElement("a");
    link.href = annotated.url || "#";
    link.target = "_blank";
    link.rel = "noreferrer";
    const img = document.createElement("img");
    if (annotated.url) {
      img.src = `${annotated.url}?ts=${encodeURIComponent(item.uploaded_at || "")}`;
    }
    img.alt = "识别历史照片";
    link.appendChild(img);
    const title = document.createElement("strong");
    title.textContent = `${Number(item.detection_count || 0)} 个目标`;
    const meta = document.createElement("small");
    meta.textContent = `${fmtAge(item.uploaded_at)} · ${item.backend || "-"} · ${fmtNumber(item.inference_ms, 1)} ms`;
    const detail = document.createElement("p");
    detail.textContent = detectionSummary(item);
    card.append(link, title, meta, detail);
    target.appendChild(card);
  }
}

function renderCommands() {
  const target = $("command-history");
  const history = state.cloud?.commands?.history || [];
  const items = Array.isArray(history)
    ? history.filter((item) => item && item.action === "capture_recognition").slice(0, 8)
    : [];
  target.innerHTML = "";
  if (!items.length) {
    target.textContent = "还没有图像识别指令记录";
    return;
  }
  for (const item of items) {
    const node = document.createElement("div");
    node.className = "command-item";
    const title = document.createElement("strong");
    title.textContent = `${actionLabel(item.action)} · ${statusLabel(item.status)}`;
    const time = document.createElement("span");
    time.textContent = item.updated_at ? fmtAge(item.updated_at) : "-";
    const result = document.createElement("small");
    result.textContent = item.result?.message || item.id || "";
    node.append(title, time, result);
    target.appendChild(node);
  }
}

function render() {
  renderStatus();
  renderLatest();
  renderHistory();
  renderCommands();
}

async function fetchJson(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}

async function fetchAll() {
  const [cloud, recognition] = await Promise.all([
    fetchJson("/api/cloud/state"),
    fetchJson("/api/cloud/recognition/history?limit=80"),
  ]);
  state.cloud = cloud;
  state.recognition = recognition;
  render();
}

async function sendRecognitionCommand() {
  if (state.commandBusy) return;
  state.commandBusy = true;
  const button = $("cmd-capture-recognition");
  const status = $("recognition-command-status");
  const conf = Number($("recognition-conf")?.value || 0.25);
  button.disabled = true;
  status.textContent = "已下发拍照识别指令，等待板端执行。";
  try {
    const response = await fetch("/api/cloud/commands", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Command-Token": commandToken,
      },
      body: JSON.stringify({
        action: "capture_recognition",
        params: {
          conf: Number.isFinite(conf) ? Math.min(Math.max(conf, 0.01), 0.99) : 0.25,
        },
      }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    status.textContent = "指令已进入队列，板端 uploader 会自动执行。";
    await fetchAll();
  } catch (error) {
    status.textContent = `下发失败：${error}`;
  } finally {
    state.commandBusy = false;
    button.disabled = false;
  }
}

window.addEventListener("load", () => {
  $("btn-refresh")?.addEventListener("click", () => fetchAll().catch(() => {}));
  $("cmd-capture-recognition")?.addEventListener("click", () => sendRecognitionCommand());
  fetchAll().catch((error) => {
    $("board-meta").textContent = `状态获取失败：${error}`;
  });
  window.setInterval(() => {
    fetchAll().catch(() => {});
  }, 1500);
});
