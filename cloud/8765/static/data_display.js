const DATA_REFRESH_MS = 2000;
const FRESH_WINDOW_MS = 30000;

function $(id) {
  return document.getElementById(id);
}

function setText(id, text) {
  const element = $(id);
  if (element) element.textContent = text;
}

function ageMs(iso) {
  if (!iso) return Number.POSITIVE_INFINITY;
  const value = Date.now() - new Date(iso).getTime();
  return Number.isFinite(value) ? value : Number.POSITIVE_INFINITY;
}

function fmtAge(iso) {
  if (!iso) return "未上报";
  const diff = ageMs(iso);
  if (!Number.isFinite(diff)) return "未上报";
  if (diff < 1000) return "刚刚";
  if (diff < 60_000) return `${Math.floor(diff / 1000)} 秒前`;
  if (diff < 3_600_000) return `${Math.floor(diff / 60_000)} 分钟前`;
  return `${Math.floor(diff / 3_600_000)} 小时前`;
}

function finiteNumber(value) {
  const num = Number(value);
  return Number.isFinite(num) ? num : null;
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function metricValue(metric) {
  if (!metric || typeof metric !== "object") return null;
  return finiteNumber(metric.value);
}

function metricFromAliases(metric, valueKeys = []) {
  if (!metric || typeof metric !== "object") return {};
  const normalized = { ...metric };
  if (finiteNumber(normalized.value) === null) {
    for (const key of valueKeys) {
      const candidate = finiteNumber(metric[key]);
      if (candidate !== null) {
        normalized.value = candidate;
        break;
      }
    }
  }
  return normalized;
}

function pickMetric(candidates, valueKeys = []) {
  for (const candidate of candidates) {
    const metric = metricFromAliases(candidate, valueKeys);
    if (metricValue(metric) !== null) return metric;
  }
  return metricFromAliases(candidates.find((candidate) => candidate && typeof candidate === "object") || {}, valueKeys);
}

function fmtMetric(metric, fallbackUnit = "", digits = 1) {
  const value = metricValue(metric);
  const unit = metric?.unit || fallbackUnit || "";
  if (value === null) return `--${unit ? ` ${unit}` : ""}`;
  return `${value.toFixed(digits)}${unit ? ` ${unit}` : ""}`;
}

function normalizeStatus(status, hasValue = false) {
  const raw = String(status || "").trim().toLowerCase();
  if (["ok", "normal", "ready", "running", "on", "正常", "在线"].includes(raw)) {
    return { tone: "ok", label: "正常" };
  }
  if (["warn", "warning", "注意", "预警"].includes(raw)) {
    return { tone: "warn", label: "预警" };
  }
  if (["bad", "error", "alarm", "alert", "danger", "fault", "告警", "报警", "故障", "异常"].includes(raw)) {
    return { tone: "bad", label: "告警" };
  }
  if (hasValue) return { tone: "ok", label: "已上报" };
  return { tone: "unknown", label: "未上报" };
}

function applyTone(element, tone) {
  if (!element) return;
  element.classList.remove("tone-ok", "tone-warn", "tone-bad", "tone-unknown");
  if (tone) element.classList.add(`tone-${tone}`);
}

function setMeter(prefix, metric, maxDefault, digits) {
  const value = metricValue(metric);
  const min = finiteNumber(metric?.min) ?? 0;
  const max = finiteNumber(metric?.max) ?? maxDefault;
  const pct = value === null || max <= min ? 0 : clamp((value - min) / (max - min), 0, 1);
  const angle = -70 + pct * 140;
  const needle = $(`${prefix}-needle`);
  if (needle) needle.style.setProperty("--needle-angle", `${angle}deg`);
  setText(`${prefix}-value`, fmtMetric(metric, prefix === "voltage" ? "V" : "A", digits));
  const status = normalizeStatus(metric?.status, value !== null);
  setText(`${prefix}-state`, status.label);
  applyTone(document.querySelector(`.instrument-card[data-kind="${prefix}"]`), status.tone);
}

function truthyState(value) {
  if (value === true) return true;
  const raw = String(value || "").trim().toLowerCase();
  return ["1", "true", "on", "yes", "active", "亮", "启动", "按下", "触发"].includes(raw);
}

function setLamp(cardId, textId, active, activeText = "已触发", inactiveText = "未触发") {
  const card = $(cardId);
  if (card) card.classList.toggle("is-active", Boolean(active));
  setText(textId, active ? activeText : inactiveText);
}

function normalizeMode(mode) {
  const raw = String(mode || "关闭").trim();
  const lower = raw.toLowerCase();
  if (["manual", "手动"].includes(lower)) return "手动";
  if (["off", "close", "closed", "stop", "stopped", "关闭", "停止"].includes(lower)) return "关闭";
  return "关闭";
}

function setSelector(containerId, mode) {
  const normalized = normalizeMode(mode);
  const container = $(containerId);
  if (!container) return;
  container.querySelectorAll("span").forEach((item) => {
    item.classList.toggle("is-active", item.dataset.mode === normalized);
  });
}

function setSensor(prefix, metric, fallbackUnit = "ppm", digits = 1) {
  const value = metricValue(metric);
  const status = normalizeStatus(metric?.status, value !== null);
  setText(`${prefix}-value`, fmtMetric(metric, fallbackUnit, digits));
  setText(`${prefix}-status`, status.label);
  applyTone($(`${prefix}-card`), status.tone);
}

function highVoltageState(value) {
  if (value === true) return { tone: "warn", label: "有电" };
  if (value === false) return { tone: "ok", label: "无异常" };
  const raw = String(value || "").trim().toLowerCase();
  if (["alarm", "alert", "danger", "on", "有电", "告警", "报警"].includes(raw)) {
    return { tone: "warn", label: "有电警示" };
  }
  if (["normal", "ok", "off", "safe", "正常", "安全"].includes(raw)) {
    return { tone: "ok", label: "正常" };
  }
  return { tone: "unknown", label: "未上报" };
}

function doorState(value) {
  const raw = String(value || "").trim().toLowerCase();
  if (["open", "opened", "打开", "开启"].includes(raw)) return { tone: "warn", label: "打开" };
  if (["closed", "close", "关闭", "关"].includes(raw)) return { tone: "ok", label: "关闭" };
  return { tone: "unknown", label: "未上报" };
}

function renderCabinet(payload) {
  const data = payload?.data || {};
  const lastSeen = payload?.last_seen_at || "";
  const updatedAt = data.updated_at || lastSeen || payload?.server_time || "";
  const boardOnline = ageMs(lastSeen) <= FRESH_WINDOW_MS;

  setText("cabinet-online", boardOnline ? "在线" : "离线");
  setText("cabinet-last-seen", lastSeen ? `最近上报 ${fmtAge(lastSeen)}` : "等待板端上报");
  setText("cabinet-updated-at", updatedAt ? `更新 ${fmtAge(updatedAt)}` : "等待上报");
  setText("data-page-meta", payload?.board_id ? `当前设备：${payload.board_label || payload.board_id}` : "查看配电柜面板状态和柜内环境数据。");

  setMeter("voltage", data.voltage || {}, 450, 1);
  setMeter("current", data.current || {}, 1, 2);
  setText("voltage-status-value", fmtMetric(data.voltage || {}, "V", 1));
  setText("current-status-value", fmtMetric(data.current || {}, "A", 2));
  setText("voltage-status-text", normalizeStatus(data.voltage?.status, metricValue(data.voltage) !== null).label);
  setText("current-status-text", normalizeStatus(data.current?.status, metricValue(data.current) !== null).label);

  const tempCtrl = data.temperature_controller || {};
  const pv = finiteNumber(tempCtrl.pv);
  const sv = finiteNumber(tempCtrl.sv);
  const tempUnit = tempCtrl.unit || "℃";
  setText("temp-pv", pv === null ? `-- ${tempUnit}` : `${pv.toFixed(1)} ${tempUnit}`);
  setText("temp-sv", sv === null ? `-- ${tempUnit}` : `${sv.toFixed(1)} ${tempUnit}`);
  setText("temp-controller-state", normalizeStatus(tempCtrl.status, pv !== null || sv !== null).label);
  applyTone(document.querySelector(".temperature-controller"), normalizeStatus(tempCtrl.status, pv !== null || sv !== null).tone);

  const motor1 = data.motor_1 || {};
  const motor2 = data.motor_2 || {};
  setLamp("motor1-start-card", "motor1-start-text", truthyState(motor1.start), "绿灯亮起", "绿灯熄灭");
  setLamp("motor1-stop-card", "motor1-stop-text", truthyState(motor1.stop), "红灯亮起", "红灯熄灭");
  setLamp("motor2-start-card", "motor2-start-text", truthyState(motor2.start), "绿灯亮起", "绿灯熄灭");
  setLamp("motor2-stop-card", "motor2-stop-text", truthyState(motor2.stop), "红灯亮起", "红灯熄灭");
  setSelector("motor1-mode", motor1.mode);
  setSelector("motor2-mode", motor2.mode);

  const warning = highVoltageState(data.warning?.high_voltage);
  setText("high-voltage-text", warning.label);
  applyTone($("high-voltage-card"), warning.tone);

  const door = doorState(data.door?.state);
  setText("door-state", door.label);
  applyTone($("door-card"), door.tone);

  const env = data.environment || {};
  setSensor("smoke", env.smoke || {}, "ppm", 1);
  setSensor("hydrogen", env.hydrogen || {}, "ppm", 1);
  setSensor("carbon-monoxide", env.carbon_monoxide || {}, "ppm", 1);

  const envTemp = env.temperature || {};
  const humidity = env.humidity || {};
  const envTempText = fmtMetric(envTemp, "℃", 1);
  const humidityText = fmtMetric(humidity, "%RH", 1);
  const envTempStatus = normalizeStatus(envTemp.status, metricValue(envTemp) !== null);
  const humidityStatus = normalizeStatus(humidity.status, metricValue(humidity) !== null);
  const tones = [envTempStatus.tone, humidityStatus.tone];
  const combinedTone = tones.includes("bad")
    ? "bad"
    : (tones.includes("warn") ? "warn" : (metricValue(envTemp) !== null || metricValue(humidity) !== null ? "ok" : "unknown"));

  setText("temperature-humidity-value", `${envTempText} / ${humidityText}`);
  setText("temperature-humidity-status", combinedTone === "unknown" ? "未上报" : "已上报");
  applyTone($("temperature-humidity-card"), combinedTone);

  const infraredTemperature = pickMetric(
    [
      env.infrared_temperature,
      env.infrared,
      env.thermal,
      data.infrared_temperature,
      data.infrared,
    ],
    ["temperature", "target_temperature", "object_temperature", "surface_temperature"]
  );
  const soundLevel = pickMetric(
    [
      env.sound_level,
      env.sound,
      env.noise,
      env.microphone,
      data.sound_level,
      data.sound,
    ],
    ["db", "level", "relative_db", "dbfs", "rms_dbfs"]
  );
  setSensor("infrared-temperature", infraredTemperature, "℃", 1);
  setSensor("sound-level", soundLevel, "dB", 1);

  const envSummary = metricValue(env.smoke) !== null
    || metricValue(env.hydrogen) !== null
    || metricValue(env.carbon_monoxide) !== null
    || metricValue(envTemp) !== null
    || metricValue(humidity) !== null
    || metricValue(infraredTemperature) !== null
    || metricValue(soundLevel) !== null;
  setText("env-status-value", envSummary ? "已上报" : "--");
  setText("env-status-text", envSummary ? "柜内环境、红外与声音数据正常刷新" : "等待烟雾 / 氢气 / 一氧化碳 / 温湿度 / 红外测温 / 声音数据");
}

async function fetchCabinetData() {
  const response = await fetch("/api/cloud/cabinet-data", { cache: "no-store" });
  const payload = await response.json();
  if (!response.ok || !payload.ok) throw new Error(payload.message || `HTTP ${response.status}`);
  renderCabinet(payload);
}

function start() {
  const refreshButton = $("btn-refresh-data");
  if (refreshButton) refreshButton.addEventListener("click", () => fetchCabinetData().catch(console.error));
  fetchCabinetData().catch((error) => {
    setText("data-page-meta", `数据获取失败：${error}`);
  });
  window.setInterval(() => fetchCabinetData().catch(() => {}), DATA_REFRESH_MS);
}

window.addEventListener("load", start);
