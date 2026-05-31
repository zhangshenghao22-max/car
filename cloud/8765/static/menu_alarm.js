const ALARM_REFRESH_MS = 2000;
const ARM_STORAGE_KEY = "carCloudAlarmArmOpen";

const T = {
  none: "无警告",
  normal: "正常",
  level3: "三级警告",
  level3Short: "三级",
  level2: "二级警告",
  level2Short: "二级",
  level1: "一级警告",
  level1Short: "一级",
  missing: "未上报",
  justNow: "刚刚",
  secondsAgo: "秒前",
  minutesAgo: "分钟前",
  hoursAgo: "小时前",
  triggerThreshold: "触发阈值",
  level3Threshold: "三级阈值",
  noThreshold: "未触发阈值",
  tempCtrl: "温控表",
  cabinetTemp: "柜内温度",
  infraredTemp: "红外测温模块",
  smoke: "柜内烟雾浓度",
  hydrogen: "氢气浓度",
  carbonMonoxide: "一氧化碳浓度",
  allNormalRecent: "所有监测模块处于阈值范围内，最近更新",
  allNormal: "所有监测模块处于阈值范围内。",
  alarmCountPrefix: "个模块超过阈值，最高为",
  noAlarmSummary: "温度与气体浓度未超过报警阈值。",
  fetchFailed: "报警数据获取失败：",
  cannotRead: "无法读取 /api/cloud/cabinet-data，请检查云平台服务或网络状态。",
  collapseDetails: "收起异常数据",
  showDetails: "查看异常数据",
  armOpen: "已打开（未下发）",
  armClosed: "已关闭（未下发）",
};

const LEVEL_PRIORITY = { 0: 0, 3: 1, 2: 2, 1: 3 };
const LEVEL_META = {
  0: { title: T.none, badge: T.normal, className: "alarm-state-none" },
  3: { title: T.level3, badge: T.level3Short, className: "alarm-state-3" },
  2: { title: T.level2, badge: T.level2Short, className: "alarm-state-2" },
  1: { title: T.level1, badge: T.level1Short, className: "alarm-state-1" },
};
const TEMP_THRESHOLDS = [
  { level: 1, min: 60 },
  { level: 2, min: 45 },
  { level: 3, min: 35 },
];
const GAS_THRESHOLDS = [
  { level: 1, min: 200 },
  { level: 2, min: 100 },
  { level: 3, min: 50 },
];

let alarmExpanded = false;
let armOpen = false;
let latestItems = [];

function $(id) {
  return document.getElementById(id);
}

function setText(id, text) {
  const element = $(id);
  if (element) element.textContent = text;
}

function finiteNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function fmtValue(item) {
  if (item.value === null) {
    return item.rawValue === null || item.rawValue === undefined || item.rawValue === "" ? T.missing : String(item.rawValue);
  }
  const digits = Number.isInteger(item.digits) ? item.digits : 1;
  return `${item.value.toFixed(digits)} ${item.unit || ""}`.trim();
}

function fmtAge(iso) {
  if (!iso) return T.missing;
  const diffMs = Date.now() - new Date(iso).getTime();
  if (!Number.isFinite(diffMs)) return T.missing;
  if (diffMs < 1000) return T.justNow;
  if (diffMs < 60_000) return `${Math.floor(diffMs / 1000)} ${T.secondsAgo}`;
  if (diffMs < 3_600_000) return `${Math.floor(diffMs / 60_000)} ${T.minutesAgo}`;
  return `${Math.floor(diffMs / 3_600_000)} ${T.hoursAgo}`;
}

function fmtDateTime(iso) {
  if (!iso) return T.alarmTimeEmpty;
  const date = new Date(iso);
  if (!Number.isFinite(date.getTime())) return T.alarmTimeEmpty;
  const pad = (value) => String(value).padStart(2, "0");
  return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())} ${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
}

function levelLabel(level) {
  return LEVEL_META[level]?.title || T.none;
}

function itemStateLabel(item) {
  if (item.value === null) return T.missing;
  return item.level > 0 ? levelLabel(item.level) : T.normal;
}

function evaluateLevel(value, thresholds) {
  if (value === null) return 0;
  for (const threshold of thresholds) {
    if (value >= threshold.min) return threshold.level;
  }
  return 0;
}

function triggeredThreshold(item) {
  if (item.threshold) return String(item.threshold);
  if (!item.level || !Array.isArray(item.thresholds)) return T.noThreshold;
  const threshold = item.thresholds.find((entry) => entry.level === item.level);
  return threshold ? `${T.triggerThreshold} >= ${threshold.min} ${item.unit}`.trim() : T.noThreshold;
}

function metricItem(label, value, unit, thresholds, digits = 1) {
  const cleanValue = finiteNumber(value);
  const item = { label, value: cleanValue, unit, thresholds, digits, level: 0 };
  item.level = evaluateLevel(cleanValue, thresholds);
  return item;
}

function buildMonitoredItems(data) {
  const env = data?.environment || {};
  const tempCtrl = data?.temperature_controller || {};
  return [
    metricItem(`${T.tempCtrl} PV`, tempCtrl.pv, tempCtrl.unit || "℃", TEMP_THRESHOLDS, 1),
    metricItem(T.cabinetTemp, env.temperature?.value, env.temperature?.unit || "℃", TEMP_THRESHOLDS, 1),
    metricItem(T.infraredTemp, env.infrared_temperature?.value, env.infrared_temperature?.unit || "℃", TEMP_THRESHOLDS, 1),
    metricItem(T.smoke, env.smoke?.value, env.smoke?.unit || "ppm", GAS_THRESHOLDS, 1),
    metricItem(T.hydrogen, env.hydrogen?.value, env.hydrogen?.unit || "ppm", GAS_THRESHOLDS, 1),
    metricItem(T.carbonMonoxide, env.carbon_monoxide?.value, env.carbon_monoxide?.unit || "ppm", GAS_THRESHOLDS, 1),
  ];
}


function normalizeAssessmentItem(item) {
  const value = finiteNumber(item?.value);
  return {
    label: item?.name || item?.key || "异常模块",
    value,
    rawValue: item?.value,
    unit: item?.unit || "",
    level: Number(item?.level || 0),
    threshold: item?.threshold || "",
    message: item?.message || "",
  };
}

function assessmentItems(assessment) {
  const items = Array.isArray(assessment?.items) ? assessment.items : [];
  return items.map(normalizeAssessmentItem);
}

function clearChildren(element) {
  if (!element) return;
  while (element.firstChild) element.removeChild(element.firstChild);
}

function createSummaryRow(item) {
  const row = document.createElement("div");
  row.className = `alarm-module-item alarm-row-level-${item.level}`;

  const copy = document.createElement("div");
  const title = document.createElement("strong");
  title.textContent = item.label;
  const meta = document.createElement("small");
  meta.textContent = `${fmtValue(item)} · ${triggeredThreshold(item)}`;
  copy.append(title, meta);

  const badge = document.createElement("span");
  badge.textContent = levelLabel(item.level);
  row.append(copy, badge);
  return row;
}

function createDetailRow(item) {
  const row = document.createElement("div");
  const missing = item.value === null;
  row.className = `alarm-detail-row ${missing ? "alarm-row-missing" : `alarm-row-level-${item.level}`}`;

  const label = document.createElement("strong");
  label.textContent = item.label;
  const value = document.createElement("small");
  value.textContent = `${fmtValue(item)} · ${missing ? T.missing : levelLabel(item.level)} · ${triggeredThreshold(item)}`;
  const badge = document.createElement("span");
  badge.textContent = missing ? T.missing : levelLabel(item.level);
  row.append(label, value, badge);
  return row;
}

function renderSummary(items, payload) {
  const container = $("alarm-module-list");
  clearChildren(container);
  const assessment = payload?.assessment || {};
  const anomalies = items.filter((item) => item.level > 0);

  if (!container) return;
  if (!anomalies.length) {
    const empty = document.createElement("div");
    empty.className = "alarm-empty";
    const updatedAt = assessment.updated_at || payload?.data?.updated_at || payload?.last_seen_at || payload?.server_time || "";
    empty.textContent = assessment.summary || (updatedAt
      ? `${T.allNormalRecent} ${fmtAge(updatedAt)}?`
      : T.allNormal);
    container.append(empty);
    return;
  }

  anomalies
    .sort((left, right) => LEVEL_PRIORITY[right.level] - LEVEL_PRIORITY[left.level])
    .slice(0, 4)
    .forEach((item) => container.append(createSummaryRow(item)));
}

function renderDetails(items) {
  const container = $("alarm-detail-list");
  clearChildren(container);
  if (!container) return;
  items
    .slice()
    .sort((left, right) => {
      const priorityDiff = LEVEL_PRIORITY[right.level] - LEVEL_PRIORITY[left.level];
      if (priorityDiff) return priorityDiff;
      return left.label.localeCompare(right.label, "zh-CN");
    })
    .forEach((item) => container.append(createDetailRow(item)));
}

function highestLevel(items) {
  return items.reduce((best, item) => (
    LEVEL_PRIORITY[item.level] > LEVEL_PRIORITY[best] ? item.level : best
  ), 0);
}

function applyPanelState(level) {
  const panel = $("menu-alarm-panel");
  const meta = LEVEL_META[level] || LEVEL_META[0];
  if (panel) {
    panel.classList.remove("alarm-state-none", "alarm-state-3", "alarm-state-2", "alarm-state-1");
    panel.classList.add(meta.className);
  }
  setText("alarm-title", meta.title);
  setText("alarm-level-badge", meta.badge);
}

function renderAlarm(payload) {
  const assessment = payload?.assessment || {};
  latestItems = assessmentItems(assessment);
  const level = Number(assessment.level || 0);
  const anomalies = latestItems.filter((item) => item.level > 0);
  const alarmStamp = level > 0 ? (assessment.started_at || assessment.updated_at || payload?.server_time || "") : "";

  setText("alarm-time", alarmStamp ? fmtDateTime(alarmStamp) : T.alarmTimeEmpty);
  applyPanelState(level);
  setText(
    "alarm-summary",
    assessment.summary || (anomalies.length
      ? `${anomalies.length} ${T.alarmCountPrefix}${levelLabel(level)}?`
      : T.noAlarmSummary)
  );
  renderSummary(latestItems, payload);
  renderDetails(latestItems);
}

function renderFetchError(error) {
  applyPanelState(3);
  setText("alarm-summary", `${T.fetchFailed}${error.message || error}`);
  const container = $("alarm-module-list");
  clearChildren(container);
  if (container) {
    const item = document.createElement("div");
    item.className = "alarm-empty";
    item.textContent = T.cannotRead;
    container.append(item);
  }
}

async function fetchAlarmData() {
  const response = await fetch("/api/cloud/cabinet-data", { cache: "no-store" });
  const payload = await response.json();
  if (!response.ok || !payload.ok) throw new Error(payload.message || `HTTP ${response.status}`);
  renderAlarm(payload);
}

function latencyLabel(latencyMs) {
  if (!Number.isFinite(latencyMs)) return { text: T.networkUnknown, className: "latency-warn" };
  if (latencyMs <= 150) return { text: T.networkNormal, className: "latency-ok" };
  if (latencyMs <= 400) return { text: T.networkSlow, className: "latency-warn" };
  return { text: T.networkBad, className: "latency-bad" };
}

function renderNetworkLatency(latencyMs) {
  const target = $("robot-network-latency");
  if (!target) return;
  const label = latencyLabel(latencyMs);
  target.classList.remove("latency-ok", "latency-warn", "latency-bad");
  target.classList.add(label.className);
  target.textContent = Number.isFinite(latencyMs) ? `${Math.round(latencyMs)} ms ? ${label.text}` : label.text;
}

function renderRobotState(payload, latencyMs) {
  renderNetworkLatency(latencyMs);
  const board = payload?.board || {};
  const ros = board.ros_status || {};
  const nav = board.nav_status || {};
  const cruise = board.cruise_status || {};
  const navigationMode = String(ros.navigation_mode || "").toLowerCase();
  const cruiseMode = String(nav.web_cruise_mode || cruise.mode || "").toLowerCase();
  const working = Boolean(
    nav.task_running
      || nav.goal_active
      || cruise.running
      || ros.mapping_live
      || ["mapping", "navigation"].includes(navigationMode)
      || (cruiseMode && cruiseMode !== "idle" && cruiseMode !== "stopped")
  );
  const dot = $("robot-status-dot");
  if (dot) {
    dot.classList.remove("is-working", "is-error");
    if (working) dot.classList.add("is-working");
  }
  setText("robot-work-state", working ? T.robotWorking : T.robotIdle);
  setText("robot-current-task", T.currentTask);
}

function renderRobotError(error) {
  const dot = $("robot-status-dot");
  if (dot) {
    dot.classList.remove("is-working");
    dot.classList.add("is-error");
  }
  setText("robot-work-state", T.robotIdle);
  setText("robot-current-task", T.currentTask);
  const target = $("robot-network-latency");
  if (target) {
    target.classList.remove("latency-ok", "latency-warn");
    target.classList.add("latency-bad");
    target.textContent = `${T.networkBad}${error?.message ? ` ? ${error.message}` : ""}`;
  }
}

async function fetchRobotState() {
  const startedAt = performance.now();
  const response = await fetch("/api/cloud/state", { cache: "no-store" });
  const latencyMs = performance.now() - startedAt;
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.message || `HTTP ${response.status}`);
  renderRobotState(payload, latencyMs);
}

function setExpanded(expanded) {
  alarmExpanded = Boolean(expanded);
  const detail = $("alarm-detail");
  const panel = $("menu-alarm-panel");
  const toggle = $("alarm-detail-toggle");
  if (detail) detail.hidden = !alarmExpanded;
  if (panel) panel.setAttribute("aria-expanded", String(alarmExpanded));
  if (toggle) {
    toggle.setAttribute("aria-expanded", String(alarmExpanded));
    toggle.textContent = alarmExpanded ? T.collapseDetails : T.showDetails;
  }
  renderDetails(latestItems);
}

function setArmOpen(nextOpen) {
  armOpen = Boolean(nextOpen);
  setText("alarm-arm-state", armOpen ? T.armOpen : T.armClosed);
  $("alarm-arm-open")?.classList.toggle("is-active", armOpen);
  $("alarm-arm-close")?.classList.toggle("is-active", !armOpen);
  try {
    window.localStorage.setItem(ARM_STORAGE_KEY, armOpen ? "1" : "0");
  } catch (error) {
    // Local storage is optional; the button still works for the current page.
  }
}

function bindAlarmUi() {
  const panel = $("menu-alarm-panel");
  const toggle = $("alarm-detail-toggle");
  const openButton = $("alarm-arm-open");
  const closeButton = $("alarm-arm-close");

  panel?.addEventListener("click", (event) => {
    if (event.target instanceof Element && event.target.closest("button, a")) return;
    setExpanded(!alarmExpanded);
  });
  panel?.addEventListener("keydown", (event) => {
    if (event.key !== "Enter" && event.key !== " ") return;
    if (event.target instanceof Element && event.target.closest("button, a")) return;
    event.preventDefault();
    setExpanded(!alarmExpanded);
  });
  toggle?.addEventListener("click", (event) => {
    event.stopPropagation();
    setExpanded(!alarmExpanded);
  });
  openButton?.addEventListener("click", (event) => {
    event.stopPropagation();
    setArmOpen(true);
  });
  closeButton?.addEventListener("click", (event) => {
    event.stopPropagation();
    setArmOpen(false);
  });

  try {
    armOpen = window.localStorage.getItem(ARM_STORAGE_KEY) === "1";
  } catch (error) {
    armOpen = false;
  }
  setArmOpen(armOpen);
}

window.addEventListener("load", () => {
  bindAlarmUi();
  fetchAlarmData().catch(renderFetchError);
  fetchRobotState().catch(renderRobotError);
  window.setInterval(() => {
    fetchAlarmData().catch(renderFetchError);
    fetchRobotState().catch(renderRobotError);
  }, ALARM_REFRESH_MS);
});
