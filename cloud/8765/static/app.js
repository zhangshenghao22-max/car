const state = {
  payload: null,
  mapFrameStamp: "",
  scanFrameStamp: "",
  commandBusy: false,
  viewport: {
    scale: 1,
    minScale: 1,
    maxScale: 6,
    offsetX: 0,
    offsetY: 0,
    dragging: false,
    pointerId: null,
    dragStartX: 0,
    dragStartY: 0,
    dragOriginX: 0,
    dragOriginY: 0,
    pointerMoved: false,
  },
  navSession: {
    key: "",
    startPose: null,
    goalPose: null,
  },
  navDraft: {
    mapKey: "",
    clickMode: "none",
    startPose: null,
    points: [],
  },
  teleop: {
    controllerId: "",
    enabled: false,
    pressedKeys: new Set(),
    speedLevel: 2,
    seq: 0,
    heartbeatTimer: 0,
    lastError: "",
  },
};

const pageMode = document.body.dataset.page || "menu";
const commandToken = document.body.dataset.commandToken || "";
const LIVE_UPLOAD_WINDOW_MS = 15000;
const NAV_DRAFT_STORAGE_PREFIX = "carCloudNavDraft:";
const TELEOP_CONTROLLER_STORAGE_KEY = "carCloudTeleopControllerId";
const TELEOP_HEARTBEAT_MS = 250;
const TELEOP_LIVE_MS = 1400;
const TELEOP_KEYS = ["w", "a", "s", "d", "q", "e", " "];

function $(id) {
  return document.getElementById(id);
}

function fmtText(value, fallback = "-") {
  if (value === null || value === undefined || value === "") return fallback;
  return String(value);
}

function makeControllerId() {
  if (window.crypto && typeof window.crypto.randomUUID === "function") {
    return window.crypto.randomUUID();
  }
  return `teleop-${Date.now()}-${Math.random().toString(16).slice(2, 10)}`;
}

function ensureTeleopControllerId() {
  if (state.teleop.controllerId) return state.teleop.controllerId;
  let value = "";
  try {
    value = window.localStorage.getItem(TELEOP_CONTROLLER_STORAGE_KEY) || "";
  } catch (error) {
    value = "";
  }
  value = String(value || "").trim();
  if (!value) {
    value = makeControllerId();
    try {
      window.localStorage.setItem(TELEOP_CONTROLLER_STORAGE_KEY, value);
    } catch (error) {
      // Ignore storage errors and keep the in-memory id.
    }
  }
  state.teleop.controllerId = value;
  return value;
}

function ageMs(iso) {
  if (!iso) return Number.POSITIVE_INFINITY;
  const diffMs = Date.now() - new Date(iso).getTime();
  return Number.isFinite(diffMs) ? diffMs : Number.POSITIVE_INFINITY;
}

function isFreshIso(iso, maxAgeMs = LIVE_UPLOAD_WINDOW_MS) {
  return ageMs(iso) <= maxAgeMs;
}

function teleopSession() {
  const payload = state.payload?.teleop;
  return payload && typeof payload === "object" ? payload : {};
}

function teleopIsLive(session = teleopSession()) {
  return Boolean(session.enabled) && ageMs(session.updated_at) <= TELEOP_LIVE_MS;
}

function teleopOwnedByThisPage(session = teleopSession()) {
  return Boolean(session.controller_id) && session.controller_id === ensureTeleopControllerId();
}

function teleopPressedArray() {
  return Array.from(state.teleop.pressedKeys.values()).sort((a, b) => TELEOP_KEYS.indexOf(a) - TELEOP_KEYS.indexOf(b));
}

function teleopEditableTarget(target) {
  return target instanceof HTMLInputElement ||
    target instanceof HTMLTextAreaElement ||
    target instanceof HTMLSelectElement ||
    target?.isContentEditable;
}

function teleopKeyFromEvent(event) {
  if (!event || typeof event.key !== "string") return "";
  if (event.key === " ") return " ";
  const key = event.key.toLowerCase();
  return TELEOP_KEYS.includes(key) ? key : "";
}

function fmtAge(iso) {
  if (!iso) return "-";
  const diffMs = ageMs(iso);
  if (!Number.isFinite(diffMs)) return "-";
  if (diffMs < 1000) return "刚刚";
  if (diffMs < 60_000) return `${Math.floor(diffMs / 1000)} 秒前`;
  if (diffMs < 3_600_000) return `${Math.floor(diffMs / 60_000)} 分钟前`;
  return `${Math.floor(diffMs / 3_600_000)} 小时前`;
}

function defaultMapPlaceholder() {
  return pageMode === "navigation"
    ? "开始导航后这里会显示地图和路径"
    : "点击“开始建图”后这里会显示地图图像";
}

function defaultScanPlaceholder() {
  return "点击“开始建图”后这里会显示雷达图像";
}

function setFrame(kind, imageId, placeholderId, stamp, fallbackText, shouldShow) {
  const frame = state.payload?.frames?.[kind];
  const image = $(imageId);
  const placeholder = $(placeholderId);
  if (!image || !placeholder) return false;

  if (!shouldShow || !frame || !isFreshIso(frame.uploaded_at)) {
    image.removeAttribute("src");
    image.dataset.currentSrc = "";
    image.dataset.sourceKind = "";
    image.style.display = "none";
    placeholder.textContent = fallbackText;
    placeholder.style.display = "grid";
    return false;
  }

  const desiredSrc = `${frame.url}?ts=${encodeURIComponent(frame.uploaded_at)}`;
  if (image.dataset.currentSrc !== desiredSrc) {
    state[`${kind}FrameStamp`] = frame.uploaded_at;
    image.dataset.currentSrc = desiredSrc;
    image.dataset.sourceKind = "live";
    image.src = desiredSrc;
  }
  image.style.display = "block";
  placeholder.style.display = "none";
  return true;
}

function renderList(targetId, entries) {
  const target = $(targetId);
  if (!target) return;
  target.innerHTML = "";
  for (const [label, value] of entries) {
    const dt = document.createElement("dt");
    dt.textContent = label;
    const dd = document.createElement("dd");
    dd.textContent = fmtText(value);
    target.append(dt, dd);
  }
}

function worldToCanvas(point, mapMeta, naturalW, naturalH, canvasW, canvasH) {
  if (!mapMeta || !Number.isFinite(mapMeta.resolution) || !Number.isFinite(mapMeta.origin_x) || !Number.isFinite(mapMeta.origin_y)) {
    return null;
  }
  const px = (point.x - mapMeta.origin_x) / mapMeta.resolution;
  const py = naturalH - ((point.y - mapMeta.origin_y) / mapMeta.resolution);
  const sx = canvasW / naturalW;
  const sy = canvasH / naturalH;
  return { x: px * sx, y: py * sy };
}

function hasUsableMapMeta(mapMeta) {
  return Boolean(
    mapMeta &&
    Number.isFinite(Number(mapMeta.width)) &&
    Number.isFinite(Number(mapMeta.height)) &&
    Number.isFinite(Number(mapMeta.resolution)) &&
    Number.isFinite(Number(mapMeta.origin_x)) &&
    Number.isFinite(Number(mapMeta.origin_y))
  );
}

function clamp(value, minValue, maxValue) {
  return Math.min(Math.max(value, minValue), maxValue);
}

function hasFinitePose(pose) {
  return Boolean(
    pose &&
    Number.isFinite(Number(pose.x)) &&
    Number.isFinite(Number(pose.y))
  );
}

function clonePose(pose) {
  if (!hasFinitePose(pose)) return null;
  return {
    x: Number(pose.x),
    y: Number(pose.y),
    yaw: Number(pose.yaw || 0),
  };
}

function formatPoseText(pose) {
  if (!hasFinitePose(pose)) return "等待位姿";
  const yawDeg = Number(pose.yaw || 0) * 180 / Math.PI;
  return `x=${Number(pose.x).toFixed(2)} y=${Number(pose.y).toFixed(2)} yaw=${yawDeg.toFixed(1)}°`;
}

function currentCruiseStatus() {
  const status = state.payload?.board?.cruise_status;
  return status && typeof status === "object" ? status : {};
}

function activeMapMeta() {
  const liveMapMeta = state.payload?.board?.map_meta || {};
  const savedMapMeta = selectedSavedMapMeta();
  return hasUsableMapMeta(liveMapMeta) ? liveMapMeta : savedMapMeta;
}

function currentDraftMapKey() {
  return preferredSavedMapKey() || "default";
}

function emptyNavDraft(mapKey = currentDraftMapKey()) {
  return {
    mapKey,
    clickMode: "none",
    startPose: null,
    points: [],
  };
}

function normalizeDraftPose(pose, fallbackLabel = "") {
  if (!hasFinitePose(pose)) return null;
  return {
    x: Number(pose.x),
    y: Number(pose.y),
    yaw: Number(pose.yaw || 0),
    label: String(pose.label || fallbackLabel || "").trim(),
  };
}

function navDraftStorageKey(mapKey) {
  return `${NAV_DRAFT_STORAGE_PREFIX}${String(mapKey || "default").trim() || "default"}`;
}

function loadNavDraft(mapKey = currentDraftMapKey()) {
  const draft = emptyNavDraft(mapKey);
  try {
    const raw = window.localStorage.getItem(navDraftStorageKey(mapKey));
    if (!raw) return draft;
    const payload = JSON.parse(raw);
    const startPose = normalizeDraftPose(payload?.startPose || null, "S");
    const points = Array.isArray(payload?.points)
      ? payload.points.map((item, index) => normalizeDraftPose(item, `P${index + 1}`)).filter(Boolean)
      : [];
    return {
      mapKey,
      clickMode: "none",
      startPose,
      points,
    };
  } catch (_error) {
    return draft;
  }
}

function persistNavDraft() {
  if (pageMode !== "navigation") return;
  try {
    window.localStorage.setItem(
      navDraftStorageKey(state.navDraft.mapKey || currentDraftMapKey()),
      JSON.stringify({
        startPose: state.navDraft.startPose,
        points: state.navDraft.points,
      })
    );
  } catch (_error) {
    // ignore storage failures
  }
}

function syncNavDraft(mapKey = currentDraftMapKey()) {
  const normalizedKey = String(mapKey || "default").trim() || "default";
  if (state.navDraft.mapKey === normalizedKey) return;
  state.navDraft = loadNavDraft(normalizedKey);
}

function setNavClickMode(mode) {
  state.navDraft.clickMode = mode;
  updateNavClickButtons();
}

function updateNavClickButtons() {
  const mode = state.navDraft?.clickMode || "none";
  $("nav-mark-start")?.classList.toggle("is-active", mode === "start");
  $("nav-mark-waypoint")?.classList.toggle("is-active", mode === "waypoint");
  $("nav-mark-stop")?.classList.toggle("is-active", mode === "none");
  const hint = $("nav-click-hint");
  if (!hint) return;
  if (mode === "start") {
    hint.textContent = "当前是起点标注模式：点击地图设置 AMCL 起点。";
  } else if (mode === "waypoint") {
    hint.textContent = "当前是巡航点模式：每点击一次地图就会追加一个巡航点。";
  } else {
    hint.textContent = "先加载地图，再选择标点模式，然后直接在地图上点击。";
  }
}

function nextDraftYawDeg() {
  const raw = Number($("nav-yaw-input")?.value || 0);
  return Number.isFinite(raw) ? raw : 0;
}

function draftLastPoint() {
  const points = Array.isArray(state.navDraft?.points) ? state.navDraft.points : [];
  return points.length ? points[points.length - 1] : null;
}

function renderNavDraftList() {
  const container = $("nav-draft-list");
  if (!container) return;
  const startPose = state.navDraft?.startPose;
  const points = Array.isArray(state.navDraft?.points) ? state.navDraft.points : [];
  const cards = [];
  if (hasFinitePose(startPose)) {
    cards.push(`
      <div class="nav-draft-card">
        <strong>起点 S</strong>
        <small>${formatPoseText(startPose)}</small>
      </div>
    `);
  }
  points.forEach((point, index) => {
    const yawDeg = Number(point.yaw || 0) * 180 / Math.PI;
    cards.push(`
      <div class="nav-draft-card">
        <strong>巡航点 ${index + 1}</strong>
        <small>x=${Number(point.x).toFixed(2)} y=${Number(point.y).toFixed(2)}</small>
        <div class="nav-draft-row">
          <input data-draft-yaw="${index}" type="number" step="1" value="${yawDeg.toFixed(1)}" />
          <button data-draft-use="${index}" type="button" class="subtle-btn">跑这个点</button>
          <button data-draft-remove="${index}" type="button" class="subtle-btn">删除</button>
        </div>
      </div>
    `);
  });
  container.innerHTML = cards.length ? cards.join("") : '<div class="nav-draft-empty">暂无起点和巡航点</div>';
}

function renderCruiseSummary() {
  const target = $("nav-cruise-summary");
  if (!target) return;
  const cruise = currentCruiseStatus();
  if (cruise?.task_running) {
    const currentIndex = Number(cruise.current_index || 0) + 1;
    const total = Array.isArray(cruise.points) ? cruise.points.length : 0;
    const loopIndex = Number(cruise.loop_index || 0);
    const loopCount = Number(cruise.loop_count || 1);
    target.textContent = `巡航执行中：点 ${currentIndex}/${total}，循环 ${loopIndex}/${loopCount}`;
    return;
  }
  if (cruise?.last_error) {
    target.textContent = `巡航失败：${cruise.last_error}`;
    return;
  }
  if (cruise?.last_result) {
    target.textContent = `巡航结果：${cruise.last_result}`;
    return;
  }
  const draftPoints = Array.isArray(state.navDraft?.points) ? state.navDraft.points.length : 0;
  target.textContent = draftPoints ? `已编辑 ${draftPoints} 个巡航点，可随时下发。` : "巡航未启动";
}

function eventToDraftPose(clientX, clientY) {
  const shell = currentMapShell();
  const image = $("map-frame");
  const mapMeta = activeMapMeta();
  if (!shell || !image || !currentMapImageVisible() || !hasUsableMapMeta(mapMeta) || !image.naturalWidth || !image.naturalHeight) {
    return null;
  }
  const rect = shell.getBoundingClientRect();
  const localX = (clientX - rect.left - state.viewport.offsetX) / state.viewport.scale;
  const localY = (clientY - rect.top - state.viewport.offsetY) / state.viewport.scale;
  if (localX < 0 || localY < 0 || localX > rect.width || localY > rect.height) return null;
  const px = (localX / rect.width) * image.naturalWidth;
  const py = (localY / rect.height) * image.naturalHeight;
  return {
    x: Number(mapMeta.origin_x) + (px * Number(mapMeta.resolution)),
    y: Number(mapMeta.origin_y) + ((image.naturalHeight - py) * Number(mapMeta.resolution)),
    yaw: nextDraftYawDeg() * Math.PI / 180,
  };
}

function handleNavMapClick(event) {
  if (pageMode !== "navigation") return;
  if ((state.navDraft?.clickMode || "none") === "none") return;
  if (state.viewport.pointerMoved) return;
  const pose = eventToDraftPose(event.clientX, event.clientY);
  if (!pose) return;
  if (state.navDraft.clickMode === "start") {
    state.navDraft.startPose = normalizeDraftPose(pose, "S");
  } else if (state.navDraft.clickMode === "waypoint") {
    state.navDraft.points.push(normalizeDraftPose(pose, `P${state.navDraft.points.length + 1}`));
  }
  persistNavDraft();
  renderNavDraftList();
  renderNavPoseSummary();
  renderCruiseSummary();
  renderOverlay();
}

function navGoalPose(board, nav) {
  if (hasFinitePose(board?.goal_pose)) return board.goal_pose;
  if (hasFinitePose(nav?.current_goal)) return nav.current_goal;
  if (hasFinitePose(currentCruiseStatus()?.current_goal)) return currentCruiseStatus().current_goal;
  return null;
}

function navSessionKey(goalPose) {
  if (!hasFinitePose(goalPose)) return "";
  return [
    Number(goalPose.x).toFixed(3),
    Number(goalPose.y).toFixed(3),
    Number(goalPose.yaw || 0).toFixed(3),
  ].join("|");
}

function drawRobot(ctx, pose, mapMeta, naturalW, naturalH, canvasW, canvasH) {
  const p = worldToCanvas(pose, mapMeta, naturalW, naturalH, canvasW, canvasH);
  if (!p) return;
  const yaw = Number(pose.yaw || 0);
  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#1f9d74";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-8, -8);
  ctx.lineTo(-2, 0);
  ctx.lineTo(-8, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawGoal(ctx, pose, mapMeta, naturalW, naturalH, canvasW, canvasH) {
  const p = worldToCanvas(pose, mapMeta, naturalW, naturalH, canvasW, canvasH);
  if (!p) return;
  ctx.save();
  ctx.strokeStyle = "#d45c4b";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
  ctx.moveTo(p.x - 12, p.y);
  ctx.lineTo(p.x + 12, p.y);
  ctx.moveTo(p.x, p.y - 12);
  ctx.lineTo(p.x, p.y + 12);
  ctx.stroke();
  ctx.restore();
}

function drawPoseMarker(ctx, pose, mapMeta, naturalW, naturalH, canvasW, canvasH, options = {}) {
  const p = worldToCanvas(pose, mapMeta, naturalW, naturalH, canvasW, canvasH);
  if (!p) return;
  const stroke = options.stroke || "#2563eb";
  const fill = options.fill || "rgba(37, 99, 235, 0.14)";
  const label = options.label || "";
  const yaw = Number(pose.yaw || 0);
  const radius = Number(options.radius || 12);

  ctx.save();
  ctx.translate(p.x, p.y);
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.rotate(-yaw);
  ctx.fillStyle = stroke;
  ctx.beginPath();
  ctx.moveTo(radius + 10, 0);
  ctx.lineTo(-radius * 0.55, -radius * 0.7);
  ctx.lineTo(-radius * 0.1, 0);
  ctx.lineTo(-radius * 0.55, radius * 0.7);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  if (!label) return;
  ctx.save();
  ctx.fillStyle = stroke;
  ctx.font = "700 14px 'Microsoft YaHei UI', sans-serif";
  ctx.fillText(label, p.x + radius + 8, p.y - radius - 4);
  ctx.restore();
}

function drawWaypointBadge(ctx, pose, index, mapMeta, naturalW, naturalH, canvasW, canvasH, options = {}) {
  drawPoseMarker(ctx, pose, mapMeta, naturalW, naturalH, canvasW, canvasH, {
    stroke: options.stroke || "#b45309",
    fill: options.fill || "rgba(245, 158, 11, 0.16)",
    label: String(index + 1),
    radius: options.radius || 10,
  });
}

function drawDraftOverlay(ctx, image, canvas, mapMeta) {
  if (pageMode !== "navigation") return;
  const cruise = currentCruiseStatus();
  const startPose = state.navDraft?.startPose || cruise.start_pose;
  const points = Array.isArray(state.navDraft?.points) && state.navDraft.points.length
    ? state.navDraft.points
    : (Array.isArray(cruise.points) ? cruise.points : []);
  if (hasFinitePose(startPose)) {
    drawPoseMarker(ctx, startPose, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height, {
      stroke: "#0f766e",
      fill: "rgba(15, 118, 110, 0.14)",
      label: "S",
      radius: 10,
    });
  }
  points.forEach((point, index) => {
    drawWaypointBadge(ctx, point, index, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height);
  });
}

function renderOverlay() {
  const image = $("map-frame");
  const canvas = $("map-overlay");
  const summary = $("path-summary");
  const shell = currentMapShell() || image?.parentElement?.parentElement;
  if (!image || !canvas || !summary) return;

  const payload = state.payload || {};
  const liveMapMeta = payload.board?.map_meta || {};
  const fallbackMapMeta = pageMode === "navigation" ? selectedSavedMapMeta() : {};
  const mapMeta = hasUsableMapMeta(liveMapMeta) ? liveMapMeta : fallbackMapMeta;
  const navPath = payload.board?.nav_path || {};
  const robotPose = payload.board?.robot_pose || {};
  const showNavigationOverlay = pageMode === "navigation";
  const goalPose = showNavigationOverlay
    ? (state.navSession.goalPose || navGoalPose(payload.board, payload.board?.nav_status || {}) || draftLastPoint() || {})
    : {};
  const startPose = showNavigationOverlay
    ? (state.navSession.startPose || state.navDraft?.startPose || {})
    : {};

  if (!image.naturalWidth || !image.naturalHeight || image.style.display === "none") {
    canvas.width = 0;
    canvas.height = 0;
    summary.textContent = pageMode === "navigation" ? "开始导航后这里会显示路径信息" : "开始建图后这里会显示地图信息";
    return;
  }

  const rect = shell ? shell.getBoundingClientRect() : image.getBoundingClientRect();
  canvas.width = Math.max(1, Math.round(rect.width));
  canvas.height = Math.max(1, Math.round(rect.height));
  canvas.style.width = `${Math.round(rect.width)}px`;
  canvas.style.height = `${Math.round(rect.height)}px`;

  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const points = showNavigationOverlay && Array.isArray(navPath.points) ? navPath.points : [];
  if (showNavigationOverlay && points.length > 1) {
    ctx.save();
    ctx.strokeStyle = "#2563eb";
    ctx.lineWidth = 4;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    points.forEach((point, index) => {
      const mapped = worldToCanvas(point, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height);
      if (!mapped) return;
      if (index === 0) ctx.moveTo(mapped.x, mapped.y);
      else ctx.lineTo(mapped.x, mapped.y);
    });
    ctx.stroke();
    ctx.restore();
  }

  if (Number.isFinite(Number(robotPose.x)) && Number.isFinite(Number(robotPose.y))) {
    drawRobot(ctx, robotPose, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height);
  }
  if (showNavigationOverlay && hasFinitePose(startPose)) {
    drawPoseMarker(ctx, startPose, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height, {
      stroke: "#0f766e",
      fill: "rgba(15, 118, 110, 0.18)",
      label: "S",
      radius: 11,
    });
  }
  if (showNavigationOverlay && Number.isFinite(Number(goalPose.x)) && Number.isFinite(Number(goalPose.y))) {
    drawGoal(ctx, goalPose, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height);
    drawPoseMarker(ctx, goalPose, mapMeta, image.naturalWidth, image.naturalHeight, canvas.width, canvas.height, {
      stroke: "#d45c4b",
      fill: "rgba(212, 92, 75, 0.16)",
      label: "G",
      radius: 11,
    });
  }

  if (pageMode === "navigation") {
    summary.textContent = points.length > 1
      ? `当前路径共 ${points.length} 个点`
      : "当前还没有规划出路径";
  } else {
    summary.textContent = "地图正在等待更多数据";
  }
}

function commandLabel(action) {
  const mapping = {
    start_mapping: "开始建图",
    stop_mapping: "停止建图",
    save_map: "保存地图",
    start_navigation: "开始导航",
    stop_navigation: "停止导航",
    set_initial_pose: "下发起点",
    start_cruise: "开始巡航",
    stop_cruise: "停止巡航",
    capture_recognition: "拍照识别",
  };
  return mapping[action] || action;
}

function isMappingAction(action) {
  return ["start_mapping", "stop_mapping", "save_map"].includes(String(action || ""));
}

function isNavigationAction(action) {
  return ["start_navigation", "stop_navigation", "set_initial_pose", "start_cruise", "stop_cruise"].includes(String(action || ""));
}

function savedMaps() {
  const maps = state.payload?.board?.saved_maps;
  return Array.isArray(maps) ? maps : [];
}

function preferredSavedMapKey() {
  const selectValue = ($("saved-map-select")?.value || "").trim();
  const inputValue = ($("nav-map-input")?.value || "").trim();
  return selectValue || inputValue;
}

function selectedSavedMap() {
  const key = preferredSavedMapKey();
  if (!key) return null;
  return savedMaps().find((item) => {
    const name = String(item?.name || "").trim();
    const yaml = String(item?.yaml || "").trim();
    return key === name || key === yaml || key === yaml.replace(/\.yaml$/i, "");
  }) || null;
}

function selectedSavedMapMeta() {
  const selected = selectedSavedMap();
  if (!selected) return {};
  const previewMeta = selected.preview_meta || selected.map_meta || {};
  return hasUsableMapMeta(previewMeta) ? previewMeta : {};
}

function showSavedMapPreview() {
  const selected = selectedSavedMap();
  const image = $("map-frame");
  const placeholder = $("map-placeholder");
  if (!image || !placeholder || !selected?.preview_url) {
    return false;
  }

  const stamp = encodeURIComponent(selected.preview_uploaded_at || selected.updated_at || selected.name || "");
  const previewSrc = `${selected.preview_url}?ts=${stamp}`;
  if (image.dataset.currentSrc !== previewSrc) {
    image.dataset.currentSrc = previewSrc;
    image.dataset.sourceKind = "saved-preview";
    image.src = previewSrc;
  }
  image.style.display = "block";
  placeholder.style.display = "none";
  $("map-meta").textContent = selected.name || selected.yaml || "已保存地图";
  return true;
}

function currentMapShell() {
  return $("map-shell");
}

function currentMapStack() {
  return $("map-stack");
}

function currentMapImageVisible() {
  const image = $("map-frame");
  return Boolean(image && image.style.display !== "none");
}

function constrainViewport() {
  const shell = currentMapShell();
  if (!shell) return;
  if (state.viewport.scale <= 1.001) {
    state.viewport.scale = 1;
    state.viewport.offsetX = 0;
    state.viewport.offsetY = 0;
    return;
  }
  const rect = shell.getBoundingClientRect();
  const scaledWidth = rect.width * state.viewport.scale;
  const scaledHeight = rect.height * state.viewport.scale;
  const minX = Math.min(0, rect.width - scaledWidth);
  const minY = Math.min(0, rect.height - scaledHeight);
  state.viewport.offsetX = clamp(state.viewport.offsetX, minX, 0);
  state.viewport.offsetY = clamp(state.viewport.offsetY, minY, 0);
}

function applyViewport() {
  const stack = currentMapStack();
  const resetButton = $("map-zoom-reset");
  if (!stack) return;
  constrainViewport();
  stack.style.transform = `translate(${state.viewport.offsetX}px, ${state.viewport.offsetY}px) scale(${state.viewport.scale})`;
  stack.style.cursor = state.viewport.dragging ? "grabbing" : (state.viewport.scale > 1.001 ? "grab" : "default");
  if (resetButton) {
    resetButton.textContent = `${Math.round(state.viewport.scale * 100)}%`;
  }
}

function resetViewport() {
  state.viewport.scale = 1;
  state.viewport.offsetX = 0;
  state.viewport.offsetY = 0;
  state.viewport.dragging = false;
  state.viewport.pointerId = null;
  applyViewport();
}

function zoomViewport(factor, clientX, clientY) {
  const shell = currentMapShell();
  if (!shell || !currentMapImageVisible()) return;
  const rect = shell.getBoundingClientRect();
  const previousScale = state.viewport.scale;
  const nextScale = clamp(previousScale * factor, state.viewport.minScale, state.viewport.maxScale);
  if (Math.abs(nextScale - previousScale) < 0.001) return;
  const anchorX = clientX - rect.left;
  const anchorY = clientY - rect.top;
  state.viewport.offsetX = anchorX - ((anchorX - state.viewport.offsetX) * (nextScale / previousScale));
  state.viewport.offsetY = anchorY - ((anchorY - state.viewport.offsetY) * (nextScale / previousScale));
  state.viewport.scale = nextScale;
  applyViewport();
}

function centerZoom(factor) {
  const shell = currentMapShell();
  if (!shell) return;
  const rect = shell.getBoundingClientRect();
  zoomViewport(factor, rect.left + rect.width / 2, rect.top + rect.height / 2);
}

function clearNavSession() {
  state.navSession.key = "";
  state.navSession.startPose = null;
  state.navSession.goalPose = null;
}

function updateNavSession(board, nav) {
  if (pageMode !== "navigation") return;
  const goalPose = clonePose(navGoalPose(board, nav));
  const robotPose = clonePose(board?.robot_pose);
  const key = navSessionKey(goalPose);
  const navActive = Boolean(
    key ||
    (Array.isArray(board?.nav_path?.points) && board.nav_path.points.length > 1) ||
    String(nav?.nav_state || "") !== "idle"
  );

  if (!navActive) {
    clearNavSession();
    return;
  }

  if (key && key !== state.navSession.key) {
    state.navSession.key = key;
    state.navSession.startPose = robotPose;
    state.navSession.goalPose = goalPose;
    return;
  }

  if (!state.navSession.startPose && robotPose) {
    state.navSession.startPose = robotPose;
  }
  if (goalPose) {
    state.navSession.goalPose = goalPose;
  }
}

function renderNavPoseSummary() {
  const startSummary = $("nav-start-summary");
  const goalSummary = $("nav-goal-summary");
  if (!startSummary || !goalSummary) return;
  const startPose = state.navSession.startPose || state.navDraft?.startPose || currentCruiseStatus()?.start_pose;
  const goalPose = state.navSession.goalPose || navGoalPose(state.payload?.board || {}, state.payload?.board?.nav_status || {}) || draftLastPoint();
  startSummary.textContent = startPose
    ? `起点位姿：${formatPoseText(startPose)}`
    : "起点位姿：等待设置";
  goalSummary.textContent = goalPose
    ? `终点位姿：${formatPoseText(goalPose)}`
    : "终点位姿：等待目标点";
}

function renderCommandHistory() {
  const target = $("command-history");
  const latestTarget = $("latest-command-summary");
  if (!target) return;
  const history = Array.isArray(state.payload?.commands?.history) ? state.payload.commands.history : [];
  const filtered = history.filter((item) => {
    if (!item || !item.action) return false;
    if (pageMode === "mapping") return isMappingAction(item.action);
    if (pageMode === "navigation") return isNavigationAction(item.action);
    return true;
  }).slice(0, 6);

  if (!filtered.length) {
    if (latestTarget) {
      latestTarget.textContent = pageMode === "mapping" ? "等待建图指令" : "等待导航指令";
    }
    target.textContent = "还没有指令记录";
    return;
  }

  if (latestTarget) {
    const latest = filtered[0];
    const message = fmtText(latest.result?.message || latest.status, "");
    latestTarget.textContent = `${commandLabel(latest.action)}：${fmtText(latest.status)} ${message}`;
  }

  target.innerHTML = filtered.map((item) => {
    const message = fmtText(item.result?.message || item.status, "");
    return `<div class="command-item"><strong>${commandLabel(item.action)}</strong><span>${fmtText(item.status)}</span><small>${fmtAge(item.updated_at)} ${message}</small></div>`;
  }).join("");
}

function renderSavedMaps() {
  const select = $("saved-map-select");
  const summary = $("saved-map-summary");
  if (!select || !summary) return;

  const maps = savedMaps();
  const previousValue = select.value;
  select.innerHTML = "";

  if (!maps.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "暂无已保存地图";
    select.appendChild(option);
    select.value = "";
    summary.textContent = "当前没有可加载的已保存地图。";
    return;
  }

  const defaultOption = document.createElement("option");
  defaultOption.value = "";
  defaultOption.textContent = "选择已保存地图";
  select.appendChild(defaultOption);

  for (const item of maps) {
    const option = document.createElement("option");
    option.value = item.name || item.yaml || "";
    option.textContent = item.name || item.yaml || "未命名地图";
    select.appendChild(option);
  }

  if (maps.some((item) => (item.name || item.yaml) === previousValue)) {
    select.value = previousValue;
  } else {
    select.value = "";
  }

  const latest = maps[0];
  summary.textContent = `已保存地图 ${maps.length} 份，最新一份是 ${fmtText(latest?.name || latest?.yaml, "未命名")}。`;
}

function loadSelectedMap() {
  const select = $("saved-map-select");
  const navInput = $("nav-map-input");
  if (!select || !navInput) return;
  const value = select.value.trim();
  if (!value) return;
  navInput.value = value;
  syncNavDraft(value);
  render();
}

async function postTeleopState({ enabled, force = false } = {}) {
  if (pageMode !== "mapping" || !commandToken) return false;
  const boardId = (state.payload?.board?.board_id || "").trim();
  if (!boardId) {
    state.teleop.lastError = "当前没有可用板子";
    renderTeleopPanel();
    return false;
  }

  state.teleop.seq += 1;
  const payload = {
    board_id: boardId,
    controller_id: ensureTeleopControllerId(),
    page_mode: "mapping",
    enabled: Boolean(enabled),
    pressed_keys: Boolean(enabled) ? teleopPressedArray() : [],
    speed_level: state.teleop.speedLevel,
    seq: state.teleop.seq,
    force: Boolean(force),
  };

  try {
    const response = await fetch("/api/cloud/teleop", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Command-Token": commandToken,
      },
      body: JSON.stringify(payload),
    });
    const body = await response.json();
    if (!response.ok || !body.ok) {
      throw new Error(body.message || `HTTP ${response.status}`);
    }
    state.teleop.lastError = "";
    return true;
  } catch (error) {
    state.teleop.lastError = String(error?.message || error || "teleop failed");
    renderTeleopPanel();
    return false;
  }
}

function stopTeleopHeartbeat() {
  if (state.teleop.heartbeatTimer) {
    window.clearInterval(state.teleop.heartbeatTimer);
    state.teleop.heartbeatTimer = 0;
  }
}

function startTeleopHeartbeat() {
  stopTeleopHeartbeat();
  state.teleop.heartbeatTimer = window.setInterval(() => {
    if (!state.teleop.enabled) return;
    postTeleopState({ enabled: true }).catch(() => {});
  }, TELEOP_HEARTBEAT_MS);
}

async function enableTeleop() {
  return false;
}

async function disableTeleop(message = "") {
  state.teleop.enabled = false;
  state.teleop.pressedKeys.clear();
  stopTeleopHeartbeat();
  if (message) {
    state.teleop.lastError = message;
  }
  return false;
}

function updateTeleopKeyCaps() {
}

function renderTeleopPanel() {
}

async function sendCommand(action, params = {}) {
  if (state.commandBusy) return;
  if (!commandToken) {
    console.warn("command token missing");
    return;
  }

  state.commandBusy = true;
  try {
    const response = await fetch("/api/cloud/commands", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Command-Token": commandToken,
      },
      body: JSON.stringify({
        action,
        params,
        target_board_id: state.payload?.board?.board_id || "",
      }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    fetchState().catch(() => {});
  } catch (error) {
    console.error(`发送命令失败: ${error}`);
  } finally {
    state.commandBusy = false;
  }
}

function render() {
  const payload = state.payload || {};
  const board = payload.board || {};
  const ros = board.ros_status || {};
  const nav = board.nav_status || {};
  const ingest = payload.ingest || {};
  syncNavDraft();
  const cruise = currentCruiseStatus();
  const currentGoal = nav.current_goal || cruise.current_goal || {};
  const boardOnline = isFreshIso(board.last_seen_at);
  const stateFresh = isFreshIso(ingest.last_state_upload_at);
  updateNavSession(board, nav);

  $("board-online").textContent = boardOnline ? "在线" : "离线";
  $("last-seen").textContent = board.last_seen_at ? `最近上报 ${fmtAge(board.last_seen_at)}` : "还没有上报";
  $("board-meta").textContent = boardOnline
    ? "页面会自动刷新，显示当前小车状态。"
    : "等待小车上传数据。";

  let modeText = "等待中";
  let modeDetail = "未开始";
  if (ros.mapping_live) {
    modeText = "正在建图";
    modeDetail = ros.map_fresh ? "地图正在实时更新" : "地图暂时没有刷新";
  } else if (nav.nav_state && nav.nav_state !== "idle") {
    modeText = "正在导航";
    modeDetail = fmtText(nav.nav_state, "导航进行中");
  } else if (cruise.task_running) {
    modeText = "正在巡航";
    modeDetail = fmtText(cruise.message, "多目标执行中");
  }
  $("mode-text").textContent = modeText;
  $("mode-detail").textContent = modeDetail;

  if ($("nav-text")) {
    let navText = "未开始";
    if (nav.goal_active) navText = "正在前往目标";
    else if (cruise.task_running) navText = "正在巡航";
    else if (nav.nav_state && nav.nav_state !== "idle") navText = fmtText(nav.nav_state);
    else if (cruise.last_result) navText = fmtText(cruise.last_result);
    else if (nav.last_result) navText = fmtText(nav.last_result);
    $("nav-text").textContent = navText;

    if (Number.isFinite(Number(currentGoal.x)) && Number.isFinite(Number(currentGoal.y))) {
      $("goal-text").textContent = `目标点 x=${Number(currentGoal.x).toFixed(2)} y=${Number(currentGoal.y).toFixed(2)}`;
    } else {
      $("goal-text").textContent = "当前没有目标点";
    }
  }

  $("upload-age").textContent = fmtAge(ingest.last_state_upload_at);
  $("frame-age").textContent = ingest.last_frame_upload_at
    ? `图像 ${fmtAge(ingest.last_frame_upload_at)}`
    : "还没有图像";

  const mapMeta = board.map_meta || {};
  const savedMapMeta = selectedSavedMapMeta();
  const activeMapMeta = hasUsableMapMeta(mapMeta) ? mapMeta : savedMapMeta;
  const mapMetaTextTarget = $("map-meta");
  $("map-meta").textContent = (Number.isFinite(Number(mapMeta.width)) && Number.isFinite(Number(mapMeta.height)))
    ? `${mapMeta.width} x ${mapMeta.height}`
    : "等待地图";

  const mapVisible = setFrame(
    "map",
    "map-frame",
    "map-placeholder",
    state.mapFrameStamp,
    defaultMapPlaceholder(),
    Boolean(ros.map_fresh || nav.goal_active || (Array.isArray(board.nav_path?.points) && board.nav_path.points.length > 1))
  );
  if (mapMetaTextTarget && hasUsableMapMeta(activeMapMeta)) {
    mapMetaTextTarget.textContent = `${activeMapMeta.width} x ${activeMapMeta.height}`;
  }

  let scanVisible = false;
  if (pageMode === "mapping") {
    scanVisible = setFrame(
      "scan",
      "scan-frame",
      "scan-placeholder",
      state.scanFrameStamp,
      defaultScanPlaceholder(),
      Boolean(ros.scan_fresh)
    );
  }
  const previewVisible = pageMode === "navigation" ? showSavedMapPreview() : false;

  const commonEntries = [
    ["地图图像", (mapVisible || previewVisible) ? "已显示" : "未收到"],
    ["小车位置", Number.isFinite(Number(board.robot_pose?.x)) ? "已显示" : "暂无"],
    ["最新反馈", nav.goal_feedback || nav.last_result || nav.last_error || "暂无"],
  ];

  if (pageMode === "mapping") {
    renderList("car-summary", [
      ...commonEntries,
      ["雷达图像", scanVisible ? "已收到" : "未收到"],
      ["建图状态", ros.mapping_live ? "进行中" : "未开始"],
      ["地图刷新", stateFresh && ros.map_fresh ? "正常" : "暂无"],
    ]);
  } else if (pageMode === "navigation") {
    renderList("car-summary", [
      ...commonEntries,
      ["导航状态", nav.nav_state || "未开始"],
      ["路径规划", Array.isArray(board.nav_path?.points) && board.nav_path.points.length > 1 ? "已生成" : "暂无"],
      ["已保存地图", savedMaps().length ? `${savedMaps().length} 份` : "暂无"],
    ]);
  }

  const routeSummary = $("nav-route-summary");
  if (routeSummary) {
    const points = Array.isArray(board.nav_path?.points) ? board.nav_path.points : [];
    if (points.length > 1) {
      routeSummary.textContent = `当前已经规划出路径，共 ${points.length} 个路径点。`;
    } else if (Number.isFinite(Number(currentGoal.x))) {
      routeSummary.textContent = "已收到目标点，等待路线生成。";
    } else if (previewVisible || mapVisible) {
      routeSummary.textContent = "地图已加载，设置目标点后这里会显示路径。";
    } else {
      routeSummary.textContent = "当前没有路径和目标点。";
    }
  }

  renderNavPoseSummary();
  renderSavedMaps();
  renderNavDraftList();
  renderCruiseSummary();
  updateNavClickButtons();
  renderCommandHistory();
  renderOverlay();
  applyViewport();
}

async function fetchState() {
  const response = await fetch("/api/cloud/state", { cache: "no-store" });
  const payload = await response.json();
  state.payload = payload;
  render();
}

function bindMapViewport() {
  const shell = currentMapShell();
  const zoomIn = $("map-zoom-in");
  const zoomOut = $("map-zoom-out");
  const zoomReset = $("map-zoom-reset");
  if (!shell) return;

  shell.addEventListener("wheel", (event) => {
    if (!currentMapImageVisible()) return;
    event.preventDefault();
    zoomViewport(event.deltaY < 0 ? 1.12 : (1 / 1.12), event.clientX, event.clientY);
  }, { passive: false });

  shell.addEventListener("pointerdown", (event) => {
    state.viewport.pointerMoved = false;
    if (!currentMapImageVisible() || state.viewport.scale <= 1.001) return;
    state.viewport.dragging = true;
    state.viewport.pointerId = event.pointerId;
    state.viewport.dragStartX = event.clientX;
    state.viewport.dragStartY = event.clientY;
    state.viewport.dragOriginX = state.viewport.offsetX;
    state.viewport.dragOriginY = state.viewport.offsetY;
    shell.setPointerCapture(event.pointerId);
    applyViewport();
  });

  shell.addEventListener("pointermove", (event) => {
    if (!state.viewport.dragging || event.pointerId !== state.viewport.pointerId) return;
    if (Math.abs(event.clientX - state.viewport.dragStartX) > 3 || Math.abs(event.clientY - state.viewport.dragStartY) > 3) {
      state.viewport.pointerMoved = true;
    }
    state.viewport.offsetX = state.viewport.dragOriginX + (event.clientX - state.viewport.dragStartX);
    state.viewport.offsetY = state.viewport.dragOriginY + (event.clientY - state.viewport.dragStartY);
    applyViewport();
  });

  const stopDragging = (event) => {
    if (state.viewport.pointerId !== null && event.pointerId !== undefined && event.pointerId !== state.viewport.pointerId) {
      return;
    }
    state.viewport.dragging = false;
    state.viewport.pointerId = null;
    applyViewport();
  };

  shell.addEventListener("pointerup", stopDragging);
  shell.addEventListener("pointercancel", stopDragging);
  shell.addEventListener("pointerleave", stopDragging);
  shell.addEventListener("click", (event) => handleNavMapClick(event));

  if (zoomIn) {
    zoomIn.addEventListener("click", () => centerZoom(1.2));
  }
  if (zoomOut) {
    zoomOut.addEventListener("click", () => centerZoom(1 / 1.2));
  }
  if (zoomReset) {
    zoomReset.addEventListener("click", () => resetViewport());
  }
}

function bindCommandButtons() {
  const startMapping = $("cmd-start-mapping");
  if (startMapping) {
    startMapping.addEventListener("click", () => sendCommand("start_mapping"));
  }

  const stopMapping = $("cmd-stop-mapping");
  if (stopMapping) {
    stopMapping.addEventListener("click", () => sendCommand("stop_mapping"));
  }

  const saveMap = $("cmd-save-map");
  if (saveMap) {
    saveMap.addEventListener("click", () => {
      const name = ($("map-save-input")?.value || "").trim();
      const params = name ? { name } : {};
      sendCommand("save_map", params);
    });
  }

  const loadSavedMapButton = $("btn-load-saved-map");
  if (loadSavedMapButton) {
    loadSavedMapButton.addEventListener("click", () => loadSelectedMap());
  }

  const savedMapSelect = $("saved-map-select");
  if (savedMapSelect) {
    savedMapSelect.addEventListener("change", () => loadSelectedMap());
  }

  const startNavigation = $("cmd-start-navigation");
  if (startNavigation) {
    startNavigation.addEventListener("click", () => {
      const navInput = $("nav-map-input");
      const select = $("saved-map-select");
      const mapValue = (navInput?.value || "").trim() || (select?.value || "").trim();
      if (!mapValue) return;
      if (navInput) navInput.value = mapValue;
      sendCommand("start_navigation", { map: mapValue });
    });
  }

  const stopNavigation = $("cmd-stop-navigation");
  if (stopNavigation) {
    stopNavigation.addEventListener("click", () => {
      clearNavSession();
      sendCommand("stop_navigation");
    });
  }

  $("nav-mark-start")?.addEventListener("click", () => setNavClickMode("start"));
  $("nav-mark-waypoint")?.addEventListener("click", () => setNavClickMode("waypoint"));
  $("nav-mark-stop")?.addEventListener("click", () => setNavClickMode("none"));

  $("nav-draft-clear")?.addEventListener("click", () => {
    state.navDraft.startPose = null;
    state.navDraft.points = [];
    persistNavDraft();
    render();
  });

  $("cmd-send-start-pose")?.addEventListener("click", () => {
    if (!hasFinitePose(state.navDraft?.startPose)) return;
    sendCommand("set_initial_pose", { pose: state.navDraft.startPose });
  });

  $("cmd-start-single-goal")?.addEventListener("click", () => {
    const point = draftLastPoint();
    if (!hasFinitePose(point)) return;
    sendCommand("start_cruise", {
      map: preferredSavedMapKey(),
      start_pose: state.navDraft?.startPose || {},
      loop_count: 1,
      points: [point],
    });
  });

  $("cmd-start-cruise")?.addEventListener("click", () => {
    const points = Array.isArray(state.navDraft?.points) ? state.navDraft.points : [];
    if (!points.length) return;
    sendCommand("start_cruise", {
      map: preferredSavedMapKey(),
      start_pose: state.navDraft?.startPose || {},
      loop_count: 1,
      points,
    });
  });

  $("cmd-stop-cruise")?.addEventListener("click", () => {
    sendCommand("stop_cruise");
  });

  $("nav-draft-list")?.addEventListener("input", (event) => {
    const target = event.target;
    if (!(target instanceof HTMLInputElement)) return;
    const index = Number(target.dataset.draftYaw || -1);
    if (!Number.isInteger(index) || index < 0 || index >= state.navDraft.points.length) return;
    const yawDeg = Number(target.value || 0);
    state.navDraft.points[index].yaw = (Number.isFinite(yawDeg) ? yawDeg : 0) * Math.PI / 180;
    persistNavDraft();
    renderNavDraftList();
    renderOverlay();
  });

  $("nav-draft-list")?.addEventListener("click", (event) => {
    if (!(event.target instanceof Element)) return;
    const button = event.target.closest("button");
    if (!button) return;
    const removeIndex = Number(button.dataset.draftRemove || -1);
    const useIndex = Number(button.dataset.draftUse || -1);
    if (Number.isInteger(removeIndex) && removeIndex >= 0 && removeIndex < state.navDraft.points.length) {
      state.navDraft.points.splice(removeIndex, 1);
      persistNavDraft();
      render();
      return;
    }
    if (Number.isInteger(useIndex) && useIndex >= 0 && useIndex < state.navDraft.points.length) {
      const point = state.navDraft.points[useIndex];
      sendCommand("start_cruise", {
        map: preferredSavedMapKey(),
        start_pose: state.navDraft?.startPose || {},
        loop_count: 1,
        points: [point],
      });
    }
  });
}

function bindTeleopControls() {
  // Cloud keyboard driving is disabled. Mapping/navigation commands go through the command queue only.
}

function startPolling() {
  fetchState().catch((error) => {
    const meta = $("board-meta");
    if (meta) meta.textContent = `状态获取失败: ${error}`;
  });
  window.setInterval(() => {
    fetchState().catch(() => {});
  }, 1000);
}

window.addEventListener("resize", () => {
  renderOverlay();
  applyViewport();
});
window.addEventListener("load", () => {
  if (pageMode === "menu") return;
  $("btn-refresh").addEventListener("click", () => fetchState().catch(() => {}));
  $("map-frame").addEventListener("load", () => renderOverlay());
  bindMapViewport();
  bindCommandButtons();
  bindTeleopControls();
  if (pageMode === "navigation") {
    syncNavDraft();
    renderNavDraftList();
    renderNavPoseSummary();
    renderCruiseSummary();
    updateNavClickButtons();
  }
  startPolling();
});
