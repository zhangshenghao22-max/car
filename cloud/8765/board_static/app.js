(() => {
  const state = {
    backendStatus: null,
    rosStatus: null,
    activeTab: "home",
    logsAutoRefresh: true,
    lastUploadResultKey: "",
    selectedMapName: "",
    cachedMaps: [],
    activeMotionAction: "stop",
    servoDraft: Array.from({ length: 8 }, () => 1500),
    servoEditLocks: {},
    lastLogsText: "",
    lastStatusError: "",
    lastRosStatusError: "",
    backendStatusPending: false,
    rosStatusPending: false,
    mapsPending: false,
    logsPending: false,
    cameraListPending: false,
  };

  const SERVO_MIN = 500;
  const SERVO_MAX = 2500;
  const SERVO_COUNT = 8;
  const TAB_NAMES = ["home", "control", "vision", "ros", "avoidance", "maps", "nav", "logs"];

  function $(id) {
    return document.getElementById(id);
  }

  function setText(id, value) {
    const element = $(id);
    if (element) {
      element.textContent = value == null ? "" : String(value);
    }
  }

  function setValue(id, value) {
    const element = $(id);
    if (element && String(element.value) !== String(value ?? "")) {
      element.value = value ?? "";
    }
  }

  function setChecked(id, value) {
    const element = $(id);
    if (element) {
      element.checked = !!value;
    }
  }

  function escapeHtml(value) {
    return String(value ?? "").replace(/[&<>"']/g, (char) => ({
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      "\"": "&quot;",
      "'": "&#39;",
    }[char]));
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, Number(value)));
  }

  function formatMaybe(value, fallback = "--") {
    if (value == null) {
      return fallback;
    }
    const text = String(value).trim();
    return text ? text : fallback;
  }

  function formatBool(value, yes = "在线", no = "离线") {
    return value ? yes : no;
  }

  function formatAgeMs(value) {
    if (value == null || value === "") {
      return "--";
    }
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) {
      return "--";
    }
    if (numeric < 1000) {
      return `${Math.round(numeric)} ms`;
    }
    const seconds = numeric / 1000;
    return `${seconds < 10 ? seconds.toFixed(1) : Math.round(seconds)} s`;
  }

  function formatDistanceMm(value) {
    const numeric = Number(value);
    if (!Number.isFinite(numeric) || numeric <= 0) {
      return "--";
    }
    return `${Math.round(numeric)} mm`;
  }

  function formatTimeText(value, fallback = "--") {
    const text = String(value ?? "").trim();
    return text || fallback;
  }

  function showToast(message, isError = false) {
    const toast = $("toast");
    if (!toast) {
      return;
    }
    toast.textContent = message || (isError ? "操作失败" : "操作完成");
    toast.className = isError ? "show error" : "show";
    window.clearTimeout(showToast._timer);
    showToast._timer = window.setTimeout(() => {
      toast.className = "";
    }, isError ? 5200 : 3200);
  }

  async function apiJson(url, options = {}) {
    const requestOptions = { method: "GET", ...options };
    if (requestOptions.body && !(requestOptions.body instanceof FormData)) {
      requestOptions.headers = {
        "Content-Type": "application/json",
        ...(requestOptions.headers || {}),
      };
      requestOptions.body = JSON.stringify(requestOptions.body);
    }

    const response = await fetch(url, requestOptions);
    const payload = await response.json().catch(() => ({}));
    if (!response.ok || payload.ok === false) {
      const message = payload && payload.message ? payload.message : `请求失败: ${response.status}`;
      const error = new Error(message);
      error.payload = payload;
      throw error;
    }
    return payload;
  }

  function fillSelect(id, values, selectedValue, labelBuilder) {
    const select = typeof id === "string" ? $(id) : id;
    if (!select) {
      return;
    }
    const items = Array.isArray(values) ? [...values] : [];
    const current = String(selectedValue ?? "");
    if (current && !items.some((item) => String(item) === current)) {
      items.unshift(current);
    }
    select.innerHTML = items.map((item) => {
      const value = String(item);
      const label = labelBuilder ? labelBuilder(item) : value;
      return `<option value="${escapeHtml(value)}">${escapeHtml(label)}</option>`;
    }).join("");
    if (current) {
      select.value = current;
    }
    if (select.options.length && !select.value) {
      select.selectedIndex = 0;
    }
  }

  function metricHtml(item) {
    const stateClass = item.state ? ` ${item.state}` : "";
    const hint = item.hint ? `<small>${escapeHtml(item.hint)}</small>` : "";
    return `
      <div class="metric${stateClass}">
        <span>${escapeHtml(item.label)}</span>
        <strong>${escapeHtml(item.value)}</strong>
        ${hint}
      </div>
    `;
  }

  function renderMetricCards(containerId, items) {
    const container = $(containerId);
    if (!container) {
      return;
    }
    container.innerHTML = (items || []).map(metricHtml).join("");
  }

  function renderDeviceCards(containerId, items) {
    const container = $(containerId);
    if (!container) {
      return;
    }
    container.innerHTML = (items || []).map((item) => `
      <article class="device-item">
        <header>
          <h4>${escapeHtml(item.title)}</h4>
          <span class="device-chip ${escapeHtml(item.state || "idle")}">${escapeHtml(item.status)}</span>
        </header>
        <p>${escapeHtml(item.detail || "")}</p>
        ${item.meta ? `<div class="device-meta">${escapeHtml(item.meta)}</div>` : ""}
      </article>
    `).join("");
  }

  function renderDetectionList(detections) {
    const container = $("vision-detection-list");
    if (!container) {
      return;
    }
    if (!Array.isArray(detections) || !detections.length) {
      container.innerHTML = '<div class="detection-empty">当前没有新的识别目标。</div>';
      return;
    }
    container.innerHTML = detections.map((item, index) => {
      const sourceLabel = item.source === "meter" ? "仪表模型" : "YOLO";
      const reading = item.reading ? ` · 读数 ${item.reading}` : "";
      const position = `中心 (${item.center_x ?? "--"}, ${item.center_y ?? "--"})`;
      return `
        <article class="detection-item${index === 0 ? " active" : ""}">
          <div class="detection-title">
            <strong>${escapeHtml(item.label || "目标")}</strong>
            <span>${escapeHtml(sourceLabel)} · ${escapeHtml(((Number(item.confidence) || 0) * 100).toFixed(1))}%</span>
          </div>
          <div class="detection-meta">${escapeHtml(position)}${escapeHtml(reading)}</div>
          <div class="detection-rank">框尺寸 ${escapeHtml(item.width ?? "--")} × ${escapeHtml(item.height ?? "--")}</div>
        </article>
      `;
    }).join("");
  }

  function buildUploadResultKey(result) {
    if (!result) {
      return "";
    }
    return [
      result.result_url || "",
      result.capture_mode || "",
      result.captured_at || "",
      result.message || "",
    ].join("|");
  }

  function buildVisionSnapshotStatus(vision) {
    if (!vision.running) {
      return "摄像头未运行，自动抓拍尚未开始。";
    }
    if (vision.snapshot_busy) {
      return "自动抓拍识别进行中，请稍候。";
    }
    if (vision.last_snapshot_error) {
      return `自动抓拍异常：${vision.last_snapshot_error}`;
    }
    if (vision.last_snapshot_completed_text) {
      return `自动抓拍最近完成于 ${vision.last_snapshot_completed_text}，周期 ${vision.snapshot_interval_s} 秒。`;
    }
    return `摄像头运行中，将按 ${vision.snapshot_interval_s} 秒周期自动抓拍识别。`;
  }

  function rosRvizModeLabel(mode) {
    if (mode === "compatibility") {
      return "兼容软件渲染";
    }
    if (mode === "unknown") {
      return "未知模式";
    }
    return formatMaybe(mode, "未启动");
  }

  function controlRuntimeState(control) {
    if (control.connected) {
      return {
        state: "online",
        label: "已连接",
        detail: `目标 ${formatMaybe(control.target)} · ${control.mode === "serial" ? "串口" : "蓝牙"}`,
      };
    }
    return {
      state: "offline",
      label: "未连接",
      detail: "等待连接主控链路",
    };
  }

  function visionRuntimeState(vision) {
    if (vision.running) {
      return {
        state: "online",
        label: "运行中",
        detail: `相机 ${vision.camera_index} · ${vision.frame_size} · ${vision.fps} FPS`,
      };
    }
    if (vision.model_error) {
      return {
        state: "warn",
        label: "模型异常",
        detail: vision.model_error,
      };
    }
    if (vision.yolo_model_loaded || vision.meter_model_loaded) {
      return {
        state: "idle",
        label: "已就绪",
        detail: `模型可用：${(vision.active_detectors || []).join(" / ") || "无"}`,
      };
    }
    return {
      state: "idle",
      label: "待机",
      detail: "摄像头与识别模型尚未启动",
    };
  }

  function avoidanceRuntimeState(avoidance) {
    if (avoidance.avoidance_enabled) {
      return {
        state: "online",
        label: "已启用",
        detail: `动作 ${formatMaybe(avoidance.current_action, "待机")} · 阈值 ${formatDistanceMm(avoidance.threshold_mm)}`,
      };
    }
    if (avoidance.running) {
      return {
        state: "idle",
        label: "雷达运行",
        detail: "避障雷达已启动，等待开启避障控制",
      };
    }
    if (avoidance.last_error) {
      return {
        state: "warn",
        label: "异常",
        detail: avoidance.last_error,
      };
    }
    return {
      state: "idle",
      label: "待机",
      detail: "避障未启动",
    };
  }

  function buildRosRuntimeState(ros) {
    if (ros.mapping && ros.scan_fresh && ros.map_fresh) {
      if (ros.rviz_live && ros.rviz_map_render_ok) {
        return { state: "online", label: "建图正常", detail: "网页与 RViz 都在持续更新地图。" };
      }
      if (ros.rviz_live && !ros.rviz_map_render_ok) {
        return { state: "warn", label: "建图正常", detail: "SLAM 正常，但 RViz 地图渲染异常。" };
      }
      return { state: "online", label: "建图正常", detail: "网页地图持续更新，RViz 当前未显示。" };
    }
    if (ros.scan_fresh && !ros.map_fresh) {
      return { state: "warn", label: "等待出图", detail: "雷达有数据，但 slam_toolbox 未持续出图。" };
    }
    if (!ros.driver_live && !ros.slam_live && (ros.scan_frames_received || ros.map_frames_received || ros.last_ready_error)) {
      return { state: "offline", label: "进程已退出", detail: ros.last_ready_error || "建图进程当前未运行。" };
    }
    if (ros.last_ready_error) {
      return { state: "warn", label: "等待就绪", detail: ros.last_ready_error };
    }
    if (ros.driver_live || ros.slam_live || ros.laser_tf_live || ros.odom_tf_live) {
      return { state: "idle", label: "启动中", detail: "建图链路正在拉起，等待 TF 与新帧。" };
    }
    return { state: "idle", label: "待机", detail: "等待启动 ROS2 建图。" };
  }

  function buildRosMappingToastMessage(response, fallbackMessage) {
    const segments = [response.message || fallbackMessage || "操作完成"];
    if (response.warning) {
      segments.push(response.warning);
    } else if (response.rviz_started && !response.rviz_reused) {
      segments.push(`RViz 已按${rosRvizModeLabel(response.rviz_render_mode)}打开`);
    } else if (response.rviz_reused) {
      segments.push("RViz 已在运行，直接复用");
    }
    return segments.filter(Boolean).join("；");
  }

  function resolveRosStatus(data) {
    if (data && data.driver_live !== undefined) {
      return data;
    }
    if (state.rosStatus) {
      return state.rosStatus;
    }
    if (data && data.ros) {
      return data.ros;
    }
    if (state.backendStatus && state.backendStatus.ros) {
      return state.backendStatus.ros;
    }
    return {};
  }

  function setButtonBusy(button, busy) {
    if (!button) {
      return;
    }
    button.disabled = !!busy;
    button.classList.toggle("disabled", !!busy);
  }

  function renderUploadResult(result) {
    const empty = $("vision-upload-empty");
    const image = $("vision-upload-image");
    const open = $("vision-upload-open");
    if (!result || !result.result_url) {
      if (empty) {
        empty.style.display = "grid";
      }
      if (image) {
        image.classList.remove("active");
        image.removeAttribute("src");
      }
      if (open) {
        open.href = "#";
        open.classList.add("disabled");
      }
      setText("vision-upload-message", "支持上传单张图片；摄像头运行时会每 5 秒自动抓拍一张并识别。");
      setText("vision-upload-summary", "当前未生成新的识别结果。");
      state.lastUploadResultKey = "";
      return;
    }

    const resultKey = buildUploadResultKey(result);
    if (empty) {
      empty.style.display = "none";
    }
    if (image) {
      if (state.lastUploadResultKey !== resultKey) {
        image.src = `${result.result_url}${result.result_url.includes("?") ? "&" : "?"}t=${Date.now()}`;
      }
      image.classList.add("active");
    }
    if (open) {
      open.href = result.result_url;
      open.classList.remove("disabled");
    }

    const captureLabel = result.capture_mode === "camera_snapshot" ? "自动抓拍结果" : "上传图片结果";
    const processedFrames = Number(result.processed_frames) || 0;
    const totalFrames = Number(result.total_frames) || processedFrames || 0;
    const parts = [captureLabel, "识别完成"];
    if (processedFrames > 0) {
      parts.push(`已处理 ${processedFrames}/${totalFrames} 帧`);
    }
    if (result.fps) {
      parts.push(`${result.fps} FPS`);
    }
    if (result.captured_at_text) {
      parts.push(result.captured_at_text);
    }
    setText("vision-upload-message", result.message || "识别完成");
    setText("vision-upload-summary", parts.join(" · "));
    state.lastUploadResultKey = resultKey;
  }

  function ensureServoRows() {
    const container = $("servo-list");
    if (!container || container.children.length) {
      return;
    }
    container.innerHTML = Array.from({ length: SERVO_COUNT }, (_, index) => `
      <div class="servo-row" data-servo-index="${index}">
        <strong>关节 ${index}</strong>
        <input
          type="range"
          min="${SERVO_MIN}"
          max="${SERVO_MAX}"
          step="1"
          value="${state.servoDraft[index]}"
          data-servo-range="${index}"
        >
        <div class="servo-input-group">
          <input
            type="number"
            min="${SERVO_MIN}"
            max="${SERVO_MAX}"
            step="1"
            value="${state.servoDraft[index]}"
            data-servo-number="${index}"
          >
          <button type="button" class="ghost" data-servo-send="${index}">发送</button>
        </div>
      </div>
    `).join("");
  }

  function renderSidebar(data) {
    const controlState = controlRuntimeState(data.control);
    const visionState = visionRuntimeState(data.vision);
    const rosState = buildRosRuntimeState(data.ros);
    const avoidanceState = avoidanceRuntimeState(data.avoidance);

    setText("status-control", controlState.label);
    setText("status-vision", visionState.label);
    setText("status-ros", rosState.label);
    setText("status-avoidance", avoidanceState.label);

    setText("sidebar-control-port", formatMaybe(data.control.serial_port));
    setText("sidebar-lidar-port", formatMaybe(data.ros.last_lidar_port || data.avoidance.port));
    setText("sidebar-latest-map", formatMaybe(data.ros.latest_saved_map, "暂无"));

    const latestResult = data.vision.latest_result;
    const latestResultText = latestResult
      ? `${latestResult.capture_mode === "camera_snapshot" ? "自动抓拍" : "上传识别"} · ${formatTimeText(latestResult.captured_at_text, "刚刚")}`
      : "暂无";
    setText("sidebar-latest-result", latestResultText);

    const notes = [
      rosState.detail,
      data.control.formal?.status_text,
      data.vision.model_error,
      data.avoidance.last_error,
      data.rt_bridge.bridge_ready ? "micro-ROS 桥接在线" : "",
    ].filter(Boolean);
    setText("sidebar-note", notes[0] || "等待后端状态同步。");
  }

  function renderHome(data) {
    const controlState = controlRuntimeState(data.control);
    const visionState = visionRuntimeState(data.vision);
    const rosState = buildRosRuntimeState(data.ros);
    const avoidanceState = avoidanceRuntimeState(data.avoidance);
    const latestResult = data.vision.latest_result;
    const deviceItems = [
      {
        title: "控制主链路",
        status: controlState.label,
        state: controlState.state,
        detail: controlState.detail,
        meta: `串口 ${formatMaybe(data.control.serial_port)} · 波特率 ${formatMaybe(data.control.serial_baudrate)}`,
      },
      {
        title: "正式协议",
        status: data.control.formal?.detected ? "已识别" : "未识别",
        state: data.control.formal?.detected ? "online" : "idle",
        detail: formatMaybe(data.control.formal?.status_text, "等待握手与状态上报"),
        meta: `模式 ${formatMaybe(data.control.formal?.mode)} · 急停 ${formatBool(data.control.formal?.estop, "触发", "正常")}`,
      },
      {
        title: "视觉链路",
        status: visionState.label,
        state: visionState.state,
        detail: visionState.detail,
        meta: `检测器 ${(data.vision.active_detectors || []).join(" / ") || "未加载"}`,
      },
      {
        title: "ROS 建图",
        status: rosState.label,
        state: rosState.state,
        detail: rosState.detail,
        meta: `雷达口 ${formatMaybe(data.ros.last_lidar_port)} · 地图 ${data.ros.saved_maps_count || 0} 张`,
      },
      {
        title: "避障",
        status: avoidanceState.label,
        state: avoidanceState.state,
        detail: avoidanceState.detail,
        meta: `当前动作 ${formatMaybe(data.avoidance.current_action, "待机")}`,
      },
      {
        title: "micro-ROS 桥接",
        status: data.rt_bridge.agent_disabled
          ? "已停用"
          : (data.rt_bridge.bridge_ready ? "在线" : (data.rt_bridge.agent_active ? "代理待连" : "未就绪")),
        state: data.rt_bridge.agent_disabled
          ? "warn"
          : (data.rt_bridge.bridge_ready ? "online" : (data.rt_bridge.agent_active ? "warn" : "offline")),
        detail: data.rt_bridge.agent_disabled
          ? "当前已切换到 F103 串口 odom 正式链路，RT micro-ROS odom 已停用。"
          : (data.rt_bridge.bridge_ready
              ? `目标 IP ${formatMaybe(data.rt_bridge.agent_target_ip)} · /odom ${formatBool(data.rt_bridge.odom_ready, "已发布", "未发布")}`
              : "当前板端 RT-Thread 与 ROS2 桥接未完全就绪。"),
        meta: data.rt_bridge.agent_disabled
          ? "正式 odom 来源 f103_serial"
          : `服务 ${formatBool(data.rt_bridge.agent_active, "active", "inactive")} · /cmd_vel ${formatBool(data.rt_bridge.cmd_vel_ready, "ready", "missing")}`,
      },
    ];

    setText("home-control-state", controlState.label);
    setText("home-control-detail", controlState.detail);
    setText("home-vision-state", visionState.label);
    setText("home-vision-detail", visionState.detail);
    setText("home-ros-state", rosState.label);
    setText("home-ros-detail", rosState.detail);
    setText("home-avoidance-state", avoidanceState.label);
    setText("home-avoidance-detail", avoidanceState.detail);
    setText("home-device-tag", "真实状态");

    renderDeviceCards("home-device-matrix", deviceItems);

    setText("home-device-summary", `当前共发现 ${deviceItems.length} 个核心子系统状态，全部来自后端实时接口。`);

    renderMetricCards("home-runtime-metrics", [
      { label: "控制串口", value: formatMaybe(data.control.serial_port), hint: data.control.connected ? "当前主控链路" : "未连接", state: data.control.connected ? "online" : "offline" },
      { label: "雷达串口", value: formatMaybe(data.ros.last_lidar_port || data.avoidance.port), hint: "建图 / 避障共用候选口", state: data.ros.last_lidar_port || data.avoidance.port ? "idle" : "offline" },
      { label: "摄像头", value: data.vision.running ? `#${data.vision.camera_index}` : "未开", hint: data.vision.running ? `${data.vision.frame_size} · ${data.vision.fps} FPS` : "等待启动", state: data.vision.running ? "online" : "idle" },
      { label: "地图数量", value: String(data.ros.saved_maps_count || 0), hint: formatMaybe(data.ros.latest_saved_map, "暂无已保存地图"), state: (data.ros.saved_maps_count || 0) > 0 ? "online" : "idle" },
    ]);

    setText("home-runtime-summary", [
      rosState.detail,
      data.control.formal?.status_text,
      data.rt_bridge.note,
    ].filter(Boolean).join(" | "));

    renderMetricCards("cabinet-runtime-metrics", [
      { label: "温湿度", value: "未接入", hint: "暂无真实数据", state: "offline" },
      { label: "烟雾浓度", value: "未接入", hint: "暂无真实数据", state: "offline" },
      { label: "仪表盘读数", value: "未接入", hint: "需后续接入配电柜采集", state: "offline" },
      { label: "指示灯颜色", value: "未接入", hint: "需后续接入指示灯识别", state: "offline" },
    ]);

    renderMetricCards("home-recent-metrics", [
      {
        label: "最近地图",
        value: formatMaybe(data.ros.latest_saved_map, "暂无"),
        hint: formatTimeText(data.ros.latest_saved_at, "尚未保存"),
        state: data.ros.latest_saved_map ? "online" : "idle",
      },
      {
        label: "最近识别",
        value: latestResult ? (latestResult.capture_mode === "camera_snapshot" ? "自动抓拍" : "上传识别") : "暂无",
        hint: latestResult ? formatTimeText(latestResult.captured_at_text, "刚刚") : "等待识别结果",
        state: latestResult ? "online" : "idle",
      },
      {
        label: "状态上报",
        value: formatBool(data.control.formal?.reporting, "开启", "关闭"),
        hint: formatMaybe(data.control.formal?.last_state_at ? `最近状态 ${formatAgeMs(data.control.formal?.state_age_ms)}` : "尚未收到正式状态帧"),
        state: data.control.formal?.reporting ? "online" : "idle",
      },
      {
        label: "日志条数",
        value: String((data.logs || []).length),
        hint: "首页只显示尾部摘要",
        state: (data.logs || []).length ? "online" : "idle",
      },
    ]);

    setText("home-recent-summary", latestResult
      ? `最近结果来自${latestResult.capture_mode === "camera_snapshot" ? "自动抓拍" : "上传图片"}，可在视觉页查看详情。`
      : "当前还没有新的识别结果。");
  }

  function renderControlPage(data) {
    ensureServoRows();

    const controlState = controlRuntimeState(data.control);
    const control = data.control;
    const formal = control.formal || {};
    fillSelect("control-port", control.ports || [], control.serial_port || "");
    setValue("control-baudrate", control.serial_baudrate || 115200);
    setValue("control-report-enabled", formal.reporting ? "true" : "false");

    setText("control-runtime-tag", controlState.label);
    setText("control-connection-summary", control.connected
      ? `已连接 ${formatMaybe(control.target)}，可直接发送小车和机械臂命令。`
      : "当前未连接控制链路，请先选择串口并连接。");
    setText("control-formal-summary", formal.status_text || "等待正式协议状态。");
    setText("control-motion-summary", state.activeMotionAction && state.activeMotionAction !== "stop"
      ? `当前按住动作：${state.activeMotionAction}，松开后会自动补发停止命令。`
      : "按住方向按钮发送运动命令，松开后自动补发停止命令。");

    renderMetricCards("control-runtime-metrics", [
      { label: "链路目标", value: formatMaybe(control.target), hint: `连接方式 ${formatMaybe(control.mode)}`, state: control.connected ? "online" : "offline" },
      { label: "正式模式", value: formatMaybe(formal.mode, "未知"), hint: formatMaybe(formal.status_text, "等待状态"), state: formal.detected ? "online" : "idle" },
      { label: "状态上报", value: formatBool(formal.reporting, "开启", "关闭"), hint: formal.last_state_at ? `最新状态 ${formatAgeMs(formal.state_age_ms)}` : "尚未收到状态帧", state: formal.reporting ? "online" : "idle" },
      { label: "急停状态", value: formatBool(formal.estop, "已触发", "正常"), hint: formal.last_error_text || formatMaybe(formal.error_name, "无异常"), state: formal.estop ? "warn" : "online" },
    ]);

    const wheelMetrics = Object.entries(formal.wheel_speeds_mm_s || {}).map(([key, value]) => ({
      label: `${key} 实际`,
      value: `${value} mm/s`,
      hint: `目标 ${(formal.target_speeds_mm_s || {})[key] ?? 0} mm/s`,
      state: Number(value) !== 0 ? "online" : "idle",
    }));
    renderMetricCards("control-wheel-metrics", wheelMetrics.length ? wheelMetrics : [
      { label: "轮速状态", value: "暂无", hint: "等待正式状态连续上报", state: "idle" },
    ]);

    const bleReason = control.ble_supported
      ? "当前第七步保留 BLE 入口，但按要求显示禁用，不执行扫描或连接。"
      : `当前环境未就绪：${formatMaybe(control.ble_import_error, "缺少 BLE 运行条件")}`;
    setText("control-ble-summary", bleReason);
    renderMetricCards("control-ble-metrics", [
      { label: "BLE 支持", value: formatBool(control.ble_supported, "可用", "不可用"), hint: "本步固定显示但禁用", state: control.ble_supported ? "idle" : "offline" },
      { label: "当前连接", value: formatBool(control.ble_connected, "已连接", "未连接"), hint: formatMaybe(control.ble_target, "备用链路未启用"), state: control.ble_connected ? "online" : "idle" },
    ]);

    const modeButtons = document.querySelectorAll("[data-mode]");
    modeButtons.forEach((button) => {
      button.classList.toggle("active", String(button.dataset.mode || "").toUpperCase() === String(formal.mode || "").toUpperCase());
    });

    const servoValues = Array.isArray(control.servo_values) ? control.servo_values : [];
    for (let index = 0; index < SERVO_COUNT; index += 1) {
      const serverValue = Number(servoValues[index]);
      if (Number.isFinite(serverValue) && !state.servoEditLocks[index]) {
        state.servoDraft[index] = serverValue;
      }
      const range = document.querySelector(`[data-servo-range="${index}"]`);
      const number = document.querySelector(`[data-servo-number="${index}"]`);
      if (range && !state.servoEditLocks[index]) {
        range.value = String(state.servoDraft[index]);
      }
      if (number && !state.servoEditLocks[index]) {
        number.value = String(state.servoDraft[index]);
      }
    }
  }

  function renderVisionPage(data) {
    const vision = data.vision;
    const stateInfo = visionRuntimeState(vision);

    setText("vision-runtime-tag", stateInfo.label);
    setText("vision-summary", [
      stateInfo.detail,
      vision.model_error,
      Array.isArray(vision.model_warnings) && vision.model_warnings.length ? vision.model_warnings.join(" | ") : "",
    ].filter(Boolean).join(" | "));
    setChecked("vision-enable-yolo", vision.yolo_enabled);
    setChecked("vision-enable-meter", vision.meter_enabled);
    setChecked("vision-enable-tracking", vision.tracking_enabled);
    setValue("vision-yolo-confidence", vision.yolo_confidence ?? 0.45);
    setValue("vision-meter-confidence", vision.meter_confidence ?? 0.55);

    renderMetricCards("vision-runtime-metrics", [
      { label: "检测器", value: (vision.active_detectors || []).join(" / ") || "未加载", hint: `推理设备 ${formatMaybe(vision.device_name)}`, state: (vision.active_detectors || []).length ? "online" : "idle" },
      { label: "实时帧率", value: `${vision.fps || 0} FPS`, hint: formatMaybe(vision.frame_size, "0x0"), state: vision.running ? "online" : "idle" },
      { label: "自动抓拍", value: formatBool(vision.periodic_capture_enabled, "开启", "关闭"), hint: buildVisionSnapshotStatus(vision), state: vision.periodic_capture_enabled ? "online" : "idle" },
      { label: "目标跟踪", value: formatBool(vision.tracking_enabled, "开启", "关闭"), hint: formatMaybe(vision.tracking_joint_policy, "未启用"), state: vision.tracking_enabled ? "warn" : "idle" },
    ]);

    setText("vision-snapshot-status", buildVisionSnapshotStatus(vision));
    renderUploadResult(vision.latest_result);
    renderDetectionList(vision.recent_detections || []);
  }
  function renderRosPage(data) {
    const ros = resolveRosStatus(data);
    const rosState = buildRosRuntimeState(ros);
    const currentPort = $("lidar-port")?.value || ros.last_lidar_port || (ros.lidar_ports || [])[0] || "/dev/rplidar";
    fillSelect("lidar-port", ros.lidar_ports || [], currentPort);

    setText("lidar-runtime-chip", rosState.label);
    setText("lidar-scan-summary", ros.scan_fresh
      ? `当前会话已收到 ${ros.scan_frames_received || 0} 帧 /scan，新鲜度正常，最近更新时间 ${formatAgeMs(ros.last_scan_age_ms)}。`
      : "当前会话仍未收到新的 /scan 帧。");
    setText("lidar-map-summary", ros.map_fresh
      ? `当前会话已收到 ${ros.map_frames_received || 0} 帧 /map，地图预览持续刷新。`
      : (ros.scan_fresh ? "雷达有数据，但 slam_toolbox 未持续出图。" : "等待地图帧进入当前会话。"));

    const rvizSummary = ros.rviz_live
      ? (ros.rviz_map_render_ok
        ? `RViz 已打开，模式 ${rosRvizModeLabel(ros.rviz_render_mode)}。`
        : (ros.rviz_last_error || `RViz 已打开，当前模式 ${rosRvizModeLabel(ros.rviz_render_mode)}。`))
      : "RViz 未打开/图形会话不可用。";
    setText("lidar-runtime-summary", [rosState.detail, rvizSummary].filter(Boolean).join(" | "));

    renderMetricCards("lidar-scan-metrics", [
      { label: "驱动", value: formatBool(ros.driver_live, "运行中", "未运行"), hint: `端口 ${formatMaybe(ros.last_lidar_port)}`, state: ros.driver_live ? "online" : "offline" },
      { label: "/scan 新鲜度", value: formatBool(ros.scan_fresh, "有新帧", "无新帧"), hint: `最近 ${formatAgeMs(ros.last_scan_age_ms)}`, state: ros.scan_fresh ? "online" : "warn" },
      { label: "扫描帧数", value: String(ros.scan_frames_received || 0), hint: formatBool(ros.scan_topic_ready, "topic 已发现", "topic 未发现"), state: (ros.scan_frames_received || 0) > 0 ? "online" : "idle" },
      { label: "雷达口存在", value: formatBool(ros.lidar_port_present, "是", "否"), hint: "错误选到控制串口会被拒绝", state: ros.lidar_port_present ? "online" : "warn" },
    ]);

    renderMetricCards("lidar-map-metrics", [
      { label: "SLAM", value: formatBool(ros.slam_live, "运行中", "未运行"), hint: formatBool(ros.map_topic_ready, "/map 已发现", "/map 未发现"), state: ros.slam_live ? "online" : "offline" },
      { label: "/map 新鲜度", value: formatBool(ros.map_fresh, "有新帧", "无新帧"), hint: `最近 ${formatAgeMs(ros.last_map_age_ms)}`, state: ros.map_fresh ? "online" : "warn" },
      { label: "地图尺寸", value: `${ros.map_width_m || 0} × ${ros.map_height_m || 0} m`, hint: `分辨率 ${ros.map_resolution_cm || 0} cm`, state: ros.map_fresh ? "online" : "idle" },
      { label: "已保存地图", value: String(ros.saved_maps_count || 0), hint: formatMaybe(ros.latest_saved_map, "暂无"), state: (ros.saved_maps_count || 0) > 0 ? "online" : "idle" },
    ]);

    renderMetricCards("lidar-runtime-metrics", [
      { label: "odom TF", value: formatBool(ros.odom_tf_live, "在线", "离线"), hint: "odom -> base_link", state: ros.odom_tf_live ? "online" : "warn" },
      { label: "laser TF", value: formatBool(ros.laser_tf_live, "在线", "离线"), hint: "base_link -> laser", state: ros.laser_tf_live ? "online" : "warn" },
      { label: "RViz", value: formatBool(ros.rviz_live, "已打开", "未打开"), hint: rosRvizModeLabel(ros.rviz_render_mode), state: ros.rviz_live ? "online" : "idle" },
      { label: "RViz 地图", value: formatBool(ros.rviz_map_render_ok, "正常", "异常"), hint: ros.rviz_last_error || "稳定优先兼容模式", state: ros.rviz_map_render_ok ? "online" : (ros.rviz_live ? "warn" : "idle") },
      { label: "机器人位姿", value: `${(ros.pose_m || [0, 0])[0] || 0}, ${(ros.pose_m || [0, 0])[1] || 0}`, hint: `航向 ${((ros.pose_m || [0, 0, 0])[2] || 0)}°`, state: ros.map_fresh ? "online" : "idle" },
      { label: "当前错误", value: ros.last_ready_error ? "有" : "无", hint: ros.last_ready_error || "链路正常", state: ros.last_ready_error ? "warn" : "online" },
    ]);
  }

  function renderAvoidancePage(data) {
    const avoidance = data.avoidance;
    const rosPorts = resolveRosStatus(data).lidar_ports || [];
    const currentPort = $("avoidance-port")?.value || avoidance.port || rosPorts[0] || "/dev/rplidar";
    fillSelect("avoidance-port", rosPorts, currentPort);
    setValue("avoidance-threshold", avoidance.threshold_mm || 50);

    const avoidanceState = avoidanceRuntimeState(avoidance);
    setText("avoidance-runtime-tag", avoidanceState.label);
    setText("avoidance-summary", [
      avoidanceState.detail,
      avoidance.last_error,
      avoidance.device_info ? `设备 ${avoidance.device_info}` : "",
      avoidance.health ? `健康 ${avoidance.health}` : "",
    ].filter(Boolean).join(" | "));

    renderMetricCards("avoidance-runtime-metrics", [
      { label: "雷达运行", value: formatBool(avoidance.running, "已启动", "未启动"), hint: `端口 ${formatMaybe(avoidance.port)}`, state: avoidance.running ? "online" : "offline" },
      { label: "避障状态", value: formatBool(avoidance.avoidance_enabled, "已启用", "未启用"), hint: `动作 ${formatMaybe(avoidance.current_action, "待机")}`, state: avoidance.avoidance_enabled ? "online" : "idle" },
      { label: "阈值", value: formatDistanceMm(avoidance.threshold_mm), hint: "支持网页实时修改", state: "idle" },
      { label: "扫描计数", value: String(avoidance.scan_count || 0), hint: avoidance.health || "等待雷达数据", state: (avoidance.scan_count || 0) > 0 ? "online" : "idle" },
    ]);

    renderMetricCards("avoidance-distance-metrics", [
      { label: "前方", value: formatDistanceMm(avoidance.front_mm), hint: "正前方", state: Number(avoidance.front_mm) > 0 && Number(avoidance.front_mm) <= Number(avoidance.threshold_mm || 0) ? "warn" : "online" },
      { label: "左前", value: formatDistanceMm(avoidance.front_left_mm), hint: "前左扇区", state: "idle" },
      { label: "右前", value: formatDistanceMm(avoidance.front_right_mm), hint: "前右扇区", state: "idle" },
      { label: "左侧", value: formatDistanceMm(avoidance.left_mm), hint: "左侧", state: "idle" },
      { label: "右侧", value: formatDistanceMm(avoidance.right_mm), hint: "右侧", state: "idle" },
      { label: "后方", value: formatDistanceMm(avoidance.rear_mm), hint: "后方", state: "idle" },
    ]);
  }

  function renderMapsPage() {
    const maps = Array.isArray(state.cachedMaps) ? state.cachedMaps : [];
    const tag = $("maps-runtime-tag");
    if (tag) {
      tag.textContent = maps.length ? `${maps.length} 张地图` : "暂无地图";
    }

    const previewImage = $("maps-preview-image");
    const previewEmpty = $("maps-preview-empty");
    const list = $("maps-list");
    const yamlLink = $("maps-download-yaml");
    const imageLink = $("maps-download-image");

    if (!maps.length) {
      setText("maps-selected-name", "未选择地图");
      setText("maps-selected-detail", "当前还没有已保存地图。");
      if (previewImage) {
        previewImage.classList.remove("active");
        previewImage.removeAttribute("src");
      }
      if (previewEmpty) {
        previewEmpty.style.display = "flex";
      }
      if (yamlLink) {
        yamlLink.href = "#";
        yamlLink.classList.add("disabled");
      }
      if (imageLink) {
        imageLink.href = "#";
        imageLink.classList.add("disabled");
      }
      if (list) {
        list.innerHTML = '<div class="detection-empty">暂无已保存地图。</div>';
      }
      return;
    }

    if (!maps.some((item) => item.name === state.selectedMapName)) {
      state.selectedMapName = maps[0].name;
    }

    const selected = maps.find((item) => item.name === state.selectedMapName) || maps[0];
    state.selectedMapName = selected.name;

    setText("maps-selected-name", selected.name);
    setText("maps-selected-detail", `更新时间 ${selected.updated_at} · 占用 ${selected.size_text}`);

    const previewUrl = `/api/ros/maps/image/${encodeURIComponent(selected.name)}?t=${selected.updated_ts || Date.now()}`;
    if (selected.has_image) {
      if (previewImage) {
        previewImage.src = previewUrl;
        previewImage.classList.add("active");
      }
      if (previewEmpty) {
        previewEmpty.style.display = "none";
      }
      if (imageLink) {
        imageLink.href = `/api/ros/maps/download/${encodeURIComponent(selected.name)}/image`;
        imageLink.classList.remove("disabled");
      }
    } else {
      if (previewImage) {
        previewImage.classList.remove("active");
        previewImage.removeAttribute("src");
      }
      if (previewEmpty) {
        previewEmpty.style.display = "flex";
        previewEmpty.textContent = "该地图没有预览图片。";
      }
      if (imageLink) {
        imageLink.href = "#";
        imageLink.classList.add("disabled");
      }
    }

    if (yamlLink) {
      yamlLink.href = `/api/ros/maps/download/${encodeURIComponent(selected.name)}/yaml`;
      yamlLink.classList.remove("disabled");
    }

    if (list) {
      list.innerHTML = maps.map((item) => `
        <article class="saved-map-card${item.name === state.selectedMapName ? " active" : ""}" data-map-name="${escapeHtml(item.name)}">
          <div class="saved-map-card-preview">
            ${item.has_image
              ? `<img src="/api/ros/maps/image/${encodeURIComponent(item.name)}?t=${item.updated_ts || Date.now()}" alt="${escapeHtml(item.name)}">`
              : '<div class="saved-map-thumb-empty">暂无预览</div>'}
          </div>
          <div class="saved-map-card-body">
            <div class="saved-map-card-title">${escapeHtml(item.name)}</div>
            <div class="saved-map-card-meta">更新时间 ${escapeHtml(item.updated_at)}</div>
            <div class="saved-map-card-meta">文件 ${escapeHtml(item.yaml_file || "--")} · ${escapeHtml(item.size_text || "--")}</div>
          </div>
        </article>
      `).join("");
    }
  }

  function renderLogsPage(logs) {
    const lines = Array.isArray(logs) ? logs : [];
    const text = lines.length ? lines.join("\n") : "等待日志加载。";
    state.lastLogsText = text;
    const box = $("logs-box");
    if (box) {
      box.textContent = text;
      box.scrollTop = box.scrollHeight;
    }
    setChecked("logs-auto-refresh", state.logsAutoRefresh);
    setText("logs-runtime-tag", state.logsAutoRefresh ? "自动刷新" : "手动刷新");
  }

  async function refreshBackendStatus() {
    try {
      const data = await apiJson("/api/status");
      state.lastStatusError = "";
      state.backendStatus = data;
      renderSidebar(data);
      renderHome(data);
      renderControlPage(data);
      renderVisionPage(data);
      renderRosPage(data);
      renderAvoidancePage(data);
      renderMapsPage();
      if (!state.lastLogsText && Array.isArray(data.logs)) {
        renderLogsPage(data.logs);
      }
    } catch (error) {
      const message = error.message || "后端状态刷新失败";
      if (state.lastStatusError !== message) {
        showToast(message, true);
      }
      state.lastStatusError = message;
    }
  }

  async function refreshCameraList() {
    try {
      const result = await apiJson("/api/vision/cameras");
      const fallback = state.backendStatus?.vision?.camera_index ?? 0;
      const cameras = Array.isArray(result.cameras) && result.cameras.length ? result.cameras : [fallback];
      fillSelect("camera-select", cameras, state.backendStatus?.vision?.camera_index ?? cameras[0], (item) => `相机 ${item}`);
    } catch (error) {
      setText("vision-summary", error.message || "摄像头列表获取失败");
    }
  }

  async function refreshMaps(force = false) {
    if (!force && state.activeTab !== "maps") {
      return;
    }
    try {
      const result = await apiJson("/api/ros/maps");
      state.cachedMaps = Array.isArray(result.maps) ? result.maps : [];
      renderMapsPage();
    } catch (error) {
      if (force || state.activeTab === "maps") {
        showToast(error.message || "地图列表刷新失败", true);
      }
    }
  }

  async function refreshLogs(force = false) {
    if (!force && state.activeTab !== "logs" && !state.logsAutoRefresh) {
      return;
    }
    try {
      const result = await apiJson("/api/logs");
      renderLogsPage(result.logs || []);
    } catch (error) {
      if (force || state.activeTab === "logs") {
        showToast(error.message || "日志刷新失败", true);
      }
    }
  }
  async function performAction(task, successMessage, options = {}) {
    try {
      const result = await task();
      const message = typeof successMessage === "function"
        ? successMessage(result)
        : (successMessage || result?.message || "操作完成");
      if (message) {
        showToast(message, false);
      }
      if (options.status !== false) {
        await refreshBackendStatus();
      }
      if (options.cameras) {
        await refreshCameraList();
      }
      if (options.maps) {
        await refreshMaps(true);
      }
      if (options.logs) {
        await refreshLogs(true);
      }
      return result;
    } catch (error) {
      showToast(error.message || "操作失败", true);
      return null;
    }
  }

  async function refreshBackendStatus(force = false) {
    if (state.backendStatusPending && !force) {
      return state.backendStatus;
    }
    state.backendStatusPending = true;
    try {
      const data = await apiJson("/api/status");
      state.lastStatusError = "";
      state.backendStatus = data;
      if (!state.rosStatus || state.activeTab !== "ros") {
        state.rosStatus = data.ros || state.rosStatus;
      }
      renderSidebar(data);
      renderHome(data);
      renderControlPage(data);
      renderVisionPage(data);
      renderRosPage(data);
      renderAvoidancePage(data);
      renderMapsPage();
      if (!state.lastLogsText && Array.isArray(data.logs)) {
        renderLogsPage(data.logs);
      }
      return data;
    } catch (error) {
      const message = error.message || "后端状态刷新失败";
      if (state.lastStatusError !== message) {
        showToast(message, true);
      }
      state.lastStatusError = message;
      return null;
    } finally {
      state.backendStatusPending = false;
    }
  }

  async function refreshCameraList() {
    if (state.cameraListPending) {
      return;
    }
    state.cameraListPending = true;
    try {
      const result = await apiJson("/api/vision/cameras");
      const fallback = state.backendStatus?.vision?.camera_index ?? 0;
      const cameras = Array.isArray(result.cameras) && result.cameras.length ? result.cameras : [fallback];
      fillSelect("camera-select", cameras, state.backendStatus?.vision?.camera_index ?? cameras[0], (item) => `相机 ${item}`);
    } catch (error) {
      setText("vision-summary", error.message || "摄像头列表获取失败");
    } finally {
      state.cameraListPending = false;
    }
  }

  async function refreshRosStatus(force = false) {
    if (!force && state.activeTab !== "ros") {
      return state.rosStatus;
    }
    if (state.rosStatusPending && !force) {
      return state.rosStatus;
    }
    state.rosStatusPending = true;
    try {
      const data = await apiJson("/api/ros/status");
      state.lastRosStatusError = "";
      state.rosStatus = data;
      if (state.backendStatus) {
        state.backendStatus.ros = data;
      }
      renderRosPage(data);
      renderAvoidancePage(state.backendStatus || { avoidance: state.backendStatus?.avoidance || {}, ros: data });
      return data;
    } catch (error) {
      const message = error.message || "ROS 状态刷新失败";
      if ((force || state.activeTab === "ros") && state.lastRosStatusError !== message) {
        showToast(message, true);
      }
      state.lastRosStatusError = message;
      return null;
    } finally {
      state.rosStatusPending = false;
    }
  }

  async function refreshMaps(force = false) {
    if (!force && state.activeTab !== "maps" && state.activeTab !== "ros") {
      return;
    }
    if (state.mapsPending && !force) {
      return;
    }
    state.mapsPending = true;
    try {
      const result = await apiJson("/api/ros/maps");
      state.cachedMaps = Array.isArray(result.maps) ? result.maps : [];
      renderMapsPage();
    } catch (error) {
      if (force || state.activeTab === "maps" || state.activeTab === "ros") {
        showToast(error.message || "地图列表刷新失败", true);
      }
    } finally {
      state.mapsPending = false;
    }
  }

  async function refreshLogs(force = false) {
    if (!force && state.activeTab !== "logs" && !state.logsAutoRefresh) {
      return;
    }
    if (state.logsPending && !force) {
      return;
    }
    state.logsPending = true;
    try {
      const result = await apiJson("/api/logs");
      renderLogsPage(result.logs || []);
    } catch (error) {
      if (force || state.activeTab === "logs") {
        showToast(error.message || "日志刷新失败", true);
      }
    } finally {
      state.logsPending = false;
    }
  }

  async function performAction(task, successMessage, options = {}) {
    const button = options.button || null;
    if (button) {
      setButtonBusy(button, true);
    }
    try {
      const result = await task();
      const message = typeof successMessage === "function"
        ? successMessage(result)
        : (successMessage || result?.message || "操作完成");
      if (message) {
        showToast(message, false);
      }
      if (options.status !== false) {
        if (options.ros) {
          await refreshRosStatus(true);
        } else {
          await refreshBackendStatus(true);
        }
      }
      if (options.cameras) {
        await refreshCameraList();
      }
      if (options.maps) {
        await refreshMaps(true);
      }
      if (options.logs) {
        await refreshLogs(true);
      }
      return result;
    } catch (error) {
      showToast(error.message || "操作失败", true);
      return null;
    } finally {
      if (button) {
        setButtonBusy(button, false);
      }
    }
  }

  function setActiveMotionButton(action) {
    state.activeMotionAction = action || "stop";
    document.querySelectorAll("[data-motion]").forEach((button) => {
      button.classList.toggle("active", button.dataset.motion === state.activeMotionAction && state.activeMotionAction !== "stop");
    });
    if (state.backendStatus) {
      renderControlPage(state.backendStatus);
    }
  }

  async function sendMotionCommand(action) {
    try {
      await apiJson("/api/control/motion", { method: "POST", body: { action } });
    } catch (error) {
      showToast(error.message || "动作命令发送失败", true);
    }
  }

  function initTabs() {
    document.querySelectorAll(".tab").forEach((button) => {
      button.addEventListener("click", async () => {
        const tabName = button.dataset.tab || "home";
        state.activeTab = tabName;
        document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
        TAB_NAMES.forEach((name) => {
          $(`tab-${name}`)?.classList.toggle("active", name === tabName);
        });
        if (tabName === "vision") {
          await refreshCameraList();
        }
        if (tabName === "maps") {
          await refreshMaps(true);
        }
        if (tabName === "logs") {
          await refreshLogs(true);
        }
      });
    });
  }

  function initTabs() {
    document.querySelectorAll(".tab").forEach((button) => {
      button.addEventListener("click", async () => {
        const tabName = button.dataset.tab || "home";
        state.activeTab = tabName;
        document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
        TAB_NAMES.forEach((name) => {
          $(`tab-${name}`)?.classList.toggle("active", name === tabName);
        });
        if (tabName === "vision") {
          await refreshCameraList();
        }
        if (tabName === "ros") {
          await refreshRosStatus(true);
          await refreshMaps(true);
        }
        if (tabName === "maps") {
          await refreshMaps(true);
        }
        if (tabName === "logs") {
          await refreshLogs(true);
        }
      });
    });
  }

  function bindMotionButtons() {
    document.querySelectorAll("[data-motion]").forEach((button) => {
      const action = button.dataset.motion || "stop";
      if (action === "stop") {
        button.addEventListener("click", async () => {
          setActiveMotionButton("stop");
          await sendMotionCommand("stop");
        });
        return;
      }

      button.addEventListener("pointerdown", async (event) => {
        event.preventDefault();
        button.dataset.pressed = "1";
        setActiveMotionButton(action);
        await sendMotionCommand(action);
      });

      ["pointerup", "pointerleave", "pointercancel"].forEach((eventName) => {
        button.addEventListener(eventName, async () => {
          if (button.dataset.pressed !== "1") {
            return;
          }
          button.dataset.pressed = "";
          setActiveMotionButton("stop");
          await sendMotionCommand("stop");
        });
      });
    });
  }

  function bindServoEvents() {
    const container = $("servo-list");
    if (!container) {
      return;
    }

    container.addEventListener("input", (event) => {
      const target = event.target;
      const rangeIndex = target.dataset.servoRange;
      const numberIndex = target.dataset.servoNumber;
      const index = Number(rangeIndex ?? numberIndex);
      if (!Number.isInteger(index)) {
        return;
      }
      const value = Math.round(clamp(target.value, SERVO_MIN, SERVO_MAX));
      state.servoDraft[index] = value;
      state.servoEditLocks[index] = true;
      const range = container.querySelector(`[data-servo-range="${index}"]`);
      const number = container.querySelector(`[data-servo-number="${index}"]`);
      if (range && range !== target) {
        range.value = String(value);
      }
      if (number && number !== target) {
        number.value = String(value);
      }
    });

    container.addEventListener("focusin", (event) => {
      const target = event.target;
      const index = Number(target.dataset.servoRange ?? target.dataset.servoNumber);
      if (Number.isInteger(index)) {
        state.servoEditLocks[index] = true;
      }
    });

    container.addEventListener("focusout", (event) => {
      const target = event.target;
      const index = Number(target.dataset.servoRange ?? target.dataset.servoNumber);
      if (Number.isInteger(index)) {
        window.setTimeout(() => {
          state.servoEditLocks[index] = false;
        }, 120);
      }
    });

    container.addEventListener("click", async (event) => {
      const button = event.target.closest("[data-servo-send]");
      if (!button) {
        return;
      }
      const index = Number(button.dataset.servoSend);
      if (!Number.isInteger(index)) {
        return;
      }
      const value = Math.round(clamp(state.servoDraft[index], SERVO_MIN, SERVO_MAX));
      await performAction(
        () => apiJson("/api/control/servo", { method: "POST", body: { targets: { [index]: value }, duration: 120 } }),
        `关节 ${index} 已发送到 ${value}`,
      );
    });
  }

  function bindMapEvents() {
    $("maps-list")?.addEventListener("click", (event) => {
      const card = event.target.closest("[data-map-name]");
      if (!card) {
        return;
      }
      state.selectedMapName = card.dataset.mapName || "";
      renderMapsPage();
    });
  }

  function initControls() {
    $("btn-control-connect")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/connect", {
          method: "POST",
          body: {
            port: $("control-port")?.value || "",
            baudrate: Number($("control-baudrate")?.value || 115200),
          },
        }),
        (result) => result?.message || "控制串口已连接",
      );
    });

    $("btn-control-disconnect")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/disconnect", { method: "POST" }),
        "通信已断开",
      );
    });

    $("btn-control-handshake")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/handshake", { method: "POST" }),
        (result) => result?.message || "握手命令已发送",
      );
    });

    $("btn-control-status-request")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/status/request", { method: "POST" }),
        "状态请求已发送",
      );
    });

    $("btn-control-report")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/report", {
          method: "POST",
          body: { enabled: $("control-report-enabled")?.value !== "false" },
        }),
        (result) => result?.message || "状态上报设置已更新",
      );
    });

    document.querySelectorAll("[data-mode]").forEach((button) => {
      button.addEventListener("click", async () => {
        await performAction(
          () => apiJson("/api/control/mode", {
            method: "POST",
            body: { mode: button.dataset.mode || "" },
          }),
          (result) => result?.message || `模式切换到 ${button.dataset.mode}`,
        );
      });
    });

    $("btn-control-estop")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/estop", { method: "POST" }),
        "急停命令已发送",
      );
    });

    $("btn-control-estop-clear")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/estop/clear", { method: "POST" }),
        "急停清除命令已发送",
      );
    });

    $("btn-servo-send-all")?.addEventListener("click", async () => {
      const targets = {};
      for (let index = 0; index < SERVO_COUNT; index += 1) {
        targets[index] = Math.round(clamp(state.servoDraft[index], SERVO_MIN, SERVO_MAX));
      }
      await performAction(
        () => apiJson("/api/control/servo", { method: "POST", body: { targets, duration: 160 } }),
        "全部关节命令已发送",
      );
    });

    $("btn-servo-home")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/home", { method: "POST" }),
        "机械臂已回初始位",
      );
    });

    $("btn-servo-reset")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/servo/reset", { method: "POST" }),
        "机械臂复位命令已发送",
      );
    });

    $("btn-servo-stop")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/control/servo/stop", { method: "POST" }),
        "机械臂停止命令已发送",
      );
    });

    $("btn-vision-load-models")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/vision/load_models", { method: "POST" }),
        (result) => result?.message || "模型已加载",
      );
    });

    $("btn-vision-start")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/vision/start", {
          method: "POST",
          body: { camera_index: Number($("camera-select")?.value || 0) },
        }),
        (result) => result?.message || "摄像头已启动",
        { cameras: true },
      );
    });

    $("btn-vision-stop")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/vision/stop", { method: "POST" }),
        "摄像头已停止",
      );
    });

    $("btn-vision-config")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/vision/config", {
          method: "POST",
          body: {
            yolo_enabled: $("vision-enable-yolo")?.checked,
            meter_enabled: $("vision-enable-meter")?.checked,
            tracking_enabled: $("vision-enable-tracking")?.checked,
            yolo_confidence: Number($("vision-yolo-confidence")?.value || 0.45),
            meter_confidence: Number($("vision-meter-confidence")?.value || 0.55),
          },
        }),
        "视觉配置已更新",
      );
    });

    $("btn-vision-upload")?.addEventListener("click", async () => {
      const fileInput = $("vision-upload-file");
      const file = fileInput?.files?.[0];
      if (!file) {
        showToast("请先选择图片文件", true);
        return;
      }
      const formData = new FormData();
      formData.append("file", file);
      const result = await performAction(
        () => apiJson("/api/vision/upload", { method: "POST", body: formData }),
        (payload) => payload?.message || "识别完成",
      );
      if (result?.result) {
        renderUploadResult(result.result);
      }
    });

    $("btn-lidar-start")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/ros/mapping/start", {
          method: "POST",
          body: { port: $("lidar-port")?.value || "" },
        }),
        (result) => buildRosMappingToastMessage(result, "ROS2 建图已启动"),
      );
    });

    $("btn-lidar-reset")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/ros/mapping/reset", {
          method: "POST",
          body: { port: $("lidar-port")?.value || "" },
        }),
        (result) => buildRosMappingToastMessage(result, "ROS2 建图已重启"),
      );
    });

    $("btn-lidar-stop")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/ros/mapping/stop", { method: "POST" }),
        "ROS2 建图已停止",
      );
    });

    $("btn-lidar-save")?.addEventListener("click", async () => {
      const defaultName = "";
      const mapName = window.prompt("请输入地图名称，留空则自动生成：", defaultName);
      if (mapName === null) {
        return;
      }
      await performAction(
        () => apiJson("/api/ros/mapping/save", {
          method: "POST",
          body: { name: mapName.trim() },
        }),
        "地图已保存",
        { maps: true },
      );
    });

    $("btn-rviz-open")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/ros/rviz/open", { method: "POST" }),
        (result) => buildRosMappingToastMessage(result, "RViz 打开请求已发送"),
      );
    });

    $("btn-avoidance-lidar-start")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/avoidance/lidar/start", {
          method: "POST",
          body: { port: $("avoidance-port")?.value || "" },
        }),
        "避障雷达已启动",
      );
    });

    $("btn-avoidance-lidar-stop")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/avoidance/lidar/stop", { method: "POST" }),
        "避障雷达已停止",
      );
    });

    $("btn-avoidance-start")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/avoidance/start", { method: "POST" }),
        "避障已启动",
      );
    });

    $("btn-avoidance-stop")?.addEventListener("click", async () => {
      await performAction(
        () => apiJson("/api/avoidance/stop", { method: "POST" }),
        "避障已停止",
      );
    });

    $("btn-avoidance-threshold")?.addEventListener("click", async () => {
      const threshold = Number($("avoidance-threshold")?.value || 50);
      await performAction(
        () => apiJson("/api/avoidance/threshold", {
          method: "POST",
          body: { threshold_mm: threshold },
        }),
        `避障阈值已更新为 ${threshold} mm`,
      );
    });

    $("btn-maps-refresh")?.addEventListener("click", async () => {
      await refreshMaps(true);
      showToast("地图列表已刷新");
    });

    $("btn-logs-refresh")?.addEventListener("click", async () => {
      await refreshLogs(true);
      showToast("日志已刷新");
    });

    $("logs-auto-refresh")?.addEventListener("change", async (event) => {
      state.logsAutoRefresh = !!event.target.checked;
      if (state.logsAutoRefresh) {
        await refreshLogs(true);
      } else {
        renderLogsPage(state.lastLogsText ? state.lastLogsText.split("\n") : []);
      }
    });
  }

  function bindRosControls() {
    const bind = (id, handler) => {
      const element = $(id);
      if (!element) {
        return;
      }
      const clone = element.cloneNode(true);
      element.replaceWith(clone);
      clone.addEventListener("click", handler);
    };

    bind("btn-lidar-start", async (event) => {
      const button = event.currentTarget;
      await performAction(
        () => apiJson("/api/ros/mapping/start", {
          method: "POST",
          body: { port: $("lidar-port")?.value || "" },
        }),
        (result) => buildRosMappingToastMessage(result, "ROS2 建图已启动"),
        { ros: true, button },
      );
    });

    bind("btn-lidar-reset", async (event) => {
      const button = event.currentTarget;
      await performAction(
        () => apiJson("/api/ros/mapping/reset", {
          method: "POST",
          body: { port: $("lidar-port")?.value || "" },
        }),
        (result) => buildRosMappingToastMessage(result, "ROS2 建图已重启"),
        { ros: true, button, maps: true },
      );
    });

    bind("btn-lidar-stop", async (event) => {
      const button = event.currentTarget;
      await performAction(
        () => apiJson("/api/ros/mapping/stop", { method: "POST" }),
        (result) => result?.message || "ROS2 建图已停止",
        { ros: true, button },
      );
    });

    bind("btn-lidar-save", async (event) => {
      const mapName = window.prompt("请输入地图名称，留空则自动生成：", "");
      if (mapName === null) {
        return;
      }
      const button = event.currentTarget;
      await performAction(
        () => apiJson("/api/ros/mapping/save", {
          method: "POST",
          body: { name: mapName.trim() },
        }),
        (result) => result?.message || "地图已保存",
        { ros: true, button, maps: true },
      );
    });

    bind("btn-rviz-open", async (event) => {
      const button = event.currentTarget;
      await performAction(
        () => apiJson("/api/ros/rviz/open", { method: "POST" }),
        (result) => buildRosMappingToastMessage(result, "RViz 打开请求已发送"),
        { ros: true, button },
      );
    });
  }

  function ensureStep9State() {
    if (!state.annotationCache) {
      state.annotationCache = {};
    }
    if (!state.mapEditorImage) {
      state.mapEditorImage = { mapName: "", image: null };
    }
    if (!state.navTaskCache) {
      state.navTaskCache = {};
    }
    if (!Array.isArray(state.navWaypointDraftIds)) {
      state.navWaypointDraftIds = [];
    }
    if (!("selectedPointId" in state)) {
      state.selectedPointId = "";
    }
    if (!("selectedTaskName" in state)) {
      state.selectedTaskName = "";
    }
    if (!("taskFormDirty" in state)) {
      state.taskFormDirty = false;
    }
    if (!TAB_NAMES.includes("nav")) {
      TAB_NAMES.splice(TAB_NAMES.indexOf("logs"), 0, "nav");
    }
  }

  function currentMapBundle() {
    ensureStep9State();
    return state.annotationCache[state.selectedMapName] || { map_name: state.selectedMapName || "", points: [], map: {} };
  }

  function allowedNavPoints(bundle = currentMapBundle()) {
    return (bundle.points || []).filter((point) => ["pose", "inspect"].includes(String(point.type || "pose")));
  }

  async function loadMapAnnotations(mapName, force = false) {
    ensureStep9State();
    const normalized = String(mapName || "").trim();
    if (!normalized) {
      return { map_name: "", points: [], map: {} };
    }
    if (!force && state.annotationCache[normalized]) {
      return state.annotationCache[normalized];
    }
    const bundle = await apiJson(`/api/maps/annotations/${encodeURIComponent(normalized)}`);
    state.annotationCache[normalized] = {
      map_name: bundle.map_name || normalized,
      points: Array.isArray(bundle.points) ? bundle.points : [],
      map: bundle.map || {},
    };
    return state.annotationCache[normalized];
  }

  async function loadNavTasks(mapName, force = false) {
    ensureStep9State();
    const normalized = String(mapName || "").trim();
    if (!normalized) {
      return { map_name: "", tasks: [], points: [] };
    }
    if (!force && state.navTaskCache[normalized]) {
      return state.navTaskCache[normalized];
    }
    const bundle = await apiJson(`/api/nav/tasks/${encodeURIComponent(normalized)}`);
    state.navTaskCache[normalized] = {
      map_name: bundle.map_name || normalized,
      tasks: Array.isArray(bundle.tasks) ? bundle.tasks : [],
      points: Array.isArray(bundle.points) ? bundle.points : [],
    };
    return state.navTaskCache[normalized];
  }

  function worldToPixel(meta, x, y) {
    const resolution = Number(meta.resolution || 0.05) || 0.05;
    const origin = Array.isArray(meta.origin) ? meta.origin : [0, 0, 0];
    const width = Number(meta.width_px || 0);
    return {
      x: (Number(x) - Number(origin[0] || 0)) / resolution,
      y: width ? Number(meta.height_px || 0) - ((Number(y) - Number(origin[1] || 0)) / resolution) : 0,
    };
  }

  function pixelToWorld(meta, px, py) {
    const resolution = Number(meta.resolution || 0.05) || 0.05;
    const origin = Array.isArray(meta.origin) ? meta.origin : [0, 0, 0];
    const height = Number(meta.height_px || 0);
    return {
      x: Number(origin[0] || 0) + (Number(px) * resolution),
      y: Number(origin[1] || 0) + ((height - Number(py)) * resolution),
    };
  }

  function nextPointId() {
    return `point_${Date.now().toString().slice(-8)}`;
  }

  function fillPointForm(point = null) {
    const item = point || { id: nextPointId(), name: "", type: "pose", x: 0, y: 0, yaw: 0, note: "" };
    setValue("maps-point-id", item.id || nextPointId());
    setValue("maps-point-name", item.name || "");
    setValue("maps-point-type", item.type || "pose");
    setValue("maps-point-x", Number(item.x || 0).toFixed(2));
    setValue("maps-point-y", Number(item.y || 0).toFixed(2));
    setValue("maps-point-yaw", Number(item.yaw || 0).toFixed(1));
    setValue("maps-point-note", item.note || "");
  }

  function pointFromForm() {
    return {
      id: String($("maps-point-id")?.value || "").trim() || nextPointId(),
      name: String($("maps-point-name")?.value || "").trim(),
      type: String($("maps-point-type")?.value || "pose").trim() || "pose",
      x: Number($("maps-point-x")?.value || 0),
      y: Number($("maps-point-y")?.value || 0),
      yaw: Number($("maps-point-yaw")?.value || 0),
      note: String($("maps-point-note")?.value || "").trim(),
    };
  }

  function replaceBundlePoints(points) {
    const bundle = currentMapBundle();
    state.annotationCache[state.selectedMapName] = {
      ...bundle,
      points: points.map((item) => ({ ...item })),
    };
  }

  function selectPoint(pointId) {
    ensureStep9State();
    state.selectedPointId = String(pointId || "");
    const point = currentMapBundle().points.find((item) => item.id === state.selectedPointId) || null;
    fillPointForm(point);
    renderMapsPage();
  }

  function renderPointCards() {
    const container = $("maps-point-list");
    if (!container) {
      return;
    }
    const points = currentMapBundle().points || [];
    if (!points.length) {
      container.innerHTML = '<div class="detection-empty">当前地图还没有点位。</div>';
      return;
    }
    container.innerHTML = points.map((point) => `
      <article class="point-card${point.id === state.selectedPointId ? " active" : ""}" data-point-id="${escapeHtml(point.id)}">
        <div class="point-card-title">${escapeHtml(point.name || point.id)}</div>
        <div class="point-card-meta">ID ${escapeHtml(point.id)} · 类型 ${escapeHtml(point.type || "pose")}</div>
        <div class="point-card-meta">X ${Number(point.x || 0).toFixed(2)} · Y ${Number(point.y || 0).toFixed(2)} · Yaw ${Number(point.yaw || 0).toFixed(1)}°</div>
        <div class="point-card-meta">${escapeHtml(point.note || "无备注")}</div>
      </article>
    `).join("");
  }

  function drawMapEditor() {
    const canvas = $("maps-editor-canvas");
    const empty = $("maps-editor-empty");
    if (!canvas) {
      return;
    }
    const imageState = state.mapEditorImage || {};
    const image = imageState.image;
    const bundle = currentMapBundle();
    const meta = bundle.map || {};
    if (!image || !image.complete || !meta.width_px || !meta.height_px) {
      canvas.classList.remove("active");
      if (empty) {
        empty.style.display = "flex";
        empty.textContent = state.selectedMapName ? "等待地图预览加载" : "请先选择地图";
      }
      return;
    }
    canvas.width = image.naturalWidth || meta.width_px;
    canvas.height = image.naturalHeight || meta.height_px;
    const context = canvas.getContext("2d");
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.drawImage(image, 0, 0, canvas.width, canvas.height);
    (bundle.points || []).forEach((point) => {
      const pixel = worldToPixel(meta, point.x, point.y);
      const selected = point.id === state.selectedPointId;
      context.beginPath();
      context.arc(pixel.x, pixel.y, selected ? 9 : 7, 0, Math.PI * 2);
      context.fillStyle = selected ? "#c75846" : "#1787b2";
      context.fill();
      context.lineWidth = 2;
      context.strokeStyle = "#ffffff";
      context.stroke();
      context.fillStyle = "#163446";
      context.font = "14px 'Segoe UI', 'Microsoft YaHei UI', sans-serif";
      context.fillText(point.name || point.id, pixel.x + 10, pixel.y - 10);
    });
    canvas.classList.add("active");
    if (empty) {
      empty.style.display = "none";
    }
  }

  function ensureMapEditorImage(mapName) {
    ensureStep9State();
    const normalized = String(mapName || "").trim();
    if (!normalized) {
      drawMapEditor();
      return;
    }
    if (state.mapEditorImage.mapName === normalized && state.mapEditorImage.image) {
      drawMapEditor();
      return;
    }
    const image = new Image();
    image.onload = () => {
      state.mapEditorImage = { mapName: normalized, image };
      drawMapEditor();
    };
    image.onerror = () => {
      state.mapEditorImage = { mapName: normalized, image: null };
      drawMapEditor();
    };
    image.src = `/api/maps/image/${encodeURIComponent(normalized)}?t=${Date.now()}`;
  }

  function setTaskDraft(task = null) {
    ensureStep9State();
    const draft = task || {
      task_name: "",
      mode: "inspect",
      loop_count: 1,
      arrival_tolerance: 0.25,
      avoidance_threshold_mm: 50,
      waypoint_ids: [],
    };
    state.selectedTaskName = draft.task_name || "";
    state.navWaypointDraftIds = Array.isArray(draft.waypoint_ids) ? [...draft.waypoint_ids] : [];
    state.taskFormDirty = false;
    setValue("nav-task-name", draft.task_name || "");
    setValue("nav-task-mode", draft.mode || "inspect");
    setValue("nav-task-loop-count", draft.loop_count || 1);
    setValue("nav-arrival-tolerance", draft.arrival_tolerance ?? 0.25);
    setValue("nav-avoidance-threshold", draft.avoidance_threshold_mm ?? 50);
    setValue("nav-task-select", draft.task_name || "");
  }

  function renderTaskDraft() {
    ensureStep9State();
    const container = $("nav-task-waypoints");
    if (!container) {
      return;
    }
    const lookup = new Map((currentMapBundle().points || []).map((point) => [point.id, point]));
    if (!state.navWaypointDraftIds.length) {
      container.innerHTML = '<div class="detection-empty">当前任务还没有点位。</div>';
      return;
    }
    container.innerHTML = state.navWaypointDraftIds.map((pointId, index) => {
      const point = lookup.get(pointId) || { id: pointId, name: pointId };
      return `
        <div class="chip-item">
          <span>${index + 1}. ${escapeHtml(point.name || point.id)} (${escapeHtml(point.id)})</span>
          <button class="ghost" type="button" data-waypoint-remove="${escapeHtml(point.id)}">移除</button>
        </div>
      `;
    }).join("");
  }

  function renderNavPointPreview() {
    const container = $("nav-points-preview");
    if (!container) {
      return;
    }
    const points = allowedNavPoints();
    if (!points.length) {
      container.innerHTML = '<div class="detection-empty">当前地图没有可用于导航的 pose / inspect 点位。</div>';
      return;
    }
    container.innerHTML = points.map((point) => `
      <article class="point-card">
        <div class="point-card-title">${escapeHtml(point.name || point.id)}</div>
        <div class="point-card-meta">ID ${escapeHtml(point.id)} · ${escapeHtml(point.type || "pose")}</div>
        <div class="point-card-meta">X ${Number(point.x || 0).toFixed(2)} · Y ${Number(point.y || 0).toFixed(2)} · Yaw ${Number(point.yaw || 0).toFixed(1)}°</div>
        <div class="point-card-meta">${escapeHtml(point.note || "无备注")}</div>
      </article>
    `).join("");
  }

  function renderMapsPage() {
    ensureStep9State();
    const maps = Array.isArray(state.cachedMaps) ? state.cachedMaps : [];
    setText("maps-runtime-tag", maps.length ? `${maps.length} 张地图` : "暂无地图");
    if (!maps.length) {
      setText("maps-selected-name", "未选择地图");
      setText("maps-selected-detail", "当前还没有已保存地图。");
      $("maps-preview-image")?.classList.remove("active");
      $("maps-preview-image")?.removeAttribute("src");
      if ($("maps-preview-empty")) {
        $("maps-preview-empty").style.display = "flex";
      }
      $("maps-download-yaml")?.classList.add("disabled");
      $("maps-download-image")?.classList.add("disabled");
      $("maps-list").innerHTML = '<div class="detection-empty">暂无已保存地图。</div>';
      renderPointCards();
      drawMapEditor();
      return;
    }

    if (!maps.some((item) => item.name === state.selectedMapName)) {
      state.selectedMapName = maps[0].name;
    }
    const selected = maps.find((item) => item.name === state.selectedMapName) || maps[0];
    state.selectedMapName = selected.name;
    setText("maps-selected-name", selected.name);
    setText("maps-selected-detail", `更新时间 ${selected.updated_at} · 占用 ${selected.size_text}`);

    const previewImage = $("maps-preview-image");
    const previewEmpty = $("maps-preview-empty");
    if (selected.has_image && previewImage) {
      previewImage.src = `/api/ros/maps/image/${encodeURIComponent(selected.name)}?t=${selected.updated_ts || Date.now()}`;
      previewImage.classList.add("active");
      if (previewEmpty) {
        previewEmpty.style.display = "none";
      }
      $("maps-download-image").href = `/api/ros/maps/download/${encodeURIComponent(selected.name)}/image`;
      $("maps-download-image").classList.remove("disabled");
    } else {
      previewImage?.classList.remove("active");
      previewImage?.removeAttribute("src");
      if (previewEmpty) {
        previewEmpty.style.display = "flex";
        previewEmpty.textContent = "该地图没有预览图。";
      }
      $("maps-download-image").href = "#";
      $("maps-download-image").classList.add("disabled");
    }
    $("maps-download-yaml").href = `/api/ros/maps/download/${encodeURIComponent(selected.name)}/yaml`;
    $("maps-download-yaml").classList.remove("disabled");
    $("maps-list").innerHTML = maps.map((item) => `
      <article class="saved-map-card${item.name === state.selectedMapName ? " active" : ""}" data-map-name="${escapeHtml(item.name)}">
        <div class="saved-map-card-preview">
          ${item.has_image
            ? `<img src="/api/ros/maps/image/${encodeURIComponent(item.name)}?t=${item.updated_ts || Date.now()}" alt="${escapeHtml(item.name)}">`
            : '<div class="saved-map-thumb-empty">暂无预览</div>'}
        </div>
        <div class="saved-map-card-body">
          <div class="saved-map-card-title">${escapeHtml(item.name)}</div>
          <div class="saved-map-card-meta">更新时间 ${escapeHtml(item.updated_at)}</div>
          <div class="saved-map-card-meta">文件 ${escapeHtml(item.yaml_file || "--")} · ${escapeHtml(item.size_text || "--")}</div>
        </div>
      </article>
    `).join("");

    const bundle = currentMapBundle();
    setText("maps-editor-tag", bundle.points?.length ? `${bundle.points.length} 个点位` : "点击地图选点");
    setText("maps-point-summary", `当前地图 ${selected.name}，分辨率 ${Number(bundle.map?.resolution || 0.05).toFixed(3)} m/px。`);
    renderPointCards();
    ensureMapEditorImage(selected.name);
  }

  function renderNavPage(data) {
    ensureStep9State();
    const nav = data?.nav || {};
    const ros = resolveRosStatus(data || state.backendStatus || {});
    const maps = Array.isArray(state.cachedMaps) ? state.cachedMaps : [];
    if (!maps.some((item) => item.name === state.selectedMapName)) {
      state.selectedMapName = maps[0]?.name || "";
    }
    fillSelect("nav-map-select", maps.map((item) => item.name), state.selectedMapName);
    fillSelect("nav-lidar-port", ros.lidar_ports || [], $("nav-lidar-port")?.value || ros.last_lidar_port || "/dev/rplidar");

    const points = allowedNavPoints();
    fillSelect("nav-goal-point", points.map((item) => item.id), $("nav-goal-point")?.value || points[0]?.id || "", (id) => {
      const point = points.find((item) => item.id === id) || { name: id, id };
      return `${point.name || point.id} (${point.id})`;
    });
    fillSelect("nav-waypoint-picker", points.map((item) => item.id), $("nav-waypoint-picker")?.value || points[0]?.id || "", (id) => {
      const point = points.find((item) => item.id === id) || { name: id, id };
      return `${point.name || point.id} (${point.id})`;
    });

    const taskBundle = state.navTaskCache[state.selectedMapName] || { tasks: [] };
    fillSelect("nav-task-select", taskBundle.tasks.map((item) => item.task_name), state.selectedTaskName || "");
    setText("nav-runtime-tag", formatMaybe(nav.nav_state, "idle"));
    setText("nav-task-tag", taskBundle.tasks.length ? `${taskBundle.tasks.length} 个任务` : "任务");
    setText("nav-summary", [
      nav.localization_map ? `定位地图 ${nav.localization_map}` : "定位未启动",
      nav.goal_feedback,
      nav.last_result,
      nav.safety_interlock_reason,
      nav.last_error && nav.last_error !== nav.last_result ? nav.last_error : "",
    ].filter(Boolean).join(" | "));
    setText("nav-task-summary", nav.active_task
      ? `执行中: ${nav.active_task.task_name} · loop ${nav.active_task.current_loop}/${nav.active_task.loop_count}`
      : "任务按点位 ID 引用，不复制坐标。");

    renderMetricCards("nav-runtime-metrics", [
      { label: "导航状态", value: formatMaybe(nav.nav_state, "idle"), hint: formatMaybe(nav.navigation_mode, "idle"), state: nav.blocked ? "warn" : (nav.goal_active || nav.task_running ? "online" : "idle") },
      { label: "定位栈", value: formatBool(nav.localization_active, "已启动", "未启动"), hint: formatMaybe(nav.localization_map, "未选择地图"), state: nav.localization_active ? "online" : "idle" },
      { label: "动作服务", value: formatBool(nav.action_server_ready, "就绪", "未就绪"), hint: formatMaybe(nav.goal_feedback, "等待目标"), state: nav.action_server_ready ? "online" : "warn" },
      { label: "安全联锁", value: nav.safety_interlock_reason ? "已触发" : "正常", hint: formatMaybe(nav.safety_interlock_reason, "前向阈值监控中"), state: nav.safety_interlock_reason ? "warn" : "online" },
    ]);
    renderTaskDraft();
    renderNavPointPreview();
  }

  function syncPointFormWithCurrentMap(preferBlank = false) {
    ensureStep9State();
    const points = currentMapBundle().points || [];
    if (preferBlank || !points.length) {
      state.selectedPointId = "";
      fillPointForm();
      return;
    }
    const selected = points.find((item) => item.id === state.selectedPointId);
    if (selected) {
      fillPointForm(selected);
      return;
    }
    state.selectedPointId = points[0].id || "";
    fillPointForm(points[0]);
  }

  function taskDraftFromForm() {
    ensureStep9State();
    return {
      map_name: state.selectedMapName || "",
      task_name: String($("nav-task-name")?.value || "").trim(),
      mode: String($("nav-task-mode")?.value || "inspect").trim() || "inspect",
      loop_count: Math.max(1, Math.round(Number($("nav-task-loop-count")?.value || 1) || 1)),
      arrival_tolerance: Math.max(0.05, Number($("nav-arrival-tolerance")?.value || 0.25) || 0.25),
      avoidance_threshold_mm: Math.max(10, Math.round(Number($("nav-avoidance-threshold")?.value || 50) || 50)),
      waypoint_ids: [...state.navWaypointDraftIds],
    };
  }

  function closestMapPoint(meta, px, py, radiusPx = 14) {
    const points = currentMapBundle().points || [];
    let bestPoint = null;
    let bestDistance = radiusPx;
    points.forEach((point) => {
      const pixel = worldToPixel(meta, point.x, point.y);
      const distance = Math.hypot(pixel.x - px, pixel.y - py);
      if (distance <= bestDistance) {
        bestDistance = distance;
        bestPoint = point;
      }
    });
    return bestPoint;
  }

  async function selectMap(mapName, force = false) {
    ensureStep9State();
    const normalized = String(mapName || "").trim();
    state.selectedMapName = normalized;
    state.selectedPointId = "";
    state.selectedTaskName = "";
    if (!normalized) {
      syncPointFormWithCurrentMap(true);
      setTaskDraft();
      renderMapsPage();
      renderNavPage(state.backendStatus || {});
      return;
    }
    await loadMapAnnotations(normalized, force);
    await loadNavTasks(normalized, force);
    syncPointFormWithCurrentMap(false);
    setTaskDraft();
    renderMapsPage();
    renderNavPage(state.backendStatus || {});
  }

  async function saveCurrentAnnotations(button = null) {
    ensureStep9State();
    if (!state.selectedMapName) {
      showToast("请先选择地图", true);
      return null;
    }
    if (button) {
      setButtonBusy(button, true);
    }
    try {
      const bundle = currentMapBundle();
      const result = await apiJson(`/api/maps/annotations/${encodeURIComponent(state.selectedMapName)}`, {
        method: "POST",
        body: {
          map_name: state.selectedMapName,
          points: Array.isArray(bundle.points) ? bundle.points : [],
        },
      });
      await loadMapAnnotations(state.selectedMapName, true);
      renderMapsPage();
      renderNavPage(state.backendStatus || {});
      showToast(result.message || "地图标注已保存");
      return result;
    } catch (error) {
      showToast(error.message || "地图标注保存失败", true);
      return null;
    } finally {
      if (button) {
        setButtonBusy(button, false);
      }
    }
  }

  async function saveCurrentTask(button = null) {
    ensureStep9State();
    if (!state.selectedMapName) {
      showToast("请先选择地图", true);
      return null;
    }
    const draft = taskDraftFromForm();
    if (!draft.task_name) {
      showToast("请先填写任务名称", true);
      return null;
    }
    if (!draft.waypoint_ids.length) {
      showToast("任务至少需要一个点位", true);
      return null;
    }
    const bundle = state.navTaskCache[state.selectedMapName] || { map_name: state.selectedMapName, tasks: [] };
    const existingTasks = Array.isArray(bundle.tasks) ? [...bundle.tasks] : [];
    const originalName = String(state.selectedTaskName || "").trim();
    const nameConflict = existingTasks.some((item) => item.task_name === draft.task_name && item.task_name !== originalName);
    if (nameConflict) {
      showToast("任务名称已存在", true);
      return null;
    }
    const nextTasks = existingTasks.filter((item) => item.task_name !== originalName);
    nextTasks.push(draft);

    if (button) {
      setButtonBusy(button, true);
    }
    try {
      const result = await apiJson(`/api/nav/tasks/${encodeURIComponent(state.selectedMapName)}`, {
        method: "POST",
        body: {
          map_name: state.selectedMapName,
          tasks: nextTasks,
        },
      });
      const savedBundle = result.tasks || {
        map_name: state.selectedMapName,
        tasks: nextTasks,
        points: allowedNavPoints(),
      };
      state.navTaskCache[state.selectedMapName] = savedBundle;
      setTaskDraft(draft);
      renderNavPage(state.backendStatus || {});
      showToast(result.message || "任务配置已保存");
      return result;
    } catch (error) {
      showToast(error.message || "任务保存失败", true);
      return null;
    } finally {
      if (button) {
        setButtonBusy(button, false);
      }
    }
  }

  function bindStep9MapEvents() {
    $("maps-list")?.addEventListener("click", async (event) => {
      const card = event.target.closest("[data-map-name]");
      if (!card) {
        return;
      }
      try {
        await selectMap(card.dataset.mapName || "", false);
      } catch (error) {
        showToast(error.message || "地图切换失败", true);
      }
    });

    $("maps-point-list")?.addEventListener("click", (event) => {
      const card = event.target.closest("[data-point-id]");
      if (!card) {
        return;
      }
      selectPoint(card.dataset.pointId || "");
    });

    $("maps-editor-canvas")?.addEventListener("click", (event) => {
      const canvas = event.currentTarget;
      const bundle = currentMapBundle();
      const meta = bundle.map || {};
      if (!canvas || !meta.width_px || !meta.height_px || !canvas.width || !canvas.height) {
        return;
      }
      const rect = canvas.getBoundingClientRect();
      if (!rect.width || !rect.height) {
        return;
      }
      const scaleX = canvas.width / rect.width;
      const scaleY = canvas.height / rect.height;
      const px = (event.clientX - rect.left) * scaleX;
      const py = (event.clientY - rect.top) * scaleY;
      const selected = closestMapPoint(meta, px, py);
      if (selected) {
        selectPoint(selected.id);
        return;
      }
      const world = pixelToWorld(meta, px, py);
      const draft = pointFromForm();
      draft.x = Number(world.x.toFixed(2));
      draft.y = Number(world.y.toFixed(2));
      fillPointForm(draft);
    });

    $("btn-maps-point-new")?.addEventListener("click", () => {
      state.selectedPointId = "";
      syncPointFormWithCurrentMap(true);
      renderMapsPage();
    });

    $("btn-maps-point-save")?.addEventListener("click", () => {
      if (!state.selectedMapName) {
        showToast("请先选择地图", true);
        return;
      }
      const point = pointFromForm();
      const points = [...(currentMapBundle().points || [])];
      const previousPoint = state.selectedPointId
        ? points.find((item) => item.id === state.selectedPointId)
        : null;
      const activeIndex = state.selectedPointId
        ? points.findIndex((item) => item.id === state.selectedPointId)
        : -1;
      const duplicateIndex = points.findIndex((item) => item.id === point.id);
      if (duplicateIndex >= 0 && duplicateIndex !== activeIndex) {
        showToast(`点位 ID 已存在: ${point.id}`, true);
        return;
      }
      if (activeIndex >= 0) {
        points[activeIndex] = point;
      } else {
        points.push(point);
      }
      if (previousPoint && previousPoint.id !== point.id) {
        state.navWaypointDraftIds = state.navWaypointDraftIds.map((item) => (
          item === previousPoint.id ? point.id : item
        ));
      }
      replaceBundlePoints(points);
      state.selectedPointId = point.id;
      fillPointForm(point);
      renderMapsPage();
      renderNavPage(state.backendStatus || {});
      showToast("点位已更新到当前地图，记得保存全部标注");
    });

    $("btn-maps-point-delete")?.addEventListener("click", () => {
      if (!state.selectedMapName) {
        showToast("请先选择地图", true);
        return;
      }
      const pointId = String(state.selectedPointId || $("maps-point-id")?.value || "").trim();
      if (!pointId) {
        showToast("当前没有可删除的点位", true);
        return;
      }
      const currentPoints = currentMapBundle().points || [];
      const points = currentPoints.filter((item) => item.id !== pointId);
      if (points.length === currentPoints.length) {
        showToast("未找到要删除的点位", true);
        return;
      }
      replaceBundlePoints(points);
      state.navWaypointDraftIds = state.navWaypointDraftIds.filter((item) => item !== pointId);
      state.selectedPointId = "";
      syncPointFormWithCurrentMap(points.length === 0);
      renderMapsPage();
      renderNavPage(state.backendStatus || {});
      showToast("点位已从当前地图移除，记得保存全部标注");
    });

    $("btn-maps-annotations-save")?.addEventListener("click", async (event) => {
      await saveCurrentAnnotations(event.currentTarget);
    });
  }

  function bindStep9NavEvents() {
    $("nav-map-select")?.addEventListener("change", async (event) => {
      try {
        await selectMap(event.target.value || "", false);
      } catch (error) {
        showToast(error.message || "地图切换失败", true);
      }
    });

    ["nav-task-name", "nav-task-mode", "nav-task-loop-count", "nav-arrival-tolerance", "nav-avoidance-threshold"]
      .forEach((id) => {
        const element = $(id);
        if (!element) {
          return;
        }
        const eventName = element.tagName === "SELECT" ? "change" : "input";
        element.addEventListener(eventName, () => {
          state.taskFormDirty = true;
        });
      });

    $("nav-task-select")?.addEventListener("change", (event) => {
      state.selectedTaskName = String(event.target.value || "").trim();
    });

    $("btn-nav-task-load")?.addEventListener("click", () => {
      const taskName = String($("nav-task-select")?.value || state.selectedTaskName || "").trim();
      const taskBundle = state.navTaskCache[state.selectedMapName] || { tasks: [] };
      const task = (taskBundle.tasks || []).find((item) => item.task_name === taskName);
      if (!task) {
        showToast("未找到任务配置", true);
        return;
      }
      setTaskDraft(task);
      renderNavPage(state.backendStatus || {});
    });

    $("btn-nav-task-new")?.addEventListener("click", () => {
      setTaskDraft({
        task_name: "",
        mode: "inspect",
        loop_count: 1,
        arrival_tolerance: Number($("nav-arrival-tolerance")?.value || 0.25) || 0.25,
        avoidance_threshold_mm: Number($("nav-avoidance-threshold")?.value || 50) || 50,
        waypoint_ids: [],
      });
      renderNavPage(state.backendStatus || {});
    });

    $("btn-nav-waypoint-add")?.addEventListener("click", () => {
      const waypointId = String($("nav-waypoint-picker")?.value || "").trim();
      if (!waypointId) {
        showToast("请先选择点位", true);
        return;
      }
      state.navWaypointDraftIds.push(waypointId);
      state.taskFormDirty = true;
      renderTaskDraft();
    });

    $("nav-task-waypoints")?.addEventListener("click", (event) => {
      const button = event.target.closest("[data-waypoint-remove]");
      if (!button) {
        return;
      }
      const targetId = String(button.dataset.waypointRemove || "").trim();
      const index = state.navWaypointDraftIds.findIndex((item) => item === targetId);
      if (index < 0) {
        return;
      }
      state.navWaypointDraftIds.splice(index, 1);
      state.taskFormDirty = true;
      renderTaskDraft();
    });

    $("btn-nav-task-save")?.addEventListener("click", async (event) => {
      await saveCurrentTask(event.currentTarget);
    });

    $("btn-nav-localization-start")?.addEventListener("click", async (event) => {
      const mapName = String($("nav-map-select")?.value || state.selectedMapName || "").trim();
      if (!mapName) {
        showToast("请先选择地图", true);
        return;
      }
      await performAction(
        () => apiJson("/api/nav/localization/start", {
          method: "POST",
          body: {
            map_name: mapName,
            port: $("nav-lidar-port")?.value || "",
          },
        }),
        (result) => result?.message || "定位已启动",
        { button: event.currentTarget },
      );
    });

    $("btn-nav-localization-stop")?.addEventListener("click", async (event) => {
      await performAction(
        () => apiJson("/api/nav/localization/stop", { method: "POST" }),
        (result) => result?.message || "定位已停止",
        { button: event.currentTarget },
      );
    });

    $("btn-nav-goal-start")?.addEventListener("click", async (event) => {
      const mapName = String($("nav-map-select")?.value || state.selectedMapName || "").trim();
      const waypointId = String($("nav-goal-point")?.value || "").trim();
      if (!mapName) {
        showToast("请先选择地图", true);
        return;
      }
      if (!waypointId) {
        showToast("请先选择单点目标", true);
        return;
      }
      await performAction(
        () => apiJson("/api/nav/goal/start", {
          method: "POST",
          body: {
            map_name: mapName,
            port: $("nav-lidar-port")?.value || "",
            waypoint_id: waypointId,
            arrival_tolerance: Math.max(0.05, Number($("nav-arrival-tolerance")?.value || 0.25) || 0.25),
            avoidance_threshold_mm: Math.max(10, Math.round(Number($("nav-avoidance-threshold")?.value || 50) || 50)),
          },
        }),
        (result) => result?.message || "单点导航已启动",
        { button: event.currentTarget },
      );
    });

    $("btn-nav-goal-cancel")?.addEventListener("click", async (event) => {
      await performAction(
        () => apiJson("/api/nav/goal/cancel", { method: "POST" }),
        (result) => result?.message || "单点导航已取消",
        { button: event.currentTarget },
      );
    });

    $("btn-nav-task-start")?.addEventListener("click", async (event) => {
      const mapName = String($("nav-map-select")?.value || state.selectedMapName || "").trim();
      const taskName = String($("nav-task-select")?.value || state.selectedTaskName || "").trim();
      const taskBundle = state.navTaskCache[mapName] || { tasks: [] };
      const task = (taskBundle.tasks || []).find((item) => item.task_name === taskName);
      if (!mapName) {
        showToast("请先选择地图", true);
        return;
      }
      if (!task) {
        showToast("请先保存并选择要执行的任务", true);
        return;
      }
      if (state.taskFormDirty && String($("nav-task-name")?.value || "").trim() === taskName) {
        showToast("当前任务有未保存修改，请先保存", true);
        return;
      }
      await performAction(
        () => apiJson("/api/nav/task/start", {
          method: "POST",
          body: {
            map_name: mapName,
            task_name: taskName,
            port: $("nav-lidar-port")?.value || "",
          },
        }),
        (result) => result?.message || "任务已启动",
        { button: event.currentTarget },
      );
    });

    $("btn-nav-task-stop")?.addEventListener("click", async (event) => {
      await performAction(
        () => apiJson("/api/nav/task/stop", { method: "POST" }),
        (result) => result?.message || "任务已停止",
        { button: event.currentTarget },
      );
    });
  }

  async function refreshMaps(force = false) {
    ensureStep9State();
    if (!force && state.activeTab !== "maps" && state.activeTab !== "nav") {
      return;
    }
    if (state.mapsPending && !force) {
      return;
    }
    state.mapsPending = true;
    try {
      const result = await apiJson("/api/ros/maps");
      state.cachedMaps = Array.isArray(result.maps) ? result.maps : [];
      if (!state.selectedMapName || !state.cachedMaps.some((item) => item.name === state.selectedMapName)) {
        state.selectedMapName = state.cachedMaps[0]?.name || "";
      }
      if (state.selectedMapName) {
        await loadMapAnnotations(state.selectedMapName, force);
        await loadNavTasks(state.selectedMapName, force);
      }
      renderMapsPage();
      if (state.backendStatus) {
        renderNavPage(state.backendStatus);
      }
    } catch (error) {
      if (force || state.activeTab === "maps" || state.activeTab === "nav") {
        showToast(error.message || "地图数据刷新失败", true);
      }
    } finally {
      state.mapsPending = false;
    }
  }

  async function refreshBackendStatus(force = false) {
    ensureStep9State();
    if (state.backendStatusPending && !force) {
      return state.backendStatus;
    }
    state.backendStatusPending = true;
    try {
      const data = await apiJson("/api/status");
      state.lastStatusError = "";
      state.backendStatus = data;
      if (!state.rosStatus || state.activeTab !== "ros") {
        state.rosStatus = data.ros || state.rosStatus;
      }
      renderSidebar(data);
      renderHome(data);
      renderControlPage(data);
      renderVisionPage(data);
      renderRosPage(data);
      renderAvoidancePage(data);
      await refreshMaps(force || state.activeTab === "maps" || state.activeTab === "nav");
      renderNavPage(data);
      if (!state.lastLogsText && Array.isArray(data.logs)) {
        renderLogsPage(data.logs);
      }
      return data;
    } catch (error) {
      const message = error.message || "后端状态刷新失败";
      if (state.lastStatusError !== message) {
        showToast(message, true);
      }
      state.lastStatusError = message;
      return null;
    } finally {
      state.backendStatusPending = false;
    }
  }

  function initTabs() {
    ensureStep9State();
    document.querySelectorAll(".tab").forEach((button) => {
      button.addEventListener("click", async () => {
        const tabName = button.dataset.tab || "home";
        state.activeTab = tabName;
        document.querySelectorAll(".tab").forEach((item) => item.classList.toggle("active", item === button));
        TAB_NAMES.forEach((name) => {
          $(`tab-${name}`)?.classList.toggle("active", name === tabName);
        });
        if (tabName === "vision") {
          await refreshCameraList();
        }
        if (tabName === "ros") {
          await refreshRosStatus(true);
          await refreshMaps(false);
        }
        if (tabName === "maps" || tabName === "nav") {
          await refreshMaps(false);
        }
        if (tabName === "logs") {
          await refreshLogs(true);
        }
      });
    });
  }

  async function init() {
    ensureStep9State();
    ensureServoRows();
    syncPointFormWithCurrentMap(true);
    setTaskDraft();
    initTabs();
    bindMotionButtons();
    bindServoEvents();
    bindStep9MapEvents();
    initControls();
    bindRosControls();
    bindStep9NavEvents();

    await refreshBackendStatus(true);
    await refreshCameraList();
    await refreshMaps(true);
    await refreshRosStatus(true);
    await refreshLogs(true);

    window.setInterval(() => {
      if (state.activeTab === "ros") {
        refreshRosStatus();
      } else {
        refreshBackendStatus();
      }
    }, 2000);
    window.setInterval(() => {
      refreshMaps(false);
    }, 8000);
    window.setInterval(() => {
      refreshLogs(false);
    }, 5000);
  }

  document.addEventListener("DOMContentLoaded", init);
})();
