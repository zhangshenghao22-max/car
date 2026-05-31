(() => {
  const TEXT_STORAGE_KEY = "carCloudSandboxDataTextOverrides";
  const CLASS_STORAGE_KEY = "carCloudSandboxDataClassOverrides";
  const editorState = {
    activeId: "",
    input: null,
    originalText: "",
  };

  const EDITABLE_TEXT_IDS = [
    "cabinet-last-seen",
    "cabinet-updated-at",
    "data-page-meta",
    "voltage-status-value",
    "voltage-status-text",
    "current-status-value",
    "current-status-text",
    "env-status-value",
    "env-status-text",
    "voltage-value",
    "voltage-state",
    "current-value",
    "current-state",
    "temp-pv",
    "temp-sv",
    "temp-controller-state",
    "motor1-start-text",
    "motor1-stop-text",
    "motor2-start-text",
    "motor2-stop-text",
    "high-voltage-text",
    "door-state",
    "smoke-value",
    "smoke-status",
    "hydrogen-value",
    "hydrogen-status",
    "carbon-monoxide-value",
    "carbon-monoxide-status",
    "temperature-humidity-value",
    "temperature-humidity-status",
    "infrared-temperature-value",
    "infrared-temperature-status",
    "sound-level-value",
    "sound-level-status",
  ];

  const TOGGLE_CARD_IDS = [
    "motor1-start-card",
    "motor1-stop-card",
    "motor2-start-card",
    "motor2-stop-card",
  ];

  function loadJson(key) {
    try {
      const raw = window.localStorage.getItem(key);
      const parsed = raw ? JSON.parse(raw) : {};
      return parsed && typeof parsed === "object" ? parsed : {};
    } catch (error) {
      return {};
    }
  }

  function saveJson(key, value) {
    try {
      window.localStorage.setItem(key, JSON.stringify(value));
    } catch (error) {
      // Keep the page editable even if localStorage is blocked.
    }
  }

  function forceOnline() {
    setText("cabinet-online", "在线");
    const online = $("cabinet-online");
    if (online) {
      online.classList.remove("tone-warn", "tone-bad", "tone-unknown");
      online.classList.add("tone-ok");
    }
  }

  function applyTextOverrides() {
    if (editorState.activeId) {
      forceOnline();
      return;
    }
    const overrides = loadJson(TEXT_STORAGE_KEY);
    for (const [id, text] of Object.entries(overrides)) {
      if (id === "cabinet-online") continue;
      const element = $(id);
      if (element) element.textContent = String(text);
    }
    forceOnline();
  }

  function rememberText(id, text) {
    if (id === "cabinet-online") return;
    const overrides = loadJson(TEXT_STORAGE_KEY);
    overrides[id] = text;
    saveJson(TEXT_STORAGE_KEY, overrides);
  }

  function applyClassOverrides() {
    if (editorState.activeId) return;
    const overrides = loadJson(CLASS_STORAGE_KEY);
    for (const id of TOGGLE_CARD_IDS) {
      const card = $(id);
      if (!card || !(id in overrides)) continue;
      card.classList.toggle("is-active", Boolean(overrides[id]));
    }
    for (const containerId of ["motor1-mode", "motor2-mode"]) {
      const activeMode = overrides[containerId];
      const container = $(containerId);
      if (!container || !activeMode) continue;
      container.querySelectorAll("span").forEach((item) => {
        item.classList.toggle("is-active", item.dataset.mode === activeMode);
      });
    }
  }

  function rememberClass(id, value) {
    const overrides = loadJson(CLASS_STORAGE_KEY);
    overrides[id] = value;
    saveJson(CLASS_STORAGE_KEY, overrides);
  }

  function editableTarget(target) {
    return Boolean(target?.closest?.(".sandbox-inline-editor"));
  }

  function closeEditor(commit) {
    const input = editorState.input;
    const id = editorState.activeId;
    if (!input || !id) return;

    const element = $(id);
    const nextText = input.value;
    input.remove();
    editorState.activeId = "";
    editorState.input = null;
    editorState.originalText = "";

    if (!element) return;
    element.textContent = commit ? nextText : editorState.originalText;
    if (commit) rememberText(id, nextText);
    applyTextOverrides();
    applyClassOverrides();
  }

  function openEditor(element) {
    if (!element || !element.id || element.id === "cabinet-online") return;
    if (editorState.activeId === element.id) return;
    closeEditor(true);

    const currentText = element.textContent || "";
    const input = document.createElement("input");
    input.className = "sandbox-inline-editor";
    input.type = "text";
    input.value = currentText;
    input.style.boxSizing = "border-box";
    input.style.width = "100%";
    input.style.maxWidth = "100%";
    input.style.minWidth = "6em";
    input.style.font = "inherit";
    input.style.fontWeight = "inherit";
    input.style.color = "inherit";
    input.style.background = "rgba(255, 255, 255, 0.92)";
    input.style.border = "1px solid rgba(20, 30, 40, 0.28)";
    input.style.borderRadius = "6px";
    input.style.padding = "2px 6px";

    editorState.activeId = element.id;
    editorState.input = input;
    editorState.originalText = currentText;

    element.textContent = "";
    element.appendChild(input);
    input.focus();
    input.select();

    input.addEventListener("click", (event) => event.stopPropagation());
    input.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        closeEditor(true);
      } else if (event.key === "Escape") {
        event.preventDefault();
        const original = editorState.originalText;
        const id = editorState.activeId;
        input.remove();
        editorState.activeId = "";
        editorState.input = null;
        editorState.originalText = "";
        const target = $(id);
        if (target) target.textContent = original;
        applyTextOverrides();
        applyClassOverrides();
      }
    });
    input.addEventListener("blur", () => closeEditor(true));
  }

  function installTextEditing() {
    for (const id of EDITABLE_TEXT_IDS) {
      const element = $(id);
      if (!element || element.dataset.sandboxEditable === "1") continue;
      element.dataset.sandboxEditable = "1";
      element.addEventListener("click", (event) => {
        event.preventDefault();
        event.stopPropagation();
        openEditor(element);
      });
    }
  }

  function installClassEditing() {
    for (const id of TOGGLE_CARD_IDS) {
      const card = $(id);
      if (!card || card.dataset.sandboxToggleReady === "1") continue;
      card.dataset.sandboxToggleReady = "1";
      card.addEventListener("click", (event) => {
        if (editableTarget(event.target)) return;
        const active = !card.classList.contains("is-active");
        card.classList.toggle("is-active", active);
        rememberClass(id, active);
      });
    }

    for (const containerId of ["motor1-mode", "motor2-mode"]) {
      const container = $(containerId);
      if (!container || container.dataset.sandboxSelectorReady === "1") continue;
      container.dataset.sandboxSelectorReady = "1";
      container.querySelectorAll("span").forEach((item) => {
        item.addEventListener("click", () => {
          container.querySelectorAll("span").forEach((candidate) => candidate.classList.remove("is-active"));
          item.classList.add("is-active");
          rememberClass(containerId, item.dataset.mode || "");
        });
      });
    }
  }

  function installEditing() {
    installTextEditing();
    installClassEditing();
    applyTextOverrides();
    applyClassOverrides();
  }

  if (typeof renderCabinet === "function") {
    const originalRenderCabinet = renderCabinet;
    renderCabinet = function renderCabinetSandboxAware(payload) {
      if (editorState.activeId) {
        forceOnline();
        return;
      }
      originalRenderCabinet(payload);
      forceOnline();
      installEditing();
    };
  }

  if (typeof fetchCabinetData === "function") {
    const originalFetchCabinetData = fetchCabinetData;
    fetchCabinetData = function fetchCabinetDataSandboxAware() {
      if (editorState.activeId) return Promise.resolve();
      return originalFetchCabinetData();
    };
  }

  window.addEventListener("load", () => {
    window.setTimeout(installEditing, 0);
    window.setInterval(() => {
      forceOnline();
      if (!editorState.activeId) {
        applyTextOverrides();
        applyClassOverrides();
      }
    }, 500);
  });
})();
