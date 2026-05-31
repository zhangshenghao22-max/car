(() => {
  const selected = {
    mode: "",
    item: null,
    localUrl: "",
    localName: "",
  };

  function imageUrlForItem(item) {
    const annotated = item?.annotated_image || {};
    if (!annotated.url) return "";
    return `${annotated.url}?ts=${encodeURIComponent(item.uploaded_at || "")}`;
  }

  function showSelectedImage() {
    const image = $("recognition-image");
    const placeholder = $("recognition-placeholder");
    const summary = $("recognition-summary");
    const pill = $("recognition-pill");
    if (!image || !placeholder || !summary || !pill) return false;

    if (selected.mode === "local" && selected.localUrl) {
      image.src = selected.localUrl;
      image.style.display = "block";
      placeholder.style.display = "none";
      summary.textContent = selected.localName || "本地照片";
      pill.textContent = "本地照片";
      return true;
    }

    if (selected.mode === "history" && selected.item) {
      const url = imageUrlForItem(selected.item);
      if (!url) return false;
      image.src = url;
      image.style.display = "block";
      placeholder.style.display = "none";
      summary.textContent = detectionSummary(selected.item);
      pill.textContent = Number(selected.item.detection_count || 0) > 0
        ? `识别到 ${Number(selected.item.detection_count || 0)} 个目标`
        : "未识别到目标";
      return true;
    }

    return false;
  }

  if (typeof renderLatest === "function") {
    const originalRenderLatest = renderLatest;
    renderLatest = function renderLatestSandboxAware() {
      if (showSelectedImage()) return;
      originalRenderLatest();
    };
  }

  if (typeof sendRecognitionCommand === "function") {
    const originalSendRecognitionCommand = sendRecognitionCommand;
    sendRecognitionCommand = function sendRecognitionCommandSandboxAware() {
      selected.mode = "";
      selected.item = null;
      return originalSendRecognitionCommand();
    };
  }

  function findHistoryItemFromCard(card) {
    const link = card?.querySelector("a");
    const href = link?.getAttribute("href") || "";
    if (!href || typeof recognitionHistory !== "function") return null;
    return recognitionHistory().find((item) => item?.annotated_image?.url === href) || null;
  }

  function installHistoryPicker() {
    const target = $("recognition-history");
    if (!target || target.dataset.sandboxPickerReady === "1") return;
    target.dataset.sandboxPickerReady = "1";
    target.addEventListener("click", (event) => {
      const card = event.target.closest(".recognition-history-card");
      if (!card) return;
      const item = findHistoryItemFromCard(card);
      if (!item) return;
      event.preventDefault();
      selected.mode = "history";
      selected.item = item;
      if (selected.localUrl) {
        URL.revokeObjectURL(selected.localUrl);
        selected.localUrl = "";
      }
      showSelectedImage();
    });
  }

  function installLocalPicker() {
    const shell = document.querySelector(".recognition-frame-shell");
    if (!shell || shell.dataset.sandboxLocalReady === "1") return;
    shell.dataset.sandboxLocalReady = "1";

    const input = document.createElement("input");
    input.type = "file";
    input.accept = "image/*";
    input.hidden = true;
    document.body.appendChild(input);

    shell.addEventListener("click", () => {
      input.value = "";
      input.click();
    });

    input.addEventListener("change", () => {
      const file = input.files && input.files[0];
      if (!file) return;
      if (selected.localUrl) URL.revokeObjectURL(selected.localUrl);
      selected.mode = "local";
      selected.item = null;
      selected.localUrl = URL.createObjectURL(file);
      selected.localName = file.name || "本地照片";
      showSelectedImage();
    });
  }

  window.addEventListener("load", () => {
    installHistoryPicker();
    installLocalPicker();
  });
})();
