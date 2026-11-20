const elements = {
  serviceList: document.getElementById("serviceList"),
  serviceCount: document.getElementById("serviceCount"),
  refreshAll: document.getElementById("refreshAll"),
  statusBadge: document.getElementById("statusBadge"),
  statusText: document.getElementById("statusText"),
  webVisualizeButton: document.getElementById("webVisualizeButton"),
  webvizSection: document.getElementById("webvizSection"),
  webvizCanvas: document.getElementById("webvizCanvas"),
  webvizStatus: document.getElementById("webvizStatus"),
  webvizLidarSelect: document.getElementById("webvizLidarSelect"),
  webvizDefaultView: document.getElementById("webvizDefaultView"),
  webvizSaveView: document.getElementById("webvizSaveView"),
  webvizReconnect: document.getElementById("webvizReconnect"),
  webvizClose: document.getElementById("webvizClose"),
  webvizTopic: document.getElementById("webvizTopic"),
  progressSection: document.getElementById("progressSection"),
  progressText: document.getElementById("progressText"),
  progressPercent: document.getElementById("progressPercent"),
  progressBar: document.getElementById("progressBar"),
  progressMeta: document.getElementById("progressMeta"),
  plcStatusCard: document.getElementById("plcStatusCard"),
  plcStatusGrid: document.getElementById("plcStatusGrid"),
  rslidarStatusCard: document.getElementById("rslidarStatusCard"),
  rslidarStatusSummary: document.getElementById("rslidarStatusSummary"),
  rslidarStatusGrid: document.getElementById("rslidarStatusGrid"),
  statusMeta: document.getElementById("statusMeta"),
  enableToggle: document.getElementById("enableToggle"),
  configEditor: document.getElementById("configEditor"),
  configPath: document.getElementById("configPath"),
  restartAfterSave: document.getElementById("restartAfterSave"),
  saveConfig: document.getElementById("saveConfig"),
  openLogs: document.getElementById("openLogs"),
  closeLogs: document.getElementById("closeLogs"),
  logModal: document.getElementById("logModal"),
  logBackdrop: document.getElementById("logBackdrop"),
  logLines: document.getElementById("logLines"),
  refreshLogs: document.getElementById("refreshLogs"),
  logViewer: document.getElementById("logViewer"),
  toast: document.getElementById("toast"),
};

const state = {
  services: [],
  currentServiceId: null,
  currentStatus: null,
  currentProgress: null,
  webviz: {
    ros: null,
    scene: null,
    renderer: null,
    camera: null,
    controls: null,
    animationFrame: null,
    pointTopic: null,
    pointCloudObject: null,
    protectionTopic: null,
    protectionGroup: null,
    activeLidar: "down",
    resizeHandler: null,
    lastPointCount: 0,
    cameraTarget: null,
    shouldAutoFocus: true,
  },
  plcRos: null,
  plcPoseTopic: null,
  plcPose: null,
  rslidar: {
    ros: null,
    topics: new Map(),
    statuses: {},
  },
};

const WEBVIZ_LIDARS = ["down", "front", "left", "right"];
const WEBVIZ_DEFAULT_VIEW_KEY = "collision-guard-webviz-default-view";
const DEFAULT_WEBVIZ_POINT_TOPIC_TEMPLATE = "/collision/{lidar}/processed_points";
const RSLIDAR_STATUS_TIMEOUT_MS = 4000;

function loadSavedDefaultView() {
  try {
    const raw = window.localStorage.getItem(WEBVIZ_DEFAULT_VIEW_KEY);
    if (!raw) {
      return null;
    }
    const parsed = JSON.parse(raw);
    if (!parsed?.position || !parsed?.target) {
      return null;
    }
    return parsed;
  } catch {
    return null;
  }
}

function saveDefaultViewConfig(view) {
  try {
    window.localStorage.setItem(WEBVIZ_DEFAULT_VIEW_KEY, JSON.stringify(view));
  } catch {
  }
}

async function request(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.error || `Request failed: ${response.status}`);
  }
  return data;
}

function showToast(message, isError = false) {
  elements.toast.textContent = message;
  elements.toast.className = `toast ${isError ? "error" : ""}`;
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => {
    elements.toast.className = "toast hidden";
  }, 2500);
}

function currentServiceId() {
  return state.currentServiceId;
}

function renderServices(services) {
  state.services = services;
  if (!state.currentServiceId && services.length > 0) {
    state.currentServiceId = services[0].id;
  }
  elements.serviceCount.textContent = String(services.length);
  elements.serviceList.innerHTML = services
    .map(
      (service) =>
        `<button class="service-item ${service.id === state.currentServiceId ? "active" : ""}" type="button" data-service-id="${service.id}"><strong>${service.display_name}</strong><span>${service.unit}</span></button>`,
    )
    .join("");
}

function renderStatus(status) {
  state.currentServiceId = status.id;
  state.currentStatus = status;
  elements.statusBadge.textContent = status.active_state;
  elements.statusBadge.dataset.state = status.active_state;
  elements.statusText.textContent = status.sub_state || status.active_state;
  elements.webVisualizeButton.classList.toggle("hidden", !status.web_visualization_unit);
  renderWebvizLidarOptions(status);
  elements.enableToggle.checked = Boolean(status.enabled);
  elements.restartAfterSave.checked = Boolean(status.restart_after_save);
  elements.progressSection.classList.toggle("hidden", !status.progress_path);
  elements.plcStatusCard.classList.toggle("hidden", status.id !== "plc-interface");
  elements.rslidarStatusCard.classList.toggle("hidden", status.id !== "rslidar-sdk");
  const fields = [
    ["服务名", status.display_name],
    ["Unit", status.unit],
    ["描述", status.description || "-"],
    ["LoadState", status.load_state],
    ["SubState", status.sub_state],
    ["UnitFileState", status.unit_file_state],
    ["Main PID", status.main_pid],
    ["启动时间", status.started_at || "-"],
  ];
  elements.statusMeta.innerHTML = fields
    .map(([label, value]) => `<div><dt>${label}</dt><dd>${value}</dd></div>`)
    .join("");
  elements.configPath.textContent = status.config_path || "未配置 config_path";

  if (!status.web_visualization_unit) {
    closeWebviz();
  }
  if (status.id === "plc-interface") {
    ensurePlcPoseSubscription();
  } else {
    disconnectPlcPoseSubscription();
  }
  if (status.id === "rslidar-sdk") {
    ensureRslidarStatusSubscription();
  } else {
    disconnectRslidarStatusSubscription();
  }
}

function parsePlcConfig(content) {
  const matchValue = (key) => {
    const pattern = new RegExp(`${key}:\\s*"?([^"\\n]+)"?`);
    return content.match(pattern)?.[1]?.trim() || "-";
  };

  return {
    plcIp: matchValue("plc_ip"),
    readPort: matchValue("plc_read_port"),
    writePort: matchValue("plc_write_port"),
    registerCount: matchValue("read_register_count"),
  };
}

function renderPlcStatus(configText) {
  if (state.currentServiceId !== "plc-interface") {
    elements.plcStatusGrid.innerHTML = "";
    return;
  }
  const plc = parsePlcConfig(configText || "");
  const fields = [
    ["PLC IP", plc.plcIp],
    ["读端口", plc.readPort],
    ["写端口", plc.writePort],
    ["读取寄存器数", plc.registerCount],
    ["行走距离", formatPoseValue(state.plcPose?.walking_dist)],
    ["俯仰角", formatPoseValue(state.plcPose?.luffing_angle)],
    ["回转角", formatPoseValue(state.plcPose?.slewing_angle)],
    ["伸缩距离", formatPoseValue(state.plcPose?.stretch_dist)],
  ];
  elements.plcStatusGrid.innerHTML = fields
    .map(
      ([label, value]) =>
        `<article class="plc-item"><span class="plc-label">${label}</span><strong class="plc-value">${value}</strong></article>`,
    )
    .join("");
}

function formatPoseValue(value) {
  if (typeof value !== "number" || Number.isNaN(value)) {
    return "--";
  }
  return value.toFixed(3);
}

function disconnectPlcPoseSubscription() {
  if (state.plcPoseTopic) {
    state.plcPoseTopic.unsubscribe();
    state.plcPoseTopic = null;
  }
  if (state.plcRos) {
    state.plcRos.close();
    state.plcRos = null;
  }
  state.plcPose = null;
}

function ensurePlcPoseSubscription() {
  if (state.currentServiceId !== "plc-interface") {
    return;
  }
  if (!window.ROSLIB || state.plcPoseTopic) {
    return;
  }

  const ros = new ROSLIB.Ros({ url: webvizUrl() });
  state.plcRos = ros;

  ros.on("connection", () => {
    const topic = new ROSLIB.Topic({
      ros,
      name: "/plc/ship_loader_pos",
      messageType: "custom_msgs/ShipLoaderPose",
      throttle_rate: 200,
    });
    state.plcPoseTopic = topic;
    topic.subscribe((message) => {
      state.plcPose = message;
      renderPlcStatus(elements.configEditor.value);
    });
  });

  ros.on("error", () => {
    state.plcPose = null;
    renderPlcStatus(elements.configEditor.value);
  });

  ros.on("close", () => {
    state.plcPoseTopic = null;
  });
}

function renderProgress(progress) {
  state.currentProgress = progress;
  const percent = Math.max(0, Math.min(Number(progress.progress || 0), 100));
  const totalLidars = Number(progress.total_lidars || 0);
  const completedLidars = Number(progress.completed_lidars || 0);
  elements.progressText.textContent = progress.message || "未开始";
  elements.progressPercent.textContent = `${percent}%`;
  elements.progressBar.style.width = `${percent}%`;
  elements.progressSection.dataset.state = progress.state || "idle";

  if (!totalLidars) {
    elements.progressMeta.textContent = "等待校准任务启动";
    return;
  }

  elements.progressMeta.textContent = `已完成 ${completedLidars}/${totalLidars} 个雷达，帧进度 ${progress.current_frames || 0}/${progress.total_frames || 0}`;
}

function renderConfig(config) {
  elements.configEditor.value = config.content;
  renderPlcStatus(config.content);
  renderRslidarStatusFromConfig(config.content);
  if (state.currentServiceId === "rslidar-sdk" && state.rslidar.ros) {
    subscribeRslidarStatusTopics(parseRslidarConfig(config.content));
  }
}

function renderLogs(logs) {
  elements.logViewer.textContent = logs.content || "暂无日志";
  elements.logViewer.scrollTop = elements.logViewer.scrollHeight;
}

function updateWebvizStatus(message, isError = false) {
  elements.webvizStatus.textContent = message;
  elements.webvizStatus.classList.toggle("webviz-error", isError);
}

function renderWebvizLidarOptions(status) {
  const lidars = status.webviz_lidars?.length ? status.webviz_lidars : WEBVIZ_LIDARS;
  const activeLidar = lidars.includes(state.webviz.activeLidar)
    ? state.webviz.activeLidar
    : lidars[0];
  elements.webvizLidarSelect.innerHTML = lidars
    .map((lidar) => `<option value="${lidar}">${lidar}</option>`)
    .join("");
  elements.webvizLidarSelect.value = activeLidar;
  state.webviz.activeLidar = activeLidar;
  updateWebvizTopicLabel();
}

function formatTopic(template, lidar) {
  return template.replaceAll("{lidar}", lidar);
}

function webvizPointTopicName(lidar) {
  return formatTopic(
    state.currentStatus?.webviz_point_topic_template || DEFAULT_WEBVIZ_POINT_TOPIC_TEMPLATE,
    lidar,
  );
}

function webvizProtectionTopicName(lidar) {
  const template = state.currentStatus?.webviz_protection_topic_template;
  return template ? formatTopic(template, lidar) : null;
}

function updateWebvizTopicLabel() {
  if (!elements.webvizTopic) {
    return;
  }
  elements.webvizTopic.textContent = `PointCloud2: ${webvizPointTopicName(state.webviz.activeLidar)}`;
}

function parseRslidarConfig(content) {
  const lidars = [];
  const blocks = content.split(/\n\s*-\s+driver:/).slice(1);
  for (const block of blocks) {
    const topic = block.match(/ros_send_point_cloud_topic:\s*([^\n#]+)/)?.[1]?.trim() || "";
    const msopPort = block.match(/msop_port:\s*([^\n#]+)/)?.[1]?.trim() || "-";
    const difopPort = block.match(/difop_port:\s*([^\n#]+)/)?.[1]?.trim() || "-";
    const lidarType = block.match(/lidar_type:\s*([^\n#]+)/)?.[1]?.trim() || "-";
    const name = topic.split("/").filter(Boolean).pop() || `lidar-${lidars.length + 1}`;
    lidars.push({
      name,
      topic,
      msopPort,
      difopPort,
      lidarType,
    });
  }
  return lidars;
}

function topicRateLabel(timestamp) {
  if (!timestamp) {
    return "--";
  }
  const ageMs = Date.now() - timestamp;
  return `${(ageMs / 1000).toFixed(1)} 秒前`;
}

function rslidarRuntimeState(item) {
  if (!item.lastMessageAt) {
    return { key: "offline", label: "离线" };
  }
  const ageMs = Date.now() - item.lastMessageAt;
  if (ageMs <= RSLIDAR_STATUS_TIMEOUT_MS) {
    return { key: "online", label: "在线" };
  }
  return { key: "waiting", label: "等待数据" };
}

function renderRslidarStatusCards(lidars) {
  if (state.currentServiceId !== "rslidar-sdk") {
    elements.rslidarStatusGrid.innerHTML = "";
    elements.rslidarStatusSummary.textContent = "未连接";
    return;
  }
  if (!lidars.length) {
    elements.rslidarStatusGrid.innerHTML = "";
    elements.rslidarStatusSummary.textContent = "配置中未解析到雷达项";
    return;
  }

  const onlineCount = lidars.filter((item) => rslidarRuntimeState(item).key === "online").length;
  elements.rslidarStatusSummary.textContent = `共 ${lidars.length} 个雷达，在线 ${onlineCount} 个`;
  elements.rslidarStatusGrid.innerHTML = lidars
    .map((item) => {
      const runtime = rslidarRuntimeState(item);
      return `
        <article class="rslidar-item" data-state="${runtime.key}">
          <div class="rslidar-head">
            <strong>${item.name}</strong>
            <span class="rslidar-state">${runtime.label}</span>
          </div>
          <div class="rslidar-meta">
            <div class="rslidar-row"><span>Topic</span><span>${item.topic || "-"}</span></div>
            <div class="rslidar-row"><span>点数</span><span>${item.lastPointCount ?? "--"}</span></div>
            <div class="rslidar-row"><span>更新</span><span>${topicRateLabel(item.lastMessageAt)}</span></div>
            <div class="rslidar-row"><span>型号</span><span>${item.lidarType}</span></div>
            <div class="rslidar-row"><span>MSOP</span><span>${item.msopPort}</span></div>
            <div class="rslidar-row"><span>DIFOP</span><span>${item.difopPort}</span></div>
          </div>
        </article>
      `;
    })
    .join("");
}

function mergedRslidarItems(configItems = []) {
  return configItems.map((item) => ({
    ...item,
    ...(state.rslidar.statuses[item.name] || {}),
  }));
}

function renderRslidarStatusFromConfig(content) {
  const items = mergedRslidarItems(parseRslidarConfig(content || ""));
  renderRslidarStatusCards(items);
}

function updateRslidarStatus(name, message) {
  state.rslidar.statuses[name] = {
    lastMessageAt: Date.now(),
    lastPointCount: estimatePointCount(message),
  };
  renderRslidarStatusFromConfig(elements.configEditor.value);
}

function subscribeRslidarStatusTopics(items) {
  if (!state.rslidar.ros) {
    return;
  }
  for (const topic of state.rslidar.topics.values()) {
    topic.unsubscribe();
  }
  state.rslidar.topics.clear();
  items.forEach((item) => {
    if (!item.topic) {
      return;
    }
    const topic = new ROSLIB.Topic({
      ros: state.rslidar.ros,
      name: item.topic,
      messageType: "sensor_msgs/PointCloud2",
      throttle_rate: 500,
    });
    topic.subscribe((message) => updateRslidarStatus(item.name, message));
    state.rslidar.topics.set(item.name, topic);
  });
}

function estimatePointCount(message) {
  const width = Number(message?.width || 0);
  const height = Number(message?.height || 1);
  if (width > 0) {
    return width * Math.max(height, 1);
  }
  const data = decodePointCloudData(message?.data);
  const pointStep = Number(message?.point_step || 0);
  if (!pointStep) {
    return "--";
  }
  return Math.floor(data.length / pointStep);
}

function disconnectRslidarStatusSubscription() {
  for (const topic of state.rslidar.topics.values()) {
    topic.unsubscribe();
  }
  state.rslidar.topics.clear();
  if (state.rslidar.ros) {
    state.rslidar.ros.close();
    state.rslidar.ros = null;
  }
  state.rslidar.statuses = {};
  if (state.currentServiceId !== "rslidar-sdk") {
    elements.rslidarStatusGrid.innerHTML = "";
    elements.rslidarStatusSummary.textContent = "未连接";
  }
}

async function ensureRslidarStatusSubscription() {
  if (state.currentServiceId !== "rslidar-sdk") {
    return;
  }
  const bridgeUnit = state.currentStatus?.status_bridge_unit;
  if (!bridgeUnit) {
    elements.rslidarStatusSummary.textContent = "未配置状态桥接服务";
    return;
  }
  if (!window.ROSLIB) {
    elements.rslidarStatusSummary.textContent = "ROSLIB 未加载";
    return;
  }
  if (state.rslidar.ros) {
    renderRslidarStatusFromConfig(elements.configEditor.value);
    return;
  }

  elements.rslidarStatusSummary.textContent = "正在连接状态桥接...";
  const ros = new ROSLIB.Ros({ url: webvizUrl() });
  state.rslidar.ros = ros;

  ros.on("connection", () => {
    const items = parseRslidarConfig(elements.configEditor.value);
    subscribeRslidarStatusTopics(items);
    renderRslidarStatusFromConfig(elements.configEditor.value);
  });

  ros.on("error", () => {
    elements.rslidarStatusSummary.textContent = "无法连接状态桥接，请确认 rosbridge 已启动";
    renderRslidarStatusFromConfig(elements.configEditor.value);
  });

  ros.on("close", () => {
    state.rslidar.topics.clear();
    state.rslidar.ros = null;
    renderRslidarStatusFromConfig(elements.configEditor.value);
  });
}

function webvizUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname}:9090`;
}

function clearProtectionGroup() {
  if (!state.webviz.protectionGroup || !state.webviz.scene) {
    return;
  }
  state.webviz.scene.remove(state.webviz.protectionGroup);
  state.webviz.protectionGroup = null;
}

function clearPointCloudObject() {
  if (!state.webviz.pointCloudObject || !state.webviz.scene) {
    return;
  }
  state.webviz.scene.remove(state.webviz.pointCloudObject);
  state.webviz.pointCloudObject.geometry?.dispose?.();
  state.webviz.pointCloudObject.material?.dispose?.();
  state.webviz.pointCloudObject = null;
  state.webviz.lastPointCount = 0;
}

function vectorToObject(vector) {
  return { x: vector.x, y: vector.y, z: vector.z };
}

function defaultViewConfig() {
  return (
    loadSavedDefaultView() || {
      position: { x: 20, y: -20, z: 12 },
      target: { x: 0, y: 0, z: 0 },
    }
  );
}

function applyViewConfig(view) {
  if (!state.webviz.camera || !state.webviz.controls || !window.THREE) {
    return;
  }
  const position = view?.position || { x: 20, y: -20, z: 12 };
  const target = view?.target || { x: 0, y: 0, z: 0 };
  state.webviz.camera.position.set(position.x, position.y, position.z);
  state.webviz.controls.target.set(target.x, target.y, target.z);
  state.webviz.camera.lookAt(new THREE.Vector3(target.x, target.y, target.z));
  state.webviz.controls.update();
}

function currentViewConfig() {
  if (!state.webviz.camera || !state.webviz.controls) {
    return null;
  }
  return {
    position: vectorToObject(state.webviz.camera.position),
    target: vectorToObject(state.webviz.controls.target),
  };
}

function focusWebvizOnBox(box) {
  if (!state.webviz.camera || !state.webviz.controls || !window.THREE) {
    return;
  }
  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);
  const radius = Math.max(size.length() * 0.9, 6);
  state.webviz.cameraTarget = center;
  state.webviz.controls.target.copy(center);
  state.webviz.camera.position.set(center.x + radius, center.y - radius, center.z + radius * 0.6);
  state.webviz.camera.lookAt(center);
  state.webviz.controls.update();
  state.webviz.shouldAutoFocus = false;
}

function unsubscribeWebvizTopics() {
  if (state.webviz.protectionTopic) {
    state.webviz.protectionTopic.unsubscribe();
    state.webviz.protectionTopic = null;
  }
  if (state.webviz.pointTopic) {
    state.webviz.pointTopic.unsubscribe();
    state.webviz.pointTopic = null;
  }
  clearPointCloudObject();
  clearProtectionGroup();
}

function ensureWebvizViewer() {
  if (!window.ROSLIB || !window.THREE || !window.THREE.OrbitControls) {
    throw new Error("网页可视化依赖加载失败，请检查网络或 CDN 访问");
  }
  if (state.webviz.renderer) {
    return;
  }
  elements.webvizCanvas.innerHTML = "";
  const width = Math.max(elements.webvizCanvas.clientWidth, 640);
  const height = Math.max(elements.webvizCanvas.clientHeight, 480);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setClearColor(0x08111f, 1);
  elements.webvizCanvas.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 5000);
  camera.up.set(0, 0, 1);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.screenSpacePanning = true;

  scene.add(new THREE.AmbientLight(0xffffff, 0.9));
  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.6);
  directionalLight.position.set(20, -10, 30);
  scene.add(directionalLight);
  scene.add(new THREE.AxesHelper(3));

  state.webviz.scene = scene;
  state.webviz.camera = camera;
  state.webviz.renderer = renderer;
  state.webviz.controls = controls;
  state.webviz.cameraTarget = new THREE.Vector3(0, 0, 0);
  applyViewConfig(defaultViewConfig());

  const animate = () => {
    state.webviz.animationFrame = window.requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  };
  animate();

  state.webviz.resizeHandler = () => {
    if (!state.webviz.renderer || !state.webviz.camera) {
      return;
    }
    const nextWidth = Math.max(elements.webvizCanvas.clientWidth, 640);
    const nextHeight = Math.max(elements.webvizCanvas.clientHeight, 480);
    state.webviz.renderer.setSize(nextWidth, nextHeight);
    state.webviz.camera.aspect = nextWidth / nextHeight;
    state.webviz.camera.updateProjectionMatrix();
  };
  window.addEventListener("resize", state.webviz.resizeHandler);
}

function fieldMap(fields = []) {
  return Object.fromEntries(fields.map((field) => [field.name, field]));
}

function rgbToColor(value) {
  return [(value >> 16) & 0xff, (value >> 8) & 0xff, value & 0xff];
}

function normalizePointColor(r, g, b) {
  if (r === 0 && g === 0 && b === 0) {
    return [0.12, 0.55, 0.95];
  }
  return [r / 255, g / 255, b / 255];
}

function decodePointCloudData(data) {
  if (Array.isArray(data)) {
    return new Uint8Array(data);
  }
  if (typeof data === "string") {
    const binary = window.atob(data);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i += 1) {
      bytes[i] = binary.charCodeAt(i);
    }
    return bytes;
  }
  return new Uint8Array();
}

function renderPointCloudMessage(message) {
  if (!state.webviz.scene || !window.THREE) {
    return;
  }
  const bytes = decodePointCloudData(message.data);
  const fields = fieldMap(message.fields);
  const xField = fields.x;
  const yField = fields.y;
  const zField = fields.z;
  if (!xField || !yField || !zField || !message.point_step) {
    updateWebvizStatus("点云字段缺失，无法渲染", true);
    return;
  }

  const rgbField = fields.rgb || fields.rgba;
  const pointStep = message.point_step;
  const totalPoints = Math.floor(bytes.length / pointStep);
  const stride = Math.max(1, Math.ceil(totalPoints / 15000));
  const count = Math.max(0, Math.floor(totalPoints / stride));
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !message.is_bigendian;
  const bounds = {
    minX: Infinity,
    minY: Infinity,
    minZ: Infinity,
    maxX: -Infinity,
    maxY: -Infinity,
    maxZ: -Infinity,
  };

  let index = 0;
  for (let i = 0; i < totalPoints && index < count; i += stride) {
    const base = i * pointStep;
    const x = view.getFloat32(base + xField.offset, littleEndian);
    const y = view.getFloat32(base + yField.offset, littleEndian);
    const z = view.getFloat32(base + zField.offset, littleEndian);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      continue;
    }
    positions[index * 3] = x;
    positions[index * 3 + 1] = y;
    positions[index * 3 + 2] = z;
    bounds.minX = Math.min(bounds.minX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.minZ = Math.min(bounds.minZ, z);
    bounds.maxX = Math.max(bounds.maxX, x);
    bounds.maxY = Math.max(bounds.maxY, y);
    bounds.maxZ = Math.max(bounds.maxZ, z);

    let rgb = 0xffffff;
    if (rgbField) {
      rgb = view.getUint32(base + rgbField.offset, littleEndian);
    }
    const [r, g, b] = rgbToColor(rgb);
    const [nr, ng, nb] = normalizePointColor(r, g, b);
    colors[index * 3] = nr;
    colors[index * 3 + 1] = ng;
    colors[index * 3 + 2] = nb;
    index += 1;
  }

  if (!index) {
    updateWebvizStatus(`已连接，当前雷达: ${state.webviz.activeLidar}，点云为空`, true);
    clearPointCloudObject();
    return;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions.slice(0, index * 3), 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors.slice(0, index * 3), 3));
  geometry.computeBoundingSphere();
  const material = new THREE.PointsMaterial({ size: 0.14, vertexColors: true, sizeAttenuation: true });
  const points = new THREE.Points(geometry, material);
  points.frustumCulled = false;

  clearPointCloudObject();
  state.webviz.scene.add(points);
  state.webviz.pointCloudObject = points;
  state.webviz.lastPointCount = index;
  if (state.webviz.shouldAutoFocus) {
    focusWebvizOnBox(
      new THREE.Box3(
        new THREE.Vector3(bounds.minX, bounds.minY, bounds.minZ),
        new THREE.Vector3(bounds.maxX, bounds.maxY, bounds.maxZ),
      ),
    );
  }
  updateWebvizStatus(`已连接，当前雷达: ${state.webviz.activeLidar}，点数: ${index}`);
}

function renderProtectionZones(message) {
  if (!state.webviz.scene || !window.THREE) {
    return;
  }
  clearProtectionGroup();
  const group = new THREE.Group();

  (message.markers || []).forEach((marker) => {
    const positions = [];
    for (const point of marker.points || []) {
      positions.push(point.x || 0, point.y || 0, point.z || 0);
    }
    if (!positions.length) {
      return;
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const material = new THREE.LineBasicMaterial({
      color: new THREE.Color(marker.color?.r || 1, marker.color?.g || 1, marker.color?.b || 1),
      transparent: true,
      opacity: marker.color?.a ?? 1,
    });
    group.add(new THREE.LineSegments(geometry, material));
  });

  state.webviz.scene.add(group);
  state.webviz.protectionGroup = group;
}

function subscribeWebvizTopics() {
  if (!state.webviz.ros || !state.webviz.scene) {
    return;
  }
  unsubscribeWebvizTopics();
  const lidar = state.webviz.activeLidar;
  const pointTopicName = webvizPointTopicName(lidar);
  const protectionTopicName = webvizProtectionTopicName(lidar);
  state.webviz.shouldAutoFocus = true;

  state.webviz.pointTopic = new ROSLIB.Topic({
    ros: state.webviz.ros,
    name: pointTopicName,
    messageType: "sensor_msgs/PointCloud2",
    throttle_rate: 200,
  });
  state.webviz.pointTopic.subscribe(renderPointCloudMessage);

  if (protectionTopicName) {
    state.webviz.protectionTopic = new ROSLIB.Topic({
      ros: state.webviz.ros,
      name: protectionTopicName,
      messageType: "visualization_msgs/MarkerArray",
      throttle_rate: 200,
    });
    state.webviz.protectionTopic.subscribe(renderProtectionZones);
  }
  updateWebvizTopicLabel();
  updateWebvizStatus(`已连接，当前雷达: ${lidar}，等待点云...`);
}

function disconnectWebviz() {
  unsubscribeWebvizTopics();
  if (state.webviz.ros) {
    state.webviz.ros.close();
    state.webviz.ros = null;
  }
}

function closeWebviz() {
  elements.webvizSection.classList.add("hidden");
  disconnectWebviz();
  state.webviz.shouldAutoFocus = true;
  updateWebvizStatus("未连接");
}

async function openWebviz() {
  if (!state.currentStatus?.web_visualization_unit) {
    return;
  }
  elements.webvizSection.classList.remove("hidden");
  updateWebvizStatus("正在启动网页可视化桥接...");
  await runAction("web_visualize", { silentToast: true });
  ensureWebvizViewer();
  disconnectWebviz();

  state.webviz.activeLidar = elements.webvizLidarSelect.value;
  const ros = new ROSLIB.Ros({ url: webvizUrl() });
  state.webviz.ros = ros;

  ros.on("connection", () => {
    subscribeWebvizTopics();
  });
  ros.on("error", () => {
    updateWebvizStatus("无法连接 rosbridge，请确认桥接服务已启动", true);
  });
  ros.on("close", () => {
    updateWebvizStatus("连接已关闭", true);
  });
}

function isLogModalOpen() {
  return !elements.logModal.classList.contains("hidden");
}

function openLogModal() {
  elements.logModal.classList.remove("hidden");
  elements.logModal.setAttribute("aria-hidden", "false");
}

function closeLogModal() {
  elements.logModal.classList.add("hidden");
  elements.logModal.setAttribute("aria-hidden", "true");
}

async function loadServices() {
  const data = await request("/api/services");
  renderServices(data.services);
}

async function loadStatus() {
  const serviceId = currentServiceId();
  const data = await request(`/api/services/${serviceId}`);
  renderStatus(data);
  renderServices(state.services);
}

async function loadConfig() {
  const serviceId = currentServiceId();
  if (!state.currentStatus?.config_path) {
    elements.configEditor.value = "该服务未配置可编辑的配置文件路径。";
    renderPlcStatus("");
    return;
  }
  try {
    const data = await request(`/api/services/${serviceId}/config`);
    renderConfig(data);
  } catch (error) {
    elements.configEditor.value = "";
    renderPlcStatus("");
    showToast(error.message, true);
  }
}

async function loadLogs() {
  const serviceId = currentServiceId();
  const lines = Number(elements.logLines.value || 200);
  const data = await request(`/api/services/${serviceId}/logs?lines=${lines}`);
  renderLogs(data);
}

async function loadProgress() {
  const serviceId = currentServiceId();
  const data = await request(`/api/services/${serviceId}/progress`);
  renderProgress(data);
}

async function refreshAll() {
  await loadStatus();
  await loadConfig();
  if (state.currentStatus?.progress_path) {
    await loadProgress();
  }
  if (isLogModalOpen()) {
    await loadLogs();
  }
}

async function runAction(action, options = {}) {
  const serviceId = currentServiceId();
  await request(`/api/services/${serviceId}/action`, {
    method: "POST",
    body: JSON.stringify({ action }),
  });
  if (!options.silentToast) {
    showToast(`${action} 成功`);
  }
  await loadStatus();
  if (isLogModalOpen()) {
    await loadLogs();
  }
}

async function setEnabled(enabled) {
  await runAction(enabled ? "enable" : "disable");
}

async function saveConfig() {
  const serviceId = currentServiceId();
  const content = elements.configEditor.value;
  const restart = elements.restartAfterSave.checked;
  await request(`/api/services/${serviceId}/config`, {
    method: "PUT",
    body: JSON.stringify({ content, restart }),
  });
  showToast("配置已保存");
  await loadStatus();
  if (isLogModalOpen()) {
    await loadLogs();
  }
}

async function bootstrap() {
  await loadServices();
  if (!state.services.length) {
    showToast("services.json 中没有可管理的服务", true);
    return;
  }
  await refreshAll();
}

elements.serviceList.addEventListener("click", async (event) => {
  const button = event.target.closest("[data-service-id]");
  if (!button) {
    return;
  }
  const nextServiceId = button.dataset.serviceId;
  if (nextServiceId === state.currentServiceId) {
    return;
  }
  state.currentServiceId = nextServiceId;
  renderServices(state.services);
  await refreshAll();
});

elements.refreshAll.addEventListener("click", async () => {
  await refreshAll();
  showToast("已刷新");
});

document.querySelectorAll("[data-action]").forEach((button) => {
  button.addEventListener("click", async () => {
    try {
      await runAction(button.dataset.action);
    } catch (error) {
      showToast(error.message, true);
    }
  });
});

elements.enableToggle.addEventListener("change", async () => {
  try {
    await setEnabled(elements.enableToggle.checked);
  } catch (error) {
    elements.enableToggle.checked = !elements.enableToggle.checked;
    showToast(error.message, true);
  }
});

elements.saveConfig.addEventListener("click", async () => {
  try {
    await saveConfig();
  } catch (error) {
    showToast(error.message, true);
  }
});

elements.webVisualizeButton.addEventListener("click", async () => {
  try {
    await openWebviz();
  } catch (error) {
    updateWebvizStatus(error.message, true);
    showToast(error.message, true);
  }
});

elements.webvizReconnect.addEventListener("click", async () => {
  try {
    await openWebviz();
  } catch (error) {
    updateWebvizStatus(error.message, true);
    showToast(error.message, true);
  }
});

elements.webvizDefaultView.addEventListener("click", () => {
  applyViewConfig(defaultViewConfig());
  state.webviz.shouldAutoFocus = false;
  updateWebvizStatus(`已恢复默认视角，当前雷达: ${state.webviz.activeLidar}`);
});

elements.webvizSaveView.addEventListener("click", () => {
  const view = currentViewConfig();
  if (!view) {
    return;
  }
  saveDefaultViewConfig(view);
  state.webviz.shouldAutoFocus = false;
  showToast("已设置为默认视角");
});

elements.webvizClose.addEventListener("click", closeWebviz);

elements.webvizLidarSelect.addEventListener("change", () => {
  state.webviz.activeLidar = elements.webvizLidarSelect.value;
  updateWebvizTopicLabel();
  if (state.webviz.ros) {
    subscribeWebvizTopics();
  }
});

elements.refreshLogs.addEventListener("click", async () => {
  try {
    await loadLogs();
    showToast("日志已刷新");
  } catch (error) {
    showToast(error.message, true);
  }
});

elements.openLogs.addEventListener("click", async () => {
  openLogModal();
  try {
    await loadLogs();
  } catch (error) {
    showToast(error.message, true);
  }
});

elements.closeLogs.addEventListener("click", closeLogModal);
elements.logBackdrop.addEventListener("click", closeLogModal);

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && isLogModalOpen()) {
    closeLogModal();
  }
});

bootstrap().catch((error) => showToast(error.message, true));

window.setInterval(() => {
  if (!state.currentServiceId) {
    return;
  }
  loadStatus().catch(() => {});
}, 5000);

window.setInterval(() => {
  if (!state.currentServiceId || !state.currentStatus?.progress_path) {
    return;
  }
  loadProgress().catch(() => {});
}, 1000);

window.setInterval(() => {
  if (state.currentServiceId !== "rslidar-sdk") {
    return;
  }
  renderRslidarStatusFromConfig(elements.configEditor.value);
}, 1000);
