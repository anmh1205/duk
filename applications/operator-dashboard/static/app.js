import {decimal, drawBalance, drawKinematics, drawTrace, traceColors} from "./plots.js";

const element = (id) => document.getElementById(id);
const format = (number, digits = 3) => decimal(number, digits, true);
const degrees = (radians) => radians * 180 / Math.PI;
const peak = (values) => Math.max(0, ...values.map(Math.abs));
const controllers = {position: "Giữ vị trí", balance: "Giữ thăng bằng"};
const outcomes = {
  completed: "Đã chạy hết thời lượng",
  lost_contact: "Mất tiếp xúc",
  pitch_limit: "Chạm ngưỡng góc nghiêng",
  traction_infeasible: "Hết miền mô-men khả thi",
};
const limits = {
  within_limits: "Trong giới hạn",
  motor: "Giới hạn động cơ",
  traction: "Giới hạn độ bám",
  infeasible: "Không khả thi",
};
const channels = {
  lean: {fields: [["lean", "Góc nghiêng"]], unit: "°"},
  slip: {fields: [["slip", "Độ trượt"]], unit: "m/s"},
  torque: {fields: [["requested", "Yêu cầu"], ["applied", "Áp dụng"]], unit: "N·m", held: true},
  speed: {fields: [["speed", "Tại trục"], ["speed_estimate", "Phản hồi"]], unit: "m/s"},
  force: {fields: [["normal", "Pháp tuyến"], ["tangent", "Tiếp tuyến"]], unit: "N"},
};
const balanceFields = [
  "friction", "initial_lean_deg", "delay_samples", "noise_seed", "accel_noise", "pitch_noise",
];

let config;
let current;
let baseline;
let pose;
let mode = "balance";
let playFrame;
let poseTimer;
let poseRevision = 0;

async function request(path, payload) {
  const options = payload === undefined ? {} : {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload),
  };
  const response = await fetch("/api/" + path, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  return result;
}

function tractionEnabled(enabled) {
  element("use-traction").checked = enabled;
  element("traction_budget").disabled = !enabled;
  element("budget-field").hidden = !enabled;
}

function applyPreset(name) {
  const settings = config.presets[name];
  element("friction").value = settings.friction;
  element("traction_budget").value = settings.traction_budget ?? config.presets.low_grip_limited.traction_budget;
  tractionEnabled(settings.traction_budget !== null);
}

function balanceOptions() {
  return {
    ...Object.fromEntries(balanceFields.map((key) => [key, element(key).valueAsNumber])),
    controller: element("controller").value,
    feedback: element("feedback").value,
    traction_budget: element("use-traction").checked ? element("traction_budget").valueAsNumber : null,
  };
}

function runLabel(run) {
  const {friction, controller, traction_budget: budget} = run.configuration;
  return "μ " + format(friction, 2) + " · " + controllers[controller] + (
    budget === null ? "" : " · Giới hạn μ " + format(budget, 2)
  );
}

function prepareRun(record) {
  const {trace} = record;
  const positions = trace.state.map((row) => row[0]);
  return {
    record,
    ...record,
    positionRange: [Math.min(...positions), Math.max(...positions)],
    end: trace.time.at(-1),
    signals: {...trace, lean: trace.state.map((row) => degrees(row[2])), speed: trace.state.map((row) => row[1])},
  };
}

async function runBalance() {
  if (!element("balance-form").reportValidity()) return;
  pause();
  element("run").disabled = true;
  element("run-status").classList.remove("error");
  element("run-status").textContent = "Đang tính mô hình và bộ điều khiển…";
  try {
    current = prepareRun(await request("balance", balanceOptions()));
    renderRun();
    element("run-status").textContent = "";
  } catch (error) {
    element("run-status").classList.add("error");
    element("run-status").textContent = error.message;
  } finally {
    element("run").disabled = false;
  }
}

function matrix(name, rows) {
  const table = document.createElement("table");
  table.createCaption().textContent = name;
  const body = table.createTBody();
  for (const values of rows) {
    const row = body.insertRow();
    for (const value of values) row.insertCell().textContent = format(value, 3);
  }
  return table;
}

function renderRun() {
  const {trace, signals, end} = current;
  const complete = trace.outcome === "completed";
  const outcome = element("outcome");
  outcome.textContent = outcomes[trace.outcome] ?? trace.outcome;
  outcome.parentElement.classList.toggle("ok", complete);
  outcome.parentElement.classList.toggle("stopped", !complete);
  element("outcome-detail").textContent = format(end) + " s / " + format(config.duration_s) + " s · " + runLabel(current);
  element("peak-slip").textContent = format(peak(trace.slip)) + " m/s";
  element("peak-lean").textContent = format(peak(signals.lean), 2) + "°";
  element("peak-torque").textContent = trace.time.length > 1 ? format(peak(trace.applied.slice(0, -1))) + " N·m" : "—";
  element("pin").disabled = false;
  element("download").disabled = false;
  const time = element("time");
  time.max = end;
  time.value = 0;
  time.disabled = !end;
  element("play").disabled = !end;

  const model = current.linear_model;
  element("model-math-meta").textContent =
    "x = [" + model.state_order.join(", ") + "]. u là mô-men tổng (N·m). " +
    "Trạng thái phản hồi LQR [" + model.feedback_state_order.join(", ") + "]. " +
    "Chu kỳ " + format(config.period_s) + " s · ρ = " + format(model.spectral_radius, 5);
  element("matrices").replaceChildren(
    matrix("A liên tục", model.continuous_a),
    matrix("B liên tục", model.continuous_b),
    matrix("K rời rạc", [model.gain]),
  );
  renderLegend();
  renderFrame();
}

function scenarioSeries(run, definition, dashed) {
  return definition.fields.map(([name, label], index) => ({
    data: run.signals[name], times: run.trace.time,
    color: traceColors[index], held: definition.held ?? false, dashed, label,
  }));
}

function chartSeries(definition) {
  if (!current) return [];
  const series = scenarioSeries(current, definition, false);
  if (baseline && baseline !== current) series.push(...scenarioSeries(baseline, definition, true));
  return series;
}

function renderLegend() {
  const definition = channels[element("channel").value];
  const items = chartSeries(definition).map(({label, color, dashed}) => {
    const item = document.createElement("span");
    const dash = document.createElement("i");
    item.className = "legend-item";
    dash.style.borderColor = color;
    dash.classList.toggle("dashed", dashed);
    item.append(dash, label + (dashed ? " · tham chiếu" : ""));
    return item;
  });
  element("chart-legend").replaceChildren(...items);
}

function renderFrame() {
  if (mode !== "balance") return;
  const definition = channels[element("channel").value];
  const cursor = Number(element("time").value);
  drawTrace(element("chart"), chartSeries(definition), Math.max(current?.end ?? 0, baseline?.end ?? 0), cursor, definition.unit);
  if (!current) {
    drawBalance(element("balance-model"), null);
    return;
  }
  const {trace, end} = current;
  let index = trace.time.findIndex((time) => time > cursor);
  index = index === -1 ? trace.time.length - 1 : Math.max(0, index - 1);
  const state = trace.state[index];
  const activeInterval = index + 1 < trace.time.length;
  const stateLabel = element("model-state");
  stateLabel.hidden = false;
  stateLabel.textContent = "θ " + format(degrees(state[2]), 2) + "°";
  element("time-label").textContent = format(cursor) + " s";
  element("live-lean").textContent = format(degrees(state[2]), 2) + "°";
  element("live-slip").textContent = format(trace.slip[index]) + " m/s";
  element("live-request").textContent = activeInterval ? format(trace.requested[index]) + " N·m" : "—";
  element("live-torque").textContent = activeInterval ? format(trace.applied[index]) + " N·m" : "—";
  element("live-limit").textContent = activeInterval ? limits[trace.limit_reason[index]] : "Kết thúc ca chạy";
  drawBalance(element("balance-model"), current, state, Math.min(cursor, end));
}

function pause() {
  cancelAnimationFrame(playFrame);
  playFrame = undefined;
  element("play").textContent = "▶";
  element("play").ariaLabel = "Phát lại";
}

function play() {
  if (playFrame !== undefined) {
    pause();
    return;
  }
  const slider = element("time");
  if (Number(slider.value) >= current.end) slider.value = 0;
  let previous = performance.now();
  element("play").textContent = "❚❚";
  element("play").ariaLabel = "Tạm dừng";
  const advance = (now) => {
    const speed = Number(element("speed").value);
    slider.value = Math.min(current.end, Number(slider.value) + (now - previous) / 1000 * speed);
    previous = now;
    renderFrame();
    if (Number(slider.value) >= current.end) pause();
    else playFrame = requestAnimationFrame(advance);
  };
  playFrame = requestAnimationFrame(advance);
}

function pinRun(run) {
  baseline = run;
  element("baseline").hidden = !run;
  element("clear-baseline").hidden = !run;
  if (run) {
    element("baseline").textContent = "Ca tham chiếu · " + runLabel(run) + " · " + outcomes[run.trace.outcome];
  }
  renderLegend();
  renderFrame();
}

function downloadRun() {
  const blob = new Blob([JSON.stringify(current.record, null, 2)], {type: "application/json"});
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "duk-nb4-" + new Date().toISOString().replaceAll(":", "-") + ".json";
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}

function poseInputs(left, right) {
  element("left_deg").value = left;
  element("right_deg").value = right;
  element("left-range").value = left;
  element("right-range").value = right;
}

async function calculatePose() {
  clearTimeout(poseTimer);
  if (!element("kinematics-form").reportValidity()) return;
  const revision = ++poseRevision;
  const left = element("left_deg").valueAsNumber;
  const right = element("right_deg").valueAsNumber;
  const status = element("pose-status");
  status.classList.remove("error");
  status.textContent = "Đang tính cơ cấu…";
  try {
    const result = await request("kinematics", {left_deg: left, right_deg: right});
    if (revision !== poseRevision) return;
    pose = result;
    status.textContent = "Đã cập nhật tư thế.";
    renderPose();
  } catch (error) {
    if (revision !== poseRevision) return;
    status.classList.add("error");
    status.textContent = error.message;
  }
}

function renderPose() {
  if (!pose || mode !== "kinematics") return;
  drawKinematics(element("kinematic-model"), pose);
  const sections = pose.legs.map((leg, index) => {
    const section = document.createElement("section");
    const title = document.createElement("h3");
    title.textContent = (index ? "Chân phải" : "Chân trái") + " · " + format(leg.q_deg, 2) + "°";
    const values = document.createElement("dl");
    values.className = "signal-list";
    const scientific = (number) => number.toExponential(1);
    const vector = (pair) => pair.map((number) => format(number, 2)).join(" / ");
    for (const [label, value] of [
      ["Tâm bánh X / Z (mm)", vector(leg.points_mm.O)],
      ["dO/dq (mm/rad)", vector(leg.wheel_jacobian_mm_rad)],
      ["Sai số khép vòng (mm)", scientific(leg.closure_error_mm)],
      ["Sai số BCO (mm)", scientific(leg.carrier_error_mm)],
      ["Cách biên hình học", format(leg.boundary_margin_deg, 2) + "°"],
    ]) {
      const row = document.createElement("div");
      const term = document.createElement("dt");
      const detail = document.createElement("dd");
      term.textContent = label;
      detail.textContent = value;
      row.append(term, detail);
      values.append(row);
    }
    section.append(title, values);
    return section;
  });
  element("kinematic-values").replaceChildren(...sections);
}

function selectMode(next) {
  mode = next;
  pause();
  for (const key of ["balance", "kinematics"]) {
    element(key + "-view").hidden = mode !== key;
    element("mode-" + key).ariaPressed = String(mode === key);
  }
  if (mode === "balance") renderFrame();
  else if (pose) renderPose();
  else if (config) calculatePose();
}

function connectControls() {
  element("mode-balance").onclick = () => selectMode("balance");
  element("mode-kinematics").onclick = () => selectMode("kinematics");
  element("preset").onchange = (event) => applyPreset(event.target.value);
  for (const key of ["friction", "traction_budget"]) {
    element(key).oninput = () => { element("preset").value = "custom"; };
  }
  element("use-traction").onchange = (event) => {
    tractionEnabled(event.target.checked);
    element("preset").value = "custom";
  };
  element("balance-form").onsubmit = (event) => {
    event.preventDefault();
    runBalance();
  };
  element("play").onclick = play;
  element("time").oninput = () => { pause(); renderFrame(); };
  element("channel").onchange = () => { renderLegend(); renderFrame(); };
  element("pin").onclick = () => pinRun(current);
  element("clear-baseline").onclick = () => pinRun();
  element("download").onclick = downloadRun;
  element("kinematics-form").onsubmit = (event) => {
    event.preventDefault();
    calculatePose();
  };
  element("reset-pose").onclick = () => {
    poseInputs(config.kinematics.initial_deg, config.kinematics.initial_deg);
    calculatePose();
  };
  for (const side of ["left", "right"]) {
    const slider = element(side + "-range");
    const field = element(side + "_deg");
    slider.oninput = () => {
      field.value = slider.value;
      clearTimeout(poseTimer);
      poseRevision++;
      poseTimer = setTimeout(calculatePose, 60);
    };
    field.oninput = () => {
      clearTimeout(poseTimer);
      poseRevision++;
      if (field.validity.valid) slider.value = field.value;
      element("pose-status").classList.remove("error");
      element("pose-status").textContent = "Chọn Tính tư thế để cập nhật.";
    };
  }
  const resize = new ResizeObserver(() => mode === "balance" ? renderFrame() : renderPose());
  for (const id of ["balance-model", "chart", "kinematic-model"]) resize.observe(element(id));
}

async function start() {
  try {
    config = await request("config");
    for (const key of [...balanceFields, "controller", "feedback"]) element(key).value = config.balance[key];
    element("initial_lean_deg").min = -config.fall_angle_deg;
    element("initial_lean_deg").max = config.fall_angle_deg;
    element("delay_samples").max = config.max_delay_samples;
    applyPreset("dry");
    const [low, high] = config.kinematics.bounds_deg;
    for (const key of ["left-range", "right-range", "left_deg", "right_deg"]) {
      element(key).min = low;
      element(key).max = high;
    }
    poseInputs(config.kinematics.initial_deg, config.kinematics.initial_deg);
    element("kinematic-bounds").textContent = "Miền khảo sát " + format(low, 2) + "° đến " + format(high, 2) + "°.";
    connectControls();
    await runBalance();
  } catch (error) {
    const message = element("app-error");
    message.textContent = "Không tải được cấu hình mô phỏng. " + error.message;
    message.hidden = false;
  }
}

start();
