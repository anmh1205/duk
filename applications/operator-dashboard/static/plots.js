const stage = {
  background: "#10232d", grid: "#213943", axis: "#64818b",
  text: "#b7cdd2", lime: "#caf48b", cyan: "#78d4dc",
};
export const traceColors = ["#006d59", "#98512d"];
const mono = 'ui-monospace, "DejaVu Sans Mono", monospace';
const decimal = (value, digits) => value.toLocaleString("vi", {maximumFractionDigits: digits});

function canvasContext(canvas) {
  const {width, height} = canvas.getBoundingClientRect();
  if (!width || !height) return null;
  const ratio = window.devicePixelRatio || 1;
  const pixels = [Math.round(width * ratio), Math.round(height * ratio)];
  if (canvas.width !== pixels[0] || canvas.height !== pixels[1]) {
    [canvas.width, canvas.height] = pixels;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);
  return {context, width, height};
}

function line(context, points, color, width = 1, dash = []) {
  context.beginPath();
  context.setLineDash(dash);
  points.forEach(([x, y], index) => index ? context.lineTo(x, y) : context.moveTo(x, y));
  context.strokeStyle = color;
  context.lineWidth = width;
  context.stroke();
  context.setLineDash([]);
}

function disk(context, x, y, radius, fill, border, width = 1) {
  context.beginPath();
  context.arc(x, y, radius, 0, 2 * Math.PI);
  context.fillStyle = fill;
  context.fill();
  context.strokeStyle = border;
  context.lineWidth = width;
  context.stroke();
}

function text(context, message, x, y, color = stage.text, size = 11, align = "left") {
  context.font = size + "px " + mono;
  context.textAlign = align;
  context.fillStyle = color;
  context.fillText(message, x, y);
}

function stageGrid(context, width, height) {
  context.fillStyle = stage.background;
  context.fillRect(0, 0, width, height);
  for (let x = width / 2 % 22; x < width; x += 22) {
    line(context, [[x, 0], [x, height]], stage.grid, .7);
  }
  for (let y = height % 22; y < height; y += 22) {
    line(context, [[0, y], [width, y]], stage.grid, .7);
  }
}

export function drawBalance(canvas, run, state, time) {
  const view = canvasContext(canvas);
  if (!view) return;
  const {context, width, height} = view;
  stageGrid(context, width, height);
  if (!run) {
    text(context, "Đang tải mô hình", width / 2, height / 2, stage.text, 12, "center");
    return;
  }
  const {radius_m: radius, length_m: length} = run.plant;
  const [position, , lean, , spin] = state;
  const [minimum, maximum] = run.positionRange;
  const scale = Math.min(height / (4.4 * radius + length), (width - 32) / (maximum - minimum + 4.6 * radius + 2 * length));
  const ground = height * .8;
  const origin = width / 2 - (maximum + minimum) * scale / 2;
  const hubX = origin + position * scale;
  const hubY = ground - radius * scale;
  const wheel = radius * scale;
  const centerX = hubX + Math.sin(lean) * length * scale;
  const centerY = hubY - Math.cos(lean) * length * scale;
  line(context, [[16, ground], [width - 16, ground]], stage.axis);
  line(context, [[hubX, hubY], [hubX, hubY - (length + .065) * scale]], stage.axis, 1, [3, 4]);
  text(context, "z = 0", width - 17, ground + 15, stage.text, 10, "right");
  text(context, "t " + decimal(time, 3) + " s", 16, 23);
  text(context, "x " + decimal(position, 3) + " m", width - 16, 23, stage.text, 11, "right");

  context.save();
  context.translate(hubX, hubY);
  context.rotate(lean);
  line(context, [[0, 0], [0, -length * scale]], "#4b7477", 12);
  line(context, [[0, 0], [0, -length * scale]], "#9ac6bc", 3);
  const bodyWidth = wheel * 1.12;
  const bodyHeight = wheel * 1.3;
  context.beginPath();
  context.roundRect(-bodyWidth / 2, -length * scale - bodyHeight / 2, bodyWidth, bodyHeight, Math.min(8, bodyWidth / 5));
  context.fillStyle = "#2a4947";
  context.fill();
  context.strokeStyle = stage.lime;
  context.lineWidth = 1.4;
  context.stroke();
  context.restore();
  disk(context, centerX, centerY, 4, stage.lime, stage.background, 1.5);

  disk(context, hubX, hubY, wheel, "#162c32", stage.cyan, 2);
  disk(context, hubX, hubY, wheel * .76, "#1e3c46", "#506e75", 1);
  for (let spoke = 0; spoke < 6; spoke++) {
    const angle = spin + spoke * Math.PI / 3;
    const point = (r) => [hubX + r * Math.sin(angle), hubY - r * Math.cos(angle)];
    line(context, [point(wheel * .25), point(wheel * .72)], "#85aab1", 1.5);
  }
  disk(context, hubX, hubY, Math.max(3, wheel * .14), stage.lime, stage.background);

  const rule = .1 * scale;
  const bottom = height - 22;
  line(context, [[17, bottom - 4], [17, bottom], [17 + rule, bottom], [17 + rule, bottom - 4]], stage.text);
  text(context, "0,1 m", 20 + rule, bottom + 3, stage.text, 10);
  text(context, "X →", width - 17, bottom + 3, stage.text, 10, "right");
}

export function drawKinematics(canvas, result) {
  const view = canvasContext(canvas);
  if (!view) return;
  const {context, width, height} = view;
  stageGrid(context, width, height);
  if (!result) return;
  const radius = result.wheel_radius_mm;
  const coordinates = result.legs.flatMap(({points_mm: outer}) => [
    ...Object.values(outer), [outer.O[0] - radius, outer.O[1] - radius],
    [outer.O[0] + radius, outer.O[1] + radius],
  ]);
  const xs = coordinates.map(([x]) => x);
  const zs = coordinates.map(([, z]) => z);
  const minimumX = Math.min(...xs);
  const maximumX = Math.max(...xs);
  const minimumZ = Math.min(...zs);
  const maximumZ = Math.max(...zs);
  const scale = Math.min((width / 2 - 48) / (maximumX - minimumX), (height - 76) / (maximumZ - minimumZ));
  line(context, [[width / 2, 0], [width / 2, height]], "#42616a");
  result.legs.forEach(({points_mm: points, q_deg: angle}, index) => {
    const color = index ? stage.cyan : stage.lime;
    const xOrigin = width * (1 + 2 * index) / 4 - (maximumX + minimumX) * scale / 2;
    const zOrigin = (height + 12) / 2 + (maximumZ + minimumZ) * scale / 2;
    const screen = ([x, z]) => [xOrigin + x * scale, zOrigin - z * scale];
    const point = (key) => screen(points[key]);
    const [ox, oz] = point("O");
    text(context, (index ? "Phải" : "Trái") + "  " + decimal(angle, 2) + "°", 14 + index * width / 2, 23, color);
    line(context, [[width / 2 * index + 9, zOrigin], [width / 2 * (index + 1) - 9, zOrigin]], stage.axis, 1, [3, 4]);
    disk(context, ox, oz, radius * scale, "#142e38", "#719aa4", 1.6);
    disk(context, ox, oz, radius * scale * .78, "#1f3b45", "#4d6b75");
    context.beginPath();
    ["B", "C", "O"].forEach((name, i) => i ? context.lineTo(...point(name)) : context.moveTo(...point(name)));
    context.closePath();
    context.fillStyle = "#34555b";
    context.fill();
    line(context, ["A", "B", "C", "D", "A"].map(point), color, 2);
    line(context, ["B", "O", "C"].map(point), color, 1.4);
    line(context, ["B", "H", "O"].map(point), stage.axis, 1, [4, 3]);
    for (const name of "ABCDHO") {
      const [x, z] = point(name);
      const auxiliary = name === "H";
      disk(context, x, z, auxiliary ? 2.3 : 3.4, stage.background, auxiliary ? stage.text : color, 1.5);
      text(context, name, x + 6, z - 6, auxiliary ? stage.text : color);
    }
  });
}

export function drawTrace(canvas, series, end, cursor, unit) {
  const view = canvasContext(canvas);
  if (!view) return;
  const {context, width, height} = view;
  const [left, right, top, bottom] = [58, width - 26, 21, height - 28];
  if (!series.length) {
    text(context, "Chưa có kết quả", width / 2, height / 2, "#59707b", 11, "center");
    return;
  }
  const values = [0];
  for (const {data, held} of series) {
    // The terminal row has no following hold interval.
    for (let index = 0; index < data.length - Number(held); index++) values.push(data[index]);
  }
  let low = Math.min(...values), high = Math.max(...values);
  const padding = (high - low || Math.abs(high) || 1) * .08;
  low -= padding;
  high += padding;
  const x = (time) => left + time / (end || 1) * (right - left);
  const y = (value) => bottom - (value - low) / (high - low) * (bottom - top);
  text(context, unit, 10, 12, "#59707b", 10);
  text(context, "t (s)", right, 12, "#59707b", 10, "right");
  for (let index = 0; index <= 4; index++) {
    const value = low + (high - low) * index / 4;
    const yy = y(value);
    line(context, [[left, yy], [right, yy]], "#e1eaed");
    text(context, decimal(value, 3), left - 6, yy + 3, "#526a74", 10, "right");
  }
  if (end) {
    for (let index = 0; index <= 4; index++) {
      const time = index * end / 4;
      const xx = x(time);
      line(context, [[xx, top], [xx, bottom]], "#e1eaed");
      text(context, decimal(time, 3), xx, bottom + 15, "#526a74", 10, "center");
    }
  }
  for (const {times, data, held, color, dashed} of series) {
    if (held && times.length < 2) continue;
    const points = [[x(times[0]), y(data[0])]];
    for (let index = 1; index < times.length; index++) {
      if (held) points.push([x(times[index]), y(data[index - 1])]);
      if (!held || index + 1 < times.length) points.push([x(times[index]), y(data[index])]);
    }
    if (points.length === 1) disk(context, ...points[0], 2.5, color, color);
    else line(context, points, color, dashed ? 1.6 : 2, dashed ? [5, 4] : []);
  }
  line(context, [[x(cursor), top], [x(cursor), bottom]], "#536873", 1, [3, 4]);
}
