// Shared position for quadruped movement
let currentX = 0;
let direction = 1;
let animatedCurveOffset = 0;

function animateIMUPlot() {
  const canvas = document.getElementById("imuCanvas");
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  let t = 0;

  function draw() {
    ctx.clearRect(0, 0, width, height);
    ctx.beginPath();
    ctx.moveTo(0, height / 2);

    for (let x = 0; x < width; x++) {
      const acc = Math.sin((t + x) * 0.05) * 12 + Math.random() * 2;
      const y = height / 2 - acc;
      ctx.lineTo(x, y);
    }

    ctx.strokeStyle = "#2d8cff";
    ctx.lineWidth = 2;
    ctx.stroke();

    t += 1;
    requestAnimationFrame(draw);
  }

  draw();
}

function animateCameraPlot(quadRef) {
  const canvas = document.getElementById("cameraCanvas");
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  let t2 = 0;

  function draw() {
    ctx.clearRect(0, 0, width, height);
    ctx.beginPath();
    ctx.moveTo(0, height / 2);

    for (let x = 0; x < width; x++) {
      const displacement = Math.sin((t2 + x) * 0.02) * 10 + Math.cos((t2 + x) * 0.07) * 5;
      const y = height / 2 - displacement;
      ctx.lineTo(x, y);
    }

    ctx.strokeStyle = "#00ffaa";
    ctx.lineWidth = 2;
    ctx.stroke();

    if (quadRef) {
      const poseY = Math.sin(t2 * 0.1) * 6 + Math.cos(t2 * 0.05) * 3;
      quadRef.style.transform = `translate(${currentX}px, ${poseY}px)`;
    }

    t2 += 1;
    requestAnimationFrame(draw);
  }

  draw();
}

function simulateQuadrupedMotion() {
  const cluster = document.querySelector(".quad-cluster");
  const status = document.getElementById("quadStatus");
  const ur5 = document.getElementById("ur5");
  const container = document.querySelector(".diagram-wrapper");

  currentX = 0;
  let lastTime = performance.now();
  const speed = 100; // pixels per second

  function move(timestamp) {
    if (!container || !cluster) return;

    const deltaTime = (timestamp - lastTime) / 1000;
    lastTime = timestamp;

    const padding = 32;
    const maxX = container.clientWidth - cluster.clientWidth - padding;

    currentX += speed * deltaTime * direction;
    currentX = Math.max(0, Math.min(currentX, maxX));

    let yOffset = 0;
    if (direction === -1) {
      const xNorm = currentX / maxX;
      yOffset = -160 * Math.pow((xNorm - 0.5), 2) + 40;
    }

    cluster.style.transform = `translate(${currentX}px, ${yOffset}px)`;

    if (status) {
      status.textContent = direction === 1 ? "Moving →" : "← Returning (UR5)";
    }

    if (ur5) {
      ur5.classList.toggle("active-glow", direction === -1);
    }

    if ((currentX >= maxX && direction === 1) || (currentX <= 0 && direction === -1)) {
      setTimeout(() => {
        direction *= -1;
        lastTime = performance.now();
        requestAnimationFrame(move);
      }, 1000);
      return;
    }

    if ((currentX >= maxX && direction === 1) || (currentX <= 0 && direction === -1)) {
      setTimeout(() => {
        direction *= -1;
        lastTime = performance.now();
        requestAnimationFrame(move);
      }, 1000);
      return;
    }

    requestAnimationFrame(move);
  }

  requestAnimationFrame(move);
}

function drawArrow(fromEl, toEl, label = "", curvature = 0.3, animateOffset = 0) {
  if (!fromEl || !toEl) return;

  const svg = document.getElementById("arrow-canvas");
  const svgRect = svg.getBoundingClientRect();
  const fromRect = fromEl.getBoundingClientRect();
  const toRect = toEl.getBoundingClientRect();

  const x1 = fromRect.left + fromRect.width / 2 - svgRect.left;
  const y1 = fromRect.top + fromRect.height / 2 - svgRect.top;
  const x2 = toRect.left + toRect.width / 2 - svgRect.left;
  const y2 = toRect.top + toRect.height / 2 - svgRect.top;

  const cx = (x1 + x2) / 2;
  const cy = Math.min(y1, y2) - 160; // height of the parabola

  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  const d = direction === -1
    ? `M ${x1},${y1} Q ${cx},${cy} ${x2},${y2}`
    : `M ${x1},${y1} C ${x1 + (x2 - x1) * curvature},${y1}, ${x2 - (x2 - x1) * curvature},${y2}, ${x2},${y2}`;

  path.setAttribute("d", d);
  path.setAttribute("fill", "none");
  path.setAttribute("stroke", "#2d8cff");
  path.setAttribute("stroke-width", "2");
  path.setAttribute("marker-end", "url(#arrowhead)");
  svg.appendChild(path);

  if (label) {
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    const labelX = (x1 + x2) / 2;
    const labelY = direction === -1 ? (y1 + y2) / 2 - Math.abs(cy - ((y1 + y2) / 2)) * 0.5 : ((y1 + y2) / 2);
    text.setAttribute("x", labelX);
    text.setAttribute("y", labelY - 10);
    text.setAttribute("fill", "#ffffff");
    text.setAttribute("font-size", "14");
    text.setAttribute("text-anchor", "middle");
    text.textContent = label;
    svg.appendChild(text);
  }
}

function drawArrowsDynamically() {
  const svg = document.getElementById("arrow-canvas");

  function drawFrame() {
    const quad = document.getElementById("quadrupedImg");
    const imu = document.getElementById("imu-plot");
    const cam = document.getElementById("camera-pose");
    const ur5 = document.getElementById("ur5");

    while (svg.children.length > 1) {
      svg.removeChild(svg.lastChild);
    }

    drawArrow(quad, imu);
    drawArrow(quad, cam);
    if (direction === -1 && ur5) {
      drawArrow(ur5, quad, "Pick & Place at Origin", 0.6);
    }

    animatedCurveOffset++;
    requestAnimationFrame(drawFrame);
  }

  svg.innerHTML = `
    <defs>
      <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="10" refY="3.5" orient="auto">
        <polygon points="0 0, 10 3.5, 0 7" fill="#2d8cff" />
      </marker>
    </defs>
  `;

  requestAnimationFrame(drawFrame);
}

window.addEventListener("DOMContentLoaded", () => {
  const quadImg = document.getElementById("quadrupedImg");
  animateIMUPlot();
  animateCameraPlot(quadImg);
  simulateQuadrupedMotion();
  drawArrowsDynamically();
});
