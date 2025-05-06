console.log("Custom JS File Loaded");

// --- Global Animation Variables ---
let currentX = 0;
let animationDirection = 1;
let isDiagramSlideActive = false;
const diagramSlideIndex = 0; // Assuming diagram is always the first slide (index 0)
let imuAnimationId = null;
let cameraAnimationId = null;
let motionAnimationId = null;
let arrowAnimationId = null;
let currentSlide = 0; // Carousel slide index

// --- Helper Function ---
function isMobileView() {
  // Match the breakpoint used in CSS
  return window.matchMedia("(max-width: 768px)").matches;
}

// --- Carousel Logic ---
function showSlide(index) {
  const slides = document.querySelectorAll(".carousel-slide");
  if (!slides.length || index < 0 || index >= slides.length) {
    console.error("Invalid slide index or no slides found:", index);
    return;
  }
  if (index === currentSlide && slides[index].classList.contains("active")) {
    return;
  }
  const prevSlideIndex = currentSlide;
  console.log(`Carousel: Showing slide ${index}, leaving ${prevSlideIndex}`);
  if (prevSlideIndex === diagramSlideIndex) {
    stopDiagramAnimations(); // Stop regardless of view type
  }
  slides.forEach((slide, i) => {
    if (i === index) {
      slide.style.visibility = "visible";
      slide.style.opacity = "0";
      slide.classList.add("active");
      void slide.offsetWidth; // Force reflow
      slide.style.opacity = "1";
    } else {
      slide.style.opacity = "0";
      setTimeout(() => {
        if (!slide.classList.contains("active")) {
          slide.style.visibility = "hidden";
        }
      }, 600); // Match CSS transition duration
      slide.classList.remove("active");
    }
  });
  currentSlide = index;
  if (currentSlide === diagramSlideIndex) {
    // Use a delay before starting animations OR drawing static arrows
    setTimeout(() => {
      if (isMobileView()) {
        drawStaticArrows(); // Draw static arrows on mobile
        const status = document.querySelector(
          ".carousel-slide.active #quadStatus"
        );
        if (status) status.textContent = "System Overview"; // Static text
      } else {
        startDiagramAnimations(); // Start dynamic animations on desktop
      }
    }, 300); // Increased timeout
  }
}

function changeSlide(delta) {
  const slides = document.querySelectorAll(".carousel-slide");
  if (!slides.length) return;
  let newIndex = (currentSlide + delta + slides.length) % slides.length;
  showSlide(newIndex);
}

// --- Diagram Animation Functions ---

// --- IMU Plot ---
function _imuPlotLoop(ctx, width, height, t) {
  if (!isDiagramSlideActive || isMobileView()) {
    imuAnimationId = null;
    return;
  }
  ctx.clearRect(0, 0, width, height);
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  for (let x = 0; x < width; x++) {
    const noise = (Math.random() - 0.5) * 4;
    const amplitude = 15;
    const frequency = 0.05;
    const yValue = Math.sin((t + x) * frequency) * amplitude + noise;
    const y = height / 2 - yValue;
    ctx.lineTo(x, y);
  }
  ctx.strokeStyle = "#2d8cff";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  t += 1;
  if (isDiagramSlideActive && !isMobileView()) {
    imuAnimationId = requestAnimationFrame(() =>
      _imuPlotLoop(ctx, width, height, t)
    );
  } else {
    imuAnimationId = null;
  }
}
function startIMUAnimation() {
  if (!isDiagramSlideActive || isMobileView()) return;
  const canvas = document.querySelector(".carousel-slide.active #imuCanvas");
  if (!canvas) {
    console.warn("IMU Canvas not found in active slide");
    return;
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    console.warn("Could not get 2D context for IMU canvas");
    return;
  }
  const width = canvas.width;
  const height = canvas.height;
  console.log("Starting IMU plot loop");
  if (!imuAnimationId) {
    _imuPlotLoop(ctx, width, height, 0);
  }
}

// --- Camera Plot ---
function _cameraPlotLoop(ctx, width, height, t2) {
  if (!isDiagramSlideActive || isMobileView()) {
    cameraAnimationId = null;
    return;
  }
  ctx.clearRect(0, 0, width, height);
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  for (let x = 0; x < width; x++) {
    const amplitude1 = 10;
    const frequency1 = 0.02;
    const amplitude2 = 5;
    const frequency2 = 0.07;
    const displacement =
      Math.sin((t2 + x) * frequency1) * amplitude1 +
      Math.cos((t2 + x) * frequency2) * amplitude2;
    const y = height / 2 - displacement;
    ctx.lineTo(x, y);
  }
  ctx.strokeStyle = "#00ffaa";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  t2 += 1;
  if (isDiagramSlideActive && !isMobileView()) {
    cameraAnimationId = requestAnimationFrame(() =>
      _cameraPlotLoop(ctx, width, height, t2)
    );
  } else {
    cameraAnimationId = null;
  }
}
function startCameraAnimation() {
  if (!isDiagramSlideActive || isMobileView()) return;
  const canvas = document.querySelector(".carousel-slide.active #cameraCanvas");
  if (!canvas) {
    console.warn("Camera Canvas not found in active slide");
    return;
  }
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    console.warn("Could not get 2D context for Camera canvas");
    return;
  }
  const width = canvas.width;
  const height = canvas.height;
  console.log("Starting Camera plot loop");
  if (!cameraAnimationId) {
    _cameraPlotLoop(ctx, width, height, 0);
  }
}

// --- Quadruped Motion ---
let motionLastTime = performance.now();
function _motionLoop(timestamp) {
  if (!isDiagramSlideActive || isMobileView()) {
    motionAnimationId = null;
    return;
  }
  const activeDiagramWrapper = document.querySelector(
    ".carousel-slide.active .diagram-wrapper"
  );
  if (!activeDiagramWrapper) {
    if (isDiagramSlideActive && !isMobileView())
      motionAnimationId = requestAnimationFrame(_motionLoop);
    else motionAnimationId = null;
    return;
  }
  const cluster = activeDiagramWrapper.querySelector(".quad-cluster");
  const status = activeDiagramWrapper.querySelector("#quadStatus");
  const ur5 = activeDiagramWrapper.querySelector("#ur5");
  if (!cluster || !ur5) {
    if (isDiagramSlideActive && !isMobileView())
      motionAnimationId = requestAnimationFrame(_motionLoop);
    else motionAnimationId = null;
    return;
  }
  const deltaTime = (timestamp - motionLastTime) / 1000;
  motionLastTime = timestamp;
  const wrapperStyle = window.getComputedStyle(activeDiagramWrapper);
  const containerWidth =
    activeDiagramWrapper.clientWidth -
    parseFloat(wrapperStyle.paddingLeft) -
    parseFloat(wrapperStyle.paddingRight);
  const clusterWidth = cluster.offsetWidth;
  const padding = 10;
  if (clusterWidth <= 0 || containerWidth <= 0) {
    if (isDiagramSlideActive && !isMobileView())
      motionAnimationId = requestAnimationFrame(_motionLoop);
    else motionAnimationId = null;
    return;
  }
  const minX = padding;
  const maxX = containerWidth - clusterWidth - padding;
  const speed = 100;
  const pauseDuration = 1500;
  currentX += speed * deltaTime * animationDirection;
  currentX = Math.max(minX, Math.min(currentX, maxX));
  cluster.style.left = `${currentX}px`;
  const bobbleFrequency = 10;
  const bobbleAmplitude = 3;
  const yOffset =
    Math.sin((timestamp / 1000) * bobbleFrequency) * bobbleAmplitude;
  cluster.style.transform = `translateY(${yOffset}px)`;
  if (status)
    status.textContent =
      animationDirection === 1 ? "Moving →" : "← Returning (UR5)";
  ur5.classList.toggle("active-glow", animationDirection === -1);
  if (
    (currentX >= maxX && animationDirection === 1) ||
    (currentX <= minX && animationDirection === -1)
  ) {
    animationDirection *= -1;
    if (motionAnimationId) cancelAnimationFrame(motionAnimationId);
    motionAnimationId = setTimeout(() => {
      if (isDiagramSlideActive && !isMobileView()) {
        motionLastTime = performance.now();
        motionAnimationId = requestAnimationFrame(_motionLoop);
      } else {
        motionAnimationId = null;
      }
    }, pauseDuration);
    return;
  }
  if (isDiagramSlideActive && !isMobileView()) {
    motionAnimationId = requestAnimationFrame(_motionLoop);
  } else {
    motionAnimationId = null;
  }
}
function startMotionAnimation() {
  if (!isDiagramSlideActive || isMobileView()) return;
  console.log("Starting Motion loop");
  if (!motionAnimationId) {
    const cluster = document.querySelector(
      ".carousel-slide.active .quad-cluster"
    );
    const activeDiagramWrapper = document.querySelector(
      ".carousel-slide.active .diagram-wrapper"
    );
    if (cluster && activeDiagramWrapper) {
      const padding = 10;
      currentX = padding;
      cluster.style.left = `${currentX}px`;
      cluster.style.transform = `translateY(0px)`;
    } else {
      console.warn(
        "Could not find cluster or wrapper to reset motion position."
      );
      currentX = 10;
    }
    animationDirection = 1;
    motionLastTime = performance.now();
    motionAnimationId = requestAnimationFrame(_motionLoop);
  }
}

// --- Arrow Drawing ---
function getAnchorPoint(element, anchor, svgRect) {
  if (!element) {
    return null;
  }
  const rect = element.getBoundingClientRect();
  if (
    !svgRect ||
    svgRect.width === 0 ||
    svgRect.height === 0 ||
    rect.width === 0 ||
    rect.height === 0 ||
    rect.bottom <= 0 ||
    rect.top >= window.innerHeight ||
    rect.right <= 0 ||
    rect.left >= window.innerWidth
  ) {
    return null;
  }
  let x = rect.left - svgRect.left;
  let y = rect.top - svgRect.top;
  if (anchor.includes("top")) y = y;
  else if (anchor.includes("bottom")) y = y + rect.height;
  else y = y + rect.height / 2;
  if (anchor.includes("left")) x = x;
  else if (anchor.includes("right")) x = x + rect.width;
  else x = x + rect.width / 2;
  const offset = 5;
  if (anchor.includes("top")) y += offset;
  if (anchor.includes("bottom")) y -= offset;
  if (anchor.includes("left")) x += offset;
  if (anchor.includes("right")) x -= offset;
  return { x, y };
}
function drawArrow(
  svg,
  fromEl,
  toEl,
  label = "",
  fromAnchor = "center",
  toAnchor = "center",
  options = {}
) {
  if (!svg || !fromEl || !toEl) {
    console.warn("drawArrow: Missing SVG or anchor element.");
    return;
  }
  const svgRect = svg.getBoundingClientRect();
  if (svgRect.width === 0 || svgRect.height === 0) {
    console.warn("drawArrow: SVG not ready (zero dimensions).");
    return;
  }
  const start = getAnchorPoint(fromEl, fromAnchor, svgRect);
  const end = getAnchorPoint(toEl, toAnchor, svgRect);
  if (!start || !end) {
    return;
  }
  const x1 = start.x;
  const y1 = start.y;
  const x2 = end.x;
  const y2 = end.y;
  const d = `M ${x1.toFixed(1)},${y1.toFixed(1)} L ${x2.toFixed(
    1
  )},${y2.toFixed(1)}`;
  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", d);
  path.setAttribute("fill", "none");
  path.setAttribute("stroke", options.color || "#2d8cff");
  path.setAttribute("stroke-width", options.strokeWidth || "1.5");
  path.setAttribute("stroke-dasharray", options.dashed ? "4 4" : "none");
  const arrowheadIdBase = "arrowhead";
  const colorSuffix = options.color ? `-${options.color.replace("#", "")}` : "";
  const specificArrowheadId = arrowheadIdBase + colorSuffix;
  const defaultArrowheadId = arrowheadIdBase;
  if (svg.querySelector(`defs #${specificArrowheadId}`)) {
    path.setAttribute("marker-end", `url(#${specificArrowheadId})`);
  } else if (svg.querySelector(`defs #${defaultArrowheadId}`)) {
    path.setAttribute("marker-end", `url(#${defaultArrowheadId})`);
  } else {
    console.warn(
      "drawArrow: Arrowhead definition not found for color:",
      options.color || "default"
    );
  }
  svg.appendChild(path);
  if (label) {
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    const midX = x1 + (x2 - x1) * 0.5;
    const midY = y1 + (y2 - y1) * 0.5;
    const labelOffset = 15;
    const length = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    let offsetX = 0;
    let offsetY = -labelOffset;
    if (length > 0) {
      offsetX = ((y2 - y1) / length) * labelOffset;
      offsetY = -((x2 - x1) / length) * labelOffset;
    }
    text.setAttribute("x", midX + offsetX);
    text.setAttribute("y", midY + offsetY);
    text.setAttribute("fill", options.labelColor || "#ffffff");
    const fontSize = options.labelSize || "14";
    text.style.fontSize = `${fontSize}px`;
    text.setAttribute("text-anchor", "middle");
    text.setAttribute("dominant-baseline", "middle");
    text.textContent = label;
    svg.appendChild(text);
  }
}
function ensureArrowheadDefs(svg) {
  if (!svg) return;
  let defs = svg.querySelector("defs");
  if (!defs) {
    defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
    svg.insertBefore(defs, svg.firstChild);
  }
  const defaultArrowheadId = "arrowhead";
  if (!defs.querySelector(`#${defaultArrowheadId}`)) {
    const marker = document.createElementNS(
      "http://www.w3.org/2000/svg",
      "marker"
    );
    marker.setAttribute("id", defaultArrowheadId);
    marker.setAttribute("markerWidth", "8");
    marker.setAttribute("markerHeight", "6");
    marker.setAttribute("refX", "8");
    marker.setAttribute("refY", "3");
    marker.setAttribute("orient", "auto");
    marker.setAttribute("markerUnits", "strokeWidth");
    marker.innerHTML = `<path d="M0,0 L8,3 L0,6 Z" fill="#2d8cff" />`;
    defs.appendChild(marker);
    console.log("Added default arrowhead def");
  }
  const yellowArrowheadId = "arrowhead-ffcc00";
  if (!defs.querySelector(`#${yellowArrowheadId}`)) {
    const markerYellow = document.createElementNS(
      "http://www.w3.org/2000/svg",
      "marker"
    );
    markerYellow.setAttribute("id", yellowArrowheadId);
    markerYellow.setAttribute("markerWidth", "8");
    markerYellow.setAttribute("markerHeight", "6");
    markerYellow.setAttribute("refX", "8");
    markerYellow.setAttribute("refY", "3");
    markerYellow.setAttribute("orient", "auto");
    markerYellow.setAttribute("markerUnits", "strokeWidth");
    markerYellow.innerHTML = `<path d="M0,0 L8,3 L0,6 Z" fill="#ffcc00" />`;
    defs.appendChild(markerYellow);
    console.log("Added yellow arrowhead def");
  }
}

// --- Dynamic Arrow Loop ---
function _arrowLoop() {
  if (!isDiagramSlideActive || isMobileView()) {
    arrowAnimationId = null;
    return;
  }
  const activeSlide = document.querySelector(".carousel-slide.active");
  if (!activeSlide) {
    if (isDiagramSlideActive && !isMobileView())
      arrowAnimationId = requestAnimationFrame(_arrowLoop);
    else arrowAnimationId = null;
    return;
  }
  const svg = activeSlide.querySelector("#arrow-canvas");
  if (!svg) {
    console.warn("_arrowLoop: #arrow-canvas SVG not found in active slide.");
    if (isDiagramSlideActive && !isMobileView())
      arrowAnimationId = requestAnimationFrame(_arrowLoop);
    else arrowAnimationId = null;
    return;
  }
  const diagramWrapper = activeSlide.querySelector(".diagram-wrapper");
  if (!diagramWrapper) {
    console.warn("_arrowLoop: .diagram-wrapper not found in active slide.");
    if (isDiagramSlideActive && !isMobileView())
      arrowAnimationId = requestAnimationFrame(_arrowLoop);
    else arrowAnimationId = null;
    return;
  }
  const quadrupedImg = diagramWrapper.querySelector("#quadrupedImg");
  const imuPlot = diagramWrapper.querySelector("#imu-plot");
  const camPose = diagramWrapper.querySelector("#camera-pose");
  const ur5 = diagramWrapper.querySelector("#ur5");
  if (!quadrupedImg || !imuPlot || !camPose || !ur5) {
    console.warn(
      "_arrowLoop: One or more anchor elements not found within active diagram.",
      { quadrupedImg, imuPlot, camPose, ur5 }
    );
    if (isDiagramSlideActive && !isMobileView())
      arrowAnimationId = requestAnimationFrame(_arrowLoop);
    else arrowAnimationId = null;
    return;
  }
  const arrows = svg.querySelectorAll(
    "path:not(defs path), text:not(defs text)"
  );
  arrows.forEach((el) => el.remove());
  if (quadrupedImg && imuPlot) {
    drawArrow(svg, quadrupedImg, imuPlot, "IMU Data", "center", "center", {
      labelSize: "21",
    });
  }
  if (quadrupedImg && camPose) {
    drawArrow(svg, quadrupedImg, camPose, "Camera Pose", "center", "center", {
      labelSize: "21",
    });
  }
  if (animationDirection === -1 && ur5 && quadrupedImg) {
    drawArrow(
      svg,
      ur5,
      quadrupedImg,
      "Pick & Place",
      "right-center",
      "top-left",
      { color: "#ffcc00", strokeWidth: "2", labelSize: "21" }
    );
  }
  if (isDiagramSlideActive && !isMobileView()) {
    arrowAnimationId = requestAnimationFrame(_arrowLoop);
  } else {
    arrowAnimationId = null;
  }
}
function startArrowAnimation() {
  if (!isDiagramSlideActive || isMobileView()) return;
  const activeSlide = document.querySelector(".carousel-slide.active");
  if (!activeSlide) {
    console.warn("startArrowAnimation: Active slide not found.");
    return;
  }
  const svg = activeSlide.querySelector("#arrow-canvas");
  if (!svg) {
    console.warn(
      "startArrowAnimation: #arrow-canvas SVG not found in active slide."
    );
    return;
  }
  console.log("Starting Arrow loop");
  ensureArrowheadDefs(svg);
  if (!arrowAnimationId) {
    arrowAnimationId = requestAnimationFrame(_arrowLoop);
  }
}

// --- Static Arrow Drawing ---
function drawStaticArrows() {
  if (!isDiagramSlideActive || !isMobileView()) return; // Only run on mobile when active
  const activeSlide = document.querySelector(".carousel-slide.active");
  if (!activeSlide) {
    console.warn("drawStaticArrows: Active slide not found.");
    return;
  }
  const svg = activeSlide.querySelector("#arrow-canvas");
  if (!svg) {
    console.warn("drawStaticArrows: #arrow-canvas SVG not found.");
    return;
  }
  const diagramWrapper = activeSlide.querySelector(".diagram-wrapper");
  if (!diagramWrapper) {
    console.warn("drawStaticArrows: .diagram-wrapper not found.");
    return;
  }
  const ur5 = diagramWrapper.querySelector("#ur5");
  const quadrupedImg = diagramWrapper.querySelector("#quadrupedImg");
  const imuPlot = diagramWrapper.querySelector("#imu-plot");
  const camPose = diagramWrapper.querySelector("#camera-pose");
  if (!ur5 || !quadrupedImg || !imuPlot || !camPose) {
    console.warn(
      "drawStaticArrows: Missing one or more elements for static arrows."
    );
    return;
  }
  console.log("Drawing static arrows for mobile view...");
  ensureArrowheadDefs(svg);
  clearArrows(svg); // Clear previous arrows
  // Draw static arrows based on the mobile layout (adjust anchors as needed)
  drawArrow(
    svg,
    ur5,
    quadrupedImg,
    "Placement",
    "bottom-center",
    "top-center",
    { color: "#ffcc00", strokeWidth: "2", labelSize: "18" }
  );
  drawArrow(
    svg,
    quadrupedImg,
    imuPlot,
    "IMU Data",
    "bottom-center",
    "top-center",
    { labelSize: "18" }
  );
  drawArrow(
    svg,
    quadrupedImg,
    camPose,
    "Camera Pose",
    "bottom-center",
    "top-center",
    { labelSize: "18" }
  );
}

// --- Helper to clear arrows ---
function clearArrows(svg) {
  if (!svg) {
    const activeSlide = document.querySelector(".carousel-slide.active");
    if (activeSlide) {
      svg = activeSlide.querySelector("#arrow-canvas");
    }
  }
  if (svg) {
    const arrows = svg.querySelectorAll(
      "path:not(defs path), text:not(defs text)"
    );
    arrows.forEach((el) => el.remove());
  }
}

// --- Master Start/Stop ---
function startDiagramAnimations() {
  const activeDiagramSlide = document.querySelector(
    `.carousel-slide:nth-child(${diagramSlideIndex + 1}).active`
  );
  if (!activeDiagramSlide || isDiagramSlideActive) {
    return;
  }
  if (isMobileView()) {
    console.log("Mobile view detected: Drawing static arrows.");
    isDiagramSlideActive = true;
    drawStaticArrows();
    const status = activeDiagramSlide.querySelector("#quadStatus");
    if (status) status.textContent = "System Overview";
  } else {
    console.log("Desktop view detected: Starting ALL dynamic animations...");
    isDiagramSlideActive = true;
    startIMUAnimation();
    startCameraAnimation();
    startMotionAnimation();
    startArrowAnimation();
  }
}
function stopDiagramAnimations() {
  if (!isDiagramSlideActive) return;
  console.log("Stopping ALL diagram animations / Clearing static arrows...");
  isDiagramSlideActive = false;
  if (imuAnimationId) cancelAnimationFrame(imuAnimationId);
  if (cameraAnimationId) cancelAnimationFrame(cameraAnimationId);
  if (motionAnimationId) {
    cancelAnimationFrame(motionAnimationId);
    clearTimeout(motionAnimationId);
  }
  if (arrowAnimationId) cancelAnimationFrame(arrowAnimationId);
  imuAnimationId = null;
  cameraAnimationId = null;
  motionAnimationId = null;
  arrowAnimationId = null;
  clearArrows(); // Clear static/dynamic arrows
  const ur5 = document.querySelector("#ur5");
  if (ur5) ur5.classList.remove("active-glow");
  const cluster = document.querySelector(".quad-cluster");
  if (cluster && cluster.style.position === "absolute") {
    const padding = 10;
    cluster.style.left = `${padding}px`;
    cluster.style.transform = `translateY(0px)`;
  }
  const status = document.querySelector("#quadStatus");
  if (status) status.textContent = "Initializing...";
}

// --- Scroll Animation Logic ---
function initializeScrollAnimations() {
  console.log("DEBUG: Initializing Scroll Animations...");
  const sections = document.querySelectorAll(
    '#main.wrapper[class*="style"] .container > section'
  );
  console.log(`DEBUG: Found ${sections.length} sections for scroll animation.`);
  if (!sections.length) return;
  sections.forEach((section) => {
    section.classList.add("scroll-animate-init");
  });
  const observerOptions = { root: null, rootMargin: "0px", threshold: 0.15 };
  const observerCallback = (entries, observer) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        console.log(
          `DEBUG: Section ${entry.target.id || "without ID"} is intersecting.`
        );
        entry.target.classList.add("animate-in");
        observer.unobserve(entry.target);
      }
    });
  };
  const scrollObserver = new IntersectionObserver(
    observerCallback,
    observerOptions
  );
  sections.forEach((section) => {
    console.log(`DEBUG: Observing section: ${section.id || "without ID"}`);
    scrollObserver.observe(section);
  });
}

// --- Tab Functionality ---
window.openTab = function (evt, tabName) {
  var i, tabcontent, tablinks;
  tabcontent = document.getElementsByClassName("tabcontent");
  for (i = 0; i < tabcontent.length; i++) {
    tabcontent[i].style.display = "none";
    tabcontent[i].classList.remove("active-content");
  }
  tablinks = document.getElementsByClassName("tablinks");
  for (i = 0; i < tablinks.length; i++) {
    tablinks[i].className = tablinks[i].className.replace(" active", "");
  }
  const tabElement = document.getElementById(tabName);
  if (tabElement) {
    tabElement.style.display = "block";
    tabElement.classList.add("active-content");
    if (typeof Prism !== "undefined") {
      Prism.highlightAllUnder(tabElement);
    } else {
      console.warn("Prism not loaded when trying to highlight tab:", tabName);
    }
  } else {
    console.error("Tab content element not found:", tabName);
  }
  if (evt && evt.currentTarget) {
    evt.currentTarget.className += " active";
  } else {
    const buttons = document.getElementsByClassName("tablinks");
    for (let btn of buttons) {
      const onclickAttr = btn.getAttribute("onclick");
      if (onclickAttr && onclickAttr.includes(`'${tabName}'`)) {
        btn.className += " active";
        break;
      }
    }
  }
};

// --- Gallery Expansion ---
function initializeGalleryExpansion() {
  const galleryContainer = document.querySelector(".gallery-container");
  const overlay = document.querySelector(".card-overlay");
  if (!galleryContainer || !overlay) {
    console.warn("Gallery container or overlay not found. Expansion disabled.");
    return;
  }
  console.log("DEBUG: Initializing Gallery Expansion Features...");
  let currentlyExpandedCard = null;
  function closeExpandedCard() {
    if (currentlyExpandedCard) {
      currentlyExpandedCard.classList.remove("card-expanded");
      overlay.classList.remove("active");
      document.body.classList.remove("no-scroll");
      currentlyExpandedCard = null;
    }
  }
  galleryContainer.addEventListener("click", (event) => {
    const card = event.target.closest(".gallery-card");
    if (!card) return;
    const isButtonOrLinkClick = event.target.closest(
      ".card-content .button, .card-content a:not(.card-drive-link-placeholder)"
    );
    const isCloseButtonClick =
      event.target.classList.contains("card-close-btn");
    if (
      !isButtonOrLinkClick &&
      !isCloseButtonClick &&
      !card.classList.contains("card-expanded")
    ) {
      closeExpandedCard();
      card.classList.add("card-expanded");
      overlay.classList.add("active");
      document.body.classList.add("no-scroll");
      currentlyExpandedCard = card;
    }
    if (isCloseButtonClick) {
      closeExpandedCard();
    }
  });
  overlay.addEventListener("click", closeExpandedCard);
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      closeExpandedCard();
    }
  });
}

// --- Sticky Header ---
function initializeStickyHeader() {
  const header = document.getElementById("header");
  const pageWrapper = document.getElementById("page-wrapper");
  if (!header || !pageWrapper) {
    console.warn(
      "DEBUG: Header or Page Wrapper element not found for sticky header."
    );
    return;
  }
  console.log("DEBUG: Initializing Sticky Header...");
  const headerHeight = header.offsetHeight;
  pageWrapper.style.paddingTop = `${headerHeight}px`;
  console.log(`DEBUG: Set page-wrapper padding-top to ${headerHeight}px`);
  const scrollThreshold = 10;
  let scrollTimeout;
  window.addEventListener(
    "scroll",
    () => {
      clearTimeout(scrollTimeout);
      scrollTimeout = setTimeout(() => {
        if (window.scrollY > scrollThreshold) {
          header.classList.add("scrolled");
        } else {
          header.classList.remove("scrolled");
        }
      }, 50);
    },
    { passive: true }
  );
}
function initializeSidebarHighlighting() {
  const sidebar = document.getElementById('sidebar');
  const mainContent = document.getElementById('main'); // Target the main content area

  // Check if sidebar and main content exist on the page
  if (!sidebar || !mainContent) {
      // console.log("DEBUG: Sidebar or main content not found, skipping highlighting init.");
      return;
  }
  console.log("DEBUG: Initializing Sidebar Highlighting...");

  const navLinks = sidebar.querySelectorAll('.sidebar-nav a[href^="#"]'); // Select only internal links
  const sections = [];
  navLinks.forEach(link => {
      const sectionId = link.getAttribute('href').substring(1); // Get ID from href
      const section = document.getElementById(sectionId);
      if (section) {
          sections.push(section);
      } else {
          console.warn(`Sidebar link points to non-existent section: #${sectionId}`);
      }
  });

  if (sections.length === 0) {
      console.warn("DEBUG: No sections found for sidebar highlighting.");
      return;
  }

  let scrollTimeout;
  const offset = 150; // Pixels offset from top to trigger highlight change

  function highlightLink() {
      let currentSectionId = '';
      const scrollPosition = window.scrollY + offset;

      // Iterate backwards to find the last section whose top is above the scroll position
      for (let i = sections.length - 1; i >= 0; i--) {
          if (sections[i].offsetTop <= scrollPosition) {
              currentSectionId = sections[i].id;
              break; // Found the current section
          }
      }

      // If scrolled past the last section, keep the last one active
      const bottomScrollPosition = document.documentElement.scrollHeight - window.innerHeight;
      if (window.scrollY >= bottomScrollPosition - offset && sections.length > 0) {
           currentSectionId = sections[sections.length - 1].id;
      }


      navLinks.forEach(link => {
          link.classList.remove('active');
          if (link.getAttribute('href') === `#${currentSectionId}`) {
              link.classList.add('active');
          }
      });
  }

  // Initial highlight on load
  highlightLink();

  // Highlight on scroll (debounced)
  window.addEventListener('scroll', () => {
      clearTimeout(scrollTimeout);
      scrollTimeout = setTimeout(highlightLink, 50); // Adjust debounce time if needed
  }, { passive: true });

   // Highlight on sidebar link click (for smooth scroll)
   navLinks.forEach(link => {
      link.addEventListener('click', () => {
          // Remove active class immediately for visual feedback
          navLinks.forEach(l => l.classList.remove('active'));
          // Add active class to the clicked link
          link.classList.add('active');
          // Note: The actual scroll happens due to the href="#...",
          // the highlightLink on scroll will confirm later.
      });
  });
}
// --- Initialization --- //
document.addEventListener("DOMContentLoaded", () => {
  console.log("DOM Content Loaded - Initializing Custom JS Features...");

  // --- Initialize Carousel/Diagram IF PRESENT ---
  const carouselContainer = document.querySelector(".carousel-container");
  if (carouselContainer) {
    console.log("DEBUG: Initializing Carousel/Diagram Features...");
    const slides = carouselContainer.querySelectorAll(".carousel-slide");
    if (slides.length > 0) {
      // ... (slide initialization logic remains the same) ...
       slides.forEach((slide, index) => { if (index === currentSlide) { slide.classList.add("active"); slide.style.visibility = "visible"; slide.style.opacity = "1"; console.log(`DEBUG: Initial active slide set to ${index}`); } else { slide.classList.remove("active"); slide.style.visibility = "hidden"; slide.style.opacity = "0"; } });
    } else {
      console.log("DEBUG: No slides found in carousel container.");
    }

    // --- DEBUG BUTTON LISTENERS ---
    const prevButton = carouselContainer.querySelector(".carousel-btn.prev");
    const nextButton = carouselContainer.querySelector(".carousel-btn.next");

    // Check Prev Button
    if (prevButton) {
        console.log("DEBUG: Found Prev button element:", prevButton);
        if (typeof changeSlide === 'function') {
            prevButton.addEventListener("click", () => {
                console.log("DEBUG: Prev button clicked!"); // Log click
                changeSlide(-1);
            });
            console.log("DEBUG: Prev button listener ADDED.");
        } else {
            console.error("DEBUG: changeSlide function not found for Prev button.");
        }
    } else {
        console.error("DEBUG: Prev button element NOT FOUND."); // Log if not found
    }

    // Check Next Button (Keep existing check)
    if (nextButton) {
        console.log("DEBUG: Found Next button element:", nextButton);
        if (typeof changeSlide === 'function') {
            nextButton.addEventListener("click", () => {
                 console.log("DEBUG: Next button clicked!"); // Log click
                 changeSlide(1);
            });
            console.log("DEBUG: Next button listener ADDED.");
        } else {
             console.error("DEBUG: changeSlide function not found for Next button.");
        }
    } else {
        console.error("DEBUG: Next button element NOT FOUND.");
    }
    // --- END DEBUG BUTTON LISTENERS ---


    // Check if initial slide is the diagram slide and start animations
    if (currentSlide === diagramSlideIndex) {
        // ... (rest of initial animation start logic) ...
        console.log("DEBUG: Initial slide is diagram slide."); setTimeout(() => { if (isMobileView()) { console.log("DEBUG: Initial draw static arrows (mobile)."); isDiagramSlideActive = true; drawStaticArrows(); const status = document.querySelector(".carousel-slide.active #quadStatus"); if(status) status.textContent = "System Overview"; } else if (typeof startDiagramAnimations === 'function') { console.log("DEBUG: Initial start dynamic animations (desktop)."); startDiagramAnimations(); } }, 300);
    }
  } else {
    console.log("DEBUG: Carousel container not found on this page.");
  }    if (currentSlide === diagramSlideIndex) {
      console.log("DEBUG: Initial slide is diagram slide.");
      setTimeout(() => {
        if (isMobileView()) {
          console.log("DEBUG: Initial draw static arrows (mobile).");
          isDiagramSlideActive = true;
          drawStaticArrows();
          const status = document.querySelector(
            ".carousel-slide.active #quadStatus"
          );
          if (status) status.textContent = "System Overview";
        } else if (typeof startDiagramAnimations === "function") {
          console.log("DEBUG: Initial start dynamic animations (desktop).");
          startDiagramAnimations();
        }
      }, 300);
    }

  // --- Initialize Gallery Expansion IF PRESENT ---
  initializeGalleryExpansion(); // Call the function

  // --- Initialize Scroll Animations IF PRESENT ---
  if (
    document.querySelector('#main.wrapper[class*="style"] .container > section')
  ) {
    console.log("DEBUG: Initializing Scroll Animations...");
    if (typeof initializeScrollAnimations === "function") {
      initializeScrollAnimations();
    }
  } else {
    console.log(
      "DEBUG: Scroll animation target structure not found on this page."
    );
  }

  // --- Initialize Tabs IF PRESENT ---
  const defaultTabButton = document.getElementById("defaultOpen");
  if (defaultTabButton || document.querySelector(".tablinks")) {
    console.log("DEBUG: Initializing Tab Features...");
    let initialTabName = "Overview";
    if (defaultTabButton) {
      const onclickAttr = defaultTabButton.getAttribute("onclick");
      const tabNameMatch = onclickAttr
        ? onclickAttr.match(/openTab\(event, '(.*?)'\)/)
        : null;
      if (tabNameMatch && tabNameMatch[1]) {
        initialTabName = tabNameMatch[1];
      }
    } else {
      const firstTabButton = document.querySelector(".tablinks");
      if (firstTabButton) {
        const onclickAttr = firstTabButton.getAttribute("onclick");
        const tabNameMatch = onclickAttr
          ? onclickAttr.match(/openTab\(event, '(.*?)'\)/)
          : null;
        if (tabNameMatch && tabNameMatch[1]) {
          initialTabName = tabNameMatch[1];
        }
      }
    }
    if (typeof window.openTab === "function") {
      window.openTab(null, initialTabName);
      console.log(`DEBUG: Attempted to open initial tab '${initialTabName}'.`);
    } else {
      console.error(
        "DEBUG: openTab function not found for tab initialization."
      );
    }
    const MAX_PRISM_CHECKS = 20;
    let prismCheckCount = 0;
    const prismCheckInterval = setInterval(() => {
      prismCheckCount++;
      if (typeof Prism !== "undefined") {
        clearInterval(prismCheckInterval);
        Prism.highlightAll();
        console.log("DEBUG: Prism loaded and initial highlighting done.");
      } else if (prismCheckCount >= MAX_PRISM_CHECKS) {
        clearInterval(prismCheckInterval);
        console.error("Prism failed to load after multiple checks.");
      }
    }, 100);
  } else {
    console.log("DEBUG: Tab elements not found on this page.");
  }

  // --- Initialize Sticky Header ---
  initializeStickyHeader(); // Call the function
  initializeSidebarHighlighting();
  // --- Resize Listener for Animation Handling ---
  let resizeTimeout;
  window.addEventListener("resize", () => {
    clearTimeout(resizeTimeout);
    resizeTimeout = setTimeout(() => {
      console.log("Window resized");
      if (currentSlide === diagramSlideIndex) {
        // Only act if diagram slide is active
        const mobileNow = isMobileView();
        const wasAnimating = motionAnimationId !== null; // Check if dynamic animations were running

        if (mobileNow && wasAnimating) {
          // Switched TO mobile FROM desktop
          console.log(
            "Resized into mobile view: Stopping dynamic, drawing static."
          );
          stopDiagramAnimations();
          isDiagramSlideActive = true; // Set flag for static view
          drawStaticArrows();
          const status = document.querySelector(
            ".carousel-slide.active #quadStatus"
          );
          if (status) status.textContent = "System Overview";
        } else if (!mobileNow && !wasAnimating && isDiagramSlideActive) {
          // Switched TO desktop FROM mobile (static was showing)
          console.log(
            "Resized into desktop view: Clearing static, starting dynamic."
          );
          stopDiagramAnimations(); // Clears static arrows and resets flag
          startDiagramAnimations();
        } else if (mobileNow && !wasAnimating && isDiagramSlideActive) {
          // Still mobile, redraw static arrows in case layout changed
          console.log("Still mobile view: Redrawing static arrows.");
          drawStaticArrows();
        }
        // No action needed if still desktop and animations were running
      }

      // Recalculate header padding on resize if using dynamic method
      const header = document.getElementById("header");
      const pageWrapper = document.getElementById("page-wrapper");
      if (header && pageWrapper) {
        const headerHeight = header.offsetHeight;
        pageWrapper.style.paddingTop = `${headerHeight}px`;
        // console.log(`DEBUG: Recalculated page-wrapper padding-top to ${headerHeight}px`);
      }
    }, 250); // Debounce resize events
  });
}); // End of DOMContentLoaded listener
