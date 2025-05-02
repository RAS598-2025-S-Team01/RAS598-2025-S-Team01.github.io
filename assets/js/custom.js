// --- PASTE THE ENTIRE JAVASCRIPT BLOCK FROM THE PREVIOUS ANSWER HERE ---
console.log("Custom JS File Loaded"); // Check if file loads

// --- Global variables for animations --- //
let currentX = 0; // Shared X position for quadruped (relative to diagram-wrapper)
let animationDirection = 1; // 1 for right, -1 for left
let isDiagramSlideActive = false; // Track if the diagram slide is active
const diagramSlideIndex = 0; // The index of the slide containing the diagram

// Animation Frame IDs
let imuAnimationId = null;
let cameraAnimationId = null;
let motionAnimationId = null;
let arrowAnimationId = null;

// --- Carousel Logic --- //
let currentSlide = 0; // Keep track of the active slide index

function showSlide(index) {
  const slides = document.querySelectorAll(".carousel-slide");
  if (!slides.length || index < 0 || index >= slides.length) {
    console.error("Invalid slide index or no slides found:", index);
    return;
  }
  // Prevent re-triggering on same slide unless it's not active
  if (index === currentSlide && slides[index].classList.contains("active")) {
    return;
  }

  const prevSlideIndex = currentSlide;
  console.log(
    `Carousel: Attempting to show slide ${index}, leaving ${prevSlideIndex}`
  );

  // Stop animations if leaving the diagram slide
  if (prevSlideIndex === diagramSlideIndex) {
    stopDiagramAnimations();
  }

  slides.forEach((slide, i) => {
    if (i === index) {
      // Prepare the incoming slide
      slide.style.visibility = "visible"; // Make it visible before transition starts
      slide.style.opacity = "0"; // Start transparent
      slide.classList.add("active");
      // Force reflow/repaint before adding opacity class
      void slide.offsetWidth;
      slide.style.opacity = "1"; // Fade in
    } else {
      // Fade out the outgoing slide
      slide.style.opacity = "0";
      // After fade out, hide it and remove active class
      setTimeout(() => {
        if (!slide.classList.contains("active")) {
          // Check again in case user switched quickly
          slide.style.visibility = "hidden";
        }
      }, 600); // Match transition duration
      slide.classList.remove("active");
    }
  });

  currentSlide = index; // Update index

  // Start animations if entering the diagram slide
  if (currentSlide === diagramSlideIndex) {
    // Use a small delay to ensure the slide is fully visible and CSS applied
    setTimeout(startDiagramAnimations, 150);
  }
}

function changeSlide(delta) {
  const slides = document.querySelectorAll(".carousel-slide");
  if (!slides.length) return;
  let newIndex = (currentSlide + delta + slides.length) % slides.length;
  showSlide(newIndex);
}

// --- Diagram Animation Functions --- //

// --- IMU Plot ---
function _imuPlotLoop(ctx, width, height, t) {
  // Check flag at the beginning of each frame
  if (!isDiagramSlideActive) {
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
  // Request next frame only if still active
  if (isDiagramSlideActive) {
    imuAnimationId = requestAnimationFrame(() =>
      _imuPlotLoop(ctx, width, height, t)
    );
  } else {
    imuAnimationId = null;
  }
}
function startIMUAnimation() {
  if (!isDiagramSlideActive) return; // Check flag before starting
  const canvas = document.querySelector(
    ".carousel-slide.active #imuCanvas"
  ); // Target active slide
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return; // Add null check for context
  const width = canvas.width;
  const height = canvas.height;
  console.log("Starting IMU plot loop");
  // Ensure only one loop runs
  if (!imuAnimationId) {
    _imuPlotLoop(ctx, width, height, 0);
  }
}

// --- Camera Plot ---
function _cameraPlotLoop(ctx, width, height, t2) {
  // Check flag at the beginning of each frame
  if (!isDiagramSlideActive) {
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
  // Request next frame only if still active
  if (isDiagramSlideActive) {
    cameraAnimationId = requestAnimationFrame(() =>
      _cameraPlotLoop(ctx, width, height, t2)
    );
  } else {
    cameraAnimationId = null;
  }
}
function startCameraAnimation() {
  if (!isDiagramSlideActive) return; // Check flag before starting
  const canvas = document.querySelector(
    ".carousel-slide.active #cameraCanvas"
  ); // Target active slide
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return; // Add null check for context
  const width = canvas.width;
  const height = canvas.height;
  console.log("Starting Camera plot loop");
  // Ensure only one loop runs
  if (!cameraAnimationId) {
    _cameraPlotLoop(ctx, width, height, 0);
  }
}

// --- Quadruped Motion ---
let motionLastTime = performance.now();
function _motionLoop(timestamp) {
  // Check flag at the beginning of each frame
  if (!isDiagramSlideActive) {
    motionAnimationId = null;
    return;
  }
  // Target elements ONLY in active slide's diagram wrapper
  const activeDiagramWrapper = document.querySelector(
    ".carousel-slide.active .diagram-wrapper"
  );
  if (!activeDiagramWrapper) {
    // If diagram wrapper not found in active slide, wait for next frame
    if (isDiagramSlideActive)
      motionAnimationId = requestAnimationFrame(_motionLoop);
    else motionAnimationId = null;
    return;
  }

  const cluster = activeDiagramWrapper.querySelector(".quad-cluster");
  const status = activeDiagramWrapper.querySelector("#quadStatus");
  const ur5 = activeDiagramWrapper.querySelector("#ur5");

  // Important: Only proceed if the elements exist in the active slide
  if (!cluster || !ur5) {
    if (isDiagramSlideActive)
      motionAnimationId = requestAnimationFrame(_motionLoop);
    else motionAnimationId = null;
    return;
  }

  const deltaTime = (timestamp - motionLastTime) / 1000;
  motionLastTime = timestamp;

  // Use the diagram wrapper as the container for bounds calculation
  // Get computed style to account for padding accurately
  const wrapperStyle = window.getComputedStyle(activeDiagramWrapper);
  const containerWidth =
    activeDiagramWrapper.clientWidth -
    parseFloat(wrapperStyle.paddingLeft) -
    parseFloat(wrapperStyle.paddingRight);
  const clusterWidth = cluster.offsetWidth;
  const padding = 10; // Reduced padding for tighter bounds inside wrapper

  if (clusterWidth <= 0 || containerWidth <= 0) {
    if (isDiagramSlideActive)
      motionAnimationId = requestAnimationFrame(_motionLoop);
    else motionAnimationId = null;
    return;
  }

  // Calculate bounds based on absolute positioning within diagram-wrapper's content area
  const minX = padding; // Left bound (relative to wrapper padding edge)
  const maxX = containerWidth - clusterWidth - padding; // Right bound (relative to wrapper padding edge)

  const speed = 100;
  const pauseDuration = 1500;

  currentX += speed * deltaTime * animationDirection;
  // Clamp position
  currentX = Math.max(minX, Math.min(currentX, maxX));

  // Apply transform using left style for absolute positioning
  // The 'left' value is relative to the diagram-wrapper's padding box
  cluster.style.left = `${currentX}px`;

  // Optional: Add bobble effect using transform (relative to its current position)
  const bobbleFrequency = 10;
  const bobbleAmplitude = 3;
  const yOffset =
    Math.sin((timestamp / 1000) * bobbleFrequency) * bobbleAmplitude;
  cluster.style.transform = `translateY(${yOffset}px)`;

  if (status)
    status.textContent =
      animationDirection === 1 ? "Moving →" : "← Returning (UR5)";
  ur5.classList.toggle("active-glow", animationDirection === -1);

  // Check bounds and reverse direction
  if (
    (currentX >= maxX && animationDirection === 1) ||
    (currentX <= minX && animationDirection === -1)
  ) {
    animationDirection *= -1;
    // Use setTimeout for pause, then requestAnimationFrame
    // Clear previous animation frame ID before setting timeout ID
    if (motionAnimationId) cancelAnimationFrame(motionAnimationId);
    motionAnimationId = setTimeout(() => {
      // Check flag again after timeout before restarting
      if (isDiagramSlideActive) {
        motionLastTime = performance.now(); // Reset time after pause
        motionAnimationId = requestAnimationFrame(_motionLoop);
      } else {
        motionAnimationId = null;
      }
    }, pauseDuration);
    return; // Exit current frame to prevent immediate next step
  }

  // Continue animation if bounds not hit and still active
  if (isDiagramSlideActive) {
    motionAnimationId = requestAnimationFrame(_motionLoop);
  } else {
    motionAnimationId = null;
  }
}

function startMotionAnimation() {
  if (!isDiagramSlideActive) return; // Check flag before starting
  console.log("Starting Motion loop");
  // Ensure only one loop runs
  if (!motionAnimationId) {
    // Reset position when starting animation on this slide
    const cluster = document.querySelector(
      ".carousel-slide.active .quad-cluster"
    );
    const activeDiagramWrapper = document.querySelector(
      ".carousel-slide.active .diagram-wrapper"
    );
    if (cluster && activeDiagramWrapper) {
      const padding = 10; // Use same padding as in loop
      currentX = padding; // Start near the left padding edge
      cluster.style.left = `${currentX}px`; // Set initial style
      cluster.style.transform = `translateY(0px)`; // Reset bobble
    } else {
      console.warn(
        "Could not find cluster or wrapper to reset motion position."
      );
      currentX = 10; // Default start
    }
    animationDirection = 1; // Ensure starting right
    motionLastTime = performance.now();
    motionAnimationId = requestAnimationFrame(_motionLoop);
  }
}

// --- Arrow Drawing (Enhanced with Anchors & Curve Control) ---
function getAnchorPoint(element, anchor, svgRect) {
  if (!element) return { x: 0, y: 0 };
  const rect = element.getBoundingClientRect();
  // Check if element is visible and has dimensions
  if (
    rect.width === 0 ||
    rect.height === 0 ||
    rect.bottom <= svgRect.top || // Use SVG rect for viewport check
    rect.top >= svgRect.bottom ||
    rect.right <= svgRect.left ||
    rect.left >= svgRect.right
  ) {
    // console.warn("Anchor element not visible or has no dimensions:", element.id);
    return { x: 0, y: 0 }; // Return 0,0 if not ready/visible
  }

  // Calculate position relative to SVG top-left
  let x = rect.left - svgRect.left;
  let y = rect.top - svgRect.top;

  // Adjust based on anchor keyword
  if (anchor.includes("top")) y = y; // Already top edge
  else if (anchor.includes("bottom")) y = y + rect.height;
  else y = y + rect.height / 2; // Default vertical center

  if (anchor.includes("left")) x = x; // Already left edge
  else if (anchor.includes("right")) x = x + rect.width;
  else x = x + rect.width / 2; // Default horizontal center

  // Apply small offset to avoid starting exactly on edge
  const offset = 5;
  if (anchor.includes("top")) y += offset;
  if (anchor.includes("bottom")) y -= offset;
  if (anchor.includes("left")) x += offset;
  if (anchor.includes("right")) x -= offset;

  // Clamp coordinates to be within SVG bounds (with a small margin)
  const margin = 2;
  x = Math.max(margin, Math.min(svgRect.width - margin, x));
  y = Math.max(margin, Math.min(svgRect.height - margin, y));

  return { x, y };
}

function drawArrow(
  svg,
  fromEl,
  toEl,
  label = "",
  fromAnchor = "center",
  toAnchor = "center",
  options = {} // Options object is the last parameter
) {
  if (!svg || !fromEl || !toEl) return;

  const svgRect = svg.getBoundingClientRect();
  if (svgRect.width === 0 || svgRect.height === 0) return; // SVG not ready

  const start = getAnchorPoint(fromEl, fromAnchor, svgRect);
  const end = getAnchorPoint(toEl, toAnchor, svgRect);

  // Check if points are valid
  if (
    (start.x === 0 && start.y === 0) ||
    (end.x === 0 && end.y === 0)
  ) {
    return;
  }

  const x1 = start.x;
  const y1 = start.y;
  const x2 = end.x;
  const y2 = end.y;

  // --- Straight Line Path ---
  const d = `M ${x1},${y1} L ${x2},${y2}`;

  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", d);
  path.setAttribute("fill", "none");
  path.setAttribute("stroke", options.color || "#2d8cff");
  path.setAttribute("stroke-width", options.strokeWidth || "1.5");
  path.setAttribute(
    "stroke-dasharray",
    options.dashed ? "4 4" : "none"
  );

  // Determine arrowhead ID
  const arrowheadIdBase = "arrowhead";
  const colorSuffix = options.color ? `-${options.color.replace("#", "")}` : "";
  const specificArrowheadId = arrowheadIdBase + colorSuffix;
  const defaultArrowheadId = arrowheadIdBase;

  if (svg.querySelector(`defs #${specificArrowheadId}`)) {
    path.setAttribute("marker-end", `url(#${specificArrowheadId})`);
  } else if (svg.querySelector(`defs #${defaultArrowheadId}`)) {
    path.setAttribute("marker-end", `url(#${defaultArrowheadId})`);
  }

  svg.appendChild(path);

  // --- Label Positioning ---
  if (label) {
    const text = document.createElementNS(
      "http://www.w3.org/2000/svg",
      "text"
    );
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

    // Use the style attribute instead of the font-size attribute
    const fontSize = options.labelSize || "14"; // Get desired size or default
    text.style.fontSize = `${fontSize}px`; // Set via style property (add 'px')

    text.setAttribute("text-anchor", "middle");
    text.setAttribute("dominant-baseline", "middle");
    text.textContent = label;

    svg.appendChild(text);
  }
}

// --- Arrow Loop (Calls match updated drawArrow signature) ---
function _arrowLoop() {
  // Check flag at the beginning of each frame
  if (!isDiagramSlideActive) {
    arrowAnimationId = null;
    return;
  }
  const svg = document.querySelector(
    ".carousel-slide.active #arrow-canvas"
  );
  if (!svg) {
    if (isDiagramSlideActive)
      arrowAnimationId = requestAnimationFrame(_arrowLoop);
    else arrowAnimationId = null;
    return;
  }

  const diagramWrapper = svg.closest(".diagram-wrapper");
  if (!diagramWrapper) {
    if (isDiagramSlideActive)
      arrowAnimationId = requestAnimationFrame(_arrowLoop);
    else arrowAnimationId = null;
    return;
  }

  // Get elements
  const quadCluster = diagramWrapper.querySelector(".quad-cluster");
  const imuPlot = diagramWrapper.querySelector("#imu-plot");
  const camPose = diagramWrapper.querySelector("#camera-pose");
  const ur5 = diagramWrapper.querySelector("#ur5");
  const quadrupedImg = diagramWrapper.querySelector("#quadrupedImg");

  // Clear previous arrows
  const arrows = svg.querySelectorAll(
    "path:not(defs path), text:not(defs text)"
  );
  arrows.forEach((el) => el.remove());

  // --- Draw Arrows ---

  // Quad Image -> IMU Plot
  if (quadrupedImg && imuPlot) {
    drawArrow(
      svg,
      quadrupedImg,
      imuPlot,
      "IMU Data",
      "center",
      "top-left",
      // Options object is the LAST argument
      { labelSize: "21" } // INCREASED FONT SIZE
    );
  }

  // Quad Image -> Camera Plot
  if (quadrupedImg && camPose) {
    drawArrow(
      svg,
      quadrupedImg,
      camPose,
      "Camera Pose",
      "center",
      "top-left",
      // Options object is the LAST argument
      { labelSize: "21" } // INCREASED FONT SIZE
    );
  }

  // UR5 -> Quad Image (Only when returning)
  if (animationDirection === -1 && ur5 && quadrupedImg) {
    drawArrow(
      svg,
      ur5,
      quadrupedImg,
      "Pick & Place",
      "right-center",
      "top-left",
      // Options object is the LAST argument
      { color: "#ffcc00", strokeWidth: "2", labelSize: "21" } // Explicitly set size if needed
    );
  }

  // Request next frame
  if (isDiagramSlideActive) {
    arrowAnimationId = requestAnimationFrame(_arrowLoop);
  } else {
    arrowAnimationId = null;
  }
}

// --- Ensure Arrowhead Definition Exists ---
function ensureArrowheadDefs(svg) {
  if (!svg) return;
  let defs = svg.querySelector("defs");
  if (!defs) {
    defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
    svg.insertBefore(defs, svg.firstChild);
  }

  // Define default arrowhead if missing
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
    marker.innerHTML = `<path d="M0,0 L8,3 L0,6 Z" fill="#2d8cff" />`; // Default color
    defs.appendChild(marker);
    console.log("Added default arrowhead def");
  }

  // Define yellow arrowhead if missing
  const yellowArrowheadId = "arrowhead-ffcc00"; // ID based on color #ffcc00
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
    markerYellow.innerHTML = `<path d="M0,0 L8,3 L0,6 Z" fill="#ffcc00" />`; // Yellow color
    defs.appendChild(markerYellow);
    console.log("Added yellow arrowhead def");
  }
}

function startArrowAnimation() {
  if (!isDiagramSlideActive) return; // Check flag before starting
  const svg = document.querySelector(".carousel-slide.active #arrow-canvas");
  if (!svg) return;
  console.log("Starting Arrow loop");
  ensureArrowheadDefs(svg); // Make sure definitions exist
  // Ensure only one loop runs
  if (!arrowAnimationId) {
    _arrowLoop();
  }
}

// --- Master Start/Stop for Diagram Animations ---
function startDiagramAnimations() {
  // Check if the diagram slide is actually the active one
  const activeDiagramSlide = document.querySelector(
    `.carousel-slide:nth-child(${diagramSlideIndex + 1}).active`
  );
  // Only start if the correct slide is active AND animations aren't already running
  if (!activeDiagramSlide || isDiagramSlideActive) {
    return;
  }

  console.log("Starting ALL diagram animations...");
  isDiagramSlideActive = true; // Set flag immediately
  // Start individual animations
  startIMUAnimation();
  startCameraAnimation();
  startMotionAnimation();
  startArrowAnimation();
}

function stopDiagramAnimations() {
  if (!isDiagramSlideActive) return; // Only stop if active
  console.log("Stopping ALL diagram animations...");
  isDiagramSlideActive = false; // Set flag first

  // Cancel animation frames and clear timeouts
  if (imuAnimationId) cancelAnimationFrame(imuAnimationId);
  if (cameraAnimationId) cancelAnimationFrame(cameraAnimationId);
  if (motionAnimationId) {
    cancelAnimationFrame(motionAnimationId);
    clearTimeout(motionAnimationId); // Clear potential pause timeout
  }
  if (arrowAnimationId) cancelAnimationFrame(arrowAnimationId);

  // Reset IDs
  imuAnimationId = null;
  cameraAnimationId = null;
  motionAnimationId = null;
  arrowAnimationId = null;

  // Also reset UR5 glow when stopping
  // Use a more general selector in case the active slide changes during stop
  const ur5 = document.querySelector("#ur5");
  if (ur5) ur5.classList.remove("active-glow");

  // Optional: Reset quad position visually if needed when stopped
  const cluster = document.querySelector(".quad-cluster"); // General selector
  if (cluster && cluster.style.position === "absolute") {
    const padding = 10;
    cluster.style.left = `${padding}px`;
    cluster.style.transform = `translateY(0px)`;
  }
  const status = document.querySelector("#quadStatus"); // General selector
  if (status) status.textContent = "Initializing...";

  // Clear any remaining arrows in the SVG of the diagram slide
  const svg = document.querySelector(
    `.carousel-slide:nth-child(${diagramSlideIndex + 1}) #arrow-canvas`
  );
  if (svg) {
    const arrows = svg.querySelectorAll(
      "path:not(defs path), text:not(defs text)"
    );
    arrows.forEach((el) => el.remove());
  }
}

// --- Scroll Animation Logic --- //
function initializeScrollAnimations() {
  console.log("DEBUG: Initializing Scroll Animations...");
  // Target sections within #main that have style1, style2, or style3 classes
  const sections = document.querySelectorAll(
    '#main.wrapper[class*="style"] .container > section' // More robust selector
  );
  console.log(`DEBUG: Found ${sections.length} sections for scroll animation.`);
  if (!sections.length) return;

  // Add initial state class for animation preparation
  sections.forEach((section) => {
    section.classList.add("scroll-animate-init");
  });

  const observerOptions = {
    root: null, // relative to the viewport
    rootMargin: "0px",
    threshold: 0.15, // Trigger when 15% of the section is visible
  };

  const observerCallback = (entries, observer) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        console.log(
          `DEBUG: Section ${entry.target.id || "without ID"} is intersecting.`
        );
        entry.target.classList.add("animate-in");
        // Optional: Stop observing after animation triggers
        observer.unobserve(entry.target);
      }
      // No need for an 'else' block if we only animate in once
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

// --- Tab Functionality --- //
function openTab(evt, tabName) {
  // Declare all variables
  var i, tabcontent, tablinks;

  // Get all elements with class="tabcontent" and hide them
  tabcontent = document.getElementsByClassName("tabcontent");
  for (i = 0; i < tabcontent.length; i++) {
    tabcontent[i].style.display = "none";
    // Remove active-content class if present
    tabcontent[i].classList.remove("active-content");
  }

  // Get all elements with class="tablinks" and remove the class "active"
  tablinks = document.getElementsByClassName("tablinks");
  for (i = 0; i < tablinks.length; i++) {
    tablinks[i].className = tablinks[i].className.replace(" active", "");
  }

  // Show the current tab, and add an "active" class to the button that opened the tab
  const tabElement = document.getElementById(tabName);
  if (tabElement) {
    tabElement.style.display = "block";
    // Add active-content class for potential styling
    tabElement.classList.add("active-content");
    // Re-highlight code in the newly displayed tab IF Prism is loaded
    if (typeof Prism !== 'undefined') {
        Prism.highlightAllUnder(tabElement);
    } else {
        console.warn("Prism not loaded when trying to highlight tab:", tabName);
    }
  } else {
    console.error("Tab content element not found:", tabName);
  }
  // Ensure evt and evt.currentTarget exist before adding class
  if (evt && evt.currentTarget) {
    evt.currentTarget.className += " active";
  } else {
    // Handle case where function is called without an event (e.g., initial load)
    // Find the button corresponding to tabName and add active class
    const buttons = document.getElementsByClassName("tablinks");
    for (let btn of buttons) {
      // Check if the onclick attribute contains the correct tabName string
      const onclickAttr = btn.getAttribute("onclick");
      if (onclickAttr && onclickAttr.includes(`'${tabName}'`)) {
        btn.className += " active";
        break;
      }
    }
  }
}


// --- Initialization --- //
document.addEventListener("DOMContentLoaded", () => {
  console.log("DOM Content Loaded - Initializing Custom JS...");

  // Initialize Carousel if container exists
  const carouselContainer = document.querySelector(".carousel-container");
  if (carouselContainer) {
    console.log("DEBUG: Initializing Carousel");
    const slides = carouselContainer.querySelectorAll(".carousel-slide");
    if (slides.length > 0) {
      // Initialize slide visibility and active state
      slides.forEach((slide, index) => {
        if (index === currentSlide) {
          slide.classList.add("active");
          slide.style.visibility = "visible";
          slide.style.opacity = "1";
          console.log(`DEBUG: Initial active slide set to ${index}`);
        } else {
          slide.classList.remove("active");
          slide.style.visibility = "hidden";
          slide.style.opacity = "0";
        }
      });
    } else {
      console.log("DEBUG: No slides found in carousel container.");
    }

    // Add button listeners
    const prevButton = carouselContainer.querySelector(".carousel-btn.prev");
    const nextButton = carouselContainer.querySelector(".carousel-btn.next");
    if (prevButton) {
      prevButton.addEventListener("click", () => changeSlide(-1));
      console.log("DEBUG: Prev button listener added.");
    } else {
      console.log("DEBUG: Prev button not found.");
    }
    if (nextButton) {
      nextButton.addEventListener("click", () => changeSlide(1));
      console.log("DEBUG: Next button listener added.");
    } else {
      console.log("DEBUG: Next button not found.");
    }

    // Check if initial slide is the diagram slide and start animations
    if (currentSlide === diagramSlideIndex) {
      console.log("DEBUG: Initial slide is diagram slide. Starting animations.");
      // Delay slightly to ensure DOM is fully ready and CSS applied
      setTimeout(startDiagramAnimations, 250);
    }
  } else {
    console.log("DEBUG: Carousel container not found.");
  }

  // Initialize Scroll Animations
  // Check if we are on a page that likely has sections to animate
  if (
    document.querySelector('#main.wrapper[class*="style"] .container > section')
  ) {
    initializeScrollAnimations();
  } else {
    console.log(
      "DEBUG: Scroll animation target structure not found on this page."
    );
  }

  // --- TAB INITIALIZATION --- //
  // Get the element with id="defaultOpen" and trigger the openTab function for it
  const defaultTabButton = document.getElementById("defaultOpen");
  let initialTabName = "Overview"; // Default tab name

  if (defaultTabButton) {
      // Extract tab name from the button's onclick attribute if possible
      const onclickAttr = defaultTabButton.getAttribute("onclick");
      const tabNameMatch = onclickAttr ? onclickAttr.match(/openTab\(event, '(.*?)'\)/) : null;
      if (tabNameMatch && tabNameMatch[1]) {
          initialTabName = tabNameMatch[1];
      }
      // Call openTab directly instead of simulating a click
      openTab(null, initialTabName);
      console.log(`DEBUG: Default tab '${initialTabName}' opened.`);
  } else {
      console.warn("DEBUG: Default open tab button ('#defaultOpen') not found.");
      // Fallback: Open the first tab if default isn't found
      const firstTabButton = document.querySelector(".tablinks");
      if (firstTabButton) {
          const onclickAttr = firstTabButton.getAttribute("onclick");
          const tabNameMatch = onclickAttr ? onclickAttr.match(/openTab\(event, '(.*?)'\)/) : null;
          if (tabNameMatch && tabNameMatch[1]) {
              initialTabName = tabNameMatch[1];
              openTab(null, initialTabName);
              console.log(`DEBUG: Opening first tab '${initialTabName}' as fallback.`);
          }
      }
  }

  // Initial highlighting for the default tab (already visible)
  // Ensure Prism is loaded before calling highlightAll
  if (typeof Prism !== 'undefined') {
      Prism.highlightAll();
      console.log("DEBUG: Initial Prism highlighting done.");
  } else {
      // If Prism isn't loaded yet, wait a bit and try again
      // This is a fallback, ideally Prism scripts load before this runs
      setTimeout(() => {
          if (typeof Prism !== 'undefined') {
              Prism.highlightAll();
              console.log("DEBUG: Delayed initial Prism highlighting done.");
          } else {
              console.error("Prism failed to load in time for initial highlighting.");
          }
      }, 500); // Wait 500ms
  }
  // --- END TAB INITIALIZATION --- //

}); // End of DOMContentLoaded listener
