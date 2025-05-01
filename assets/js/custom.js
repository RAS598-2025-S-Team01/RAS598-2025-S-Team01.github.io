console.log("Custom JS File Loaded"); // Check if file loads

// --- Global variables for animations --- //
let currentX = 0; // Shared X position for quadruped
let animationDirection = 1; // 1 for right, -1 for left
let animatedCurveOffset = 0; // For potential future curve variations

// --- Carousel Logic --- //
let currentSlide = 0;

function showSlide(index) {
  const slides = document.querySelectorAll(".carousel-slide");
  if (!slides.length || index === currentSlide || index < 0 || index >= slides.length) {
    // console.log("Slide change condition not met:", index, currentSlide, slides.length);
    return;
  }

  const prevSlide = slides[currentSlide];
  const nextSlide = slides[index];
  const directionClass = index > currentSlide ? "right" : "left";
  const oppositeDirectionClass = directionClass === "right" ? "left" : "right";

  console.log(`Carousel: Changing slide from ${currentSlide} to ${index}`);

  // Prepare next slide for entry
  nextSlide.classList.add(`enter-${directionClass}`);
  nextSlide.style.visibility = 'visible'; // Make it visible before animation starts

  // Force reflow to ensure transition starts correctly
  void nextSlide.offsetWidth;

  // Start transition: make next slide active, trigger exit on previous
  nextSlide.classList.add("active");
  nextSlide.classList.remove(`enter-${directionClass}`); // Remove entry class to start transition
  prevSlide.classList.add(`exit-${oppositeDirectionClass}`);
  prevSlide.classList.remove("active");


  // Cleanup after transition ends (match CSS transition duration)
  setTimeout(() => {
    prevSlide.classList.remove(`exit-${oppositeDirectionClass}`);
    prevSlide.style.visibility = 'hidden'; // Hide after transition
    currentSlide = index;
  }, 600); // Match CSS transition duration (0.6s)
}

function changeSlide(delta) {
  const slides = document.querySelectorAll(".carousel-slide");
  if (!slides.length) return;
  const newIndex = (currentSlide + delta + slides.length) % slides.length;
  showSlide(newIndex);
}
if (document.querySelector('#main.wrapper.style1 .container > section') || document.querySelector('#main.wrapper.style2 .container > section') || document.querySelector('#main.wrapper.style3 .container > section')) {
  initializeScrollAnimations();
} else {
  console.log("DEBUG: Scroll animation target structure not found on this page.");
}

// --- Diagram Animation Logic --- //

// Animate a simple sine wave plot for IMU data simulation
function animateIMUPlot() {
  const canvas = document.getElementById("imuCanvas");
  if (!canvas) {
      console.log("DEBUG: imuCanvas not found");
      return;
  }
  console.log("DEBUG: animateIMUPlot called");
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  let t = 0; // Time variable for animation

  function draw() {
    ctx.clearRect(0, 0, width, height); // Clear canvas
    ctx.beginPath();
    ctx.moveTo(0, height / 2); // Start at middle-left

    // Draw sine wave
    for (let x = 0; x < width; x++) {
      // Combine sine wave with some noise
      const noise = (Math.random() - 0.5) * 4; // Small random noise
      const amplitude = 15;
      const frequency = 0.05;
      const yValue = Math.sin((t + x) * frequency) * amplitude + noise;
      const y = height / 2 - yValue; // Invert y-axis for canvas coordinates
      ctx.lineTo(x, y);
    }

    ctx.strokeStyle = "#2d8cff"; // Blue color for the line
    ctx.lineWidth = 1.5; // Thinner line
    ctx.stroke(); // Draw the path

    t += 1; // Increment time for animation effect
    requestAnimationFrame(draw); // Loop the animation
  }

  draw(); // Start the drawing loop
}

// Animate a plot for Camera Pose data simulation
function animateCameraPlot() {
  const canvas = document.getElementById("cameraCanvas");
   if (!canvas) {
       console.log("DEBUG: cameraCanvas not found");
       return;
   }
   console.log("DEBUG: animateCameraPlot called");
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  let t2 = 0; // Separate time variable

  function draw() {
    ctx.clearRect(0, 0, width, height);
    ctx.beginPath();
    ctx.moveTo(0, height / 2);

    // Draw a different wave pattern
    for (let x = 0; x < width; x++) {
      const amplitude1 = 10;
      const frequency1 = 0.02;
      const amplitude2 = 5;
      const frequency2 = 0.07;
      const displacement = Math.sin((t2 + x) * frequency1) * amplitude1 + Math.cos((t2 + x) * frequency2) * amplitude2;
      const y = height / 2 - displacement;
      ctx.lineTo(x, y);
    }

    ctx.strokeStyle = "#00ffaa"; // Teal color for camera pose
    ctx.lineWidth = 1.5;
    ctx.stroke();

    t2 += 1;
    requestAnimationFrame(draw);
  }

  draw();
}

// Simulate the back-and-forth motion of the quadruped cluster
function simulateQuadrupedMotion() {
  const cluster = document.querySelector(".quad-cluster");
  const status = document.getElementById("quadStatus");
  const ur5 = document.getElementById("ur5");
  const container = document.querySelector(".diagram-wrapper");

  if (!cluster || !container) {
      console.log("DEBUG: Quad cluster or diagram wrapper not found for motion sim.");
      return;
  }
  console.log("DEBUG: simulateQuadrupedMotion called");

  currentX = 0; // Start at left (will be adjusted by padding)
  let lastTime = performance.now();
  const speed = 100; // Pixels per second (adjust speed)
  const pauseDuration = 1500; // ms to pause at ends

  function move(timestamp) {
    const deltaTime = (timestamp - lastTime) / 1000; // Time difference in seconds
    lastTime = timestamp;

    const padding = 50; // Padding from container edges
    const clusterWidth = cluster.offsetWidth;
    const containerWidth = container.clientWidth;

    // Ensure widths are valid before calculating maxX
    if (clusterWidth <= 0 || containerWidth <= 0) {
        requestAnimationFrame(move); // Try again next frame
        return;
    }

    const maxX = containerWidth - clusterWidth - padding; // Max X position

    // Update position based on direction and speed
    currentX += speed * deltaTime * animationDirection;

    // Clamp position within bounds
    currentX = Math.max(padding, Math.min(currentX, maxX));

    // Simple vertical bobbing effect while moving
    const bobbleFrequency = 10;
    const bobbleAmplitude = 3;
    const yOffset = Math.sin(timestamp / 1000 * bobbleFrequency) * bobbleAmplitude;

    // Apply transform
    cluster.style.transform = `translate(${currentX}px, ${yOffset}px)`;

    // Update status text and UR5 glow
    if (status) {
      status.textContent = animationDirection === 1 ? "Moving →" : "← Returning (UR5)";
    }
    if (ur5) {
      ur5.classList.toggle("active-glow", animationDirection === -1);
    }

    // Check if reached end
    if ((currentX >= maxX && animationDirection === 1) || (currentX <= padding && animationDirection === -1)) {
      // Reached an end, pause and reverse
      animationDirection *= -1; // Reverse direction
      setTimeout(() => {
        lastTime = performance.now(); // Reset time after pause
        requestAnimationFrame(move); // Continue animation
      }, pauseDuration);
      return; // Stop current animation frame request
    }

    requestAnimationFrame(move); // Continue animation
  }

  // Small delay to allow layout calculation for offsetWidth
  setTimeout(() => requestAnimationFrame(move), 100);
}

// Draw a curved arrow between two elements
function drawArrow(svg, fromEl, toEl, label = "", curvature = 0.3, options = {}) {
  if (!svg || !fromEl || !toEl) return;

  const svgRect = svg.getBoundingClientRect();
  const fromRect = fromEl.getBoundingClientRect();
  const toRect = toEl.getBoundingClientRect();

  // Check if elements have valid dimensions (might be 0 if hidden or not rendered)
   if (fromRect.width === 0 || fromRect.height === 0 || toRect.width === 0 || toRect.height === 0) {
    // console.log("DEBUG: Arrow draw skipped, element dimensions invalid", fromEl, toEl);
    return;
  }


  // Calculate start and end points relative to the SVG container
  const x1 = fromRect.left + fromRect.width / 2 - svgRect.left;
  const y1_base = fromRect.top + fromRect.height / 2 - svgRect.top;
  const x2 = toRect.left + toRect.width / 2 - svgRect.left;
  const y2_base = toRect.top + toRect.height / 2 - svgRect.top;

  // Adjust start/end points based on direction to avoid overlapping element
  const y1 = y1_base + (y2_base > y1_base ? fromRect.height / 3 : -fromRect.height / 3);
  const y2 = y2_base + (y1_base > y2_base ? toRect.height / 3 : -toRect.height / 3);


  // Control point calculation for Quadratic Bezier curve
  const dx = x2 - x1;
  const dy = y2 - y1;
  const midX = x1 + dx * 0.5;
  const midY = y1 + dy * 0.5;
  const nx = -dy; // Perpendicular vector component
  const ny = dx;  // Perpendicular vector component
  const curveIntensity = curvature * Math.sqrt(dx*dx + dy*dy) * 0.15; // Adjust intensity

  const ctrlX_Q = midX + nx * curveIntensity * 2; // Quadratic control point X
  const ctrlY_Q = midY + ny * curveIntensity * 2; // Quadratic control point Y

  // Use Quadratic Bezier curve (Q)
  const d = `M ${x1},${y1} Q ${ctrlX_Q},${ctrlY_Q} ${x2},${y2}`;

  // Create SVG path element
  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", d);
  path.setAttribute("fill", "none");
  path.setAttribute("stroke", options.color || "#2d8cff");
  path.setAttribute("stroke-width", options.strokeWidth || "1.5");
  path.setAttribute("stroke-dasharray", options.dashed ? "4 4" : "none");
  path.setAttribute("marker-end", "url(#arrowhead)"); // Reference the marker defined in SVG
  svg.appendChild(path);

  // Add label along the path if provided
  if (label) {
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    // Position label near the middle of the curve's control point influence
    const labelX = ctrlX_Q;
    const labelY = ctrlY_Q - 10; // Slightly above the curve peak/control point
    text.setAttribute("x", labelX);
    text.setAttribute("y", labelY);
    text.setAttribute("fill", options.labelColor || "#ffffff");
    text.setAttribute("font-size", options.labelSize || "13");
    text.setAttribute("text-anchor", "middle"); // Center the text
    text.textContent = label;
    svg.appendChild(text);
  }
}

// Dynamically draw and update arrows based on element positions and state
function drawArrowsDynamically() {
  const svg = document.getElementById("arrow-canvas");
  if (!svg) {
      console.log("DEBUG: arrow-canvas SVG not found");
      return;
  }
  console.log("DEBUG: drawArrowsDynamically called");

  // Define arrowhead marker once if not already present
  if (!svg.querySelector("defs")) {
      const defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
      defs.innerHTML = `
        <marker id="arrowhead" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto" markerUnits="strokeWidth">
          <path d="M0,0 L8,3 L0,6 Z" fill="#2d8cff" />
        </marker>
      `;
      svg.insertBefore(defs, svg.firstChild); // Insert defs at the beginning
  }


  function drawFrame() {
    // Get elements needed for arrows in each frame
    const quadCluster = document.querySelector(".quad-cluster"); // Use the cluster
    const imuPlot = document.getElementById("imu-plot");
    const camPose = document.getElementById("camera-pose");
    const ur5 = document.getElementById("ur5");

    // Clear previous arrows (keep defs)
    const paths = svg.querySelectorAll("path");
    const texts = svg.querySelectorAll("text");
    paths.forEach(p => svg.removeChild(p));
    texts.forEach(t => svg.removeChild(t));

    // Draw arrows based on current state
    if (quadCluster && imuPlot) {
        drawArrow(svg, quadCluster, imuPlot, "IMU Data");
    }
    if (quadCluster && camPose) {
        drawArrow(svg, quadCluster, camPose, "Camera Pose");
    }
    if (animationDirection === -1 && ur5 && quadCluster) {
      // Only draw UR5 arrow when returning
      drawArrow(svg, ur5, quadCluster, "Pick & Place", 0.4, { color: "#ffcc00", strokeWidth: "2" }); // Example options
    }

    requestAnimationFrame(drawFrame); // Loop the arrow drawing
  }

  requestAnimationFrame(drawFrame); // Start the loop
}


// --- Scroll Animation Logic --- //
function initializeScrollAnimations() {
    console.log("DEBUG: Initializing Scroll Animations..."); // Check if function runs
    const sections = document.querySelectorAll('#main .container > section');
    console.log(`DEBUG: Found ${sections.length} sections for scroll animation.`); // Check if sections are found

    if (!sections.length) return;

    const observerOptions = {
        root: null, // Use the viewport as the root
        rootMargin: '0px',
        threshold: 0.1 // Trigger when 10% of the element is visible
    };

    const observerCallback = (entries, observer) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                console.log(`DEBUG: Section ${entry.target.id || 'without ID'} is intersecting.`); // Check which section triggers
                entry.target.classList.add('animate-in');
                observer.unobserve(entry.target); // Stop observing once animated
            }
        });
    };

    const scrollObserver = new IntersectionObserver(observerCallback, observerOptions);
    sections.forEach(section => {
        console.log(`DEBUG: Observing section: ${section.id || 'without ID'}`); // Check which sections are observed
        scrollObserver.observe(section);
    });
}

// --- Initialization --- //
document.addEventListener("DOMContentLoaded", () => {
  console.log("DOM Content Loaded - Initializing Custom JS..."); // Check if DOMContentLoaded fires

  // Initialize Carousel if container exists
  const carouselContainer = document.querySelector(".carousel-container");
  if (carouselContainer) {
    console.log("DEBUG: Initializing Carousel");
    const slides = carouselContainer.querySelectorAll(".carousel-slide");
    if (slides.length > 0) {
        slides[0].classList.add("active");
        slides[0].style.visibility = 'visible';
    }
    const prevButton = carouselContainer.querySelector(".carousel-btn.prev");
    const nextButton = carouselContainer.querySelector(".carousel-btn.next");
    if (prevButton) {
        prevButton.addEventListener("click", () => changeSlide(-1));
    }
    if (nextButton) {
        nextButton.addEventListener("click", () => changeSlide(1));
    }
  } else {
      console.log("DEBUG: Carousel container not found.");
  }

  // Initialize Diagram Animations if container exists
  const diagramWrapper = document.querySelector(".diagram-wrapper");
  if (diagramWrapper) {
    console.log("DEBUG: Initializing Diagram Animations");
    // Call the actual animation functions
    animateIMUPlot();
    animateCameraPlot();
    simulateQuadrupedMotion();
    drawArrowsDynamically();
  } else {
      console.log("DEBUG: Diagram wrapper not found.");
  }

  // Initialize Scroll Animations
  // Check if we are on a page with the specific structure targeted by the CSS/JS
  if (document.querySelector('#main.wrapper.style1 .container > section')) {
      initializeScrollAnimations();
  } else {
      console.log("DEBUG: Scroll animation target structure not found on this page.");
  }

}); // End of DOMContentLoaded listener
