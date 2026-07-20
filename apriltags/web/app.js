import * as Comlink from "https://unpkg.com/comlink@4.4.2/dist/esm/comlink.mjs";

const video = document.getElementById("video");
const canvas = document.getElementById("canvas");
const statusEl = document.getElementById("status");
const tagListEl = document.getElementById("tagList");
const btnStart = document.getElementById("btnStart");
const btnStop = document.getElementById("btnStop");
const decimateSel = document.getElementById("decimate");
const poseChk = document.getElementById("pose");
const fpsEl = document.getElementById("fps");

let stream = null;
/** @type {import('comlink').Remote<any> | null} */
let apriltag = null;
let running = false;
let lastDetections = [];
let detectBusy = false;
let fpsFrames = 0;
let fpsLast = performance.now();

function setStatus(msg, kind = "") {
  statusEl.textContent = msg;
  statusEl.className = "status " + kind;
}

async function initDetector() {
  if (apriltag) return;
  setStatus("Loading AprilTag (WASM)…");
  const ApriltagClass = Comlink.wrap(new Worker("apriltag.js", { type: "classic" }));
  let resolveWasm;
  const wasmReady = new Promise((r) => {
    resolveWasm = r;
  });
  apriltag = await new ApriltagClass(
    Comlink.proxy(() => {
      resolveWasm();
      setStatus("Detector ready — starting camera…", "ready");
    })
  );
  await wasmReady;
}

function applyDetectorOptions() {
  if (!apriltag) return;
  const dec = parseFloat(decimateSel.value, 10);
  const wantPose = poseChk.checked ? 1 : 0;
  apriltag.set_quad_decimate(dec);
  apriltag.set_return_pose(wantPose);
  apriltag.set_return_solutions(wantPose);
}

decimateSel.addEventListener("change", () => {
  if (apriltag) applyDetectorOptions();
});

poseChk.addEventListener("change", () => {
  if (apriltag) applyDetectorOptions();
});

btnStart.addEventListener("click", async () => {
  btnStart.disabled = true;
  try {
    await initDetector();
    applyDetectorOptions();

    const constraints = {
      audio: false,
      video: {
        facingMode: { ideal: "environment" },
        width: { ideal: 1280 },
        height: { ideal: 720 },
      },
    };
    stream = await navigator.mediaDevices.getUserMedia(constraints);
    video.srcObject = stream;
    await video.play();

    const w = video.videoWidth;
    const h = video.videoHeight;
    canvas.width = w;
    canvas.height = h;

    const fx = w;
    const fy = w;
    const cx = w / 2;
    const cy = h / 2;
    await apriltag.set_camera_info(fx, fy, cx, cy);

    running = true;
    btnStop.disabled = false;
    requestAnimationFrame(loop);
  } catch (e) {
    console.error(e);
    setStatus(
      e.name === "NotAllowedError"
        ? "Camera permission denied."
        : "Could not open camera: " + (e.message || String(e)),
      "err"
    );
    btnStart.disabled = false;
  }
});

btnStop.addEventListener("click", () => {
  running = false;
  if (stream) {
    stream.getTracks().forEach((t) => t.stop());
    stream = null;
  }
  video.srcObject = null;
  btnStop.disabled = true;
  btnStart.disabled = false;
  lastDetections = [];
  tagListEl.innerHTML = "";
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  setStatus("Stopped.");
});

function drawOverlays(ctx, detections) {
  for (const det of detections) {
    const c = det.corners;
    ctx.beginPath();
    ctx.strokeStyle = "#34d399";
    ctx.lineWidth = 3;
    ctx.moveTo(c[0].x, c[0].y);
    for (let i = 1; i < 4; i++) ctx.lineTo(c[i].x, c[i].y);
    ctx.closePath();
    ctx.stroke();

    const cx = det.center.x;
    const cy = det.center.y;
    ctx.font = "bold 18px system-ui, sans-serif";
    ctx.fillStyle = "rgba(15, 20, 25, 0.75)";
    const label = String(det.id);
    const tw = ctx.measureText(label).width;
    ctx.fillRect(cx - tw / 2 - 6, cy - 14, tw + 12, 24);
    ctx.fillStyle = "#fff";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, cx, cy);
  }
}

function updateTagList(detections) {
  if (detections.length === 0) {
    tagListEl.innerHTML = '<li style="color:var(--muted)">No tags in view</li>';
    return;
  }
  tagListEl.innerHTML = detections
    .map((d) => {
      let extra = "";
      if (d.pose && d.pose.t) {
        const t = d.pose.t;
        const dist = Math.hypot(t[0], t[1], t[2]);
        extra = ` · ~${dist.toFixed(2)} m (rough)`;
      }
      return `<li>ID <strong>${d.id}</strong>${extra}</li>`;
    })
    .join("");
}

async function loop() {
  if (!running) return;

  const ctx = canvas.getContext("2d");
  const w = video.videoWidth;
  const h = video.videoHeight;
  if (w && h && (canvas.width !== w || canvas.height !== h)) {
    canvas.width = w;
    canvas.height = h;
    const fx = w;
    const fy = w;
    await apriltag.set_camera_info(fx, fy, w / 2, h / 2);
  }

  ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
  const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
  const pixels = imageData.data;
  const gray = new Uint8Array(canvas.width * canvas.height);

  for (let i = 0, j = 0; i < pixels.length; i += 4, j++) {
    const g = (pixels[i] + pixels[i + 1] + pixels[i + 2]) / 3;
    gray[j] = g | 0;
    pixels[i] = pixels[i + 1] = pixels[i + 2] = g | 0;
  }
  ctx.putImageData(imageData, 0, 0);

  drawOverlays(ctx, lastDetections);

  if (apriltag && !detectBusy) {
    detectBusy = true;
    const copy = new Uint8Array(gray);
    apriltag
      .detect(copy, canvas.width, canvas.height)
      .then((d) => {
        if (Array.isArray(d)) lastDetections = d;
        updateTagList(lastDetections);
      })
      .catch((e) => console.error(e))
      .finally(() => {
        detectBusy = false;
      });
  }

  fpsFrames++;
  const now = performance.now();
  if (now - fpsLast >= 1000) {
    fpsEl.textContent = String(fpsFrames);
    fpsFrames = 0;
    fpsLast = now;
  }

  requestAnimationFrame(loop);
}

btnStop.disabled = true;
setStatus('Click "Start camera". HTTPS or localhost is required.');
