#pragma once

const char PAGE_VIVE_POINTS[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot-Car Controller</title>
<style>
  body{
    background:#0c1222; color:#fff;
    font-family:sans-serif; margin:0; padding:16px; text-align:center;
  }
  .card{
    background:#141b33; border-radius:12px; padding:12px; margin-bottom:16px; text-align:left;
  }
  .row{ display:flex; flex-wrap:wrap; gap:8px; margin-bottom:8px; }
  .btn{
    padding:10px 12px; border:none; border-radius:8px; cursor:pointer; color:#fff; flex-grow:1; font-size:15px;
  }
  .btn-go{ background:#4caf50; flex-grow:0; }
  .btn-red{ background:#d32f2f; }
  .btn-blue{ background:#1976d2; }
  .btn-stop{ background:#000; width:100%; border:1px solid red; font-weight:bold; }
  
  .btn-seq { background: linear-gradient(90deg, #FF9800 0%, #FF5722 100%); font-weight:bold; width:100%; }
  .btn-wall { background: linear-gradient(90deg, #9C27B0 0%, #673AB7 100%); font-weight:bold; width:100%; }

  input[type=number]{ padding:8px; background:#111827; color:#fff; border:1px solid #4b5563; width:80px; }
  
  /* 滑动条容器 */
  .slider-container { margin: 10px 0; width: 100%; }
  input[type=range] { width: 100%; }
  label { font-size: 14px; color: #ccc; }

  .grid-drive{ display:grid; grid-template-columns:1fr 1fr 1fr; gap:6px; margin-top:10px; }
  .drive{ height:50px; border-radius:8px; border:none; background:#374151; color:#fff; font-size:16px; }
  .drive:active { background: #4b5563; }
</style>
</head>
<body>

<h1>Mission Control</h1>

<div class="card">
  <h2>Auto Tasks</h2>
  
  <div class="row">
    <button class="btn btn-seq" onclick="attackSequence()">⚡ ATTACK SEQUENCE ⚡</button>
  </div>
  
  <div class="row">
    <button class="btn btn-wall" onclick="startWallFollow()">🧱 WALL FOLLOW (Left) 🧱</button>
  </div>

  <div style="height:10px"></div>

  <div class="row">
    <button class="btn btn-red" onclick="attackRed()">Attack Red</button>
    <button class="btn btn-blue" onclick="attackBlue()">Attack Blue</button>
  </div>
  <div class="row">
    <button class="btn btn-blue" onclick="captureLowBlue()">Cap Low Blue</button>
    <button class="btn btn-red" onclick="captureLowRed()">Cap Low Red</button>
  </div>
  <div class="row">
    <button class="btn btn-blue" onclick="captureHighBlue()">Cap High Blue</button>
    <button class="btn btn-red" onclick="captureHighRed()">Cap High Red</button>
  </div>

  <hr style="border-color:#333">
  
  <div class="row">
    <input type="number" id="xval" placeholder="X">
    <input type="number" id="yval" placeholder="Y">
    <button class="btn btn-go" onclick="sendTarget()">GO</button>
  </div>

  <div class="row">
    <button class="btn btn-stop" onclick="stopRobot()">STOP ALL</button>
  </div>
  <div id="status">Ready</div>
</div>

<div class="card">
  <h2>Manual & Servo</h2>
  
  <div class="row">
    <button class="btn" style="background:#555" onclick="fetch('/servo?mode=auto')">Servo Auto</button>
    <button class="btn" style="background:#333" onclick="fetch('/servo?mode=stop')">Servo Stop</button>
  </div>
  
  <div class="slider-container">
    <label>Servo Angle: <span id="servoVal">90</span>°</label>
    <input type="range" id="servoRange" min="0" max="180" value="90">
  </div>

  <hr style="border-color:#333">

  <div class="slider-container">
    <label>Manual Speed: <span id="rpmVal">80</span> RPM</label>
    <input type="range" id="rpmRange" min="0" max="200" value="80">
  </div>

  <div class="grid-drive">
    <button class="drive" onclick="sendCmd('cw')">CW</button>
    <button class="drive" onclick="sendCmd('forward')">FWD</button>
    <button class="drive" onclick="sendCmd('ccw')">CCW</button>
    <button class="drive" onclick="sendCmd('left')">LEFT</button>
    <button class="drive" onclick="sendCmd('stop')" style="background:#b91c1c">STOP</button>
    <button class="drive" onclick="sendCmd('right')">RIGHT</button>
    <div></div>
    <button class="drive" onclick="sendCmd('back')">BACK</button>
    <div></div>
  </div>
</div>

<script>
const BASE = "";
function setStatus(t){ document.getElementById("status").innerText=t; }

// Auto Tasks
function attackSequence(){ fetch(BASE+"/attack_sequence").then(r=>r.text()).then(setStatus); }
function startWallFollow(){ fetch(BASE+"/wall_follow").then(r=>r.text()).then(setStatus); }
function stopRobot(){ fetch(BASE+"/stop").then(r=>r.text()).then(setStatus); }

function sendTarget(){
  let x=document.getElementById("xval").value;
  let y=document.getElementById("yval").value;
  fetch(`${BASE}/set_target?x=${x}&y=${y}`).then(r=>r.text()).then(setStatus);
}
function attackRed(){ fetch(BASE+"/attack_red").then(r=>r.text()).then(setStatus); }
function attackBlue(){ fetch(BASE+"/attack_blue").then(r=>r.text()).then(setStatus); }
function captureLowBlue(){ fetch(BASE+"/capture_low_blue").then(r=>r.text()).then(setStatus); }
function captureLowRed(){ fetch(BASE+"/capture_low_red").then(r=>r.text()).then(setStatus); }
function captureHighBlue(){ fetch(BASE+"/capture_high_blue").then(r=>r.text()).then(setStatus); }
function captureHighRed(){ fetch(BASE+"/capture_high_red").then(r=>r.text()).then(setStatus); }

// ================== 手动逻辑修复 ==================

// 1. 处理 RPM 滑动条
let currentRpm = 80;
document.getElementById("rpmRange").oninput = function(e) {
  currentRpm = e.target.value;
  document.getElementById("rpmVal").innerText = currentRpm;
  // 可选：如果希望拖动时实时改变速度，取消下面注释
  // sendCmd(lastMode); 
};

let lastMode = "stop";
function sendCmd(m){ 
  lastMode = m;
  // ⭐ 这里使用了 currentRpm 变量，而不是死板的 80
  fetch(`${BASE}/cmd?mode=${m}&rpm=${currentRpm}`); 
}

// 2. 处理 Servo 滑动条
document.getElementById("servoRange").oninput = function(e) {
  let val = e.target.value;
  document.getElementById("servoVal").innerText = val;
  // ⭐ 发送 fetch 请求控制角度
  fetch(`${BASE}/servo?mode=manual&angle=${val}`);
};

</script>
</body>
</html>
)rawliteral";