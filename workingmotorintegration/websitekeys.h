#pragma once
#include <Arduino.h>

const char Slider[] PROGMEM = R"===(

<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { background:#1a1a1a; color:#eee; font-family:Arial; padding:20px; }
.button {
  background:#0077cc; color:white; border:none;
  padding:12px; margin:4px; font-size:16px; border-radius:6px;
}
#status {
  position:fixed; top:15px; right:15px;
  background:black; color:#0f0;
  padding:10px; font-family:monospace;
}
</style>
</head>

<body>

<h2>Robot Control</h2>

<button class="button" onclick="fetch('/AUTOON')">AUTO</button>
<button class="button" onclick="fetch('/AUTOOFF')">MANUAL</button><br>

<button class="button" onclick="fetch('/FORWARD')">FORWARD</button>
<button class="button" onclick="fetch('/BACKWARD')">BACKWARD</button><br>

<button class="button" onclick="fetch('/LEFT')">LEFT</button>
<button class="button" onclick="fetch('/RIGHT')">RIGHT</button><br>

<button class="button" style="background:#aa0000" onclick="fetch('/STOPMOTOR')">STOP</button>
<button class="button" onclick="fetch('/ZERO')">ZERO YAW</button>

<div id="status">
Yaw: <span id="yaw">0</span><br>
LPWM: <span id="lpwm">0</span><br>
RPWM: <span id="rpwm">0</span><br>
<hr>
ToF L: <span id="l">--</span><br>
ToF F: <span id="f">--</span><br>
ToF R: <span id="r">--</span><br>
Mode: <span id="m">--</span>
</div>

<script>
setInterval(()=>{
  fetch("/STATUS")
    .then(r=>r.text())
    .then(t=>{
      let v=t.split(",");
      yaw.innerText=v[0];
      lpwm.innerText=v[2];
      rpwm.innerText=v[3];
      l.innerText=v[4];
      f.innerText=v[5];
      r.innerText=v[6];
      m.innerText=v[7];
    });
},100);
</script>

</body>
</html>

)===";
