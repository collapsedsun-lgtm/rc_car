// ESP32-CAM streaming over WebSocket with touch UI
// - Serves UI at / on port 80
// - WebSocket server on port 81 for binary JPEG frames and JSON control messages
// - Uses esp_camera driver and WiFi softAP

#include <Arduino.h>
#include "WiFi.h"
#include "esp_wifi.h"
#include "WebServer.h"
#include "WebSocketsServer.h"
#include "esp_camera.h"
#include <ArduinoJson.h>

// Camera pins for AI-Thinker module
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// AP credentials (change if desired)
const char* ssid = "ESP32-CAM-AP";
const char* password = "12345678";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Flow-control: per-client ready flags to avoid sending frames faster than client can handle
#define MAX_WS_CLIENTS 6
volatile bool client_ready[MAX_WS_CLIENTS] = {false};

// Per-client telemetry
struct ClientInfo {
  bool connected = false;
  IPAddress ip = IPAddress((uint32_t)0);
  unsigned long connectedAt = 0;
  unsigned long lastFrameMs = 0;
  size_t lastFrameBytes = 0;
  int framesSent = 0;
  int disconnects = 0;
  unsigned long lastPongMs = 0;
  unsigned long lastRttMs = 0;
  float avgRttMs = 0.0f;
  int lastRssi = 0;
  unsigned long lastCtrlMs = 0;
  // send tracking
  unsigned long lastSendMs = 0;
  unsigned long pendingPrevFrameMs = 0;
  int sendAttempts = 0;
  int sendFailures = 0;
  unsigned long backoffUntilMs = 0;
};
ClientInfo clientInfo[MAX_WS_CLIENTS];

// Default settings
volatile int target_fps = 15; // default fps
volatile int current_fps = 15;
volatile int desired_resolution = 240; // 120/240/360
  // Adaptive streaming parameters
  int current_jpeg_quality = 18;
  const int MIN_JPEG_QUALITY = 8;
  const int MAX_JPEG_QUALITY = 30;
  const size_t MAX_FRAME_BYTES = 40000; // if frames larger than this, consider lowering quality or skipping
  int consecutive_large_frames = 0;
  const int LARGE_FRAME_THRESHOLD = 6; // after this many large frames, reduce fps
// Flag to pause capture while camera is being reconfigured
volatile bool camera_reinit_in_progress = false;

// Control variables from sliders (0-255 ranges often)
volatile int ctrl_x = 128;
volatile int ctrl_y = 128;

// Motor & servo pins (change as needed). Choose pins that do not conflict with camera.
#define MOTOR_PWM_PIN 12
#define MOTOR_DIR_PIN 13
#define SERVO_PIN 14

// LEDC channels for PWM
#define MOTOR_LEDC_CH 0
#define SERVO_LEDC_CH 1

// Motor settings
const int MOTOR_PWM_FREQ = 5000; // Hz
const int MOTOR_PWM_RES = 8;     // bits (0-255)
const int MOTOR_DEADBAND = 8;    // center deadband (0-255)

// Servo settings
const int SERVO_FREQ = 50;       // 50Hz -> 20ms period
const int SERVO_RES = 16;        // bits for fine-grained duty
const int SERVO_MIN_US = 1000;   // 1.0ms
const int SERVO_MAX_US = 2000;   // 2.0ms

String statusMsg = "";

// Embedded single-page UI (mobile-focused). Served from PROGMEM.
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="utf-8">
  <title>ESP32-CAM Stream</title>
  <style>
    html,body{height:100%;margin:0;font-family:Arial;background:#000;color:#fff}
    #topbar{display:flex;gap:8px;padding:8px;background:rgba(0,0,0,0.4);align-items:center}
    select{font-size:16px;padding:6px}
    #status{margin-left:auto;font-size:14px}
    #container{position:relative;display:flex;flex-direction:column;align-items:center}
    #videoCanvas{background:#222;width:100%;max-width:960px;height:auto}
    /* Left vertical slider */
    #vslider{position:absolute;left:8px;top:56px;bottom:70px;width:44px;padding:0;transform:rotate(-90deg);transform-origin:top left}
    /* Bottom horizontal slider */
    #hslider{position:absolute;left:0;right:0;bottom:8px;margin:auto;width:90%;height:44px}
    input[type=range]{-webkit-appearance:none;background:rgba(255,255,255,0.1);height:44px;border-radius:6px}
    input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:36px;height:36px;border-radius:50%;background:#1e90ff}
    #overlayVals{position:absolute;right:12px;top:56px;background:rgba(0,0,0,0.35);padding:6px;border-radius:6px}
    @media (min-width:420px){#videoCanvas{height:360px}}
    /* Joystick styles */
    #joystick{position:absolute;right:12px;bottom:80px;width:160px;height:160px;border-radius:50%;touch-action:none;display:flex;align-items:center;justify-content:center}
    #joy-bg{position:absolute;width:100%;height:100%;border-radius:50%;background:rgba(255,255,255,0.06);border:2px solid rgba(255,255,255,0.08)}
    #joy-knob{position:absolute;width:56px;height:56px;border-radius:50%;background:rgba(30,144,255,0.95);box-shadow:0 2px 6px rgba(0,0,0,0.6);left:50%;top:50%;transform:translate(-50%,-50%)}
  </style>
</head>
<body>
  <div id="topbar">
    <label>Resolution: <select id="resSelect"><option value="360">360p</option><option value="240" selected>240p</option><option value="120">120p</option></select></label>
    <label>FPS: <select id="fpsSelect"></select></label>
    <label>Quality: <select id="qualitySelect"></select></label>
    <div id="status">Connecting...</div>
  </div>
  <div id="container">
    <canvas id="videoCanvas"></canvas>
    <div id="joystick" aria-label="joystick" role="application">
      <div id="joy-bg"></div>
      <div id="joy-knob"></div>
    </div>
    <div id="overlayVals">X: <span id="xVal">128</span> Y: <span id="yVal">128</span></div>
  </div>
  <script>
    const status = document.getElementById('status');
    const canvas = document.getElementById('videoCanvas');
    const ctx = canvas.getContext('2d');
    const resSelect = document.getElementById('resSelect');
    const fpsSelect = document.getElementById('fpsSelect');
    const joystick = document.getElementById('joystick');
    const joyBg = document.getElementById('joy-bg');
    const joyKnob = document.getElementById('joy-knob');
    const xVal = document.getElementById('xVal');
    const yVal = document.getElementById('yVal');

    // Populate FPS options
    for(let i=1;i<=30;i++){const opt=document.createElement('option');opt.value=i;opt.text=i; if(i===15) opt.selected=true; fpsSelect.appendChild(opt)}
    // Populate quality options (JPEG quality lower -> smaller image)
    const qualities = [8,10,12,14,16,18,20,22,24,26,28,30];
    for(const q of qualities){const opt=document.createElement('option');opt.value=q;opt.text=q; if(q===18) opt.selected=true; qualitySelect.appendChild(opt)}

    // WebSocket to server (port 81 for frames & JSON)
    let ws;
    function connect(){
      const loc = window.location.hostname;
      ws = new WebSocket('ws://' + loc + ':81/');
      ws.binaryType = 'arraybuffer';
      ws.onopen = ()=>{status.textContent='WS connected';sendConfig()};
      ws.onclose = ()=>{status.textContent='WS disconnected - reconnecting'; setTimeout(connect,100)};
      ws.onerror = (e)=>{console.error(e);}
      ws.onmessage = async (evt)=>{
        if(typeof evt.data === 'string'){
                        try{
                                  const j=JSON.parse(evt.data);
                                  if(j.type==='status'){
                                    status.textContent = `res:${j.resolution} fps:${j.fps} q:${j.quality}`;
                                    resSelect.value=j.resolution;
                                    if(j.fps) fpsSelect.value = j.fps;
                                    if(j.quality) qualitySelect.value = j.quality;
                                  } else if(j.type==='ping'){
                                    // respond with pong echoing server ts to allow RTT measurement
                                    if(ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({type:'pong', ts: j.ts || Date.now()}));
                                  }
                                }catch(e){}
                return;
              }
        // Binary JPEG frame
          const blob = new Blob([evt.data], {type:'image/jpeg'});
          const img = await createImageBitmap(blob);
          // COVER-scaling: upscale image to fill available container, preserve aspect ratio,
          // and crop excess (top/bottom) to better fit typical phone horizontal screens.
          const topbar = document.getElementById('topbar');
          const topbarH = topbar ? topbar.offsetHeight : 56;
          const containerWidth = Math.min(window.innerWidth * 0.9, 960);
          const containerHeight = Math.max(120, Math.floor((window.innerHeight - topbarH - 100)) * 0.9 );

          // determine scale to cover container (may crop on one axis)
          const scale = Math.max(containerWidth / img.width, containerHeight / img.height);
          const srcW = Math.round(containerWidth / scale);
          const srcH = Math.round(containerHeight / scale);
          const srcX = Math.round((img.width - srcW) / 2);
          const srcY = Math.round((img.height - srcH) / 2);

          canvas.width = containerWidth;
          canvas.height = containerHeight;
          // prefer higher-quality upscaling while keeping latency low
          ctx.imageSmoothingEnabled = true;
          try{ ctx.imageSmoothingQuality = 'high'; } catch(e){}
          // draw cropped source to canvas (covers and fills)
          ctx.drawImage(img, srcX, srcY, srcW, srcH, 0, 0, canvas.width, canvas.height);
          // notify server we're ready for the next frame (flow-control)
          if(ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({type:'ready'}));
      }
    }

    function sendConfig(){
      const cfg = {type:'config', resolution:parseInt(resSelect.value), fps:parseInt(fpsSelect.value), quality: parseInt(qualitySelect.value)};
      if(ws && ws.readyState===WebSocket.OPEN) ws.send(JSON.stringify(cfg));
    }

    // (old slider send functions removed — using joystick send instead)

    // Wire events
    resSelect.addEventListener('change', sendConfig);
    fpsSelect.addEventListener('change', sendConfig);
    // Virtual joystick implementation
    let joyActive = false;
    const JOY_SIZE = 160; // px
    const JOY_RADIUS = JOY_SIZE/2;
    let joyCenter = {x:0,y:0};

    function setKnob(px, py){
      joyKnob.style.transform = `translate(${px - (2 * JOY_RADIUS)}px, ${py - (2 * JOY_RADIUS)}px)`;
    }

    function sendJoy(x,y){
      xVal.textContent = x;
      yVal.textContent = y;
      if(!ws || ws.readyState!==WebSocket.OPEN) return;
      const msg = {type:'ctrl', x: x, y: y};
      ws.send(JSON.stringify(msg));
    }

    // Throttle using rAF
    let pendingJoy = false;
    function scheduleJoySend(x,y){ if(!pendingJoy){ pendingJoy=true; requestAnimationFrame(()=>{ sendJoy(x,y); pendingJoy=false; }); }}

    function handlePointerDown(e){
      e.preventDefault();
      joystick.setPointerCapture(e.pointerId);
      joyActive = true;
      const rect = joystick.getBoundingClientRect();
      joyCenter = {x: rect.left + rect.width/2, y: rect.top + rect.height/2};
      handlePointerMove(e);
    }
    function handlePointerMove(e){
      if(!joyActive) return;
      const dx = e.clientX - joyCenter.x;
      const dy = e.clientY - joyCenter.y;
      const dist = Math.hypot(dx,dy);
      const max = JOY_RADIUS - 24; // keep knob inside
      const scale = dist > max ? (max/dist) : 1;
      const sx = dx * scale;
      const sy = dy * scale;
      // Map to 0-255 (center 128)
      const outX = Math.round((sx / max) * 127 + 128);
      const outY = Math.round((sy / max) * 127 + 128);
      // move knob visually
      const knobX = (sx + JOY_RADIUS);
      const knobY = (sy + JOY_RADIUS);
      setKnob(knobX, knobY);
      scheduleJoySend(Math.max(0,Math.min(255,outX)), Math.max(0,Math.min(255,outY)));
    }
    function handlePointerUp(e){
      try{ joystick.releasePointerCapture(e.pointerId); } catch(e){}
      joyActive = false;
      // return knob to center
      setKnob(JOY_RADIUS, JOY_RADIUS);
      scheduleJoySend(128,128);
    }

    joystick.addEventListener('pointerdown', handlePointerDown);
    joystick.addEventListener('pointermove', handlePointerMove);
    joystick.addEventListener('pointerup', handlePointerUp);
    joystick.addEventListener('pointercancel', handlePointerUp);

    // initialize joystick knob to center
    setKnob(JOY_RADIUS, JOY_RADIUS);
    // Start
    connect();
  </script>
</body>
</html>
)rawliteral";

// Helpers for camera mapping
static framesize_t map_resolution_to_framesize(int res) {
  if (res == 120) return FRAMESIZE_QQVGA; // 160x120
  if (res == 240) return FRAMESIZE_QVGA;  // 320x240
  // 360p is not a native camera framesize; use VGA (640x480) as fallback and document client-side scaling
  return FRAMESIZE_VGA; // fallback for 360p
}

camera_config_t camera_config(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // We'll set frame_size later in init
  return config;
}

bool initCameraForResolution(int res) {
  camera_config_t config = camera_config();
  framesize_t fs = map_resolution_to_framesize(res);
  config.frame_size = fs;
  // tune JPEG quality to balance latency and visual quality (higher -> smaller)
  config.jpeg_quality = current_jpeg_quality; // use current desired JPEG quality
  Serial.printf("initCameraForResolution: using jpeg_quality=%d\n", config.jpeg_quality);
  // use 2 frame buffers when possible to allow sending while capturing
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  // record current
  current_fps = target_fps;
  Serial.printf("Camera initialized at framesize %d for requested %dp\n", (int)fs, res);
  if (res == 360) {
    Serial.println("Note: 360p is not natively supported; using 640x480 fallback. Client will crop/scale to 360p.");
  }
  return true;
}

// WebSocket event handling
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
  if(type == WStype_CONNECTED){
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("Client %u connected from %s\n", num, ip.toString().c_str());
    // send status
    DynamicJsonDocument doc(256);
    doc["type"] = "status";
    doc["resolution"] = desired_resolution;
    doc["fps"] = target_fps;
    doc["quality"] = current_jpeg_quality;
    String out; serializeJson(doc,out);
    webSocket.sendTXT(num, out);
    if(num < MAX_WS_CLIENTS) {
      client_ready[num] = true; // allow initial frame
      clientInfo[num].connected = true;
      clientInfo[num].ip = ip;
      clientInfo[num].connectedAt = millis();
      clientInfo[num].framesSent = 0;
      clientInfo[num].lastFrameMs = 0;
      clientInfo[num].lastFrameBytes = 0;
      clientInfo[num].lastPongMs = millis();
      // reset send counters so they don't accumulate across reconnects
      clientInfo[num].sendAttempts = 0;
      clientInfo[num].sendFailures = 0;
      clientInfo[num].lastSendMs = 0;
      clientInfo[num].pendingPrevFrameMs = 0;
      clientInfo[num].backoffUntilMs = 0;
    }
    return;
  }
  if(type == WStype_DISCONNECTED){
    Serial.printf("Client %u disconnected\n", num);
    if(num < MAX_WS_CLIENTS) {
      client_ready[num] = false;
      clientInfo[num].disconnects++;
      clientInfo[num].connected = false;
      // print extended telemetry for this client to aid debugging
      Serial.printf("Client %u telemetry: framesSent=%d lastFrameBytes=%u lastFrameAgeMs=%lu disconnects=%d sendAttempts=%d sendFailures=%d avgRtt=%.1f lastRssi=%d lastSendMs=%lu lastPongMs=%lu\n",
        num,
        clientInfo[num].framesSent,
        (unsigned)clientInfo[num].lastFrameBytes,
        (unsigned long)(millis() - clientInfo[num].lastFrameMs),
        clientInfo[num].disconnects,
        clientInfo[num].sendAttempts,
        clientInfo[num].sendFailures,
        clientInfo[num].avgRttMs,
        clientInfo[num].lastRssi,
        (unsigned long)clientInfo[num].lastSendMs,
        (unsigned long)clientInfo[num].lastPongMs);
      // clear transient send state on disconnect
      clientInfo[num].lastSendMs = 0;
      clientInfo[num].pendingPrevFrameMs = 0;
    }
    return;
  }
  if(type == WStype_TEXT){
    // parse JSON control/config messages (use stack allocator to avoid heap churn)
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if(err){ Serial.println("JSON parse error"); return; }
    const char* t = doc["type"];
    if(!t) return;
    if(strcmp(t,"ctrl")==0){
      unsigned long now = millis();
      const unsigned long MIN_CTRL_INTERVAL_MS = 33; // ~30Hz
      if(num < MAX_WS_CLIENTS){
        if(now - clientInfo[num].lastCtrlMs >= MIN_CTRL_INTERVAL_MS){
          int x = doc["x"] | ctrl_x;
          int y = doc["y"] | ctrl_y;
          ctrl_x = x; ctrl_y = y;
          clientInfo[num].lastCtrlMs = now;
        }
      } else {
        // fallback: accept ctrl if client index out of tracking range
        int x = doc["x"] | ctrl_x;
        int y = doc["y"] | ctrl_y;
        ctrl_x = x; ctrl_y = y;
      }
      // Avoid noisy serial prints and extra ack traffic — main loop already prints and client
      // gets immediate visual feedback. This reduces heap/I/O pressure that can cause crashes.
      return;
    }
    if(strcmp(t,"pong")==0){
      if(num < MAX_WS_CLIENTS){
        unsigned long sentTs = doc["ts"] | 0UL;
        unsigned long now = millis();
        clientInfo[num].lastPongMs = now;
        if(sentTs != 0){
          unsigned long rtt = now - sentTs;
          clientInfo[num].lastRttMs = rtt;
          // simple EMA for avg RTT
          if(clientInfo[num].avgRttMs <= 0.1f) clientInfo[num].avgRttMs = (float)rtt;
          else clientInfo[num].avgRttMs = (clientInfo[num].avgRttMs * 0.8f) + ((float)rtt * 0.2f);
          clientInfo[num].lastRssi = WiFi.RSSI();
          // log if RTT or RSSI are poor
          if(rtt > 500) Serial.printf("Client %d high RTT %lu ms RSSI %d\n", num, rtt, clientInfo[num].lastRssi);
          if(clientInfo[num].lastRssi < -80) Serial.printf("Client %d low RSSI %d dBm\n", num, clientInfo[num].lastRssi);
        }
      }
      return;
    }
    else if(strcmp(t,"config")==0){
      int fps = doc["fps"] | target_fps;
      int res = doc["resolution"] | desired_resolution;
      Serial.printf("Received config change: res=%d fps=%d\n", res, fps);
      // apply fps immediately
      target_fps = fps;
      // try to reinit camera for resolution change
      if(res != desired_resolution){
            int old_res = desired_resolution;
            // indicate reinit in progress so captureTask will pause capturing
            camera_reinit_in_progress = true;
            desired_resolution = res;
            webSocket.sendTXT(num, "{\"type\":\"ack\",\"config\":\"reinit\"}");
            // small yield to let captureTask observe the flag
            vTaskDelay(10 / portTICK_PERIOD_MS);
            esp_err_t ret = esp_camera_deinit();
            if(ret != ESP_OK) Serial.printf("Camera deinit failed: 0x%x\n", ret);
              bool ok = initCameraForResolution(desired_resolution);
            if(!ok){
              // revert on failure
              Serial.println("Reinit failed, reverting resolution");
              desired_resolution = old_res;
            }
            DynamicJsonDocument r(128);
            r["type"] = "ack";
            r["config"] = ok?"ok":"fail";
            r["resolution"] = desired_resolution;
              r["quality"] = current_jpeg_quality;
            String rr; serializeJson(r,rr);
            webSocket.broadcastTXT(rr);
            camera_reinit_in_progress = false;
      }
        // apply quality change if present
        int quality = doc["quality"] | current_jpeg_quality;
        if(quality != current_jpeg_quality){
              current_jpeg_quality = quality;
              Serial.printf("Applied quality change: current_jpeg_quality=%d\n", current_jpeg_quality);
          sensor_t * s = esp_camera_sensor_get();
          if(s && s->set_quality) s->set_quality(s, current_jpeg_quality);
          DynamicJsonDocument qd(128);
          qd["type"] = "status";
          qd["resolution"] = desired_resolution;
          qd["fps"] = target_fps;
          qd["quality"] = current_jpeg_quality;
          String qs; serializeJson(qd, qs);
          webSocket.broadcastTXT(qs);
        }
    }
    // handle flow-control ready request
    if(strcmp(t,"ready")==0){
      if(num < MAX_WS_CLIENTS) client_ready[num] = true;
      return;
    }
  }
}

// Capture task: grabs frames at target_fps and broadcasts binary JPEG frames to connected clients.
void captureTask(void *pvParameters){
  while(true){
    int fps = target_fps > 0 ? target_fps : 1;
    unsigned long start = millis();

    // only capture if at least one client connected and not reinitializing camera
    if(webSocket.connectedClients() > 0 && !camera_reinit_in_progress){
      camera_fb_t * fb = esp_camera_fb_get();
      if(!fb){
        Serial.println("Camera capture failed");
      } else {
        // log frame size for diagnostics
        Serial.printf("Frame captured: %u bytes\n", fb->len);

        if(fb->len > MAX_FRAME_BYTES){
          // oversized frame: count and try to reduce quality dynamically
          consecutive_large_frames++;
          sensor_t * s = esp_camera_sensor_get();
          if(s && current_jpeg_quality > MIN_JPEG_QUALITY){
            current_jpeg_quality = max(MIN_JPEG_QUALITY, current_jpeg_quality - 2);
            Serial.printf("Adjusting JPEG quality down to %d\n", current_jpeg_quality);
            if(s->set_quality) s->set_quality(s, current_jpeg_quality);
          }
          esp_camera_fb_return(fb);
        } else {
          consecutive_large_frames = 0;
          // per-client flow-controlled send: only send to clients that signalled ready
          for(uint8_t i=0; i<MAX_WS_CLIENTS; i++){
            if(!client_ready[i]) continue;
            if(millis() < clientInfo[i].backoffUntilMs) continue; // skip clients in backoff
            IPAddress rip = webSocket.remoteIP(i);
            if(rip == IPAddress((uint32_t)0)){
              client_ready[i] = false;
              continue;
            }
            // send frame to this client
            clientInfo[i].sendAttempts++;
            clientInfo[i].lastSendMs = millis();
            clientInfo[i].pendingPrevFrameMs = clientInfo[i].lastFrameMs;
            webSocket.sendBIN(i, fb->buf, fb->len);
            client_ready[i] = false;
            // update telemetry (ack will be reflected when client sends ready and we update lastFrameMs)
            clientInfo[i].framesSent++;
            clientInfo[i].lastFrameMs = millis();
            clientInfo[i].lastFrameBytes = fb->len;
          }
          esp_camera_fb_return(fb);
        }

        // small yield so other tasks (websocket loop) can run
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // if we observed many consecutive large frames, reduce FPS to ease load
        if(consecutive_large_frames >= LARGE_FRAME_THRESHOLD){
          int old_fps = target_fps;
          target_fps = max(5, target_fps - 5);
          Serial.printf("High bandwidth detected; reducing target_fps %d -> %d\n", old_fps, target_fps);
          consecutive_large_frames = 0;
          // notify clients of new fps
          DynamicJsonDocument doc(128);
          doc["type"] = "status";
          doc["resolution"] = desired_resolution;
          doc["fps"] = target_fps;
          String out; serializeJson(doc, out);
          webSocket.broadcastTXT(out);
        }
      }
    }

    unsigned long elapsed = millis() - start;
    long frameDelay = (1000 / fps) - elapsed;
    if(frameDelay > 0) vTaskDelay(frameDelay / portTICK_PERIOD_MS);
    else vTaskDelay(1 / portTICK_PERIOD_MS); // yield
  }
}

// Apply ctrl_x / ctrl_y to physical outputs (motor PWM + direction, servo PWM)
void applyControlOutputs(){
  int x = ctrl_x;
  int y = ctrl_y;

  // Motor: direction pin + PWM speed
  if (abs(y - 128) <= MOTOR_DEADBAND) {
    // in deadband -> stop motor
    ledcWrite(MOTOR_LEDC_CH, 0);
  } else {
    if (y > 128) digitalWrite(MOTOR_DIR_PIN, HIGH); else digitalWrite(MOTOR_DIR_PIN, LOW);
    int speed = map(abs(y - 128), 0, 127, 0, 255);
    ledcWrite(MOTOR_LEDC_CH, speed);
  }

  // Servo: map x (0-255) to pulse width (SERVO_MIN_US..SERVO_MAX_US)
  int pulse_us = map(x, 0, 255, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t max_duty = ((1UL << SERVO_RES) - 1UL);
  uint32_t duty = (uint64_t)pulse_us * max_duty / 20000UL; // 20ms period
  ledcWrite(SERVO_LEDC_CH, duty);
}

void handleRoot(){
  // serve embedded HTML
  server.sendHeader("Content-Encoding", "identity");
  server.send_P(200, "text/html", index_html);
}

void handleTelemetry(){
  DynamicJsonDocument doc(1024);
  doc["type"] = "telemetry";
  JsonArray arr = doc.createNestedArray("clients");
  for(int i=0;i<MAX_WS_CLIENTS;i++){
    JsonObject c = arr.createNestedObject();
    c["idx"] = i;
    c["connected"] = clientInfo[i].connected;
    c["ip"] = clientInfo[i].ip.toString();
    c["connectedAtMs"] = clientInfo[i].connectedAt;
    c["lastFrameMs"] = clientInfo[i].lastFrameMs;
    c["lastFrameBytes"] = (unsigned)clientInfo[i].lastFrameBytes;
    c["framesSent"] = clientInfo[i].framesSent;
    c["disconnects"] = clientInfo[i].disconnects;
    c["sendAttempts"] = clientInfo[i].sendAttempts;
    c["sendFailures"] = clientInfo[i].sendFailures;
    c["lastRttMs"] = clientInfo[i].lastRttMs;
    c["avgRttMs"] = clientInfo[i].avgRttMs;
    c["lastRssi"] = clientInfo[i].lastRssi;
    c["lastSendMs"] = clientInfo[i].lastSendMs;
    c["lastPongMs"] = clientInfo[i].lastPongMs;
  }
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void setup(){
  Serial.begin(115200);
  Serial.println("Starting ESP32-CAM WebSocket streaming");

  // Start WiFi AP
  WiFi.mode(WIFI_AP);
  bool apok = WiFi.softAP(ssid, password);
  if(!apok) Serial.println("softAP failed");
  else Serial.printf("Started AP SSID:%s IP:192.168.4.1\n", ssid);

  // Set AP beacon interval (ms) and max TX power
  // Beacon interval controls how often the AP sends beacon frames; lower values increase airtime.
  const uint16_t BEACON_INTERVAL_MS = 50; // user requested 50 ms
  wifi_config_t ap_conf;
  if(esp_wifi_get_config(WIFI_IF_AP, &ap_conf) == ESP_OK){
    ap_conf.ap.beacon_interval = BEACON_INTERVAL_MS;
    esp_err_t berr = esp_wifi_set_config(WIFI_IF_AP, &ap_conf);
    if(berr != ESP_OK) Serial.printf("Failed to set beacon interval: 0x%x\n", berr);
    else Serial.printf("Set AP beacon interval to %ums\n", BEACON_INTERVAL_MS);
  } else {
    Serial.println("Failed to read AP config for beacon interval");
  }

  // Set TX power to maximum (device-dependent). Using 78 is common to reach ~20dBm on many modules.
  const int8_t MAX_TX_PWR = 78;
  esp_err_t perr = esp_wifi_set_max_tx_power(MAX_TX_PWR);
  if(perr != ESP_OK) Serial.printf("Failed to set max TX power: 0x%x\n", perr);
  else Serial.printf("Set max TX power to %d (raw units)\n", MAX_TX_PWR);

  // Start HTTP server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.begin();

  // Start WebSocket server (port 81)
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Initialize camera with default desired_resolution
  if(!initCameraForResolution(desired_resolution)){
    Serial.println("Camera init failed. Halting.");
  }

  // Configure motor PWM and direction pin
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  ledcSetup(MOTOR_LEDC_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_LEDC_CH);

  // Configure servo PWM
  ledcSetup(SERVO_LEDC_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CH);

  // Ensure centered defaults and apply to outputs
  ctrl_x = 128;
  ctrl_y = 128;
  applyControlOutputs();

  // start capture task pinned to core 1
  xTaskCreatePinnedToCore(captureTask, "captureTask", 4096, NULL, 1, NULL, 1);
}

void loop(){
  // Web server loop
  server.handleClient();
  // WebSocket background loop (required for arduinoWebSockets)
  webSocket.loop();
  // update status periodically
  static unsigned long last = 0;
  static unsigned long lastCtrlPrint = 0;
  if(millis() - last > 1000){
    last = millis();
    DynamicJsonDocument doc(128);
    doc["type"] = "status";
    doc["resolution"] = desired_resolution;
    doc["fps"] = target_fps;
    doc["quality"] = current_jpeg_quality;
    doc["rssi"] = WiFi.RSSI();
    // attach a small clients summary
    JsonArray carr = doc.createNestedArray("clients");
    for(int i=0;i<MAX_WS_CLIENTS;i++){
      if(!clientInfo[i].connected) continue;
      JsonObject c = carr.createNestedObject();
      c["idx"] = i;
      c["ip"] = clientInfo[i].ip.toString();
      c["framesSent"] = clientInfo[i].framesSent;
      c["lastFrameAgeMs"] = (unsigned long)(millis() - clientInfo[i].lastFrameMs);
      c["lastFrameBytes"] = (unsigned)clientInfo[i].lastFrameBytes;
      c["sendAttempts"] = clientInfo[i].sendAttempts;
      c["sendFailures"] = clientInfo[i].sendFailures;
      c["lastRttMs"] = clientInfo[i].lastRttMs;
      c["avgRttMs"] = clientInfo[i].avgRttMs;
      c["lastRssi"] = clientInfo[i].lastRssi;
    }
    String out; serializeJson(doc, out);
    // broadcast status
    webSocket.broadcastTXT(out);
  }
  // Print current control values for debugging at ~2 Hz (reduced to lower serial I/O)
  if(millis() - lastCtrlPrint > 500){
    lastCtrlPrint = millis();
    Serial.printf("CTRL X=%d Y=%d\n", ctrl_x, ctrl_y);
  }

  // Application-level ping: send a lightweight ping JSON to clients and detect stale clients
  static unsigned long lastPing = 0;
  const unsigned long PING_INTERVAL_MS = 3000;
  const unsigned long PONG_TIMEOUT_MS = 10000;
  if(millis() - lastPing > PING_INTERVAL_MS){
    lastPing = millis();
    for(int i=0;i<MAX_WS_CLIENTS;i++){
      if(!clientInfo[i].connected) continue;
      // send a small ping JSON with timestamp so client can echo it back
      char buf[64];
      unsigned long ts = millis();
      snprintf(buf, sizeof(buf), "{\"type\":\"ping\",\"ts\":%lu}", ts);
      webSocket.sendTXT(i, buf);
      // if we haven't seen a pong recently, mark as stale and stop sending to it
      if(millis() - clientInfo[i].lastPongMs > PONG_TIMEOUT_MS){
        Serial.printf("Client %d pong timeout, marking disconnected\n", i);
        client_ready[i] = false;
        clientInfo[i].connected = false;
        clientInfo[i].disconnects++;
      }
    }
  }
  // Check for sends that were not acknowledged (no frame progress) within timeout
  const unsigned long SEND_ACK_TIMEOUT_MS = 2000;
  const int SEND_FAILURE_DISCONNECT_THRESHOLD = 3;
  for(int i=0;i<MAX_WS_CLIENTS;i++){
    if(!clientInfo[i].connected) continue;
    if(clientInfo[i].lastSendMs != 0){
      if(millis() - clientInfo[i].lastSendMs > SEND_ACK_TIMEOUT_MS){
        // if lastFrameMs hasn't advanced since before the send, consider it a failure
        if(clientInfo[i].lastFrameMs == clientInfo[i].pendingPrevFrameMs){
          clientInfo[i].sendFailures++;
          Serial.printf("Client %d send failure #%d (no ack in %lums) lastRtt=%.1fms RSSI=%d\n", i, clientInfo[i].sendFailures, SEND_ACK_TIMEOUT_MS, clientInfo[i].avgRttMs, clientInfo[i].lastRssi);
          // reset lastSendMs so we don't double count for the same send
          clientInfo[i].lastSendMs = 0;
          // if failures keep happening, mark client disconnected
          if(clientInfo[i].sendFailures >= SEND_FAILURE_DISCONNECT_THRESHOLD){
            Serial.printf("Client %d exceeded send failure threshold, applying backoff and reducing FPS\n", i);
            // apply a temporary backoff for this client (0.5s)
            clientInfo[i].backoffUntilMs = millis() + 500; // 0.5s
            client_ready[i] = false;
            clientInfo[i].sendFailures = 0; // reset to avoid immediate repeat
            clientInfo[i].disconnects++;
            // resync camera: deinit and reinit to drop any pending frames/buffers
            Serial.println("Resyncing camera due to send failures");
            camera_reinit_in_progress = true;
            vTaskDelay(10 / portTICK_PERIOD_MS);
            esp_err_t derr = esp_camera_deinit();
            if(derr != ESP_OK) Serial.printf("Camera deinit during resync failed: 0x%x\n", derr);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            bool ok = initCameraForResolution(desired_resolution);
            if(!ok) Serial.println("Camera reinit after resync failed");
            camera_reinit_in_progress = false;
            // reduce global FPS slightly to ease load
            static unsigned long lastAutoThrottleMs = 0;
            const unsigned long AUTO_THROTTLE_COOLDOWN_MS = 5000;
            if(millis() - lastAutoThrottleMs > AUTO_THROTTLE_COOLDOWN_MS){
              int old_fps = target_fps;
              target_fps = max(5, target_fps - 5);
              lastAutoThrottleMs = millis();
              Serial.printf("Auto-throttle: target_fps %d -> %d due to client %d issues\n", old_fps, target_fps, i);
              DynamicJsonDocument doc(128);
              doc["type"] = "status";
              doc["resolution"] = desired_resolution;
              doc["fps"] = target_fps;
              String out; serializeJson(doc, out);
              webSocket.broadcastTXT(out);
            }
          }
        } else {
          // client advanced, clear lastSendMs
          clientInfo[i].lastSendMs = 0;
          clientInfo[i].sendFailures = 0;
        }
      }
    }
  }
  // Additional auto-throttle: if any client shows sustained high RTT, reduce FPS and backoff
  const float RTT_THRESH_MS = 200.0f;
  for(int i=0;i<MAX_WS_CLIENTS;i++){
    if(!clientInfo[i].connected) continue;
    if(clientInfo[i].avgRttMs > RTT_THRESH_MS){
    Serial.printf("Client %d high avg RTT %.1fms — applying backoff and reducing FPS\n", i, clientInfo[i].avgRttMs);
    clientInfo[i].backoffUntilMs = millis() + 500;
      client_ready[i] = false;
      static unsigned long lastRttThrottleMs = 0;
      const unsigned long RTT_THROTTLE_COOLDOWN_MS = 5000;
      if(millis() - lastRttThrottleMs > RTT_THROTTLE_COOLDOWN_MS){
        int old_fps = target_fps;
        target_fps = max(5, target_fps - 5);
        lastRttThrottleMs = millis();
        Serial.printf("Auto-throttle (RTT): target_fps %d -> %d\n", old_fps, target_fps);
        DynamicJsonDocument doc(128);
        doc["type"] = "status";
        doc["resolution"] = desired_resolution;
        doc["fps"] = target_fps;
        String out; serializeJson(doc, out);
        webSocket.broadcastTXT(out);
      }
    }
  }
  // Apply outputs when control values change
  static int prev_x = 128;
  static int prev_y = 128;
  if(prev_x != ctrl_x || prev_y != ctrl_y){
    prev_x = ctrl_x; prev_y = ctrl_y;
    applyControlOutputs();
    Serial.printf("Applied outputs X=%d Y=%d\n", prev_x, prev_y);
  }
}