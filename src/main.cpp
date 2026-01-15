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

// Flow-control: single client ready flag to avoid sending frames faster than client can handle
volatile bool client_ready = false;
volatile uint8_t client_num = 255; // 255 = no client connected

// Frame queue: captureTask pushes frames, main loop pops and sends via WebSocket
// Since single-client constraint, use simple circular buffer to avoid heap allocations
#define FRAME_QUEUE_SIZE 3
struct FrameQueueEntry {
  camera_fb_t* fb;
  bool valid;
};
volatile FrameQueueEntry frame_queue[FRAME_QUEUE_SIZE];
volatile int frame_queue_head = 0;
volatile int frame_queue_tail = 0;
portMUX_TYPE frame_queue_mux = portMUX_INITIALIZER_UNLOCKED;

// Push frame to queue (called from captureTask, Core 1)
bool pushFrameToQueue(camera_fb_t* fb) {
  taskENTER_CRITICAL(&frame_queue_mux);
  int next_head = (frame_queue_head + 1) % FRAME_QUEUE_SIZE;
  if (next_head == frame_queue_tail) {
    // Queue full, return old frame and reject new one
    if (frame_queue[frame_queue_head].valid && frame_queue[frame_queue_head].fb) {
      esp_camera_fb_return(frame_queue[frame_queue_head].fb);
    }
    taskEXIT_CRITICAL(&frame_queue_mux);
    return false;
  }
  frame_queue[frame_queue_head].fb = fb;
  frame_queue[frame_queue_head].valid = true;
  frame_queue_head = next_head;
  taskEXIT_CRITICAL(&frame_queue_mux);
  return true;
}

// Pop frame from queue (called from main loop, Core 0)
camera_fb_t* popFrameFromQueue() {
  taskENTER_CRITICAL(&frame_queue_mux);
  if (frame_queue_tail == frame_queue_head) {
    taskEXIT_CRITICAL(&frame_queue_mux);
    return NULL; // Queue empty
  }
  camera_fb_t* fb = frame_queue[frame_queue_tail].fb;
  frame_queue[frame_queue_tail].valid = false;
  frame_queue[frame_queue_tail].fb = NULL;
  frame_queue_tail = (frame_queue_tail + 1) % FRAME_QUEUE_SIZE;
  taskEXIT_CRITICAL(&frame_queue_mux);
  return fb;
}

// Single client telemetry
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
ClientInfo client;

// Default settings
volatile int target_fps = 15; // default fps
volatile int current_fps = 15;
volatile int desired_resolution = 176; // 120/240/360
  // Adaptive streaming parameters
  int current_jpeg_quality = 12; // lower -> higher quality
  const int MIN_JPEG_QUALITY = 10;
  const int MAX_JPEG_QUALITY = 98;
  const size_t MAX_FRAME_BYTES = 100000; // if frames larger than this, consider lowering quality or skipping
  int consecutive_large_frames = 0;
  const int LARGE_FRAME_THRESHOLD = 6; // after this many large frames, reduce fps
// Flag to pause capture while camera is being reconfigured
volatile bool camera_reinit_in_progress = false;

// Control variables from sliders (0-255 ranges often)
volatile int ctrl_x = 128;
volatile int ctrl_y = 128;

// Motor & servo pins (change as needed). Choose pins that do not conflict with camera.
#define MOTOR_PWM_F_PIN 13
#define MOTOR_PWM_R_PIN 12
#define SERVO_L_PIN 14
#define SERVO_R_PIN 15

// LEDC channels for PWM
#define MOTOR_F_LEDC_CH 0
#define MOTOR_R_LEDC_CH 1
#define SERVO_L_LEDC_CH 2
#define SERVO_R_LEDC_CH 3

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
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
  <meta charset="utf-8">
  <title>ESP32-CAM Stream</title>
  <style>
    html,body{height:100%;margin:0;font-family:Arial;background:#000;color:#fff;overflow:hidden}
    /* topbar overlays the video so container can be full-viewport */
    #topbar{position:absolute;top:0;left:0;right:0;z-index:30;display:flex;gap:8px;padding:6px 8px;background:rgba(0,0,0,0.25);align-items:center}
    select{font-size:16px;padding:6px}
    #status{margin-left:auto;font-size:14px}
    #container{position:absolute;top:48px;left:0;right:0;bottom:0;overflow:hidden;display:flex;align-items:center;justify-content:center}
    #videoCanvas{background:#111;width:100%;height:100%;max-width:none;object-fit:cover}

    /* Left vertical throttle (large touch target) */
    .vtrack{position:absolute;left:8px;top:56px;bottom:8px;width:84px;display:flex;align-items:center;justify-content:center;touch-action:none}
    .vtrack .track{width:18px;height:80%;border-radius:12px;background:rgba(255,255,255,0.06);position:relative}
    .vtrack .thumb{position:absolute;left:50%;transform:translateX(-50%);width:56px;height:56px;border-radius:50%;background:#1e90ff;box-shadow:0 3px 8px rgba(0,0,0,0.6)}

    /* Bottom-right horizontal steering */
    .htrack{position:absolute;right:8px;bottom:8px;width:44%;max-width:420px;height:84px;display:flex;align-items:center;justify-content:center;touch-action:none}
    .htrack .track{height:18px;width:90%;border-radius:12px;background:rgba(255,255,255,0.06);position:relative}
    .htrack .thumb{position:absolute;top:50%;transform:translateY(-50%);width:56px;height:56px;border-radius:50%;background:#ff7f50;box-shadow:0 3px 8px rgba(0,0,0,0.6)}

    #overlayVals{position:absolute;left:110px;top:8px;background:rgba(0,0,0,0.35);padding:6px;border-radius:6px;font-size:15px}
    @media (orientation:landscape){#videoCanvas{height:100%}}
  </style>
</head>
<body>
  <div id="topbar">
    <label>Resolution: <select id="resSelect"><option value="480">480p</option><option value="320">320p</option><option value="296" selected>296p</option><option value="240">240p</option><option value="176">176p</option></select></label>
    <label>FPS: <select id="fpsSelect"></select></label>
    <label>Quality: <select id="qualitySelect"></select></label>
    <div id="status">Connecting...</div>
    <button id="fsBtn" style="margin-left:8px;padding:6px 10px;font-size:14px">Full</button>
  </div>
  <div id="container">
    <canvas id="videoCanvas"></canvas>

    <div class="vtrack" id="vSlider" aria-label="throttle" role="slider">
      <div class="track"></div>
      <div class="thumb" id="vThumb"></div>
    </div>

    <div class="htrack" id="hSlider" aria-label="steering" role="slider">
      <div class="track"></div>
      <div class="thumb" id="hThumb"></div>
    </div>

    <div id="overlayVals">Throttle: <span id="yVal">128</span> &nbsp; Steering: <span id="xVal">128</span></div>
  </div>
  <script>
    const status = document.getElementById('status');
    const canvas = document.getElementById('videoCanvas');
    const ctx = canvas.getContext('2d');
    const resSelect = document.getElementById('resSelect');
    const fpsSelect = document.getElementById('fpsSelect');
    const qualitySelect = document.getElementById('qualitySelect');

    // Slider elements
    const vSlider = document.getElementById('vSlider');
    const vThumb = document.getElementById('vThumb');
    const hSlider = document.getElementById('hSlider');
    const hThumb = document.getElementById('hThumb');
    const xVal = document.getElementById('xVal');
    const yVal = document.getElementById('yVal');

    // Populate FPS options
    for(let i=1;i<=30;i++){const opt=document.createElement('option');opt.value=i;opt.text=i; if(i===15) opt.selected=true; fpsSelect.appendChild(opt)}
    // Populate quality options
    const qualities = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90];
    for(const q of qualities){const opt=document.createElement('option');opt.value=q;opt.text=q; if(q===75) opt.selected=true; qualitySelect.appendChild(opt)}

    // WebSocket
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
              if(ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({type:'pong', ts: j.ts || Date.now()}));
            }
          }catch(e){}
          return;
        }
        // Binary JPEG frame
        const blob = new Blob([evt.data], {type:'image/jpeg'});
        const img = await createImageBitmap(blob);
        const topbar = document.getElementById('topbar');
        const topbarH = topbar ? topbar.offsetHeight : 48;
        const containerWidth = Math.min(window.innerWidth, 960);
        const containerHeight = Math.max(120, Math.floor((window.innerHeight - topbarH - 8)));
        const scale = Math.max(containerWidth / img.width, containerHeight / img.height);
        const srcW = Math.round(containerWidth / scale);
        const srcH = Math.round(containerHeight / scale);
        const srcX = Math.round((img.width - srcW) / 2);
        const srcY = Math.round((img.height - srcH) / 2);
        canvas.width = containerWidth;
        canvas.height = containerHeight;
        ctx.imageSmoothingEnabled = true;
        try{ ctx.imageSmoothingQuality = 'high'; } catch(e){}
        ctx.drawImage(img, srcX, srcY, srcW, srcH, 0, 0, canvas.width, canvas.height);
        if(ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({type:'ready'}));
      }
    }

    function sendConfig(){
      const cfg = {type:'config', resolution:parseInt(resSelect.value), fps:parseInt(fpsSelect.value), quality: parseInt(qualitySelect.value)};
      if(ws && ws.readyState===WebSocket.OPEN) ws.send(JSON.stringify(cfg));
    }

    resSelect.addEventListener('change', sendConfig);
    fpsSelect.addEventListener('change', sendConfig);
    qualitySelect.addEventListener('change', sendConfig);

    // Control state (0-255)
    let ctrlX = 128, ctrlY = 128;
    let pendingSend = false;
    function scheduleSend(){ if(!pendingSend){ pendingSend=true; requestAnimationFrame(()=>{ if(ws && ws.readyState===WebSocket.OPEN) ws.send(JSON.stringify({type:'ctrl', x: ctrlX, y: ctrlY})); pendingSend=false; }); }}

    // Pointer handling for vertical throttle
    let vPointer = -1;
    function vPointerDown(e){ e.preventDefault(); vSlider.setPointerCapture(e.pointerId); vPointer = e.pointerId; vPointerMove(e); }
    function vPointerMove(e){ if(e.pointerId !== vPointer) return; const rect = vSlider.getBoundingClientRect(); const y = Math.max(rect.top, Math.min(rect.bottom, e.clientY)); const rel = 1 - ((y - rect.top) / rect.height); // 0..1, top=1
      ctrlY = Math.round(rel * 255); yVal.textContent = ctrlY; // position thumb
      const thumbCenter = rect.top + (1 - rel) * rect.height; vThumb.style.top = `${( (thumbCenter - rect.top) / rect.height) * 100}%`; scheduleSend(); }
    function vPointerUp(e){ if(e.pointerId !== vPointer) return; try{ vSlider.releasePointerCapture(e.pointerId); }catch{} vPointer = -1; }

    // Pointer handling for horizontal steering
    let hPointer = -1;
    function hPointerDown(e){ e.preventDefault(); hSlider.setPointerCapture(e.pointerId); hPointer = e.pointerId; hPointerMove(e); }
    function hPointerMove(e){ if(e.pointerId !== hPointer) return; const rect = hSlider.getBoundingClientRect(); const x = Math.max(rect.left, Math.min(rect.right, e.clientX)); const rel = (x - rect.left) / rect.width; ctrlX = Math.round(rel * 255); xVal.textContent = ctrlX; const thumbCenter = rect.left + rel * rect.width; hThumb.style.left = `${( (thumbCenter - rect.left) / rect.width) * 100}%`; scheduleSend(); }
    function hPointerUp(e){ if(e.pointerId !== hPointer) return; try{ hSlider.releasePointerCapture(e.pointerId); }catch{} hPointer = -1; }

    // Initialize thumb positions and layout based on actual topbar height
    function updateThumbs(){
      const topbarEl = document.getElementById('topbar');
      const container = document.getElementById('container');
      const topH = topbarEl ? topbarEl.offsetHeight : 48;
      // ensure container sits below topbar
      container.style.top = topH + 'px';

      // vertical slider: position from topbar height
      vSlider.style.position = 'absolute';
      vSlider.style.top = (topH + 8) + 'px';
      vSlider.style.bottom = '8px';
      // horizontal slider sits at bottom: keep CSS bottom:8px
      hSlider.style.position = 'absolute';
      hSlider.style.right = '8px';

      // Size and position thumbs according to current control values
      vThumb.style.position = 'absolute';
      vThumb.style.width = '56px';
      vThumb.style.height = '56px';
      vThumb.style.left = '50%';
      const vRect = vSlider.getBoundingClientRect();
      if(vRect.height > 0){
        const rel = ctrlY / 255; // 0..1
        const topPct = (1 - rel) * 100; // convert to percentage where 0% is top of track
        vThumb.style.top = topPct + '%';
      }

      hThumb.style.position = 'absolute';
      hThumb.style.width = '56px';
      hThumb.style.height = '56px';
      hThumb.style.top = '50%';
      const hRect = hSlider.getBoundingClientRect();
      if(hRect.width > 0){
        const relx = ctrlX / 255; // 0..1
        const leftPct = relx * 100;
        hThumb.style.left = leftPct + '%';
      }
    }
    window.addEventListener('resize', ()=>{ setTimeout(updateThumbs,50); });

    vSlider.addEventListener('pointerdown', vPointerDown);
    vSlider.addEventListener('pointermove', vPointerMove);
    vSlider.addEventListener('pointerup', vPointerUp);
    vSlider.addEventListener('pointercancel', vPointerUp);

    hSlider.addEventListener('pointerdown', hPointerDown);
    hSlider.addEventListener('pointermove', hPointerMove);
    hSlider.addEventListener('pointerup', hPointerUp);
    hSlider.addEventListener('pointercancel', hPointerUp);

    // set defaults and start
    xVal.textContent = ctrlX; yVal.textContent = ctrlY; setTimeout(updateThumbs,100);
    // Fullscreen button handler
    try{
      const fsBtn = document.getElementById('fsBtn');
      if(fsBtn){
        fsBtn.addEventListener('click', async ()=>{
          try{
            if(document.fullscreenElement){
              await document.exitFullscreen();
            } else {
              await document.documentElement.requestFullscreen();
              try{ if(screen.orientation && screen.orientation.lock) await screen.orientation.lock('landscape'); } catch(e){}
            }
          }catch(e){}
          setTimeout(updateThumbs,100);
        });
      }
    }catch(e){}
    connect();
  </script>
</body>
</html>
)rawliteral";

// Helpers for camera mapping
static framesize_t map_resolution_to_framesize(int res) {

  if (res == 176) return FRAMESIZE_HQVGA; // 240x176
  if (res == 240) return FRAMESIZE_QVGA;  // 320x240
  if (res == 296) return FRAMESIZE_CIF;  // 400x296
  if (res == 320) return FRAMESIZE_HVGA;  // 480x320
  if (res == 480) return FRAMESIZE_VGA;  // 640x480
  return FRAMESIZE_CIF; // fallback for 296p
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
  sensor_t * s = esp_camera_sensor_get();
  s->set_ae_level(s, 2); // adjust ae level if needed
  s->set_exposure_ctrl(s, 1); // adjust exposure level if needed

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
    client_num = num;
    client_ready = true; // allow initial frame
    client.connected = true;
    client.ip = ip;
    client.connectedAt = millis();
    client.framesSent = 0;
    client.lastFrameMs = 0;
    client.lastFrameBytes = 0;
    client.lastPongMs = millis();
    // reset send counters so they don't accumulate across reconnects
    client.sendAttempts = 0;
    client.sendFailures = 0;
    client.lastSendMs = 0;
    client.pendingPrevFrameMs = 0;
    client.backoffUntilMs = 0;
    return;
  }
  if(type == WStype_DISCONNECTED){
    Serial.printf("Client %u disconnected\n", num);
    if(num == client_num) {
      client_ready = false;
      client.disconnects++;
      client.connected = false;
      client_num = 255; // mark no client
      // print extended telemetry for this client to aid debugging
      Serial.printf("Client %u telemetry: framesSent=%d lastFrameBytes=%u lastFrameAgeMs=%lu disconnects=%d sendAttempts=%d sendFailures=%d avgRtt=%.1f lastRssi=%d lastSendMs=%lu lastPongMs=%lu\n",
        num,
        client.framesSent,
        (unsigned)client.lastFrameBytes,
        (unsigned long)(millis() - client.lastFrameMs),
        client.disconnects,
        client.sendAttempts,
        client.sendFailures,
        client.avgRttMs,
        client.lastRssi,
        (unsigned long)client.lastSendMs,
        (unsigned long)client.lastPongMs);
      // clear transient send state on disconnect
      client.lastSendMs = 0;
      client.pendingPrevFrameMs = 0;
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
      if(num == client_num && now - client.lastCtrlMs >= MIN_CTRL_INTERVAL_MS){
        int x = doc["x"] | ctrl_x;
        int y = doc["y"] | ctrl_y;
        ctrl_x = x; ctrl_y = y;
        client.lastCtrlMs = now;
      }
      // Avoid noisy serial prints and extra ack traffic — main loop already prints and client
      // gets immediate visual feedback. This reduces heap/I/O pressure that can cause crashes.
      return;
    }
    if(strcmp(t,"pong")==0){
      if(num == client_num){
        unsigned long sentTs = doc["ts"] | 0UL;
        unsigned long now = millis();
        client.lastPongMs = now;
        if(sentTs != 0){
          unsigned long rtt = now - sentTs;
          client.lastRttMs = rtt;
          // simple EMA for avg RTT
          if(client.avgRttMs <= 0.1f) client.avgRttMs = (float)rtt;
          else client.avgRttMs = (client.avgRttMs * 0.8f) + ((float)rtt * 0.2f);
          client.lastRssi = WiFi.RSSI();
          // log if RTT or RSSI are poor
          if(rtt > 500) Serial.printf("Client %d high RTT %lu ms RSSI %d\n", num, rtt, client.lastRssi);
          if(client.lastRssi < -80) Serial.printf("Client %d low RSSI %d dBm\n", num, client.lastRssi);
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
        // int quality = doc["quality"] | current_jpeg_quality;
        // if(quality != current_jpeg_quality){
        //       current_jpeg_quality = quality;
        //       Serial.printf("Applied quality change: current_jpeg_quality=%d\n", current_jpeg_quality);
        //   sensor_t * s = esp_camera_sensor_get();
        //   if(s && s->set_quality) s->set_quality(s, current_jpeg_quality);
        //   DynamicJsonDocument qd(128);
        //   qd["type"] = "status";
        //   qd["resolution"] = desired_resolution;
        //   qd["fps"] = target_fps;
        //   qd["quality"] = current_jpeg_quality;
        //   String qs; serializeJson(qd, qs);
        //   webSocket.broadcastTXT(qs);
        // }
    }
    // handle flow-control ready request
    if(strcmp(t,"ready")==0){
      if(num == client_num) client_ready = true;
      return;
    }
  }
}

// Capture task: grabs frames at target_fps and broadcasts binary JPEG frames to connected clients.
void captureTask(void *pvParameters){
  while(true){
    int fps = target_fps > 0 ? target_fps : 1;
    unsigned long start = millis();

    // only capture if not reinitializing camera
    if(!camera_reinit_in_progress){
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
          if(s) {
            current_jpeg_quality = min(MIN_JPEG_QUALITY, max(MIN_JPEG_QUALITY, current_jpeg_quality));
            Serial.printf("Adjusting JPEG quality down to %d\n", current_jpeg_quality);
            if(s->set_quality) s->set_quality(s, current_jpeg_quality);
          }
          esp_camera_fb_return(fb);
        } else {
          consecutive_large_frames = 0;
          // Push frame to queue; main loop will send it via WebSocket
          // This avoids calling WebSocket from multiple threads (thread safety issue)
          bool queued = pushFrameToQueue(fb);
          if (!queued) {
            Serial.println("Frame queue full, dropping frame");
            esp_camera_fb_return(fb);
          }
        }

        // small yield so other tasks (websocket loop) can run
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // if we observed many consecutive large frames, reduce FPS to ease load
        // Note: FPS reduction will be handled by main loop instead
        if(consecutive_large_frames >= LARGE_FRAME_THRESHOLD){
          consecutive_large_frames = 0;
          // Signal to main loop to reduce FPS (via global variable)
          target_fps = max(5, target_fps - 5);
          Serial.printf("High bandwidth detected; reducing target_fps to %d\n", target_fps);
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
    ledcWrite(MOTOR_F_LEDC_CH, 0);
    ledcWrite(MOTOR_R_LEDC_CH, 0);
  } else {
    if (y > 128)
      {
        // forward
        ledcWrite(MOTOR_F_LEDC_CH, map(abs(y - 128), 0, 127, 0, 255));
        ledcWrite(MOTOR_R_LEDC_CH, 0);
      }
    else
      {
        // reverse
        ledcWrite(MOTOR_F_LEDC_CH, 0);
        ledcWrite(MOTOR_R_LEDC_CH, map(abs(128 - y), 0, 127, 0, 255));
      }
  }

  // Servo: map x (0-255) to pulse width (SERVO_MIN_US..SERVO_MAX_US)
  int pulse_us_l = map(x, 0, 255, SERVO_MIN_US, SERVO_MAX_US);
  int pulse_us_r = map(x, 0, 255, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t max_duty_l = ((1UL << SERVO_RES) - 1UL);
  uint32_t duty_l = (uint64_t)pulse_us_l * max_duty_l / 20000UL; // 20ms period
  uint32_t max_duty_r = ((1UL << SERVO_RES) - 1UL);
  uint32_t duty_r = (uint64_t)pulse_us_r * max_duty_r / 20000UL; // 20ms period
  ledcWrite(SERVO_L_LEDC_CH, duty_l);
  ledcWrite(SERVO_R_LEDC_CH, duty_r);
}

void handleRoot(){
  // serve embedded HTML
  server.sendHeader("Content-Encoding", "identity");
  server.send_P(200, "text/html", index_html);
}

void handleTelemetry(){
  DynamicJsonDocument doc(512);
  doc["type"] = "telemetry";
  JsonObject c = doc.createNestedObject("client");
  c["idx"] = client_num;
  c["connected"] = client.connected;
  c["ip"] = client.ip.toString();
  c["connectedAtMs"] = client.connectedAt;
  c["lastFrameMs"] = client.lastFrameMs;
  c["lastFrameBytes"] = (unsigned)client.lastFrameBytes;
  c["framesSent"] = client.framesSent;
  c["disconnects"] = client.disconnects;
  c["sendAttempts"] = client.sendAttempts;
  c["sendFailures"] = client.sendFailures;
  c["lastRttMs"] = client.lastRttMs;
  c["avgRttMs"] = client.avgRttMs;
  c["lastRssi"] = client.lastRssi;
  c["lastSendMs"] = client.lastSendMs;
  c["lastPongMs"] = client.lastPongMs;
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
  ledcSetup(MOTOR_F_LEDC_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PWM_F_PIN, MOTOR_F_LEDC_CH);
  ledcSetup(MOTOR_R_LEDC_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PWM_R_PIN, MOTOR_R_LEDC_CH);

  // Configure servo PWM
  ledcSetup(SERVO_L_LEDC_CH, SERVO_FREQ, SERVO_RES);
  ledcSetup(SERVO_R_LEDC_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_L_PIN, SERVO_L_LEDC_CH);
  ledcAttachPin(SERVO_R_PIN, SERVO_R_LEDC_CH);

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

  // Pop frames from queue and send via WebSocket (ALL WS ops on Core 0)
  camera_fb_t* fb = popFrameFromQueue();
  if (fb) {
    // Send frame to client if ready
    if(client_ready && millis() >= client.backoffUntilMs && client_num != 255){
      IPAddress rip = webSocket.remoteIP(client_num);
      if(rip != IPAddress((uint32_t)0)){
        // send frame to client
        client.sendAttempts++;
        client.lastSendMs = millis();
        client.pendingPrevFrameMs = client.lastFrameMs;
        webSocket.sendBIN(client_num, fb->buf, fb->len);
        client_ready = false;
        // update telemetry (ack will be reflected when client sends ready and we update lastFrameMs)
        client.framesSent++;
        client.lastFrameMs = millis();
        client.lastFrameBytes = fb->len;
      } else {
        client_ready = false;
      }
    }
    esp_camera_fb_return(fb);
  }

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
    // attach client summary if connected
    if(client.connected){
      JsonObject c = doc.createNestedObject("client");
      c["idx"] = client_num;
      c["ip"] = client.ip.toString();
      c["framesSent"] = client.framesSent;
      c["lastFrameAgeMs"] = (unsigned long)(millis() - client.lastFrameMs);
      c["lastFrameBytes"] = (unsigned)client.lastFrameBytes;
      c["sendAttempts"] = client.sendAttempts;
      c["sendFailures"] = client.sendFailures;
      c["lastRttMs"] = client.lastRttMs;
      c["avgRttMs"] = client.avgRttMs;
      c["lastRssi"] = client.lastRssi;
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

  // Application-level ping: send a lightweight ping JSON to client and detect stale client
  static unsigned long lastPing = 0;
  const unsigned long PING_INTERVAL_MS = 3000;
  const unsigned long PONG_TIMEOUT_MS = 10000;
  if(millis() - lastPing > PING_INTERVAL_MS){
    lastPing = millis();
    if(client.connected && client_num != 255){
      // send a small ping JSON with timestamp so client can echo it back
      char buf[64];
      unsigned long ts = millis();
      snprintf(buf, sizeof(buf), "{\"type\":\"ping\",\"ts\":%lu}", ts);
      webSocket.sendTXT(client_num, buf);
      // if we haven't seen a pong recently, mark as stale and stop sending to it
      if(millis() - client.lastPongMs > PONG_TIMEOUT_MS){
        Serial.printf("Client %d pong timeout, marking disconnected\n", client_num);
        client_ready = false;
        client.connected = false;
        client.disconnects++;
        client_num = 255;
      }
    }
  }
  // Check for sends that were not acknowledged (no frame progress) within timeout
  const unsigned long SEND_ACK_TIMEOUT_MS = 2000;
  const int SEND_FAILURE_DISCONNECT_THRESHOLD = 3;
  if(client.connected && client_num != 255){
    if(client.lastSendMs != 0){
      if(millis() - client.lastSendMs > SEND_ACK_TIMEOUT_MS){
        // if lastFrameMs hasn't advanced since before the send, consider it a failure
        if(client.lastFrameMs == client.pendingPrevFrameMs){
          client.sendFailures++;
          Serial.printf("Client %d send failure #%d (no ack in %lums) lastRtt=%.1fms RSSI=%d\n", client_num, client.sendFailures, SEND_ACK_TIMEOUT_MS, client.avgRttMs, client.lastRssi);
          // reset lastSendMs so we don't double count for the same send
          client.lastSendMs = 0;
          // if failures keep happening, mark client disconnected
          if(client.sendFailures >= SEND_FAILURE_DISCONNECT_THRESHOLD){
            Serial.printf("Client %d exceeded send failure threshold, applying backoff and reducing FPS\n", client_num);
            // apply a temporary backoff for this client (0.5s)
            client.backoffUntilMs = millis() + 500; // 0.5s
            client_ready = false;
            client.sendFailures = 0; // reset to avoid immediate repeat
            client.disconnects++;
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
              Serial.printf("Auto-throttle: target_fps %d -> %d due to client %d issues\n", old_fps, target_fps, client_num);
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
          client.lastSendMs = 0;
          client.sendFailures = 0;
        }
      }
    }
  }
  // Additional auto-throttle: if client shows sustained high RTT, reduce FPS and backoff
  const float RTT_THRESH_MS = 200.0f;
  if(client.connected && client_num != 255){
    if(client.avgRttMs > RTT_THRESH_MS){
    Serial.printf("Client %d high avg RTT %.1fms — applying backoff and reducing FPS\n", client_num, client.avgRttMs);
    client.backoffUntilMs = millis() + 500;
      client_ready = false;
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