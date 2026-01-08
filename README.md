# ESP32-CAM WebSocket Low-Latency Stream

Quick start

- Build & flash with PlatformIO (in project root):

```bash
pio run -e esp32cam -t upload
```

- Connect your phone to Wi‑Fi SSID: `ESP32-CAM-AP` password: `12345678`.
- Open http://192.168.4.1/ in the phone browser.

Defaults & notes

- Default resolution: 240p (320x240). Options: 120, 240, 360.
  - 360p is not natively supported by the camera sensor; the firmware falls back to 640x480 and the client scales/crops to 360p. This fallback is announced on serial and via the UI status.
- Default FPS: 15. You may set 1–30 from the UI; the capture task will respect the selected FPS.
- The UI connects to a WebSocket at port 81 (ws://192.168.4.1:81/). Binary JPEG frames are sent to the client; slider/control/config messages are JSON.

Protocol examples

- Client -> Server
  - `{"type":"config","resolution":"240","fps":15}`
  - `{"type":"ctrl","x":123,"y":45}`

- Server -> Client
  - Binary JPEG frames (raw binary)
  - `{"type":"status","resolution":240,"fps":15}`

Limitations & tradeoffs

- ESP32 has limited RAM; use conservative defaults (320x240 @ 15fps recommended).
- If the client cannot keep up, frames are dropped (no unbounded buffering).
- For ultra-low latency, WebRTC would be better but is not feasible on ESP32; binary WebSocket JPEG streaming is the practical compromise implemented here.

Debugging

- Open serial monitor at 115200 to see camera init, fallback messages, and WebSocket connection logs.
