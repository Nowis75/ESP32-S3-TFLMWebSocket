# ESP32-S3-TFlowLiteMicroWebSocket

This project presents an extension and optimization of a system for visualizing object detection results on the ESP32-S3 microcontroller. It builds upon previous work in which the FOMO (Faster Objects, More Objects) model was optimized for embedded systems, and focuses on a complete end-to-end solution — from camera image acquisition, through inference of a quantized neural network model, to real-time visualization of detected objects in a web browser.

The project introduces a dual-threaded architecture utilizing both cores of the ESP32-S3, including a web server for video streaming (MJPEG) and a WebSocket server for real-time transmission of detections. Emphasis is placed on optimizing computational and memory efficiency. The inference runs on a dedicated core, external PSRAM is used for tensor allocation, and tasks are parallelized using FreeRTOS, with data access synchronized through mutexes and communication bandwidth minimized by sending only JSON centroid data.

Implementation details such as image downsampling, normalization for the quantized model, efficient video stream encoding, and thread synchronization are discussed in depth. The prototype achieves a smooth video stream (~30 FPS) with overlaid detection visualization and inference latency in the order of tens of milliseconds per evaluation. The results confirm that even on highly resource-constrained hardware, it is possible to implement a fully functional edge AI application for object detection with real-time visual feedback.

<p align="center">
  <img src="images/ascii_view.png" width="600" alt="Ascii output"><br> ASCII representation of the real-time detection output
</p>
<p align="center">
  <img src="images/real_view.png" width="600" alt="Real stream output"><br>Stream output with centroids overlaid
</p>

---

# Appendix • Detailed Guide (English)

## Overview

ESP32-S3-TFlowLiteMicroWebSocket is an edge‑AI reference showcasing a complete camera → inference → visualization pipeline on ESP32‑S3:

- Captures frames from OV2640 (ESP32‑S3 camera boards)
- Runs a TensorFlow Lite Micro FOMO model on Core 1 (int8, PSRAM tensor arena)
- Streams MJPEG over HTTP for live preview
- Sends real‑time detections (centroids) as JSON via WebSocket (port 81)
- Serves a minimal HTML UI from flash

The design prioritizes throughput and responsiveness on constrained hardware by:
- Using grayscale QQVGA frames and downsampling to 96×96 for inference
- Quantized model (int8) and PSRAM‑backed tensor arena
- Dual‑core FreeRTOS tasks (inference vs. WebSocket loop)
- Lightweight JSON (only centroids above threshold)

## Hardware

- Board: ESP32‑S3 (tested with esp32‑s3‑devkitc‑1)
- Camera: OV2640 (pins defined in `camera_pins.h` for `CAMERA_MODEL_TSIMCAM_ESP32S3`)
- PSRAM: enabled (required for tensor arena)

## Folder structure (key parts)

- `src/main.cpp` – application entry, camera, HTTP stream, WebSocket, inference tasks
- `include/fomo_model.h` – embedded TensorFlow Lite Micro FOMO model (int8 FlatBuffer as C array)
- `include/README` – notes for include content
- `src/html_gui.h` – HTML UI served at `/` (included as `html_gui.h`)
- `images/` – screenshots used in readme
- `platformio.ini` – PlatformIO configuration (board, flags, dependencies)

Model note: The FOMO model is compiled into the firmware as a C array in `include/fomo_model.h` and is referenced in code as `fomo_detector_int8_tflite`.

## Build & installation (PlatformIO)

1. Open the project in VS Code with the PlatformIO extension.
2. Ensure the environment is set for ESP32‑S3 with PSRAM:

   ```ini
   [env:esp32s3]
   platform = espressif32
   board = esp32-s3-devkitc-1
   framework = arduino
   board_build.psram = enabled
   board_build.flash_size = 8MB
   build_flags =
     -DBOARD_HAS_PSRAM
     -DARDUINO_USB_MODE=1
     -DARDUINO_USB_CDC_ON_BOOT=1
     -mfix-esp32-psram-cache-issue
     -DCORE_DEBUG_LEVEL=3
     -DPSRAM_ENABLE
     -DBOARD_HAS_TENSOR_ARENA
   lib_deps =
     esp32-camera
     tanakamasayuki/TensorFlowLite_ESP32@^1.0.0
     bblanchon/ArduinoJson@^7.3.0
     Links2004/WebSockets@^2.3.6
     ottowinter/ESPAsyncWebServer-esphome@^3.0.0
   ```

   Note: The code uses `esp_http_server` (ESP-IDF) for the MJPEG stream and `WebSocketsServer` for WebSocket; `ESPAsyncWebServer` is present as a dependency for related experiments and can be kept or removed.

3. Connect the board, build and upload. Open Serial Monitor at 115200 baud.

## Runtime services

- HTTP server (port 80)
  - `GET /` – serves the minimal HTML UI (from `html_gui.h`)
  - `GET /stream` – MJPEG stream (multipart/x-mixed-replace; JPEG frames converted from grayscale)
  - `OPTIONS /*` – CORS preflight (open CORS)

- WebSocket server (port 81)
  - Broadcasts detection results as JSON arrays of centroids

### WebSocket JSON format

Each broadcast is a JSON array of detection objects:

```json
[
  { "x": 4, "y": 7, "confidence": 0.82 },
  { "x": 9, "y": 3, "confidence": 0.67 }
]
```

Coordinates `x`, `y` are indices within an output grid (`GRID_SIZE = 12` → 12×12). `confidence` is a float in [0..1]. Only cells with `value > THRESHOLD` (default 0.6) are emitted.

## How it works (pipeline)

1. Camera capture (QQVGA grayscale) → frame buffer (`esp_camera_fb_get`).
2. Resize to 96×96 (nearest neighbor) in `runInference()` and map to int8: `input[i] = resized[i] - 128`.
3. Invoke TFLite Micro on Core 1 with `interpreter->Invoke()`.
4. Read `output_tensor` (int8), dequantize to [0..1] with `(value + 128) * 1/256`.
5. Threshold and pack detections into a JSON array.
6. Send via WebSocket (port 81). In parallel, the HTTP stream serves MJPEG frames converted from current grayscale frames using `frame2jpg`.

### Dual‑core scheduling

- `inferenceTask` (Core 1) – acquires frame, resizes, feeds the model, invokes, publishes results (every ~150 ms)
- `webSocketTask` (Core 0) – drives WebSocket event loop
- HTTP server runs handlers in its own tasks (ESP‑IDF httpd)

`result_mutex` protects shared access to `output_tensor` while packaging JSON results.

## Key parameters (tuning)

- `IMG_WIDTH`, `IMG_HEIGHT` = 96×96 – model input size
- `GRID_SIZE` = 12 – output grid resolution (12×12)
- `THRESHOLD` = 0.6 – detection threshold
- `TENSOR_ARENA_SIZE` = 350 KB – PSRAM arena size for TFLM
- Camera: `PIXFORMAT_GRAYSCALE`, `FRAMESIZE_QQVGA`, `xclk_freq_hz = 20 MHz`

## Public API (functions)

- `setupWiFi()` – station mode connect to SSID/PASS, logs IP
- `startCameraServer()` – starts HTTP server, registers `/` and `/stream` routes and CORS preflight
- `stream_handler()` – pushes MJPEG frames (server‑side chunked response)
- `startWebSocketServer()` – begins WebSocket on port 81 (also see `webSocketEvent`)
- `runInference()` – capture → resize → quantize → `Invoke()`
- `sendDetectionsWebSocket()` – serialize detections as JSON and broadcast
- `inferenceTask()` / `webSocketTask()` – FreeRTOS tasks pinned to cores

## Using your own FOMO model

- Replace the model array in `include/fomo_model.h` with your converted int8 TFLite model (C array).
- Ensure the symbol name matches what `main.cpp` references: `fomo_detector_int8_tflite`.
- Keep input shape 96×96×1 and output compatible with a 12×12 grid (or adjust constants and parsing accordingly).

## Performance notes

- With grayscale QQVGA capture and simple resize, inference runs in tens of ms per frame (logged on serial).
- MJPEG stream quality is set to 80% in `frame2jpg` – increase for higher quality at the cost of bandwidth/CPU.
- Disabling Wi‑Fi power save (`WiFi.setSleep(false)`) improves stability for streaming + WebSocket.

## Limitations & tips

- No authentication; intended for lab/demo scenarios on trusted networks.
- Ensure PSRAM is enabled; low arena sizes can cause `AllocateTensors()` to fail.
- If you change input size or output grid, update the constants and parsing logic.

## Quick start

1. Flash the firmware via PlatformIO (Serial monitor 115200).
2. Check the IP address on the serial log.
3. Open a browser to `http://<device-ip>/` to see the HTML UI.
4. Preview the live stream at `/stream`.
5. Connect a WebSocket client to `ws://<device-ip>:81/` and subscribe to centroid JSON messages.

Enjoy building real‑time edge AI on ESP32‑S3!
