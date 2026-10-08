/*
 * ESP32-CAM（ESP-32S）独立车载图传 V1.2 / 2026-09-13，配套2D/3D网页 V8.5。
 * 完整接线/参数见 CameraConfig.h 和项目根目录 V8.4_ESP32-CAM烧录与接线.md。
 * 只供电并连接 GimbalCtrl；不连接STM32串口，不转发视频经过基座HTTP。
 * 共用ESP8266热点无线带宽，不能保证零延迟；本固件不含任何车辆控制接口。
 * HTTP JPEG按需抓图，320x240、最高10fps、上下翻转、单查看者；超时保护不变。
 * 本次完整接线复核不变，见CameraConfig.h；只重烧摄像头，不重烧三个控制设备。
 * Arduino-ESP32 3.3.11 / AI Thinker ESP32-CAM；旧S3程序不适用此实物。未烧录不代表实物已验证。
 */
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "CameraConfig.h"
#include "CameraHome.h"

static httpd_handle_t cameraServer = nullptr;
static esp_err_t cameraError = ESP_FAIL;
static char bootId[9] = {};
// 以下查看者状态只由同一个HTTP任务读写，loop不修改。
static char viewerId[65] = {};
static uint32_t viewerSeenMs = 0, previousCaptureMs = 0;
static bool haveCapture = false;

static void setHeaders(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
  httpd_resp_set_hdr(req, "Access-Control-Expose-Headers",
                    "X-Frame-Id, X-Frame-Age-Ms, X-Camera-Version");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, max-age=0");
  httpd_resp_set_hdr(req, "X-Camera-Version", CAMERA_API_VERSION);
}

static esp_err_t textResponse(httpd_req_t *req, const char *status, const char *text) {
  setHeaders(req);
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

static bool getViewer(httpd_req_t *req, char *out, size_t capacity) {
  char query[160];
  if (httpd_req_get_url_query_len(req) >= sizeof(query) ||
      httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "viewer", out, capacity) != ESP_OK) return false;
  const size_t length = strlen(out);
  if (length < 16 || length > 64) return false;
  for (size_t i = 0; i < length; ++i)
    if (!((out[i] >= '0' && out[i] <= '9') || (out[i] >= 'a' && out[i] <= 'f'))) return false;
  return true;
}

static esp_err_t captureHandler(httpd_req_t *req) {
  if (cameraError != ESP_OK) return textResponse(req, "503 Service Unavailable", "Camera init failed. Check /status and Serial Monitor.");
  char incoming[65] = {};
  if (!getViewer(req, incoming, sizeof(incoming)))
    return textResponse(req, "400 Bad Request", "Open the camera page or V8.5 viewer.");
  const uint32_t now = millis();
  if (viewerId[0] && strcmp(viewerId, incoming) != 0 &&
      uint32_t(now - viewerSeenMs) < CAMERA_VIEWER_LEASE_MS)
    return textResponse(req, "409 Conflict", "Another viewer is active. Close it and wait 3 seconds.");
  if (haveCapture && uint32_t(now - previousCaptureMs) < CAMERA_MIN_INTERVAL_MS)
    return textResponse(req, "429 Too Many Requests", "Camera capture rate limit; retry shortly.");
  strlcpy(viewerId, incoming, sizeof(viewerId));
  viewerSeenMs = previousCaptureMs = now;
  haveCapture = true;
  camera_fb_t *fb = nullptr;
  int64_t ageMs = 0;
  for (int attempt = 0; attempt < 2; ++attempt) {
    fb = esp_camera_fb_get();
    if (!fb) break;
    const int64_t stampUs = int64_t(fb->timestamp.tv_sec) * 1000000LL + fb->timestamp.tv_usec;
    ageMs = (esp_timer_get_time() - stampUs) / 1000;
    if (ageMs >= 0 && ageMs <= CAMERA_MAX_FRAME_AGE_MS && fb->format == PIXFORMAT_JPEG) break;
    esp_camera_fb_return(fb);
    fb = nullptr;
  }
  if (!fb) return textResponse(req, "503 Service Unavailable", "No fresh camera frame. Check power and camera cable.");
  char frameId[64], age[24];
  snprintf(frameId, sizeof(frameId), "%s-%lld-%06ld", bootId,
           (long long)fb->timestamp.tv_sec, (long)fb->timestamp.tv_usec);
  snprintf(age, sizeof(age), "%lld", (long long)ageMs);
  setHeaders(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "X-Frame-Id", frameId);
  httpd_resp_set_hdr(req, "X-Frame-Age-Ms", age);
  const esp_err_t result = httpd_resp_send(req, reinterpret_cast<const char *>(fb->buf), fb->len);
  esp_camera_fb_return(fb);
  return result;
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char body[300];
  const String ip = WiFi.localIP().toString();
  snprintf(body, sizeof(body),
      "{\"version\":\"%s\",\"cameraReady\":%s,\"cameraError\":%ld,\"psramBytes\":%u,"
      "\"ip\":\"%s\",\"rssi\":%ld,\"width\":320,\"height\":240,\"maxFps\":%u}",
      CAMERA_VERSION, cameraError == ESP_OK ? "true" : "false", (long)cameraError,
      (unsigned)ESP.getPsramSize(), ip.c_str(), (long)WiFi.RSSI(), CAMERA_MAX_FPS);
  setHeaders(req);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t homeHandler(httpd_req_t *req) {
  setHeaders(req);
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, CAMERA_HOME, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t optionsHandler(httpd_req_t *req) {
  setHeaders(req);
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, nullptr, 0);
}

static bool startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 6;
  config.max_resp_headers = 12;
  config.max_open_sockets = 3;
  config.lru_purge_enable = true;
  config.recv_wait_timeout = 1;
  config.send_wait_timeout = 1;
  config.stack_size = 6144;
  if (httpd_start(&cameraServer, &config) != ESP_OK) return false;
  struct Route { const char *path; httpd_method_t method; esp_err_t (*handler)(httpd_req_t *); };
  const Route routes[] = {
    {"/", HTTP_GET, homeHandler}, {"/status", HTTP_GET, statusHandler},
    {"/capture", HTTP_GET, captureHandler}, {"/capture", HTTP_OPTIONS, optionsHandler},
    {"/status", HTTP_OPTIONS, optionsHandler}
  };
  for (const Route &route : routes) {
    httpd_uri_t uri = {};
    uri.uri = route.path;
    uri.method = route.method;
    uri.handler = route.handler;
    if (httpd_register_uri_handler(cameraServer, &uri) != ESP_OK) {
      httpd_stop(cameraServer);
      cameraServer = nullptr;
      return false;
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300); // 仅摄像头启动一次，不在STM32控制链路中。
  Serial.printf("\nRover camera V%s / ESP32-CAM / QVGA / max %u fps\n", CAMERA_VERSION, CAMERA_MAX_FPS);
  Serial.printf("Chip: %s; Flash: %u bytes; PSRAM: %u bytes\n",
                ESP.getChipModel(), (unsigned)ESP.getFlashChipSize(), (unsigned)ESP.getPsramSize());
  pinMode(CAM_FLASH_LED, OUTPUT);
  digitalWrite(CAM_FLASH_LED, LOW); // 不使用闪光灯/TF卡，避免额外供电负担。
  snprintf(bootId, sizeof(bootId), "%08lx", (unsigned long)esp_random());
  if (!psramFound()) {
    cameraError = ESP_ERR_NO_MEM;
    Serial.println("ERROR: PSRAM not found. Select AI Thinker ESP32-CAM; check 5V power and board PSRAM. Do NOT select OPI PSRAM.");
  } else {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = CAM_D0; c.pin_d1 = CAM_D1; c.pin_d2 = CAM_D2; c.pin_d3 = CAM_D3;
    c.pin_d4 = CAM_D4; c.pin_d5 = CAM_D5; c.pin_d6 = CAM_D6; c.pin_d7 = CAM_D7;
    c.pin_xclk = CAM_XCLK; c.pin_pclk = CAM_PCLK; c.pin_vsync = CAM_VSYNC; c.pin_href = CAM_HREF;
    c.pin_sccb_sda = CAM_SDA; c.pin_sccb_scl = CAM_SCL; c.pin_pwdn = CAM_PWDN; c.pin_reset = CAM_RESET;
    c.xclk_freq_hz = 20000000; c.pixel_format = PIXFORMAT_JPEG; c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = CAMERA_JPEG_QUALITY; c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
    cameraError = esp_camera_init(&c);
    if (cameraError == ESP_OK) {
      sensor_t *sensor = esp_camera_sensor_get();
      if (sensor) {
        const int flipResult = sensor->set_vflip(sensor, CAMERA_VFLIP);
        sensor->set_hmirror(sensor, CAMERA_HMIRROR);
        Serial.printf("Vertical flip requested: %d; result: %d (0=OK); horizontal mirror: %d\n",
                      CAMERA_VFLIP, flipResult, CAMERA_HMIRROR);
        if (flipResult != 0) Serial.println("WARNING: vertical flip failed; check the actual preview orientation.");
        Serial.printf("Camera initialized; sensor PID=0x%04x; PSRAM=%u bytes\n",
                      sensor->id.PID, (unsigned)ESP.getPsramSize());
      }
    } else Serial.printf("ERROR: camera init 0x%lx. Power off before checking ribbon cable.\n", (unsigned long)cameraError);
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(CAMERA_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(CAMERA_WIFI_SSID, CAMERA_WIFI_PASSWORD);
  WiFi.setSleep(false);
  Serial.printf("Connecting to %s (2.4 GHz, DHCP)...\n", CAMERA_WIFI_SSID);
  // 状态页即使摄像头初始化失败也启动，便于区分WiFi与排线/PSRAM故障。
  if (!startServer()) Serial.println("ERROR: HTTP server start failed; press RESET to retry.");
}

void loop() {
  static bool wasConnected = false, mdnsStarted = false;
  static uint32_t retryAt = 0;
  const bool connected = WiFi.status() == WL_CONNECTED;
  const uint32_t now = millis();
  if (connected && !wasConnected) {
    if (mdnsStarted) MDNS.end();
    mdnsStarted = MDNS.begin(CAMERA_HOSTNAME);
    if (mdnsStarted) MDNS.addService("http", "tcp", 80);
    Serial.printf("WiFi connected. Camera URL: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.println("Optional hostname: http://rover-camera.local/ ; Serial Monitor: 115200");
    Serial.printf("Camera ready: %s; HTTP ready: %s\n", cameraError == ESP_OK ? "YES" : "NO", cameraServer ? "YES" : "NO");
  }
  if (!connected && wasConnected) Serial.println("WiFi lost; viewer must treat video as unavailable.");
  if (!connected && uint32_t(now - retryAt) >= 10000) {
    retryAt = now;
    Serial.printf("Still waiting for GimbalCtrl; WiFi status=%d. Check AP power/password/client count.\n", (int)WiFi.status());
    WiFi.reconnect();
  }
  wasConnected = connected;
  delay(10); // 仅本摄像头后台；HTTP在独立任务中处理。
}
