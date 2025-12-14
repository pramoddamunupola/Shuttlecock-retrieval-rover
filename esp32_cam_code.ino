#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ===================================
// 1. CONFIGURATION SECTION
// ===================================

// --- WiFi Credentials (Your Mobile Hotspot) ---
static const char* WIFI_SSID = "motoedge50fusion_2315"; 
static const char* WIFI_PASS = "12345678";             

// --- Static IP Configuration ---
IPAddress staticIP(172, 31, 43, 154);
IPAddress gateway(172, 31, 43, 221);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);

// --- Camera Pins (AI-Thinker ESP32-CAM Board) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ===================================
// 2. CAMERA INITIALIZATION
// ===================================

bool initCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; 
  config.jpeg_quality = 8;  // Lower quality for faster streaming
  config.fb_count = 2; 
  config.grab_mode = CAMERA_GRAB_LATEST; // Always get the latest frame

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  // Adjust camera settings
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 8);
  
  return true;
}

// ===================================
// 3. HTTP HANDLERS
// ===================================

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  Serial.println("Stream client connected");

  // Set response headers
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    Serial.println("Failed to set response type");
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  int frameCount = 0;
  unsigned long lastFrameTime = millis();
  
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      delay(10);
      continue;
    }

    frameCount++;
    
    // Log frame rate every 100 frames
    if (frameCount % 100 == 0) {
      unsigned long currentTime = millis();
      float fps = 100000.0 / (currentTime - lastFrameTime);
      Serial.printf("Frames: %d, FPS: %.1f\n", frameCount, fps);
      lastFrameTime = currentTime;
    }

    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    // Send boundary
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    // Send JPEG header
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    
    // Send JPEG data
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) {
      Serial.printf("Stream ended after %d frames\n", frameCount);
      break;
    }

    // Frame rate control (30ms = ~33 FPS)
    delay(30);
  }

  return res;
}

esp_err_t index_handler(httpd_req_t *req) {
  const char* html = 
    "<!DOCTYPE html><html><head>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<style>"
    "body { font-family: Arial; text-align: center; margin: 20px; background: #f0f0f0; }"
    "img { max-width: 100%; border: 3px solid #333; }"
    "h1 { color: #333; }"
    ".container { max-width: 640px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }"
    ".info { background: #e8f4f8; padding: 10px; margin: 10px 0; border-radius: 5px; }"
    "</style></head><body>"
    "<div class=\"container\">"
    "<h1>ESP32-CAM Live Stream</h1>"
    "<img src=\"/stream\">"
    "<div class=\"info\">"
    "<p><strong>Resolution:</strong> 320x240 (QVGA)</p>"
    "<p><strong>Stream URL:</strong> <code>http://10.231.208.32/stream</code></p>"
    "</div>"
    "<p><a href=\"/stream\">Direct Stream Link</a></p>"
    "</div>"
    "</body></html>";
  
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 7;  // Allow multiple connections
  config.lru_purge_enable = true;  // Enable socket reuse

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_handle_t camera_httpd = NULL;
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    Serial.println("HTTP server started successfully");
  } else {
    Serial.println("Failed to start HTTP server");
  }
}

// ===================================
// 4. SETUP & LOOP
// ===================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configure Static IP
  WiFi.config(staticIP, gateway, subnet, primaryDNS);
  
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed. Restarting...");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("\nWiFi connected successfully.");
  Serial.print("Camera Stream URL: http://");
  Serial.println(WiFi.localIP());
  
  // Initialize Camera
  if (!initCamera()) {
    Serial.println("Camera initialization failed. Restarting...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("Camera initialized successfully.");

  // Start Web Server
  startCameraServer();
  Serial.println("Ready to stream!");
}

void loop() {
  delay(1000);
  
  // Print WiFi status periodically
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Restarting...");
    delay(1000);
    ESP.restart();
  }
}