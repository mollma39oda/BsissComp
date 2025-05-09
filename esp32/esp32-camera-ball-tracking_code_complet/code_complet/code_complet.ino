#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Wi-Fi credentials
const char *WIFI_SSID = "oussama";
const char *WIFI_PASS = "12345678";

// Serveur HTTP
WebServer server(80);

// Ball position (envoyée par Python)
int ballX = -1;
int ballY = -1;

// Pins pour XIAO ESP32S3 Sense (OV2640)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

void startCamera() {
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

  // Frame size & qualité
  config.frame_size = FRAMESIZE_QVGA;  // 320x240 (modifie selon tes besoins)
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialisation
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Erreur init caméra: 0x%x", err);
  } else {
    Serial.println("Caméra initialisée avec succès.");
  }
}

void handleJpgLo() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture échouée");
    server.send(500, "text/plain", "Capture failed");
    return;
  }

  WiFiClient client = server.client();
  if (client.connected()) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(fb->len));
    client.println();
    client.write(fb->buf, fb->len);
  }

  esp_camera_fb_return(fb);
}

void handleBallPosition() {
  if (server.hasArg("x") && server.hasArg("y")) {
    ballX = server.arg("x").toInt();
    ballY = server.arg("y").toInt();
    Serial.printf("Position balle reçue: x=%d, y=%d\n", ballX, ballY);
    server.send(200, "text/plain", "Position reçue");
  } else {
    server.send(400, "text/plain", "Paramètres manquants");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  startCamera();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté. IP: ");
  Serial.println(WiFi.localIP());

  server.on("/cam-lo.jpg", HTTP_GET, handleJpgLo);
  server.on("/ball_position", HTTP_POST, handleBallPosition);
  server.begin();
  Serial.println("Serveur démarré.");
}

void loop() {
  server.handleClient();

  if (ballX != -1 && ballY != -1) {
    // Exemple de logique de position
    if (ballX < 100) {
      Serial.println("Balle à gauche → ajuster à droite");
    } else if (ballX > 220) {
      Serial.println("Balle à droite → ajuster à gauche");
    } else {
      Serial.println("Balle centrée → pas d'ajustement");
    }

    // Réinitialiser la position
    ballX = -1;
    ballY = -1;
  }
}
