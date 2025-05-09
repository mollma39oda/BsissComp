/**
 * Implémentation des fonctions de configuration de la caméra
 */

 #include "camera_config.h"
 #include <esp32cam.h>
 #include <Arduino.h>
 
 void setupCamera() {
     esp32cam::Config cfg;
     cfg.setPins(esp32cam::pins::AiThinker);
     cfg.setResolution(esp32cam::Resolution::find(320, 240));
     cfg.setBufferCount(3);
     cfg.setJpeg(80);
     
     bool ok = esp32cam::Camera.begin(cfg);
     Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
 }