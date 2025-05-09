/**
 * ESP32 Camera Ball Tracking System
 * Fichier principal qui coordonne les différents modules
 */

 #include "camera_config.h"
 #include "wifi_config.h"
 #include "web_server.h"
 
 // Variables pour la position de la balle (globales)
 int ballX = -1;
 int ballY = -1;
 
 void setup() {
     Serial.begin(115200);
     Serial.println("Démarrage du système de suivi de balle");
     
     // Initialisation des modules
     setupCamera();
     setupWiFi();
     setupWebServer();
     
     Serial.println("Système prêt");
 }
 
 void loop() {
     handleWebServer();
     
     // Traitement de la position de la balle
     if (ballX != -1 && ballY != -1) {
         // Logique pour ajuster l'orientation de la caméra
         if (ballX < 100) {
             Serial.println("Balle à gauche, ajuster à droite");
         } else if (ballX > 220) {
             Serial.println("Balle à droite, ajuster à gauche");
         } else {
             Serial.println("Balle au centre, pas d'ajustement nécessaire");
         }
         
         // Réinitialiser la position après traitement
         ballX = -1;
         ballY = -1;
     }
 }