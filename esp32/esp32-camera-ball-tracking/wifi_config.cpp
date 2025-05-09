/**
 * Implémentation des fonctions de configuration WiFi
 */

 #include "wifi_config.h"
 #include <WiFi.h>
 #include <Arduino.h>
 
 // Paramètres Wi-Fi - À modifier selon votre réseau
 const char *WIFI_SSID = "TP-LINK_A6F4B9";
 const char *WIFI_PASS = "chocho89";
 
 void setupWiFi() {
     WiFi.persistent(false);
     WiFi.mode(WIFI_STA);
     
     Serial.println("Connexion au réseau WiFi...");
     WiFi.begin(WIFI_SSID, WIFI_PASS);
     
     while (WiFi.status() != WL_CONNECTED) {
         delay(500);
         Serial.print(".");
     }
     
     Serial.println("");
     Serial.print("Connecté à ");
     Serial.println(WIFI_SSID);
     Serial.print("Adresse IP: ");
     Serial.println(WiFi.localIP());
     Serial.println("URL caméra: http://" + WiFi.localIP().toString() + "/cam-lo.jpg");
 }