/**
 * Implémentation des fonctions du serveur web
 */

#include "web_server.h"
#include <esp32cam.h>
#include <Arduino.h>

// Initialisation du serveur web
WebServer server(80);

// Déclaration des variables globales pour la position de la balle
int ballX = 0;
int ballY = 0;

void setupWebServer() {
    // Configuration des routes
    server.on("/cam-lo.jpg", HTTP_GET, handleJpgLo);
    server.on("/ball_position", HTTP_POST, handleBallPosition);
    
    // Démarrage du serveur
    server.begin();
    Serial.println("Serveur HTTP démarré");
}

void handleWebServer() {
    server.handleClient();
}

// Gestion de la requête GET pour l'image
void handleJpgLo() {
    auto frame = esp32cam::capture();
    if (frame == nullptr) {
        Serial.println("CAPTURE FAIL");
        server.send(500, "text/plain", "CAPTURE FAIL");
        return;
    }
    
    // Envoyer l'image JPEG
    server.setContentLength(frame->size());
    server.send(200, "image/jpeg");
    WiFiClient client = server.client();
    client.write(frame->data(), frame->size());
}

// Gestion de la requête POST pour la position de la balle
void handleBallPosition() {
    if (server.hasArg("x") && server.hasArg("y")) {
        ballX = server.arg("x").toInt();
        ballY = server.arg("y").toInt();
        Serial.printf("Position de la balle reçue : x=%d, y=%d\n", ballX, ballY);
        server.send(200, "text/plain", "Position reçue");
    } else {
        server.send(400, "text/plain", "Paramètres manquants");
    }
}