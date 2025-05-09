/**
 * Définition du serveur web
 */

 #ifndef WEB_SERVER_H
 #define WEB_SERVER_H
 
 #include <WebServer.h>
 
 // Variables externes pour la position de la balle
 extern int ballX;
 extern int ballY;
 
 // Déclaration du serveur web (global)
 extern WebServer server;
 
 // Fonctions de gestion du serveur web
 void setupWebServer();
 void handleWebServer();
 
 // Fonctions de gestion des requêtes
 void handleJpgLo();
 void handleBallPosition();
 
 #endif