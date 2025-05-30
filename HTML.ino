#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

/* Configuración de WiFi */
const char* ssid = "S23+ de Fercho";  // Nombre de la red WiFi
const char* password = "Sosmorro123"; // Contraseña de la red WiFi

/* Configuración de I2C */
#define I2CSlaveAddress1 0x55 // Dirección del primer esclavo I2C
#define I2CSlaveAddress2 0x56 // Dirección del segundo esclavo I2C
#define I2C_SDA 21            // Pin SDA para I2C
#define I2C_SCL 22            // Pin SCL para I2C

WebServer server(80); // Crear servidor web en el puerto 80

// Estados de los espacios de estacionamiento
uint8_t p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0, p8 = 0;
uint8_t Trans_esclavo_1 = 0; // Valor a enviar al esclavo 1
uint8_t Trans_esclavo_2 = 0; // Valor a enviar al esclavo 2
bool parkingSpaces[8];       // Arreglo para almacenar estados de los espacios
uint8_t availableSpaces = 0; // Contador de espacios disponibles

void setup() {
  Serial.begin(115200); // Iniciar comunicación serial
  Serial.println("Intentando conectar a...");
  Serial.println(ssid);

  // Conectar a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado exitosamente");
  Serial.println("=====================================");
  Serial.print("URL de la página web: http://");
  Serial.println(WiFi.localIP());
  Serial.println("=====================================");
  delay(100);

  // Iniciar I2C como maestro
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configurar rutas del servidor
  server.on("/", handle_OnConnect);      // Página principal
  server.onNotFound(handle_NotFound);    // Manejar rutas no encontradas

  server.begin(); // Iniciar el servidor
  Serial.println("Servidor HTTP iniciado");
  delay(100);
}

void loop() {
  server.handleClient(); // Manejar solicitudes de clientes
  updateParkingStatus(); // Actualizar estado de estacionamiento desde I2C

  // Imprimir periódicamente la URL cada 30 segundos
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 30000) {
    Serial.println("=====================================");
    Serial.print("URL de la página web: http://");
    Serial.println(WiFi.localIP());
    Serial.println("=====================================");
    lastPrint = millis();
  }
}

// Actualizar estado de los espacios de estacionamiento mediante I2C
void updateParkingStatus() {
  // Enviar datos al esclavo 1
  Wire.beginTransmission(I2CSlaveAddress1);
  Wire.write(Trans_esclavo_1);
  Serial.printf("Valor enviado al esclavo 1: %u\n", Trans_esclavo_1);
  Wire.endTransmission(true);

  // Solicitar datos del esclavo 1
  Wire.requestFrom(I2CSlaveAddress1, 1);
  uint8_t Reci_esclavo_1 = Wire.read();
  Serial.printf("Valor recibido del esclavo 1: %u\n", Reci_esclavo_1);

  // Enviar datos al esclavo 2
  Wire.beginTransmission(I2CSlaveAddress2);
  Wire.write(Trans_esclavo_2);
  Serial.printf("Valor enviado al esclavo 2: %u\n", Trans_esclavo_2);
  Wire.endTransmission(true);

  // Solicitar datos del esclavo 2
  Wire.requestFrom(I2CSlaveAddress2, 1);
  uint8_t Reci_esclavo_2 = Wire.read();
  Serial.printf("Valor recibido del esclavo 2: %u\n", Reci_esclavo_2);

  Trans_esclavo_1 = Reci_esclavo_2;
  Trans_esclavo_2 = Reci_esclavo_1;

  // Decodificar datos del esclavo 1 (P1-P4)
  switch (Reci_esclavo_1) {
    case 0:  p1 = 0; p2 = 0; p3 = 0; p4 = 0; break;
    case 1:  p1 = 1; p2 = 0; p3 = 0; p4 = 0; break;
    case 2:  p1 = 0; p2 = 1; p3 = 0; p4 = 0; break;
    case 3:  p1 = 0; p2 = 0; p3 = 1; p4 = 0; break;
    case 4:  p1 = 0; p2 = 0; p3 = 0; p4 = 1; break;
    case 5:  p1 = 1; p2 = 1; p3 = 0; p4 = 0; break;
    case 6:  p1 = 0; p2 = 1; p3 = 1; p4 = 0; break;
    case 7:  p1 = 0; p2 = 0; p3 = 1; p4 = 1; break;
    case 8:  p1 = 1; p2 = 0; p3 = 0; p4 = 1; break;
    case 9:  p1 = 1; p2 = 0; p3 = 1; p4 = 0; break;
    case 10: p1 = 0; p2 = 1; p3 = 0; p4 = 1; break;
    case 11: p1 = 1; p2 = 1; p3 = 1; p4 = 0; break;
    case 12: p1 = 0; p2 = 1; p3 = 1; p4 = 1; break;
    case 13: p1 = 1; p2 = 0; p3 = 1; p4 = 1; break;
    case 14: p1 = 1; p2 = 1; p3 = 0; p4 = 1; break;
    case 15: p1 = 1; p2 = 1; p3 = 1; p4 = 1; break;
    default: p1 = 0; p2 = 0; p3 = 0; p4 = 0; break;
  }

  // Decodificar datos del esclavo 2 (P5-P8)
  switch (Reci_esclavo_2) {
    case 0:  p5 = 0; p6 = 0; p7 = 0; p8 = 0; break;
    case 1:  p5 = 1; p6 = 0; p7 = 0; p8 = 0; break;
    case 2:  p5 = 0; p6 = 1; p7 = 0; p8 = 0; break;
    case 3:  p5 = 0; p6 = 0; p7 = 1; p8 = 0; break;
    case 4:  p5 = 0; p6 = 0; p7 = 0; p8 = 1; break;
    case 5:  p5 = 1; p6 = 1; p7 = 0; p8 = 0; break;
    case 6:  p5 = 0; p6 = 1; p7 = 1; p8 = 0; break;
    case 7:  p5 = 0; p6 = 0; p7 = 1; p8 = 1; break;
    case 8:  p5 = 1; p6 = 0; p7 = 0; p8 = 1; break;
    case 9:  p5 = 1; p6 = 0; p7 = 1; p8 = 0; break;
    case 10: p5 = 0; p6 = 1; p7 = 0; p8 = 1; break;
    case 11: p5 = 1; p6 = 1; p7 = 1; p8 = 0; break;
    case 12: p5 = 0; p6 = 1; p7 = 1; p8 = 1; break;
    case 13: p5 = 1; p6 = 0; p7 = 1; p8 = 1; break;
    case 14: p5 = 1; p6 = 1; p7 = 0; p8 = 1; break;
    case 15: p5 = 1; p6 = 1; p7 = 1; p8 = 1; break;
    default: p5 = 0; p6 = 0; p7 = 0; p8 = 0; break;
  }

  // Actualizar arreglo de espacios de estacionamiento
  parkingSpaces[0] = p1; parkingSpaces[1] = p2; parkingSpaces[2] = p3; parkingSpaces[3] = p4;
  parkingSpaces[4] = p5; parkingSpaces[5] = p6; parkingSpaces[6] = p7; parkingSpaces[7] = p8;

  // Contar espacios disponibles (0 = disponible, 1 = ocupado)
  availableSpaces = 0;
  for (int i = 0; i < 8; i++) {
    if (!parkingSpaces[i]) availableSpaces++;
  }
}

// Manejar conexión a la página principal
void handle_OnConnect() {
  Serial.println("Mostrando estado del estacionamiento");
  server.send(200, "text/html", SendHTML(parkingSpaces, availableSpaces));
}

// Manejar rutas no encontradas
void handle_NotFound() {
  server.send(404, "text/plain", "No encontrado");
}

// Generar HTML para la interfaz web
String SendHTML(bool spaces[8], uint8_t available) {
  String ptr = "<!DOCTYPE html><html lang=\"es\">\n";
  ptr += "<head>\n";
  ptr += "<meta charset=\"UTF-8\">\n";
  ptr += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<meta http-equiv=\"refresh\" content=\"5\">\n"; // Refrescar cada 5 segundos
  ptr += "<title>Parqueo-matic - Control de Parqueo</title>\n";
  ptr += "<link href=\"https://cdn.jsdelivr.net/npm/tailwindcss@2.2.19/dist/tailwind.min.css\" rel=\"stylesheet\">\n";
  ptr += "<style>\n";
  ptr += "body { background: linear-gradient(135deg, #e0e0e0, #ffffff); font-family: 'Helvetica', 'Arial', sans-serif; }\n";
  ptr += ".parking-lot { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; max-width: 800px; margin: 20px auto; padding: 20px; background: #f8f9fa; border-radius: 10px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); }\n";
  ptr += ".parking-space { aspect-ratio: 3/2; display: flex; align-items: center; justify-content: center; border: 2px solid #333; border-radius: 8px; transition: background-color 0.3s ease; position: relative; font-size: 18px; font-weight: bold; color: white; text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5); }\n";
  ptr += ".occupied { background-color: #dc2626; }\n";
  ptr += ".free { background-color: #16a34a; }\n";
  ptr += ".car-icon::before { content: '🚗'; font-size: 40px; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); }\n";
  ptr += ".available-counter { font-size: 24px; font-weight: bold; text-align: center; margin: 20px 0; padding: 10px; background: #2563eb; color: white; border-radius: 8px; max-width: 300px; margin-left: auto; margin-right: auto; }\n";
  ptr += ".status-list { max-width: 800px; margin: 20px auto; padding: 20px; background: #f8f9fa; border-radius: 10px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1); }\n";
  ptr += ".status-item { font-size: 18px; margin: 5px 0; }\n";
  ptr += "footer { text-align: center; margin-top: 20px; color: #4b5563; }\n";
  ptr += "@media (max-width: 600px) { .parking-lot { grid-template-columns: repeat(2, 1fr); } .parking-space { font-size: 14px; } .car-icon::before { font-size: 30px; } }\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<header class=\"text-center py-6\">\n";
  ptr += "<h1 class=\"text-4xl font-bold text-gray-800\">Parqueo-matic</h1>\n";
  ptr += "<h3 class=\"text-xl text-gray-600 mt-2\">Control de Parqueo - Nivel de Sótano</h3>\n";
  ptr += "</header>\n";
  ptr += "<div class=\"available-counter\">Espacios Disponibles: " + String(available) + "</div>\n";
  ptr += "<div class=\"parking-lot\">\n";

  // Generar espacios de estacionamiento dinámicamente
  for (int i = 0; i < 8; i++) {
    String spaceClass = spaces[i] ? "occupied car-icon" : "free";
    ptr += "<div class=\"parking-space " + spaceClass + "\">P" + String(i + 1) + "</div>\n";
  }

  ptr += "</div>\n";
  ptr += "<div class=\"status-list\">\n";
  ptr += "<h3 class=\"text-xl font-bold text-gray-800\">Estado de Parqueos:</h3>\n";
  ptr += "<ul>\n";
  // Listar espacios con su estado en orden
  for (int i = 0; i < 8; i++) {
    String status = spaces[i] ? "Ocupado" : "Disponible";
    String color = spaces[i] ? "text-red-600" : "text-green-600";
    ptr += "<li class=\"status-item " + color + "\">P" + String(i + 1) + ": " + status + "</li>\n";
  }
  ptr += "</ul>\n";
  ptr += "</div>\n";
  ptr += "<footer><p>Parqueo-matic - Universidad del Valle de Guatemala, 2025</p></footer>\n";
  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}