#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
void leerSensor();

// ---------------------------------------------------------------------------
// CALIBRACIÓN
// Valores por defecto; se sobrescriben desde la metadata de io7:
//   "pubInterval", "invertir", "antirrebote_ms"
// ---------------------------------------------------------------------------
int invertir       = 0;   // 0 = HIGH en el pin es Encendido, 1 = LOW en el pin es Encendido
int antirrebote_ms = 0;   // Tiempo que el pin debe mantenerse estable para cambiar de estado. 0 = desactivado

// --- HARDWARE (solo se cambia en el código) ---
const int PIN_SENSOR = D5;
const int LED_PIN    = LED_BUILTIN;   // Lógica invertida en ESP8266 (LOW = encendido)

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000; // 5 minutos offline antes de reiniciar radio

String user_html = "";
char* ssid_pfix = (char*)"IOT_TT";

unsigned long lastPublishMillis = 0;
int defaultPubIntervalMs = 5000;   // Respaldo si el portal no trae pubInterval

uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;

// --- VARIABLES INTERNAS ---
int Estado = 0;                     // 0 = Apagado, 1 = Encendido (valor publicado)
int estadoCrudo = 0;                // Última lectura del pin (ya con inversión aplicada)
unsigned long ultimoCambioMillis = 0;

// ---------------------------------------------------------------------------
// HANDLERS IO7
// ---------------------------------------------------------------------------
void handleUserMeta() {
    JsonVariant meta = cfg["meta"];

    if (meta.containsKey("pubInterval")) {
        pubInterval = meta["pubInterval"].as<int>();
        if (pubInterval < 200) pubInterval = 200;
    }
    if (meta.containsKey("invertir"))       invertir       = meta["invertir"].as<int>() ? 1 : 0;
    if (meta.containsKey("antirrebote_ms")) {
        antirrebote_ms = meta["antirrebote_ms"].as<int>();
        if (antirrebote_ms < 0) antirrebote_ms = 0;
    }

    Serial.printf("[META] invertir=%d antirrebote=%d ms pub=%d\n",
                  invertir, antirrebote_ms, pubInterval);
}

void handleUserCommand(char* topic, JsonDocument* root) {
    // (Manejo de comandos entrantes de tu proceso)
}

// ---------------------------------------------------------------------------
// PUBLICACIÓN DE DATOS MQTT
// ---------------------------------------------------------------------------
void publishData() {
    StaticJsonDocument<512> root;
    JsonObject data = root.createNestedObject("d");

    // --- VARIABLES DEL PROCESO ---
    data["Estado"] = Estado;   // 1 = Encendido, 0 = Apagado

    // --- VARIABLES DE ESTADO ---
    data["uptime"]    = millis() / 1000;
    data["reconn"]    = reconnecciones_wifi;
    data["heap"]      = ESP.getFreeHeap();
    data["wifi_ok"]   = (WiFi.status() == WL_CONNECTED);
    data["wifi_rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
    data["status"]    = "Online";

    serializeJson(root, msgBuffer);

    if (WiFi.status() == WL_CONNECTED && client.connected()) {
        if (client.publish(evtTopic, msgBuffer)) {
            Serial.printf("TX OK | Estado: %d\n", Estado);
        } else {
            Serial.println("[MQTT] Error al enviar a IO7");
        }
    }
}

// ---------------------------------------------------------------------------
// LECTURA DEL SENSOR (estado de voltaje en el pin)
// ---------------------------------------------------------------------------
void leerSensor() {
    int lectura = (digitalRead(PIN_SENSOR) == HIGH) ? 1 : 0;
    if (invertir) lectura = !lectura;

    if (antirrebote_ms <= 0) {
        Estado = lectura;                       // Igual que el código original
    } else {
        if (lectura != estadoCrudo) {           // Cambió el pin: reinicia el conteo
            estadoCrudo = lectura;
            ultimoCambioMillis = millis();
        }
        if (millis() - ultimoCambioMillis >= (unsigned long)antirrebote_ms) {
            Estado = estadoCrudo;               // Estable el tiempo suficiente
        }
    }

    // LED indicador de la placa (LOW = encendido)
    digitalWrite(LED_PIN, (Estado == 1) ? LOW : HIGH);
}

// ---------------------------------------------------------------------------
// FUNCIÓN COOPERATIVA DE GESTIÓN DE WIFI Y MQTT
// ---------------------------------------------------------------------------
void handleNetworkTask() {
    bool mqtt_ok = (WiFi.status() == WL_CONNECTED) && client.connected();

    if (mqtt_ok) {
        if (!wifiWasConnected) {
            wifiWasConnected = true;
            wifiDownMillis = 0;
            Serial.println("[WIFI/MQTT] Conexión estable y operativa.");
        }
        client.loop();
    }
    else {
        if (wifiWasConnected) {
            reconnecciones_wifi++;
            wifiWasConnected = false;
            wifiDownMillis = millis();
            Serial.println("[ALERTA] Enlace MQTT o WiFi perdido...");
        }

        if (WiFi.status() == WL_CONNECTED) {
            static uint32_t lastTry = 0;
            if (millis() - lastTry > 5000) {
                iot_connect();
                lastTry = millis();
            }
        }

        if (wifiDownMillis != 0 && (millis() - wifiDownMillis > RESTART_TIMEOUT)) {
            Serial.println("[CRÍTICO] 5 minutos sin reportar datos. Reiniciando radio WiFi...");
            WiFi.disconnect(true);
            delay(100);
            WiFi.mode(WIFI_OFF);
            delay(100);
            WiFi.mode(WIFI_STA);

            const char* ssid = cfg["ssid"] ? (const char*)cfg["ssid"] : nullptr;
            const char* pass = cfg["w_pw"] ? (const char*)cfg["w_pw"] : nullptr;
            WiFi.begin(ssid, pass);

            wifiDownMillis = millis();
        }
    }

    if (pubInterval > 0 && millis() - lastPublishMillis >= (unsigned long)pubInterval) {
        lastPublishMillis = millis();
        publishData();
    }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT] Iniciando sistema");

    pinMode(PIN_SENSOR, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // Apaga el LED interno al arrancar

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();   // pubInterval y calibración vienen del portal io7

    if (pubInterval <= 0) pubInterval = defaultPubIntervalMs;

    const char* ssid = cfg["ssid"] ? (const char*)cfg["ssid"] : nullptr;
    const char* pass = cfg["w_pw"] ? (const char*)cfg["w_pw"] : nullptr;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    Serial.print("Conectando a WiFi inicial...");
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] ¡Conectado con éxito al inicio! IP: %s\n", WiFi.localIP().toString().c_str());
        wifiWasConnected = true;
    } else {
        Serial.println("\n[WIFI] No se pudo conectar al inicio. Iniciando modo offline temporal.");
        wifiWasConnected = false;
        wifiDownMillis = millis();
    }

    // Configura el servidor IO7 (tu código original lo llamaba; se conserva)
    set_iot_server();
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();
    leerSensor();
    yield();
}