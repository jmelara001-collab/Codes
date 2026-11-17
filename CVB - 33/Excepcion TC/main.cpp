#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
float twoDecimals(float value);
float calcTankPercent(float h);

// ---------------------------------------------------------------------------
// CALIBRACIÓN
// Valores por defecto; se sobrescriben desde la metadata de io7:
//   "pubInterval", "diametro", "largo", "nivel_max", "vref"
// ---------------------------------------------------------------------------
float diametro  = 2.85;   // Diámetro del tanque horizontal (m)
float largo     = 8.5;    // Largo del tanque (m)
float nivel_max = 2.85;   // Nivel (m) que corresponde a vref en A0 (rango del sensor)
float vref      = 3.3;    // Voltaje máximo esperado en A0 (V)

// --- HARDWARE (solo se cambia en el código) ---
const int LEVEL_PIN = A0;            // Señal analógica del sensor de nivel
const int LED_PIN   = LED_BUILTIN;   // LED de la placa (lógica invertida: LOW = encendido)

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000; // 5 minutos offline antes de reiniciar radio

String user_html = "";
char* ssid_pfix = (char*)"Tanqueceleste";

unsigned long lastPublishMillis = 0;
int defaultPubIntervalMs = 1000;   // Respaldo si el portal no trae pubInterval

uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;

// ---------------------------------------------------------------------------
// HANDLERS IO7
// ---------------------------------------------------------------------------
void handleUserMeta() {
    JsonVariant meta = cfg["meta"];

    if (meta.containsKey("pubInterval")) {
        pubInterval = meta["pubInterval"].as<int>();
        if (pubInterval < 200) pubInterval = 200;
    }
    if (meta.containsKey("diametro"))  diametro  = meta["diametro"].as<float>();
    if (meta.containsKey("largo"))     largo     = meta["largo"].as<float>();
    if (meta.containsKey("nivel_max")) nivel_max = meta["nivel_max"].as<float>();
    if (meta.containsKey("vref"))      vref      = meta["vref"].as<float>();

    if (diametro  <= 0) diametro  = 2.85;
    if (largo     <= 0) largo     = 8.5;
    if (nivel_max <= 0) nivel_max = diametro;
    if (vref      <= 0) vref      = 3.3;
}

void handleUserCommand(char* topic, JsonDocument* root) {
    // No hay comandos que manejar
}

// ---------------------------------------------------------------------------
// CÁLCULOS DEL TANQUE
// ---------------------------------------------------------------------------
// Limita a 2 decimales
float twoDecimals(float value) {
    return roundf(value * 100) / 100.0;
}

// Porcentaje de llenado de un tanque cilíndrico horizontal (segmento circular)
float calcTankPercent(float h) {
    float radio = diametro / 2.0;

    if (h <= 0) return 0.0;
    if (h >= 2 * radio) return 100.0;

    float theta = acos((radio - h) / radio);
    float areaSegmento = radio * radio * theta - (radio - h) * sqrt(2 * radio * h - h * h);

    float volumen      = areaSegmento * largo;
    float volumenTotal = M_PI * radio * radio * largo;

    return (volumen / volumenTotal) * 100.0;
}

// ---------------------------------------------------------------------------
// PUBLICACIÓN DE DATOS MQTT (los campos se mantienen igual que el código original)
// ---------------------------------------------------------------------------
void publishData() {
    if (!(WiFi.status() == WL_CONNECTED && client.connected())) return;

    // Una sola lectura para que voltaje, nivel y porcentaje sean coherentes
    int   adcRaw = analogRead(LEVEL_PIN);            // 0..1023
    float vA0    = (adcRaw / 1023.0) * vref;         // Voltaje leído

    float nivel = (vA0 / vref) * nivel_max;          // 0 V -> 0 m, vref -> nivel_max
    if (nivel < 0) nivel = 0;
    if (nivel > nivel_max) nivel = nivel_max;

    float porcentaje = calcTankPercent(nivel);

    StaticJsonDocument<256> root;
    JsonObject data = root.createNestedObject("d");

    // --- VARIABLES PRINCIPALES ---
    data["voltaje"]    = String(twoDecimals(vA0), 2);
    data["nivel_m"]    = String(twoDecimals(nivel), 2);
    data["porcentaje"] = String(twoDecimals(porcentaje), 2);

    // --- DIAGNÓSTICO ---
    data["adc_raw"]   = adcRaw;
    data["wifi_rssi"] = WiFi.RSSI();
    data["status"]    = "Online";
    data["mac"]       = WiFi.macAddress();

    serializeJson(root, msgBuffer);

    if (client.publish(evtTopic, msgBuffer)) {
        digitalWrite(LED_PIN, LOW);    // Parpadeo: publicación exitosa
        delay(50);
        digitalWrite(LED_PIN, HIGH);
    }
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
            Serial.println("[IO7] Conectado");
        }
        client.loop();
    }
    else {
        if (wifiWasConnected) {
            reconnecciones_wifi++;
            wifiWasConnected = false;
            wifiDownMillis = millis();
            Serial.println("[IO7] Desconectado");
        }

        if (WiFi.status() == WL_CONNECTED) {
            static uint32_t lastTry = 0;
            if (millis() - lastTry > 5000) {
                iot_connect();
                lastTry = millis();
            }
        }

        if (wifiDownMillis != 0 && (millis() - wifiDownMillis > RESTART_TIMEOUT)) {
            Serial.println("[WIFI] Reiniciando radio");
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
    Serial.println("\n[BOOT]");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // LED apagado al arrancar

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();   // pubInterval y calibración vienen del portal io7

    if (pubInterval <= 0) pubInterval = defaultPubIntervalMs;

    const char* ssid = cfg["ssid"] ? (const char*)cfg["ssid"] : nullptr;
    const char* pass = cfg["w_pw"] ? (const char*)cfg["w_pw"] : nullptr;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    Serial.print("[WIFI] Conectando");
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK");
        wifiWasConnected = true;
    } else {
        Serial.println(" sin red");
        wifiWasConnected = false;
        wifiDownMillis = millis();
    }

    set_iot_server();   // Se conserva del código original

    if (WiFi.status() == WL_CONNECTED) {
        iot_connect();   // Conecta a la instancia de inmediato
    }
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();
    yield();
}