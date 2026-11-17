#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
void actualizarRele();
int  leerEstado(JsonVariant v);

// ---------------------------------------------------------------------------
// CALIBRACIÓN
int estado      = 0;   // 0 = apagado, 1 = encendido
int rele_activo = 1;   // 1 = el relé se activa con HIGH, 0 = se activa con LOW (módulos de lógica invertida)

// --- HARDWARE (solo se cambia en el código) ---
const int RELAY_PIN = D7;            // GPIO5 -> Relé/Lámpara
const int LED_PIN   = LED_BUILTIN;   // LED de la placa (lógica invertida: LOW = encendido)

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000; // 5 minutos offline antes de reiniciar radio

String user_html = "";
char* ssid_pfix = (char*)"APLOLTPM1";

unsigned long lastPublishMillis = 0;
int defaultPubIntervalMs = 2000;   // Respaldo si el portal no trae pubInterval

uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;

// ---------------------------------------------------------------------------
// CONVERSIÓN DEL VALOR RECIBIDO (1/0, true/false, "on"/"off")
// ---------------------------------------------------------------------------
int leerEstado(JsonVariant v) {
    if (v.is<bool>()) return v.as<bool>() ? 1 : 0;
    if (v.is<const char*>()) {
        String s = v.as<String>();
        s.toLowerCase();
        return (s == "on" || s == "1" || s == "true") ? 1 : 0;
    }
    return v.as<int>() ? 1 : 0;
}

// ---------------------------------------------------------------------------
// HANDLERS IO7
// ---------------------------------------------------------------------------
void handleUserMeta() {
    JsonVariant meta = cfg["meta"];

    if (meta.containsKey("pubInterval")) {
        pubInterval = meta["pubInterval"].as<int>();
        if (pubInterval < 200) pubInterval = 200;
    }
    if (meta.containsKey("estado"))      estado      = leerEstado(meta["estado"]);
    if (meta.containsKey("rele_activo")) rele_activo = meta["rele_activo"].as<int>() ? 1 : 0;

    Serial.printf("[META] estado=%d rele_activo=%s pub=%lu\n",
                  estado, rele_activo ? "HIGH" : "LOW", (unsigned long)pubInterval);

    actualizarRele();   // Aplica el cambio de inmediato
}

void handleUserCommand(char* topic, JsonDocument* root) {
    // Acepta {"d":{"estado":...}} o {"estado":...}, con cualquier Command Type
    JsonVariant d = root->as<JsonVariant>();
    if (d.containsKey("d")) {
        d = d["d"];
    }

    if (d.containsKey("estado")) {
        estado = leerEstado(d["estado"]);
        actualizarRele();
        Serial.printf("[CMD] %s -> estado=%d\n", topic, estado);
        lastPublishMillis = 0;   // Publica enseguida para confirmar el cambio en io7
    }
}

// ---------------------------------------------------------------------------
// CONTROL DEL RELÉ
// ---------------------------------------------------------------------------
void actualizarRele() {
    int nivelOn  = rele_activo ? HIGH : LOW;
    int nivelOff = rele_activo ? LOW  : HIGH;
    digitalWrite(RELAY_PIN, (estado == 1) ? nivelOn : nivelOff);
}

// ---------------------------------------------------------------------------
// PUBLICACIÓN DE DATOS MQTT
// ---------------------------------------------------------------------------
void publishData() {
    StaticJsonDocument<512> root;
    JsonObject data = root.createNestedObject("d");

    // --- VARIABLES DEL PROCESO ---
    data["estado"] = estado;

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
            digitalWrite(LED_PIN, LOW);    // Parpadeo: publicación exitosa
            delay(50);
            digitalWrite(LED_PIN, HIGH);
            Serial.printf("TX OK | Estado: %d\n", estado);
        }
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

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // LED apagado al arrancar

    pinMode(RELAY_PIN, OUTPUT);
    actualizarRele();   // Apagado por defecto mientras carga la configuración

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();   // pubInterval y estado vienen del portal io7

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

    set_iot_server();
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();
    actualizarRele();
    yield();
}