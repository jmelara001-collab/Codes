#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
void leerSensor();

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000;

String user_html = "";
char* ssid_pfix = (char*)"Rebobinadora2";

unsigned long lastPublishMillis = 0;

uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;

// ---------------------------------------------------------------------------
// CALIBRACIÓN
// ---------------------------------------------------------------------------
String unidad    = "mpm";  // nombre del campo
float  adc_min   = 0.0;    // ADC del punto inferior
float  adc_max   = 1023.0; // ADC del punto superior
float  valor_min = 0.0;    // valor en adc_min
float  valor_max = 500.0;  // valor en adc_max
float  alpha     = 0.3;    // filtro

const int   LED_PIN           = 2;
const int   MUESTRAS          = 16;   // ráfaga corta = menos bloqueo
const unsigned long T_LECTURA = 50;   // ms entre ráfagas (20 Hz)

// --- VARIABLES INTERNAS ---
int   adc_raw  = 0;       // promedio de la ráfaga (útil para calibrar)
float adc_filt = 0.0;     // ADC filtrado con EMA
float voltaje  = 0.0;     // solo informativo
float valor    = 0.0;     // valor final publicado (en "unidad")
bool  primeraLectura = true;
unsigned long lastSensorReadMillis = 0;

// ---------------------------------------------------------------------------
// HANDLERS IO7
// ---------------------------------------------------------------------------
void handleUserMeta() {
    JsonVariant meta = cfg["meta"];

    if (meta.containsKey("pubInterval")) {
        pubInterval = meta["pubInterval"].as<int>();
        if (pubInterval < 200) pubInterval = 200;
    }
    if (meta.containsKey("unidad"))    unidad    = meta["unidad"].as<String>();
    if (meta.containsKey("adc_min"))   adc_min   = meta["adc_min"].as<float>();
    if (meta.containsKey("adc_max"))   adc_max   = meta["adc_max"].as<float>();
    if (meta.containsKey("valor_min")) valor_min = meta["valor_min"].as<float>();
    if (meta.containsKey("valor_max")) valor_max = meta["valor_max"].as<float>();
    if (meta.containsKey("alpha")) {
        alpha = meta["alpha"].as<float>();
        if (alpha <= 0.0 || alpha > 1.0) alpha = 1.0;
    }

    Serial.printf("[META] %s | adc[%.0f-%.0f] -> valor[%.1f-%.1f] a=%.2f pub=%d\n",
                  unidad.c_str(), adc_min, adc_max, valor_min, valor_max,
                  alpha, pubInterval);
}

void handleUserCommand(char* topic, JsonDocument* root) {
    // (Manejo de comandos entrantes de tu proceso)
}

// ---------------------------------------------------------------------------
// PUBLICACIÓN DE DATOS MQTT
// ---------------------------------------------------------------------------
void publishData() {
    StaticJsonDocument<768> root;
    JsonObject data = root.createNestedObject("d");

    // --- VARIABLES DEL PROCESO ---
    data["adc_raw"] = adc_raw;                         
    data["voltaje"] = round(voltaje * 100.0) / 100.0;
    data[unidad]    = round(valor * 10.0) / 10.0;      

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
            digitalWrite(LED_PIN, LOW);
            delay(20);
            digitalWrite(LED_PIN, HIGH);
        }
    }
}

// ---------------------------------------------------------------------------
// LECTURA DEL SENSOR (señal del driver -> valor en "unidad")
// ---------------------------------------------------------------------------
void leerSensor() {
    // 1. Ráfaga corta: solo quita ruido del ADC (~2 ms), no suaviza la velocidad
    long suma = 0;
    for (int i = 0; i < MUESTRAS; i++) {
        suma += analogRead(A0);
        delayMicroseconds(100);
    }
    adc_raw = suma / MUESTRAS;

    // 2. Filtro EMA (con alpha = 1.0 queda desactivado)
    if (primeraLectura) {
        adc_filt = adc_raw;
        primeraLectura = false;
    } else {
        adc_filt = alpha * adc_raw + (1.0 - alpha) * adc_filt;
    }

    // 3. Voltaje solo como referencia
    voltaje = adc_filt * (3.3 / 1023.0);

    // 4. Recta definida por dos puntos (adc_min, valor_min) y (adc_max, valor_max)
    float v = valor_min;
    if (adc_max != adc_min) {
        v = valor_min + (adc_filt - adc_min) * (valor_max - valor_min) / (adc_max - adc_min);
    }

    // 5. Límites (debajo de adc_min -> valor_min, arriba de adc_max -> valor_max)
    if (v < valor_min) v = valor_min;
    if (v > valor_max) v = valor_max;

    valor = v;
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
    digitalWrite(LED_PIN, HIGH);

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();   // pubInterval y calibración vienen del portal io7

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
        Serial.println("\n[WIFI] ¡Conectado con éxito al inicio!");
        wifiWasConnected = true;
    } else {
        Serial.println("\n[WIFI] No se pudo conectar al inicio. Iniciando modo offline temporal.");
        wifiWasConnected = false;
        wifiDownMillis = millis();
    }
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();

    if (millis() - lastSensorReadMillis >= T_LECTURA) {
        lastSensorReadMillis = millis();
        leerSensor();
    }

    yield();
}