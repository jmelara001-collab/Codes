#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// --- ESTRUCTURA DE UN CANAL DE CORRIENTE ---
struct Canal {
    uint16_t mux;            // Par diferencial del ADS
    float    tc_primario;    // TC externo (1 = sin TC)
    float    tc_secundario;
    int      idxGanancia;    // Ganancia actual (se ajusta sola)
    float    amperios;       // Último valor medido
    uint8_t  codigo;         // Último código de estado
};

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
bool iniciarADS();
bool adsResponde();
bool leerDiferencial(uint16_t mux, int16_t &raw);
uint8_t muestrear(uint16_t mux, float &vrms, int16_t &picoAbs, bool &saturado);
void leerCanal(Canal &c);
void leerCorrientes();

// ---------------------------------------------------------------------------
// CALIBRACIÓN
// Valores por defecto; se sobrescriben desde la metadata de io7:
//   "pubInterval", "amps_por_volt",
//   "tc_primario_1", "tc_secundario_1",
//   "tc_primario_2", "tc_secundario_2"
// Amperios = Vrms x amps_por_volt x (tc_primario / tc_secundario)
// ---------------------------------------------------------------------------
float amps_por_volt = 50.0;   // Las dos donas (mismo modelo): amperios por voltio RMS (50 A / 1 V)

Canal canal1 = { ADS1X15_REG_CONFIG_MUX_DIFF_0_1, 1.0, 1.0, 0, 0.0, 0 };  // Dona J3 (A0-A1)
Canal canal2 = { ADS1X15_REG_CONFIG_MUX_DIFF_2_3, 1.0, 1.0, 0, 0.0, 0 };  // Dona J1 (A2-A3)

// --- HARDWARE (solo se cambia en el código) ---
const int     PIN_SDA  = 4;            // D2
const int     PIN_SCL  = 5;            // D1
const uint8_t ADS_ADDR = 0x48;         // ADDR a GND
const int     LED_PIN  = LED_BUILTIN;  // LED de la placa (lógica invertida: LOW = encendido)

// --- CÓDIGOS DE ESTADO (se publican en "sensor_codigo_1" y "sensor_codigo_2") ---
const uint8_t COD_OK            = 0;  // Lectura válida
const uint8_t COD_ADS_SIN_RESP  = 1;  // ADS no responde en I2C
const uint8_t COD_ADS_CONFIG    = 2;  // ADS responde pero no se pudo configurar
const uint8_t COD_TIMEOUT       = 3;  // ADS dejó de responder durante la lectura
const uint8_t COD_SATURADO      = 4;  // Saturado aun con la ganancia mínima (±4.096 V)
const uint8_t COD_LECTURA_INVAL = 5;  // Resultado no numérico

// --- MUESTREO (definido en código, no necesita ajuste) ---
const unsigned long VENTANA_US     = 166667;  // 10 ciclos exactos de 60 Hz
const unsigned long ADS_TIMEOUT_US = 5000;    // Máximo de espera por conversión (~1.2 ms a 860 SPS)
const int16_t  RAW_SATURACION      = 32000;   // Cerca del tope del ADC (32767) = saturado
const float    UMBRAL_SUBIR_GAN    = 0.40;    // Si el pico usa menos del 40 % del rango, sube ganancia

// Ganancias del ADS1115, de menor a mayor
const adsGain_t GANANCIAS[] = { GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN };
const int       NUM_GAN     = 5;

Adafruit_ADS1115 ads;

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000; // 5 minutos offline antes de reiniciar radio

String user_html = "";
char* ssid_pfix = (char*)"P1";

unsigned long lastPublishMillis = 0;
int defaultPubIntervalMs = 5000;   // Respaldo si el portal no trae pubInterval

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

    if (meta.containsKey("amps_por_volt"))   amps_por_volt        = meta["amps_por_volt"].as<float>();

    if (meta.containsKey("tc_primario_1"))   canal1.tc_primario   = meta["tc_primario_1"].as<float>();
    if (meta.containsKey("tc_secundario_1")) canal1.tc_secundario = meta["tc_secundario_1"].as<float>();
    if (meta.containsKey("tc_primario_2"))   canal2.tc_primario   = meta["tc_primario_2"].as<float>();
    if (meta.containsKey("tc_secundario_2")) canal2.tc_secundario = meta["tc_secundario_2"].as<float>();

    if (canal1.tc_primario   <= 0) canal1.tc_primario   = 1.0;
    if (canal1.tc_secundario <= 0) canal1.tc_secundario = 1.0;
    if (canal2.tc_primario   <= 0) canal2.tc_primario   = 1.0;
    if (canal2.tc_secundario <= 0) canal2.tc_secundario = 1.0;
}

void handleUserCommand(char* topic, JsonDocument* root) {
    // (Manejo de comandos entrantes de tu proceso)
}

// ---------------------------------------------------------------------------
// ADS1115: INICIALIZACIÓN Y VERIFICACIÓN
// ---------------------------------------------------------------------------
bool iniciarADS() {
    if (ads.begin(ADS_ADDR)) {
        ads.setDataRate(RATE_ADS1115_860SPS);
        return true;
    }
    return false;
}

// Comprueba que el ADS responda en el bus I2C
bool adsResponde() {
    Wire.beginTransmission(ADS_ADDR);
    return (Wire.endTransmission() == 0);
}

// Lee un par diferencial con límite de tiempo (nunca se queda trabado)
bool leerDiferencial(uint16_t mux, int16_t &raw) {
    ads.startADCReading(mux, false);
    unsigned long t0 = micros();
    while (!ads.conversionComplete()) {
        if (micros() - t0 > ADS_TIMEOUT_US) return false;
        yield();
    }
    raw = ads.getLastConversionResults();
    return true;
}

// Muestrea durante 10 ciclos exactos de 60 Hz y calcula el voltaje RMS (sin offset DC)
uint8_t muestrear(uint16_t mux, float &vrms, int16_t &picoAbs, bool &saturado) {
    double suma = 0, suma2 = 0;
    long n = 0;
    picoAbs = 0;
    saturado = false;

    unsigned long t0 = micros();
    while (micros() - t0 < VENTANA_US) {
        int16_t raw;
        if (!leerDiferencial(mux, raw)) return COD_TIMEOUT;

        int16_t a = (raw < 0) ? -raw : raw;
        if (a > picoAbs) picoAbs = a;
        if (a >= RAW_SATURACION) saturado = true;

        float v = ads.computeVolts(raw);
        suma  += v;
        suma2 += v * v;
        n++;
    }

    if (n < 10) return COD_LECTURA_INVAL;

    double media = suma / n;
    double var   = (suma2 / n) - (media * media);
    vrms = (var > 0) ? sqrt(var) : 0.0;
    return COD_OK;
}

// ---------------------------------------------------------------------------
// LECTURA DE UN CANAL CON GANANCIA AUTOMÁTICA
// ---------------------------------------------------------------------------
void leerCanal(Canal &c) {
    float   vrms = 0.0;
    int16_t pico = 0;
    bool    saturado = false;

    // Muestreo: si se satura, baja la ganancia y repite
    for (int intento = 0; intento < NUM_GAN; intento++) {
        ads.setGain(GANANCIAS[c.idxGanancia]);
        uint8_t cod = muestrear(c.mux, vrms, pico, saturado);

        if (cod != COD_OK) {
            c.amperios = 0.0;
            c.codigo   = cod;   // COD_TIMEOUT o COD_LECTURA_INVAL
            return;
        }
        if (saturado && c.idxGanancia > 0) {
            c.idxGanancia--;
            continue;
        }
        break;
    }

    // Si la señal es pequeña, sube la ganancia para la próxima lectura
    if (!saturado && c.idxGanancia < NUM_GAN - 1 && pico < (int16_t)(UMBRAL_SUBIR_GAN * 32767)) {
        c.idxGanancia++;
    }

    // Conversión a amperios (sin límite superior: se ve el pico de arranque)
    float a = vrms * amps_por_volt * (c.tc_primario / c.tc_secundario);

    if (isnan(a) || isinf(a)) {
        c.amperios = 0.0;
        c.codigo   = COD_LECTURA_INVAL;
        return;
    }

    c.amperios = a;
    c.codigo   = saturado ? COD_SATURADO : COD_OK;
}

// Lee los dos canales (si el ADS falla, ambos reportan el mismo código)
void leerCorrientes() {
    if (!adsResponde()) {
        canal1.amperios = canal2.amperios = 0.0;
        canal1.codigo   = canal2.codigo   = COD_ADS_SIN_RESP;
        return;
    }
    if (!iniciarADS()) {
        canal1.amperios = canal2.amperios = 0.0;
        canal1.codigo   = canal2.codigo   = COD_ADS_CONFIG;
        return;
    }
    leerCanal(canal1);
    leerCanal(canal2);
}

// ---------------------------------------------------------------------------
// PUBLICACIÓN DE DATOS MQTT (solo lee si hay conexión con io7)
// ---------------------------------------------------------------------------
void publishData() {
    if (!(WiFi.status() == WL_CONNECTED && client.connected())) return;

    leerCorrientes();

    StaticJsonDocument<256> root;
    JsonObject data = root.createNestedObject("d");

    // --- VARIABLES DEL PROCESO ---
    data["Amperios1"]      = serialized(String(canal1.amperios, 2));
    data["Amperios2"]      = serialized(String(canal2.amperios, 2));
    data["sensor_codigo_1"] = canal1.codigo;
    data["sensor_codigo_2"] = canal2.codigo;

    // --- VARIABLES DE ESTADO ---
    data["uptime"]    = millis() / 1000;
    data["reconn"]    = reconnecciones_wifi;
    data["heap"]      = ESP.getFreeHeap();
    data["wifi_ok"]   = true;
    data["wifi_rssi"] = WiFi.RSSI();
    data["status"]    = "Online";

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
// SETUP (primero io7; el ADS se inicia después)
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

    // Sensor: si falla, el ESP sigue conectado y lo reporta con su código
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);   // I2C rápido: más muestras por ventana
    iniciarADS();
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();
    yield();
}