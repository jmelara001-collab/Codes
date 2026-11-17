#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
void iniciarBus();
void detenerBus();
void preTransmission();
void postTransmission();
uint32_t combineRegisters(uint16_t high, uint16_t low);

// ---------------------------------------------------------------------------
// CALIBRACIÓN
// Valores por defecto; se sobrescriben desde la metadata de io7:
//   "pubInterval", "modbus_id", "modbus_baud"
// ---------------------------------------------------------------------------
uint8_t  modbus_id   = 1;       // ID de esclavo del Circutor Computer Smart III
uint32_t modbus_baud = 19200;   // Velocidad del bus RS-485

// --- HARDWARE (solo se cambia en el código) ---
const int RX_PIN    = 14;            // D5 -> RO del MAX485
const int TX_PIN    = 12;            // D6 -> DI del MAX485
const int DE_RE_PIN = 4;             // D2 -> DE y RE unidos
const int LED_PIN   = LED_BUILTIN;   // LED de la placa (lógica invertida: LOW = encendido)

// --- REGISTROS DEL SMART III (no necesitan ajuste) ---
const uint16_t REG_POTENCIA = 0x0052;   // Potencia activa (W), 18 registros hasta el cos phi
const uint16_t NUM_REG_POT  = 18;
const uint8_t  OFF_COSPHI   = 16;       // 0x0062 - 0x0052
const uint16_t REG_ENERGIA  = 0x0088;   // Energía activa consumida (kWh), 2 registros

// --- AHORRO DE RAM (no necesitan ajuste) ---
const int SW_BUF     = 64;    // Buffer de recepción (la respuesta más larga son ~41 bytes)
const int SW_ISR_BUF = 160;   // Buffer interno de SoftwareSerial (por defecto usa ~2.5 KB)

SoftwareSerial rs485;
ModbusMaster smart3Node;

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000; // 5 minutos offline antes de reiniciar radio

String user_html = "";
char* ssid_pfix = (char*)"IOT_Device";

unsigned long lastPublishMillis = 0;
int defaultPubIntervalMs = 5000;   // Respaldo si el portal no trae pubInterval

uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;

// --- VARIABLES INTERNAS ---
bool     busActivo   = false;   // El RS-485 solo se enciende con io7 conectado
uint8_t  id_actual   = 0;
uint32_t baud_actual = 0;

// ---------------------------------------------------------------------------
// HANDLERS IO7
// ---------------------------------------------------------------------------
void handleUserMeta() {
    JsonVariant meta = cfg["meta"];

    if (meta.containsKey("pubInterval")) {
        pubInterval = meta["pubInterval"].as<int>();
        if (pubInterval < 200) pubInterval = 200;
    }
    if (meta.containsKey("modbus_id"))   modbus_id   = meta["modbus_id"].as<int>();
    if (meta.containsKey("modbus_baud")) modbus_baud = meta["modbus_baud"].as<uint32_t>();

    if (modbus_id < 1 || modbus_id > 247) modbus_id = 1;
    if (modbus_baud == 0) modbus_baud = 19200;

    // Si el bus ya está encendido y cambió la configuración, se reinicia
    if (busActivo && (modbus_id != id_actual || modbus_baud != baud_actual)) {
        detenerBus();
        iniciarBus();
    }
}

void handleUserCommand(char* topic, JsonDocument* root) {
    // Reservado para comandos de control
}

// ---------------------------------------------------------------------------
// RS-485 / MODBUS
// ---------------------------------------------------------------------------
void preTransmission() {
    digitalWrite(DE_RE_PIN, HIGH);
    delayMicroseconds(200);
}

void postTransmission() {
    delayMicroseconds(200);
    digitalWrite(DE_RE_PIN, LOW);
}

void iniciarBus() {
    rs485.begin(modbus_baud, SWSERIAL_8N1, RX_PIN, TX_PIN, false, SW_BUF, SW_ISR_BUF);
    smart3Node.begin(modbus_id, rs485);
    smart3Node.preTransmission(preTransmission);
    smart3Node.postTransmission(postTransmission);

    id_actual   = modbus_id;
    baud_actual = modbus_baud;
    busActivo   = true;
}

// Libera la RAM del SoftwareSerial (se usa antes de cada conexión SSL)
void detenerBus() {
    if (busActivo) {
        rs485.end();
        busActivo = false;
    }
}

// Une dos registros de 16 bits (alto y bajo) en uno de 32 bits
uint32_t combineRegisters(uint16_t high, uint16_t low) {
    return ((uint32_t)high << 16) | low;
}

// ---------------------------------------------------------------------------
// PUBLICACIÓN DE DATOS MQTT (solo lee el medidor si hay conexión con io7)
// ---------------------------------------------------------------------------
void publishData() {
    if (!(WiFi.status() == WL_CONNECTED && client.connected() && busActivo)) return;

    float   kw_inst = 0;
    float   pf      = 0;
    float   mwh     = 0;
    uint8_t sensor_codigo = 0;   // 0 = OK, si no, código Modbus (ver tabla)

    // --- LECTURA 1: Potencia instantánea y cos phi ---
    uint8_t result1 = smart3Node.readInputRegisters(REG_POTENCIA, NUM_REG_POT);

    if (result1 == smart3Node.ku8MBSuccess) {
        uint32_t rawW      = combineRegisters(smart3Node.getResponseBuffer(0), smart3Node.getResponseBuffer(1));
        uint32_t rawCosPhi = combineRegisters(smart3Node.getResponseBuffer(OFF_COSPHI), smart3Node.getResponseBuffer(OFF_COSPHI + 1));

        kw_inst = rawW / 1000.0;                                          // W -> kW
        pf = (rawCosPhi > 100) ? rawCosPhi / 1000.0 : rawCosPhi / 100.0;  // Autodetección de escala
    } else {
        sensor_codigo = result1;
    }

    delay(50);   // Pausa para no saturar el bus RS-485

    // --- LECTURA 2: Energía activa consumida ---
    uint8_t result2 = smart3Node.readInputRegisters(REG_ENERGIA, 2);

    if (result2 == smart3Node.ku8MBSuccess) {
        uint32_t rawKWh = combineRegisters(smart3Node.getResponseBuffer(0), smart3Node.getResponseBuffer(1));
        mwh = rawKWh / 1000.0;                                            // kWh -> MWh
    } else {
        sensor_codigo = result2;
    }

    // Si alguna lectura falló, se envía todo en 0 (igual que el código original)
    if (sensor_codigo != 0) {
        kw_inst = 0;
        mwh     = 0;
        pf      = 0;
    }

    StaticJsonDocument<256> root;
    JsonObject data = root.createNestedObject("d");

    // --- VARIABLES DEL PROCESO ---
    data["kw_inst"]       = serialized(String(kw_inst, 2));
    data["mwh"]           = serialized(String(mwh, 2));
    data["pf"]            = serialized(String(pf, 2));
    data["sensor_codigo"] = sensor_codigo;

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
        if (!busActivo) iniciarBus();   // El Modbus arranca hasta que io7 está conectado
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
                detenerBus();     // Libera RAM para el SSL
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
// SETUP (primero io7 con toda la RAM libre; el Modbus se enciende después)
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT]");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);   // LED apagado al arrancar

    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, LOW);  // MAX485 en modo recepción

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();   // pubInterval y Modbus vienen del portal io7

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
        iot_connect();   // SSL con el bus apagado = máxima RAM disponible
    }
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();
    yield();
}