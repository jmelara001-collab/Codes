#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <IO7F8266.h>
#include <Adafruit_MAX31865.h>

// --- PROTOTIPOS DE FUNCIONES ---
void handleNetworkTask();
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
void initSensor();
bool maxResponde();
uint8_t codigoFalla(uint8_t fault);

// ---------------------------------------------------------------------------
// CALIBRACIÓN
// Valores por defecto; se sobrescriben desde la metadata de io7:
//   "pubInterval", "hilos", "rref", "rnominal", "offset_temp"
// ---------------------------------------------------------------------------
int   hilos       = 3;       // Tipo de PT100: 2, 3 o 4 hilos
float rref        = 430.0;   // Resistencia de referencia del módulo MAX31865 (430 para PT100)
float rnominal    = 100.0;   // Resistencia del sensor a 0 °C (100 para PT100)
float offset_temp = 0.0;     // Corrección en °C que se suma a la lectura (calibración contra patrón)

// --- HARDWARE (solo se cambia en el código) ---
// Pines SPI: CS(D2/GPIO4), MOSI(D7/GPIO13), MISO(D6/GPIO12), CLK(D5/GPIO14)
const int PIN_CS   = 4;
const int PIN_MOSI = 13;
const int PIN_MISO = 12;
const int PIN_CLK  = 14;
const int LED_PIN  = LED_BUILTIN;   // LED de la placa (lógica invertida: LOW = encendido)

// --- CÓDIGOS DE ESTADO DEL SENSOR (se publican en "sensor_codigo") ---
const uint8_t COD_OK            = 0;  // Lectura válida
const uint8_t COD_MAX_SIN_RESP  = 1;  // MAX31865 no responde (quemado / sin alimentación / SPI)
const uint8_t COD_RTD_ABIERTO   = 2;  // PT100 desconectado o cable cortado
const uint8_t COD_RTD_CORTO     = 3;  // PT100 en corto
const uint8_t COD_FALLA_REF     = 4;  // Falla en la referencia (REFIN-)
const uint8_t COD_FORCE_ABIERTO = 5;  // Cable FORCE- abierto
const uint8_t COD_VOLTAJE       = 6;  // Sobre/bajo voltaje en entradas
const uint8_t COD_LECTURA_INVAL = 7;  // Lectura fuera de rango
const uint8_t COD_DESCONOCIDA   = 8;  // Falla desconocida del MAX31865

// --- CONSTANTES INTERNAS (no necesitan ajuste) ---
const float TEMP_MIN_VALIDA = -200.0;  // Rango físico de un PT100
const float TEMP_MAX_VALIDA =  850.0;
const uint16_t MQTT_BUFFER  = 512;     // Tamaño máximo del mensaje MQTT (el default de PubSubClient es 256)

Adafruit_MAX31865 thermo = Adafruit_MAX31865(PIN_CS, PIN_MOSI, PIN_MISO, PIN_CLK);

// --- VARIABLES PARA EL CONTROL DE RED ---
unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000; // 5 minutos offline antes de reiniciar radio

String user_html = "";
char* ssid_pfix = (char*)"RTB-33";

unsigned long lastPublishMillis = 0;
int defaultPubIntervalMs = 5000;   // Respaldo si el portal no trae pubInterval

uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;

// ---------------------------------------------------------------------------
// CONFIGURACIÓN DEL SENSOR MAX31865
// ---------------------------------------------------------------------------
void initSensor() {
    max31865_numwires_t tipo = MAX31865_3WIRE;
    if (hilos == 2) tipo = MAX31865_2WIRE;
    if (hilos == 4) tipo = MAX31865_4WIRE;

    thermo.begin(tipo);
    thermo.setThresholds(0x0000, 0xFFFF);   // Umbrales conocidos: sirven para verificar la comunicación

    Serial.printf("[SENSOR] PT100 %d hilos | RREF: %.1f | RNOMINAL: %.1f | Offset: %.2f C | MAX: %s\n",
                  hilos, rref, rnominal, offset_temp, maxResponde() ? "OK" : "SIN RESPUESTA");
}

// ---------------------------------------------------------------------------
// DIAGNÓSTICO DEL MAX31865
// ---------------------------------------------------------------------------
// Si el chip responde, los umbrales leídos son distintos (0x0000 y 0xFFFF).
// Si está quemado, sin alimentación o con SPI desconectado, ambos se leen iguales.
bool maxResponde() {
    uint16_t bajo = thermo.getLowerThreshold();
    uint16_t alto = thermo.getUpperThreshold();
    return (bajo != alto);
}

// Convierte el registro de falla del MAX31865 a un código de la tabla
uint8_t codigoFalla(uint8_t fault) {
    if (fault & MAX31865_FAULT_HIGHTHRESH) return COD_RTD_ABIERTO;
    if (fault & MAX31865_FAULT_LOWTHRESH)  return COD_RTD_CORTO;
    if (fault & MAX31865_FAULT_REFINLOW)   return COD_FALLA_REF;
    if (fault & MAX31865_FAULT_REFINHIGH)  return COD_FALLA_REF;
    if (fault & MAX31865_FAULT_RTDINLOW)   return COD_FORCE_ABIERTO;
    if (fault & MAX31865_FAULT_OVUV)       return COD_VOLTAJE;
    return COD_DESCONOCIDA;
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
    if (meta.containsKey("hilos")) {
        hilos = meta["hilos"].as<int>();
        if (hilos != 2 && hilos != 4) hilos = 3;
    }
    if (meta.containsKey("rref"))        rref        = meta["rref"].as<float>();
    if (meta.containsKey("rnominal"))    rnominal    = meta["rnominal"].as<float>();
    if (meta.containsKey("offset_temp")) offset_temp = meta["offset_temp"].as<float>();

    Serial.printf("[META] hilos=%d rref=%.1f rnominal=%.1f offset=%.2f pub=%lu\n",
                  hilos, rref, rnominal, offset_temp, (unsigned long)pubInterval);

    initSensor();   // Reconfigura el MAX31865 con los nuevos valores
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

    // --- LECTURA DEL SENSOR (siempre se lee y se publica) ---
    bool     responde    = maxResponde();
    uint16_t rtd         = thermo.readRTD();
    uint8_t  fault       = thermo.readFault();
    float    temperatura = thermo.calculateTemperature(rtd, rnominal, rref) + offset_temp;
    if (isnan(temperatura) || isinf(temperatura)) temperatura = 0.0;   // Evita "nan" en el JSON

    // --- DIAGNÓSTICO ---
    uint8_t sensor_codigo = COD_OK;

    if (!responde) {
        // El chip no contesta: quemado, sin alimentación o SPI desconectado
        sensor_codigo = COD_MAX_SIN_RESP;
        Serial.println("[SENSOR] Codigo 1: MAX31865 sin respuesta (revisar alimentación y SPI)");
        initSensor();   // Reintenta por si fue un cable flojo
    } else if (fault) {
        // El MAX funciona pero reporta problema en el PT100 o su cableado
        sensor_codigo = codigoFalla(fault);
        Serial.printf("[SENSOR] Codigo %u: falla MAX31865 0x%02X\n", sensor_codigo, fault);
        thermo.clearFault();
    } else if (rtd == 0 || temperatura < TEMP_MIN_VALIDA || temperatura > TEMP_MAX_VALIDA) {
        // Lectura imposible para un PT100
        sensor_codigo = COD_LECTURA_INVAL;
        Serial.println("[SENSOR] Codigo 7: lectura inválida");
    }

    // --- VARIABLES DEL PROCESO ---
    data["temperatura"]   = serialized(String(temperatura, 2));   // Siempre 2 decimales (ej. 25.30)
    data["sensor_codigo"] = sensor_codigo;                         // 0 = OK, ver tabla de códigos

    // --- VARIABLES DE ESTADO ---
    data["uptime"]    = millis() / 1000;
    data["reconn"]    = reconnecciones_wifi;
    data["heap"]      = ESP.getFreeHeap();
    data["wifi_ok"]   = (WiFi.status() == WL_CONNECTED);
    data["wifi_rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
    data["status"]    = "Online";

    serializeJson(root, msgBuffer);
    size_t largo = measureJson(root);

    if (WiFi.status() == WL_CONNECTED && client.connected()) {
        if (client.publish(evtTopic, msgBuffer)) {
            digitalWrite(LED_PIN, LOW);    // Parpadeo: publicación exitosa
            delay(50);
            digitalWrite(LED_PIN, HIGH);
            Serial.printf("TX OK | Temp: %.2f C | Codigo: %u\n", temperatura, sensor_codigo);
        } else {
            Serial.printf("[MQTT] Error al enviar a IO7 | Mensaje: %u bytes + topic (límite %u)\n",
                          (unsigned)largo, MQTT_BUFFER);
        }
    } else {
        Serial.printf("[MQTT] Sin conexión, dato no enviado | WiFi: %s | MQTT: %s\n",
                      (WiFi.status() == WL_CONNECTED) ? "OK" : "NO",
                      client.connected() ? "OK" : "NO");
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

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();   // pubInterval y calibración vienen del portal io7 (ya llama a initSensor)

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

    set_iot_server();                    // Se conserva del código original
    client.setBufferSize(MQTT_BUFFER);   // Permite mensajes de más de 256 bytes
}

// ---------------------------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------------------------
void loop() {
    handleNetworkTask();
    yield();
}