#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <IO7F32.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ======================= CONFIGURACIÓN =======================
const char* UNIDAD             = "inHg";      // nombre del dato en io7: "psi", "bar", "grados"...
const int   FLASH_BRILLO       = 0;          // 0 = apagado ... 100 = máximo, en % (si no viene "flash" en meta)
const int   TOMAS_POR_REPORTE  = 5;          // fotos por cada envío (se reporta la mediana)
char* ssid_pfix                = (char*)"PBVS";
// Calibración inicial: cx, cy, radio, ángulo mín, barrido, valor mín, valor máx
const float CAL_INICIAL[7]     = {147.5f, 126.5f, 79.9f, -135.5f, 271.01f, 0.0f, 100.0f};
// =============================================================

struct Calib {
    float cx, cy;
    float r;
    float a0;
    float sw;
    float vmin, vmax;
};

struct Lectura {
    float valor;
    float ang;
    float contraste;
};

static inline int suma3x3(const uint8_t* g, int w, int h, int x, int y) {
    if (x < 1) x = 1;
    if (y < 1) y = 1;
    if (x > w - 2) x = w - 2;
    if (y > h - 2) y = h - 2;
    const uint8_t* p = g + (y - 1) * w + (x - 1);
    int s = 0;
    for (int j = 0; j < 3; j++) { s += p[0] + p[1] + p[2]; p += w; }
    return s;
}

static int cmpFloat(const void* a, const void* b) {
    float d = *(const float*)a - *(const float*)b;
    return (d > 0) - (d < 0);
}

inline Lectura leerAguja(const uint8_t* g, int w, int h, const Calib& c) {
    const float PASO = 0.5f;
    const float MARGEN = 8.0f;
    const int   NR = 40;
    Lectura L = {NAN, NAN, 0};
    if (c.sw <= 0 || c.r <= 0) return L;

    int n = (int)((c.sw + 2 * MARGEN) / PASO) + 1;
    if (n > 1600) n = 1600;
    float* osc  = (float*)malloc(n * sizeof(float));
    float* suav = (float*)malloc(n * sizeof(float));
    if (!osc || !suav) { free(osc); free(suav); return L; }

    for (int i = 0; i < n; i++) {
        float rel = -MARGEN + i * PASO;
        float t = (c.a0 + rel) * (float)M_PI / 180.0f;
        float s = sinf(t), co = cosf(t);
        long suma = 0;
        for (int k = 0; k < NR; k++) {
            float r = c.r * (0.30f + 0.50f * k / (NR - 1));
            int x = (int)lroundf(c.cx + r * s);
            int y = (int)lroundf(c.cy - r * co);
            suma += suma3x3(g, w, h, x, y);
        }
        osc[i] = 255.0f - suma / (9.0f * NR);
    }
    for (int i = 0; i < n; i++) {
        float s = osc[i]; int m = 1;
        if (i > 0)     { s += osc[i - 1]; m++; }
        if (i < n - 1) { s += osc[i + 1]; m++; }
        suav[i] = s / m;
    }
    int best = 0;
    for (int i = 1; i < n; i++) if (suav[i] > suav[best]) best = i;
    float maxOsc = suav[best];

    memcpy(osc, suav, n * sizeof(float));
    qsort(osc, n, sizeof(float), cmpFloat);
    float mediana = osc[n / 2];

    float rel = -MARGEN + best * PASO;
    L.ang = c.a0 + rel;
    L.valor = c.vmin + rel / c.sw * (c.vmax - c.vmin);
    L.contraste = maxOsc - mediana;
    free(osc); free(suav);
    return L;
}

void core0NetworkTask(void * pvParameters);
void publishData();
void handleUserMeta();
void handleUserCommand(char* topic, JsonDocument* root);
bool initCamera();
void cargarCalibracion();
void guardarCalibracion();
void iniciarWeb();
void hacerLectura();

unsigned long wifiDownMillis = 0;
const unsigned long RESTART_TIMEOUT = 300000;
String user_html = "";
unsigned long lastPublishMillis = 0;
uint32_t reconnecciones_wifi = 0;
bool wifiWasConnected = false;
TaskHandle_t NetworkTaskHandle;

const int LED_PIN = 33;
const int FLASH_PIN = 4;
const float CONTRASTE_MIN = 25.0f;
const char* calFile = "/calib.json";

Calib cal = {CAL_INICIAL[0], CAL_INICIAL[1], CAL_INICIAL[2], CAL_INICIAL[3], CAL_INICIAL[4], CAL_INICIAL[5], CAL_INICIAL[6]};
volatile float valorFiltrado = NAN;
volatile float valorCrudo = NAN;
volatile float anguloAguja = NAN;
volatile float contrasteAguja = 0;
float historial[20];
int nHist = 0, iHist = 0;
unsigned long lastLecturaMillis = 0;
int imgW = 320, imgH = 240;

void handleUserMeta() {
    if (cfg["meta"].containsKey("pubInterval")) {
        pubInterval = cfg["meta"]["pubInterval"].as<int>();
        if (pubInterval < 200) pubInterval = 200;
    }
    int flash = FLASH_BRILLO;
    if (cfg["meta"].containsKey("flash")) flash = cfg["meta"]["flash"].as<int>();
    flash = constrain(flash, 0, 100);
    ledcWrite(7, (flash * 255 + 50) / 100);
}

void handleUserCommand(char* topic, JsonDocument* root) {
}

void publishData() {
    StaticJsonDocument<768> root;
    JsonObject data = root.createNestedObject("d");

    float p = valorFiltrado, k = contrasteAguja;
    if (!isnan(p)) {
        data[UNIDAD]      = serialized(String(p, 1));
        data["contraste"] = (int)k;
    }

    data["uptime"] = millis() / 1000;
    data["reconn"] = reconnecciones_wifi;
    data["heap"]   = ESP.getFreeHeap();
    data["wifi_ok"]   = (WiFi.status() == WL_CONNECTED);
    data["wifi_rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
    data["status"] = "Online";
    data["ip"]     = WiFi.localIP().toString();

    serializeJson(root, msgBuffer);

    if (WiFi.status() == WL_CONNECTED && client.connected()) {
        if (client.publish(evtTopic, msgBuffer)) {
            digitalWrite(LED_PIN, LOW);
            delay(50);
            digitalWrite(LED_PIN, HIGH);
        }
    }
}

void core0NetworkTask(void * pvParameters) {
    Serial.printf("[CORE 0] Tarea de red iniciada en el núcleo: %d\n", xPortGetCoreID());

    for(;;) {
        bool mqtt_ok = (WiFi.status() == WL_CONNECTED) && client.connected();

        if (mqtt_ok) {
            if (!wifiWasConnected) {
                wifiWasConnected = true;
                wifiDownMillis = 0;
                Serial.println("[WIFI/MQTT] Conexión estable y operativa.");
                Serial.printf("[WEB] Calibrar en: http://%s/\n", WiFi.localIP().toString().c_str());
            }
            client.loop();
        }
        else {
            if (wifiWasConnected) {
                reconnecciones_wifi++;
                wifiWasConnected = false;
                wifiDownMillis = millis();
                Serial.println("[ALERTA] Enlace MQTT o WiFi perdido. Iniciando temporizador de tolerancia...");
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
                vTaskDelay(pdMS_TO_TICKS(100));
                WiFi.mode(WIFI_OFF);
                vTaskDelay(pdMS_TO_TICKS(100));
                WiFi.mode(WIFI_STA);
                const char* ssid = cfg["ssid"] ? (const char*)cfg["ssid"] : nullptr;
                const char* pass = cfg["w_pw"] ? (const char*)cfg["w_pw"] : nullptr;
                WiFi.begin(ssid, pass);
                wifiDownMillis = millis();
            }
        }

        if (pubInterval > 0 && millis() - lastPublishMillis > (unsigned long)pubInterval) {
            publishData();
            lastPublishMillis = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool initCamera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = 5;   config.pin_d1 = 18;  config.pin_d2 = 19;  config.pin_d3 = 21;
    config.pin_d4 = 36;  config.pin_d5 = 39;  config.pin_d6 = 34;  config.pin_d7 = 35;
    config.pin_xclk = 0; config.pin_pclk = 22; config.pin_vsync = 25; config.pin_href = 23;
    config.pin_sccb_sda = 26; config.pin_sccb_scl = 27;
    config.pin_pwdn = 32; config.pin_reset = -1;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Error al iniciar la cámara: 0x%x\n", err);
        return false;
    }
    Serial.println("[CAM] Cámara lista (QVGA, escala de grises)");
    return true;
}

camera_fb_t* fotoNueva() {
    camera_fb_t* viejo = esp_camera_fb_get();
    if (viejo) esp_camera_fb_return(viejo);
    return esp_camera_fb_get();
}

int numTomas() { return constrain(TOMAS_POR_REPORTE, 1, 20); }

void hacerLectura() {
    camera_fb_t* fb = fotoNueva();
    if (!fb) { Serial.println("[CAM] No se pudo tomar la foto"); return; }
    imgW = fb->width; imgH = fb->height;
    Lectura L = leerAguja(fb->buf, fb->width, fb->height, cal);
    esp_camera_fb_return(fb);
    if (isnan(L.valor)) return;

    historial[iHist] = L.valor;
    iHist = (iHist + 1) % numTomas();
    if (nHist < numTomas()) nHist++;
    float tmp[20];
    memcpy(tmp, historial, sizeof(float) * nHist);
    qsort(tmp, nHist, sizeof(float), cmpFloat);

    valorCrudo = L.valor;
    anguloAguja = L.ang;
    contrasteAguja = L.contraste;
    valorFiltrado = tmp[nHist / 2];
}

void cargarCalibracion() {
    if (!LittleFS.exists(calFile)) {
        Serial.println("[CAL] Usando calibración inicial");
        return;
    }
    File f = LittleFS.open(calFile, "r");
    StaticJsonDocument<256> d;
    if (deserializeJson(d, f) == DeserializationError::Ok) {
        cal.cx = d["cx"]; cal.cy = d["cy"]; cal.r = d["r"];
        cal.a0 = d["a0"]; cal.sw = d["sw"];
        cal.vmin = d["vmin"]; cal.vmax = d["vmax"];
        Serial.println("[CAL] Calibración cargada de la memoria");
    }
    f.close();
}

void guardarCalibracion() {
    StaticJsonDocument<256> d;
    d["cx"] = cal.cx; d["cy"] = cal.cy; d["r"] = cal.r;
    d["a0"] = cal.a0; d["sw"] = cal.sw;
    d["vmin"] = cal.vmin; d["vmax"] = cal.vmax;
    File f = LittleFS.open(calFile, "w");
    serializeJson(d, f);
    f.close();
    nHist = 0; iHist = 0;
}

const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Indicador de Aguja</title>
<style>
:root{--bg:#0f1419;--card:#1a2129;--borde:#2a333d;--txt:#e6e9ec;--sub:#8b96a3;--acento:#2f81f7;--ok:#3fb950;--mal:#f85149}
*{box-sizing:border-box}
body{margin:0;font-family:"Segoe UI",Roboto,Arial,sans-serif;background:var(--bg);color:var(--txt)}
header{display:flex;justify-content:space-between;align-items:center;padding:14px 20px;border-bottom:1px solid var(--borde)}
header h1{font-size:17px;font-weight:600;margin:0;letter-spacing:.3px}
#estado{font-size:13px;color:var(--sub)}
#estado b{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--ok);margin-right:6px}
main{max-width:720px;margin:0 auto;padding:18px 16px}
.card{background:var(--card);border:1px solid var(--borde);border-radius:10px;padding:18px;margin-bottom:16px}
.lectura{display:flex;align-items:baseline;gap:10px}
#valor{font-size:46px;font-weight:600;font-variant-numeric:tabular-nums}
#unidad{font-size:18px;color:var(--sub)}
#info{font-size:13px;color:var(--sub);margin-top:4px}
canvas{width:100%;border-radius:6px;display:block;background:#000}
.cv{cursor:default}.cv.marcar{cursor:crosshair}
.pasos{display:flex;gap:8px;margin-bottom:12px}
.paso{flex:1;padding:8px 10px;border-radius:6px;border:1px solid var(--borde);font-size:13px;color:var(--sub)}
.paso.activo{border-color:var(--acento);color:var(--txt)}
.paso.hecho{border-color:var(--ok);color:var(--ok)}
#instruccion{font-size:15px;margin:4px 0 12px}
.rango{display:none;gap:12px;margin:12px 0}
.rango label{flex:1;font-size:13px;color:var(--sub)}
.rango input{width:100%;margin-top:4px;padding:9px;font-size:16px;border-radius:6px;border:1px solid var(--borde);background:var(--bg);color:var(--txt)}
.acciones{display:flex;gap:10px;justify-content:flex-end;margin-top:12px}
button{font-size:15px;padding:10px 20px;border-radius:6px;border:1px solid transparent;cursor:pointer;font-weight:500}
.primario{background:var(--acento);color:#fff}
.primario:disabled{opacity:.4;cursor:not-allowed}
.secundario{background:transparent;color:var(--sub);border-color:var(--borde)}
#aviso{font-size:14px;margin-top:10px;min-height:18px}
.oculto{display:none!important}
</style></head><body>
<header><h1>Indicador de Aguja</h1><div id="estado"><b></b>En línea</div></header>
<main>
 <div class="card">
  <div class="lectura"><span id="valor">--</span><span id="unidad"></span></div>
  <div id="info">Esperando lectura...</div>
 </div>
 <div class="card">
  <div id="panelCal" class="oculto">
   <div class="pasos">
    <div class="paso" id="p0">1. Eje de la aguja</div>
    <div class="paso" id="p1">2. Valor mínimo</div>
    <div class="paso" id="p2">3. Valor máximo</div>
   </div>
   <div id="instruccion"></div>
  </div>
  <canvas id="cv" class="cv"></canvas>
  <div id="panelRango" class="rango">
   <label>Valor mínimo de la escala<input id="vmin" type="number" step="any" value="0"></label>
   <label>Valor máximo de la escala<input id="vmax" type="number" step="any" value="100"></label>
  </div>
  <div class="acciones">
   <button id="bCancelar" class="secundario oculto" onclick="cancelar()">Cancelar</button>
   <button id="bReiniciar" class="secundario oculto" onclick="reiniciar()">Reiniciar puntos</button>
   <button id="bGuardar" class="primario oculto" onclick="guardar()" disabled>Guardar calibración</button>
   <button id="bCalibrar" class="primario" onclick="calibrar()">Calibrar</button>
  </div>
  <div id="aviso"></div>
 </div>
</main>
<script>
const S=2,$=id=>document.getElementById(id),cv=$('cv'),ctx=cv.getContext('2d');
let est=null,modo=0,pts=[],img=new Image();
const INSTR=['Ubique el eje de la aguja.','Ubique el valor mínimo de la escala.','Ubique el valor máximo de la escala.'];
const COL=['#f85149','#3fb950','#2f81f7'];
function rad(a){return a*Math.PI/180}
function rayo(c,a,col,g){ctx.strokeStyle=col;ctx.lineWidth=g;ctx.beginPath();ctx.moveTo(c.cx*S,c.cy*S);
 ctx.lineTo((c.cx+c.r*Math.sin(rad(a)))*S,(c.cy-c.r*Math.cos(rad(a)))*S);ctx.stroke();}
function dibujar(){
 if(!img.width)return;
 cv.width=img.width*S;cv.height=img.height*S;ctx.drawImage(img,0,0,cv.width,cv.height);
 if(modo==0&&est){const c=est.cal;rayo(c,c.a0,COL[1],2);rayo(c,c.a0+c.sw,COL[2],2);if(est.ang!==null)rayo(c,est.ang,COL[0],3);}
 pts.forEach((p,i)=>{
  if(i>0){ctx.strokeStyle=COL[i];ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(pts[0][0]*S,pts[0][1]*S);ctx.lineTo(p[0]*S,p[1]*S);ctx.stroke();}
  ctx.fillStyle=COL[i];ctx.beginPath();ctx.arc(p[0]*S,p[1]*S,6,0,7);ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=1.5;ctx.stroke();});
}
img.onload=dibujar;
function aviso(t,ok){$('aviso').textContent=t||'';$('aviso').style.color=ok===false?'var(--mal)':'var(--ok)';}
async function actualizar(){
 try{est=await (await fetch('/estado')).json();
  const ok=est.contraste>=est.cmin;
  $('valor').textContent=est.valor===null?'--':est.valor.toFixed(1);
  $('unidad').textContent=est.unidad;
  $('valor').style.color=ok?'var(--txt)':'var(--mal)';
  $('info').textContent='Contraste '+est.contraste+(ok?'':' · Aguja poco visible');
  $('estado').innerHTML='<b></b>En línea';
  if(modo==0)img.src='/foto.jpg?t='+Date.now();
 }catch(e){$('estado').innerHTML='<b style="background:var(--mal)"></b>Sin conexión';}
}
function vista(){
 const cal=modo==1;
 $('panelCal').classList.toggle('oculto',!cal);
 $('bCalibrar').classList.toggle('oculto',cal);
 ['bCancelar','bReiniciar','bGuardar'].forEach(b=>$(b).classList.toggle('oculto',!cal));
 cv.classList.toggle('marcar',cal&&pts.length<3);
 $('panelRango').style.display=cal&&pts.length==3?'flex':'none';
 $('bGuardar').disabled=pts.length<3;
 for(let i=0;i<3;i++){const e=$('p'+i);e.className='paso'+(i<pts.length?' hecho':(i==pts.length&&cal?' activo':''));}
 $('instruccion').textContent=pts.length<3?INSTR[pts.length]:'Verifique que ambas líneas partan del eje e ingrese el rango de la escala.';
}
function calibrar(){modo=1;pts=[];aviso('');img.src='/foto.jpg?t='+Date.now();vista();}
function reiniciar(){pts=[];dibujar();vista();}
function cancelar(){modo=0;pts=[];aviso('');vista();actualizar();}
cv.onclick=e=>{if(modo!=1||pts.length>=3)return;const b=cv.getBoundingClientRect();
 pts.push([(e.clientX-b.left)*cv.width/b.width/S,(e.clientY-b.top)*cv.height/b.height/S]);dibujar();vista();};
function angulo(c,p){return Math.atan2(p[0]-c[0],-(p[1]-c[1]))*180/Math.PI}
async function guardar(){
 if(pts.length<3)return;
 const vmin=parseFloat($('vmin').value),vmax=parseFloat($('vmax').value);
 if(isNaN(vmin)||isNaN(vmax)||vmin==vmax){aviso('Ingrese un rango de escala válido.',false);return;}
 const c=pts[0],a0=angulo(c,pts[1]),a1=angulo(c,pts[2]);
 const sw=((a1-a0)%360+360)%360,r=(Math.hypot(pts[1][0]-c[0],pts[1][1]-c[1])+Math.hypot(pts[2][0]-c[0],pts[2][1]-c[1]))/2;
 const q=new URLSearchParams({cx:c[0].toFixed(1),cy:c[1].toFixed(1),r:r.toFixed(1),a0:a0.toFixed(2),sw:sw.toFixed(2),vmin:vmin,vmax:vmax});
 try{const res=await fetch('/calib?'+q);const t=await res.text();
  if(res.ok){modo=0;pts=[];vista();aviso(t,true);actualizar();}else aviso(t,false);
 }catch(e){aviso('No se pudo guardar. Revise la conexión.',false);}
}
vista();actualizar();setInterval(actualizar,2000);
</script></body></html>
)rawliteral";

void iniciarWeb() {
    server.on("/", []() {
        server.send_P(200, "text/html", PAGINA);
    });

    server.on("/foto.jpg", []() {
        camera_fb_t* fb = fotoNueva();
        if (!fb) { server.send(500, "text/plain", "Error de camara"); return; }
        uint8_t* jpg = nullptr; size_t len = 0;
        bool ok = frame2jpg(fb, 80, &jpg, &len);
        esp_camera_fb_return(fb);
        if (!ok) { server.send(500, "text/plain", "Error JPEG"); return; }
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "image/jpeg", (const char*)jpg, len);
        free(jpg);
    });

    server.on("/estado", []() {
        StaticJsonDocument<512> d;
        float p = valorFiltrado, c = valorCrudo, a = anguloAguja;
        const char* nulo = nullptr;
        if (isnan(p)) d["valor"] = nulo; else d["valor"] = p;
        if (isnan(c)) d["crudo"] = nulo;   else d["crudo"] = c;
        if (isnan(a)) d["ang"] = nulo;     else d["ang"] = a;
        d["contraste"] = (int)contrasteAguja;
        d["cmin"] = CONTRASTE_MIN;
        d["unidad"] = UNIDAD;
        d["w"] = imgW; d["h"] = imgH;
        JsonObject k = d.createNestedObject("cal");
        k["cx"] = cal.cx; k["cy"] = cal.cy; k["r"] = cal.r;
        k["a0"] = cal.a0; k["sw"] = cal.sw;
        String out; serializeJson(d, out);
        server.send(200, "application/json", out);
    });

    server.on("/calib", []() {
        const char* req[] = {"cx", "cy", "r", "a0", "sw", "vmin", "vmax"};
        for (auto n : req) {
            if (!server.hasArg(n)) { server.send(400, "text/plain", "Faltan datos de calibración."); return; }
        }
        Calib nueva = { server.arg("cx").toFloat(), server.arg("cy").toFloat(), server.arg("r").toFloat(),
                        server.arg("a0").toFloat(), server.arg("sw").toFloat(),
                        server.arg("vmin").toFloat(), server.arg("vmax").toFloat() };
        if (nueva.r < 5 || nueva.sw < 30) {
            server.send(400, "text/plain", "Calibración no válida. Repita el procedimiento.");
            return;
        }
        cal = nueva;
        guardarCalibracion();
        Serial.printf("[CAL] Nueva: centro(%.1f,%.1f) r=%.1f a0=%.1f barrido=%.1f  %g..%g\n",
                      cal.cx, cal.cy, cal.r, cal.a0, cal.sw, cal.vmin, cal.vmax);
        server.send(200, "text/plain", "Calibración guardada correctamente.");
    });

    server.begin();
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT] Iniciando LectorAgujaCAM");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    ledcSetup(7, 5000, 8);
    ledcAttachPin(FLASH_PIN, 7);

    initDevice();
    userMeta = handleUserMeta;
    userCommand = handleUserCommand;
    handleUserMeta();
    if (pubInterval <= 0) pubInterval = 5000;

    cargarCalibracion();
    if (!initCamera()) {
        Serial.println("[CAM] Revisa el cable de la cámara. Reiniciando en 5 s...");
        delay(5000);
        ESP.restart();
    }

    const char* ssid = cfg["ssid"] ? (const char*)cfg["ssid"] : nullptr;
    const char* pass = cfg["w_pw"] ? (const char*)cfg["w_pw"] : nullptr;
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, pass);

    Serial.print("Conectando a WiFi inicial...");
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WIFI] Conectado. Conectando a la plataforma...");
    } else {
        Serial.println("\n[WIFI] No se pudo conectar al inicio. Iniciando modo offline temporal.");
    }
    wifiWasConnected = false;
    wifiDownMillis = millis();

    iniciarWeb();
    if (cfg["devId"] && MDNS.begin((const char*)cfg["devId"])) {
        MDNS.addService("http", "tcp", 80);
    }

    xTaskCreatePinnedToCore(core0NetworkTask, "NetworkTask", 8192, NULL, 1, &NetworkTaskHandle, 0);
}

void loop() {
    unsigned long intervalo = max(300UL, (unsigned long)pubInterval / numTomas());
    if (millis() - lastLecturaMillis >= intervalo) {
        hacerLectura();
        lastLecturaMillis = millis();
    }
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
}