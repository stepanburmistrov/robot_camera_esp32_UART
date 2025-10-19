/*********************************************************************
 * Robot View — ESP32-CAM Web Joystick + MJPEG Stream (v2.4)
 * - UART2: 0xAA 0x55 <int8 L> <int8 R>   (TX2 = GPIO12)
 * - Joystick: symmetric v/w mapping, hemisphere (FWD>=0, REV<=0)
 * - Smoothing: EMA -> per-wheel deadband lift (to overcome start)
 * - Keep-alive: send every 30 ms; debug prints only on change
 * - Safety: hard STOP (L=0,R=0) on release/blur/hidden
 *********************************************************************/

#include "esp_camera.h"
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>

/* ===== UART / Pins ===== */
#define UART0_BAUD   115200
#define UART2_BAUD   115200
#define UART2_RX_PIN -1
#define UART2_TX_PIN 13   // <— TX2 pin to Arduino RX

/* ===== Camera / LED ===== */
#define MAX_CLIENTS  8
#define LED_PIN      4
#define DEF_RES      FRAMESIZE_VGA
#define DEF_FPS      7
#define DEF_JPEGQ    24
#define DEF_FLASH    0

/* ===== Robot control (global) ===== */
#define DEF_MINSTART 80    // визуальный «минимальный старт» в UI (0..127)
#define DEF_HOST     "robot"

/* ===== Keep-alive period (ms) ===== */
#define DRIVE_PERIOD_MS 30

/* ===== Per-wheel deadband lift (0..127 on ESP; *2 на Arduino) =====
 * Пример: на Arduino установлено «стартует с 70 и 100», значит
 * здесь ставим 35 и 50 для FWD/REV соответственно.
 */
#define DB_L_F 35   // левое колесо вперёд
#define DB_L_B 35   // левое назад
#define DB_R_F 50   // правое вперёд
#define DB_R_B 50   // правое назад

/* ===== Steering/easing params ===== */
#define DEADZONE     0.06f   // мёртвая зона по осям
#define SPIN_THR     0.15f   // если |Y| < SPIN_THR — разрешён «spin in place»
#define TURN_GAIN    1.0f    // коэффициент руля (0..1.2 обычно)
#define SPEED_GAMMA  1.2f    // кривая по модулю Y (1.0 = линейно)
#define EMA_ALPHA    0.35f   // сглаживание по L/R (0..1)

/* ===== AI-Thinker pins (OV2640) ===== */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

Preferences prefs;
WiFiServer http(80);
HardwareSerial& DRV = Serial2;

WiFiClient streamClients[MAX_CLIENTS];
String ssid, pass, hostname;
framesize_t frameSize;
uint8_t fps, jpgQ, flashPct, minStart;
uint16_t frameIntervalMs;
uint32_t tLastFrame = 0;

uint8_t* lastFrame = nullptr;
size_t lastCap = 0, lastLen = 0;

/* ===== Drive state ===== */
volatile int8_t cmdL = 0, cmdR = 0;   // current command for UART2
int8_t lastDbgL = 127, lastDbgR = 127;
uint32_t tLastDriveSend = 0;

/* ===== Helpers ===== */
static inline int8_t clip127(int v){ if(v<-127) return -127; if(v>127) return 127; return (int8_t)v; }

/* ===== LED PWM ===== */
#define LEDC_LED_MODE  LEDC_LOW_SPEED_MODE
#define LEDC_LED_TIMER LEDC_TIMER_2
#define LEDC_LED_CH    LEDC_CHANNEL_4
#define LEDC_LED_FREQ  5000
#define LEDC_LED_BITS  LEDC_TIMER_12_BIT
static inline void ledcInit(){
  ledc_timer_config_t t={}; t.speed_mode=LEDC_LED_MODE; t.timer_num=LEDC_LED_TIMER;
  t.duty_resolution=LEDC_LED_BITS; t.freq_hz=LEDC_LED_FREQ; t.clk_cfg=LEDC_AUTO_CLK; ledc_timer_config(&t);
  ledc_channel_config_t ch={}; ch.speed_mode=LEDC_LED_MODE; ch.channel=LEDC_LED_CH; ch.gpio_num=LED_PIN;
  ch.timer_sel=LEDC_LED_TIMER; ch.intr_type=LEDC_INTR_DISABLE; ch.duty=0; ch.hpoint=0; ledc_channel_config(&ch);
}
static inline void setLedPct(uint8_t pct){
  flashPct=pct; uint32_t val=(uint32_t)(pct*40.95f);
  ledc_set_duty(LEDC_LED_MODE,LEDC_LED_CH,val); ledc_update_duty(LEDC_LED_MODE,LEDC_LED_CH);
}

/* ===== Camera ===== */
static bool initCamera(){
  camera_config_t c={};
  c.ledc_channel=LEDC_CHANNEL_1; c.ledc_timer=LEDC_TIMER_0;
  c.pin_d0=Y2_GPIO_NUM; c.pin_d1=Y3_GPIO_NUM; c.pin_d2=Y4_GPIO_NUM; c.pin_d3=Y5_GPIO_NUM;
  c.pin_d4=Y6_GPIO_NUM; c.pin_d5=Y7_GPIO_NUM; c.pin_d6=Y8_GPIO_NUM; c.pin_d7=Y9_GPIO_NUM;
  c.pin_xclk=XCLK_GPIO_NUM; c.pin_pclk=PCLK_GPIO_NUM; c.pin_vsync=VSYNC_GPIO_NUM; c.pin_href=HREF_GPIO_NUM;
  c.pin_sscb_sda=SIOD_GPIO_NUM; c.pin_sscb_scl=SIOC_GPIO_NUM; c.pin_pwdn=PWDN_GPIO_NUM; c.pin_reset=RESET_GPIO_NUM;
  c.xclk_freq_hz=8000000; c.pixel_format=PIXFORMAT_JPEG; c.frame_size=frameSize; c.jpeg_quality=jpgQ;
  c.fb_count=psramFound()?2:1; c.grab_mode=CAMERA_GRAB_LATEST; c.fb_location=CAMERA_FB_IN_PSRAM;
  if(esp_camera_init(&c)!=ESP_OK){ Serial.println("STATUS|ERR=CAM"); return false; }
  if(sensor_t* s=esp_camera_sensor_get()) s->set_quality(s, jpgQ);
  return true;
}
static inline framesize_t str2size(const String& s){
  if (s=="QQVGA") return FRAMESIZE_QQVGA; if (s=="QVGA") return FRAMESIZE_QVGA; if (s=="HVGA") return FRAMESIZE_HVGA; if (s=="VGA") return FRAMESIZE_VGA;
  if (s=="SVGA") return FRAMESIZE_SVGA; if (s=="XGA") return FRAMESIZE_XGA; if (s=="SXGA") return FRAMESIZE_SXGA; if (s=="UXGA") return FRAMESIZE_UXGA;
  return (framesize_t)255;
}
static inline const char* size2str(framesize_t fs){
  switch(fs){case FRAMESIZE_QQVGA:return "QQVGA";case FRAMESIZE_QVGA:return "QVGA";case FRAMESIZE_HVGA:return "HVGA";case FRAMESIZE_VGA:return "VGA";
  case FRAMESIZE_SVGA:return "SVGA";case FRAMESIZE_XGA:return "XGA";case FRAMESIZE_SXGA:return "SXGA";case FRAMESIZE_UXGA:return "UXGA";default:return "?";}
}
static inline int activeClients(){int n=0; for(int i=0;i<MAX_CLIENTS;i++) if(streamClients[i]&&streamClients[i].connected()) n++; return n;}

/* ===== UART2 send ===== */
static inline void sendDrivePkt(int8_t L,int8_t R,bool printDbg){
  uint8_t pkt[4]={0xAA,0x55,(uint8_t)L,(uint8_t)R};
  DRV.write(pkt,4);
  if(printDbg && (L!=lastDbgL || R!=lastDbgR)){ lastDbgL=L; lastDbgR=R; Serial.print("[drive] L="); Serial.print(L); Serial.print(" R="); Serial.println(R); }
}

/* ===== HTML (UI + mapping) ===== */
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover, maximum-scale=1">
<title>Robot View</title>
<style>
  :root{--bg:#f6f7f9;--card:#fff;--txt:#222;--mut:#666;--br:#e2e5ea}
  *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--txt);font:16px/1.4 -apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Inter,Arial;user-select:none}
  .wrap{max-width:960px;margin:0 auto;padding:10px}
  .card{background:var(--card);border:1px solid var(--br);border-radius:14px;box-shadow:0 1px 4px rgba(0,0,0,.04);padding:10px}
  #video{width:100%;border-radius:10px}
  .joy-wrap{display:flex;flex-direction:column;align-items:center;gap:10px;margin-top:10px}
  #pad{touch-action:none;width:min(72vw,360px);height:min(72vw,360px);max-width:360px;max-height:360px;border-radius:50%;
       background:radial-gradient(ellipse at 50% 45%, #eef1f5, #e3e6ea); border:1px solid var(--br); position:relative; overflow:hidden}
  #knob{position:absolute;width:22%;height:22%;left:50%;top:50%;transform:translate(-50%,-50%);border-radius:50%;background:#8fa0b5;box-shadow:0 2px 6px rgba(0,0,0,.15)}
  .btns{display:flex;gap:10px;flex-wrap:wrap;justify-content:center}
  button{border:1px solid var(--br);background:#fff;border-radius:10px;padding:10px 14px;cursor:pointer}
  .stats{color:#546;text-align:center}
  .settings{margin-top:8px;color:#666;font-size:14px;text-align:center}
  @media (min-width:900px){ .row{display:flex;gap:10px} .left{flex:1 1 60%}.right{flex:1 1 40%}.settings{text-align:left}}
</style>
</head><body>
<div class="wrap">
  <div class="card">
    <img id="video" src="/stream" alt="stream">
    <div class="row">
      <div class="left joy-wrap">
        <div id="pad"><div id="knob"></div></div>
        <div class="btns">
          <button id="off">Light OFF</button>
          <button id="on">Light 100%</button>
        </div>
        <div class="stats" id="stats">Left 0 | Right 0</div>
      </div>
      <div class="right">
        <div class="card">
          <div style="margin-bottom:6px;font-weight:600;text-align:center">Настройка</div>
          <div style="text-align:center">
            Min start: <span id="minv">-</span>
            <input id="min" type="range" min="0" max="127" step="1" value="80" style="width:100%">
            <div class="settings">
              Разрешение: <span id="res">-</span> • FPS: <span id="fps">-</span> • Viewers: <span id="vw">-</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>

<script>
const pad=document.getElementById('pad'), knob=document.getElementById('knob'), stats=document.getElementById('stats');
const minR=document.getElementById('min'), minV=document.getElementById('minv');
const resV=document.getElementById('res'), fpsV=document.getElementById('fps'), vwV=document.getElementById('vw');

let minStart=80;
fetch('/info').then(r=>r.json()).then(j=>{minStart=j.min||80;minR.value=minStart;minV.textContent=minStart;resV.textContent=j.res;fpsV.textContent=j.fps;vwV.textContent=j.viewers;});

let rect,cx,cy,R,dragging=false;
function layout(){rect=pad.getBoundingClientRect();cx=rect.left+rect.width/2;cy=rect.top+rect.height/2;R=rect.width*0.45;}
addEventListener('resize',layout,{passive:true}); layout();

const DZ=0.06, SPIN_THR=0.15, SPEED_GAMMA=1.2, TURN_GAIN=1.0, ALPHA=0.35;

let sL=0, sR=0, wantL=0, wantR=0, lastSentL=999, lastSentR=999, aborter=null;

function clamp127(v){return v<-127?-127:(v>127?127:(v|0));}
function scaleSpeed(norm){ if(norm<=0) return 0; const k=Math.pow(norm,SPEED_GAMMA); return Math.round(minStart+(127-minStart)*k); }

// v/w mapping with hemisphere rule
function mapStick(nx,ny){
  let ax=Math.abs(nx), ay=Math.abs(ny);
  nx = (ax<DZ)?0:Math.sign(nx)*(ax-DZ)/(1-DZ);
  ny = (ay<DZ)?0:Math.sign(ny)*(ay-DZ)/(1-DZ);

  if (Math.abs(ny)<SPIN_THR && Math.abs(nx)>DZ){
    const s=scaleSpeed(Math.abs(nx));
    return [ -Math.sign(nx)*s, Math.sign(nx)*s ];
  }

  const v = ny;                 // -1..1
  const w = nx;                 // -1..1
  const s = scaleSpeed(Math.abs(v));  // 0..127

  const t = Math.min(1, Math.abs(w)*TURN_GAIN); // 0..1
  let L = s*(1 - t*Math.sign(v)*Math.sign(w)); // несимметричные? — знак v учитываем
  let R = s*(1 + t*Math.sign(v)*Math.sign(w));

  if (v>=0){ L=Math.max(0,L); R=Math.max(0,R); } else { L=-Math.max(0,L); R=-Math.max(0,R); }
  return [clamp127(Math.round(L)), clamp127(Math.round(R))];
}

// EMA -> show -> queue
function applyAndShow(L,R){
  sL = Math.round(sL + ALPHA*(L - sL));
  sR = Math.round(sR + ALPHA*(R - sR));
  stats.textContent=`Left ${sL} | Right ${sR}`;
  wantL=sL; wantR=sR;
}

function updateFromPointer(x,y){
  let dx=x-cx, dy=cy-y;
  const len=Math.hypot(dx,dy), k=len>R?R/len:1; dx*=k; dy*=k;
  const [L,Rv]=mapStick(dx/R, dy/R);
  knob.style.transform=`translate(calc(-50% + ${dx}px), calc(-50% + ${-dy}px))`;
  applyAndShow(L,Rv);
}

// hard STOP and immediate 0
function hardStop(){
  sL=0;sR=0;wantL=0;wantR=0;lastSentL=999;lastSentR=999;
  stats.textContent='Left 0 | Right 0';
  knob.style.transform='translate(-50%,-50%)';
  fetch('/drive?l=0&r=0',{cache:'no-store'}).catch(()=>{});
}

pad.addEventListener('pointerdown',e=>{dragging=true;pad.setPointerCapture(e.pointerId);layout();updateFromPointer(e.clientX,e.clientY);e.preventDefault();});
pad.addEventListener('pointermove',e=>{if(!dragging)return;updateFromPointer(e.clientX,e.clientY);e.preventDefault();});
function release(e){ if(!dragging)return; dragging=false; hardStop(); e.preventDefault(); }
pad.addEventListener('pointerup',release); pad.addEventListener('pointercancel',release); pad.addEventListener('pointerleave',release);
addEventListener('blur',hardStop); document.addEventListener('visibilitychange',()=>{ if(document.hidden) hardStop(); });

// Send on change only (ESP keeps periodic keep-alive itself)
function loopSend(){
  if (wantL!==lastSentL || wantR!==lastSentR){
    lastSentL=wantL; lastSentR=wantR;
    if(aborter) aborter.abort();
    aborter = new AbortController();
    fetch(`/drive?l=${lastSentL}&r=${lastSentR}`,{signal:aborter.signal,cache:'no-store'}).catch(()=>{});
  }
  requestAnimationFrame(loopSend);
}
requestAnimationFrame(loopSend);

// UI buttons
document.getElementById('off').onclick=()=>fetch('/flash?val=0');
document.getElementById('on').onclick =()=>fetch('/flash?val=100');
minR.oninput=e=>{minStart=+e.target.value;minV.textContent=minStart;};
minR.onchange =()=>fetch('/set?min='+minStart);
</script>
</body></html>
)HTML";

/* ===== HTTP ===== */
static void acceptClients(){
  WiFiClient nc=http.available(); if(!nc) return; nc.setTimeout(50);
  String line=nc.readStringUntil('\n'); line.trim();
  uint32_t t0=millis();
  while (millis()-t0<120){ if(!nc.available()){ delay(1); continue; } String h=nc.readStringUntil('\n'); if(h=="\r"||h.length()==0) break; }

  if (line.startsWith("GET /stream")){
    for(int i=0;i<MAX_CLIENTS;i++){
      if(!streamClients[i]||!streamClients[i].connected()){
        streamClients[i]=nc; streamClients[i].setNoDelay(true);
        streamClients[i].printf("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-store, no-cache, must-revalidate\r\nPragma: no-cache\r\nConnection: keep-alive\r\n\r\n");
        return;
      }
    }
    nc.printf("HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\nBusy"); nc.stop(); return;
  }

  if (line.startsWith("GET /jpg")){
    camera_fb_t* fb=esp_camera_fb_get();
    if(!fb){ nc.printf("HTTP/1.1 500\r\nConnection: close\r\n\r\n"); nc.stop(); return; }
    nc.printf("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nCache-Control: no-store, no-cache\r\nConnection: close\r\n\r\n", fb->len);
    nc.write(fb->buf, fb->len); esp_camera_fb_return(fb); nc.stop(); return;
  }

  if (line.startsWith("GET /drive?")){
    int lpos=line.indexOf("l="), rpos=line.indexOf("r=");
    int amp=line.indexOf('&',lpos), sp=line.indexOf(' ',rpos);
    int l=0,r=0; if(lpos>0&&amp>lpos) l=line.substring(lpos+2,amp).toInt(); if(rpos>0&&sp>rpos) r=line.substring(rpos+2,sp).toInt();

    // перехватываем команду от UI (уже после EMA), дальше на UART2 уходит периодически
    cmdL=clip127(l); cmdR=clip127(r);
    sendDrivePkt(cmdL,cmdR,true); // немедленно + отладка при изменении

    nc.printf("HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n"); nc.stop(); return;
  }

  if (line.startsWith("GET /flash?")){
    int p=line.indexOf("val="), sp=line.indexOf(' ',p); int v=(p>0&&sp>p)?line.substring(p+4,sp).toInt():flashPct;
    if(v<0)v=0; if(v>100)v=100; setLedPct(v); prefs.putUChar("flash",flashPct);
    nc.printf("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK"); nc.stop(); return;
  }

  if (line.startsWith("GET /set?")){
    int p=line.indexOf("min="), sp=line.indexOf(' ',p);
    if(p>0&&sp>p){ int m=line.substring(p+4,sp).toInt(); if(m<0)m=0; if(m>127)m=127; minStart=m; prefs.putUChar("min",minStart); }
    nc.printf("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK"); nc.stop(); return;
  }

  if (line.startsWith("GET /info")){
    nc.printf("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
    nc.printf("{\"min\":%u,\"res\":\"%s\",\"fps\":%u,\"viewers\":%d}", (unsigned)minStart, size2str(frameSize), (unsigned)fps, activeClients());
    nc.stop(); return;
  }

  nc.printf("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
  nc.write_P(INDEX_HTML, strlen_P(INDEX_HTML)); nc.stop();
}

/* ===== Streaming ===== */
static void broadcastFrame(const uint8_t* buf,size_t len){
  for(int i=0;i<MAX_CLIENTS;i++){
    auto &c=streamClients[i]; if(!c||!c.connected()){ c.stop(); continue; }
    if(c.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",(unsigned)len)<=0){ c.stop(); continue; }
    size_t sent=0; uint32_t t0=millis();
    while(sent<len){
      if(!c.connected()){ c.stop(); break; }
      int chunk=(int)min((size_t)1024, len-sent);
      int w=c.write(buf+sent, chunk);
      if(w<=0){ if(millis()-t0>60){ c.stop(); break; } delay(1); continue; }
      sent+=w;
    }
    if(!c||!c.connected()) continue;
    if(c.write("\r\n",2)!=2){ c.stop(); continue; }
  }
}

/* ===== UART CLI ===== */
static void handleUart(){
  static String buf;
  while(Serial.available()){
    char c=Serial.read(); if(c=='\r') continue; if(c!='\n'){ buf+=c; continue; }
    buf.trim();
    if(buf.startsWith("SETWIFI|")){ int p=buf.indexOf('|',8); if(p>8){ ssid=buf.substring(8,p); pass=buf.substring(p+1); prefs.putString("ssid",ssid); prefs.putString("pass",pass); WiFi.begin(ssid.c_str(), pass.c_str()); Serial.println("OK"); } else Serial.println("ERR"); }
    else if(buf.startsWith("SETHOST|")){ hostname=buf.substring(8); hostname.trim(); if(!hostname.length()) hostname=DEF_HOST; prefs.putString("host",hostname); WiFi.setHostname(hostname.c_str()); Serial.println("OK"); }
    else if(buf.startsWith("SETRES|")){ framesize_t ns=str2size(buf.substring(7)); if(ns!=255){ frameSize=ns; prefs.putUChar("res",ns); esp_camera_deinit(); initCamera(); Serial.println("OK"); } else Serial.println("ERR"); }
    else if(buf.startsWith("SETFPS|")){ int nf=buf.substring(7).toInt(); if(nf>=1&&nf<=30){ fps=nf; frameIntervalMs=1000/fps; prefs.putUChar("fps",fps); Serial.println("OK"); } else Serial.println("ERR"); }
    else if(buf.startsWith("SETJPG|")){ int q=buf.substring(7).toInt(); if(q>=10&&q<=63){ jpgQ=q; prefs.putUChar("jpgq",jpgQ); if(sensor_t*s=esp_camera_sensor_get()) s->set_quality(s, jpgQ); Serial.println("OK"); } else Serial.println("ERR"); }
    else if(buf.startsWith("FLASH|")){ int v=buf.substring(6).toInt(); if(v<0)v=0; if(v>100)v=100; setLedPct(v); prefs.putUChar("flash",flashPct); Serial.println("OK"); }
    else if(buf.startsWith("SETMIN|")){ int m=buf.substring(7).toInt(); if(m<0)m=0; if(m>127)m=127; minStart=m; prefs.putUChar("min",minStart); Serial.println("OK"); }
    else if(buf=="IP"){ Serial.print("IP="); Serial.print(WiFi.localIP()); Serial.print(" SSID="); Serial.print(WiFi.SSID()); Serial.print(" HOST="); Serial.print(WiFi.getHostname()); Serial.print(" RSSI="); Serial.println(WiFi.RSSI()); }
    else Serial.println("ERR");
    buf="";
  }
}

/* ===== Setup / Loop ===== */
void setup(){
  Serial.begin(UART0_BAUD);
  DRV.begin(UART2_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  ledcInit(); prefs.begin("cfg", false);

  ssid=prefs.getString("ssid",""); pass=prefs.getString("pass",""); hostname=prefs.getString("host",DEF_HOST);
  frameSize=(framesize_t)prefs.getUChar("res",DEF_RES);
  fps=prefs.getUChar("fps",DEF_FPS); if(fps<1||fps>30) fps=DEF_FPS;
  jpgQ=prefs.getUChar("jpgq",DEF_JPEGQ); if(jpgQ<10||jpgQ>63) jpgQ=DEF_JPEGQ;
  frameIntervalMs=1000/fps; setLedPct(prefs.getUChar("flash",DEF_FLASH));
  minStart=prefs.getUChar("min",DEF_MINSTART);

  WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.setHostname(hostname.c_str());
  if(ssid.length()) WiFi.begin(ssid.c_str(), pass.c_str());
  if(MDNS.begin(hostname.c_str())) MDNS.addService("http","tcp",80);

  initCamera(); http.begin();
  tLastDriveSend = millis();
}

void loop(){
  handleUart();
  acceptClients();

  // periodic keep-alive
  if (millis() - tLastDriveSend >= DRIVE_PERIOD_MS){
    tLastDriveSend = millis();
    // Пер-колёсный «подхват» (lift) в UART-пакете — чтобы ардуина видела сразу достаточную команду
    int L = cmdL, R = cmdR;

    auto lift = [](int v, int dbF, int dbB)->int{
      if (v==0) return 0;
      int s = abs(v);
      int db = (v>0) ? dbF : dbB;   // разные пороги на вперёд/назад
      // поднимаем к db, сохраняя масштаб 0..127
      int out = db + ( (127 - db) * s ) / 127;
      if (out>127) out=127;
      return (v>0)? out : -out;
    };

    L = lift(L, DB_L_F, DB_L_B);
    R = lift(R, DB_R_F, DB_R_B);

    sendDrivePkt(clip127(L), clip127(R), false);
  }

  if (millis() - tLastFrame < frameIntervalMs) return;
  tLastFrame = millis();

  camera_fb_t* fb = esp_camera_fb_get(); if(!fb) return;
  if (fb->len > lastCap){ free(lastFrame); lastFrame=(uint8_t*)ps_malloc(fb->len); lastCap=lastFrame? fb->len : 0; }
  if (lastFrame && lastCap>=fb->len){ memcpy(lastFrame, fb->buf, fb->len); lastLen = fb->len; } else lastLen = 0;
  esp_camera_fb_return(fb);
  if (lastLen) broadcastFrame(lastFrame, lastLen);
}
