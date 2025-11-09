// ESP32-S3 Base — ESP-NOW receiver with V1/V2/V3 support, multi-node overlay charts,
// proper timestamp reconstruction for V3 batches, NeoPixel alerts, and Clear button.
// NeoPixel pin set to 38. Fill WIFI_SSID/PASS. Enable PSRAM if available.

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <time.h>
#include <esp_heap_caps.h>
#include <Adafruit_NeoPixel.h>
#include <esp_idf_version.h>

// ---------------- User config ----------------
const char* WIFI_SSID = "WiFI SSID";
const char* WIFI_PASS = "PASSWORD";

#define NEOPIXEL_PIN 38
#define NEOPIXELS    1
static constexpr float LOW_BATT_V  = 3.30f;   // low-battery alert threshold
static constexpr uint32_t CADENCE_SEC = 300;  // 5 min spacing inside V3 batches

// Storage sizing (target: 2-min cadence → ~5040 pts over 7 days; PSRAM-backed)
static constexpr uint32_t SAMPLE_INTERVAL_SEC = 120;
static constexpr uint32_t DAYS_KEEP           = 7;
static constexpr uint32_t TARGET_CAP          = (DAYS_KEEP * 24 * 3600) / SAMPLE_INTERVAL_SEC; // ≈5040
static constexpr int      MAX_NODES           = 6;

// ---------------- Packet formats ----------------
struct SensorPktV1 {
  uint32_t node_uid;
  uint32_t seq;
  float    temperature_c;
  float    pressure_hpa;
  uint32_t ms;
};
struct SensorPktV2 {
  uint32_t node_uid;
  uint32_t seq;
  float    temperature_c;
  float    pressure_hpa;
  float    battery_v;
  uint32_t ms;
};
// V3 (batch)
struct SensorPktV3_Header {
  uint32_t node_uid;
  uint16_t count;     // <= 10
  uint16_t reserved;  // align
  uint32_t seq;
};
struct SensorPktV3_Sample {
  float    t_c;
  float    p_hpa;
  float    b_v;       // NaN or value
  uint32_t ts;        // sender ts in seconds; may be 0
};

// ---------------- Stored sample / buffers ----------------
struct Sample {
  uint32_t ts;    // UNIX seconds
  float    t_c;
  float    p_hpa;
  float    b_v;   // NaN if unknown
};

struct NodeBuf {
  uint32_t uid = 0;
  Sample*  buf = nullptr;
  uint16_t cap = 0;
  uint16_t head = 0;
  uint16_t count = 0;
  bool     used = false;
};

static NodeBuf   nodes[MAX_NODES];
static WebServer server(80);

// ---------------- NeoPixel ----------------
static Adafruit_NeoPixel pixel(NEOPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
static volatile uint32_t ledFlashUntilMs = 0;
static volatile bool     anyLowBattery   = false;

// ---------------- Web UI ----------------
const char* INDEX_HTML = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP-NOW Base — Multi-node Temp / Pressure / Battery</title>
<style>
  :root { --axis:#222; --grid:#e5e7eb; --text:#111; --muted:#555; }
  body { font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; margin: 1rem; color: var(--text); }
  #wrap { max-width: 1100px; margin:auto; }
  select, button { padding:.45rem .7rem; border:1px solid #cbd5e1; border-radius:8px; background:#fff; cursor:pointer; }
  button:hover { background:#f8fafc; }
  .row { display:flex; gap:1rem; align-items:center; flex-wrap:wrap; margin:.5rem 0 1rem; }
  .small { color:var(--muted); font-size:.92rem; }
  .card { border:1px solid #e5e7eb; border-radius:12px; padding:12px; margin:10px 0; box-shadow:0 1px 2px rgba(0,0,0,.03); }
  canvas { width:100%; height:320px; display:block; }
  .legend { display:flex; gap:12px; flex-wrap:wrap; margin:.5rem 0 .25rem; }
  .legend span { display:flex; gap:6px; align-items:center; font-size:.92rem; color:#444; }
  .sw { width:12px; height:12px; border-radius:3px; background:#888; display:inline-block; }
</style>
</head><body><div id="wrap">
<h2>ESP-NOW Base — Multi-node Temperature / Pressure / Battery (7-day rolling)</h2>
<div class="row">
  <button id="clearBtn" title="Wipe all stored data">Clear</button>
  <span class="small">Nodes: <span id="nodeCount">0</span></span>
  <span class="small">Last RX: <span id="last">—</span></span>
</div>

<div class="card">
  <div class="legend" id="legendTemp"></div>
  <canvas id="temp"></canvas>
</div>
<div class="card">
  <div class="legend" id="legendPress"></div>
  <canvas id="press"></canvas>
</div>
<div class="card">
  <div class="legend" id="legendBatt"></div>
  <canvas id="batt"></canvas>
</div>

<script>
const tempCtx  = document.getElementById('temp').getContext('2d');
const pressCtx = document.getElementById('press').getContext('2d');
const battCtx  = document.getElementById('batt').getContext('2d');
const nodeCountEl = document.getElementById('nodeCount');
const lastEl = document.getElementById('last');
const clearBtn = document.getElementById('clearBtn');

function sizeCanvas(ctx){
  const dpr=Math.max(1,window.devicePixelRatio||1), c=ctx.canvas;
  const W=c.clientWidth, H=c.clientHeight;
  if(c.width!==Math.round(W*dpr)||c.height!==Math.round(H*dpr)){ c.width=Math.round(W*dpr); c.height=Math.round(H*dpr); }
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return {W,H};
}
function niceNum(range, round){
  const e=Math.floor(Math.log10(range)), f=range/Math.pow(10,e);
  let nf; if(round){ nf=(f<1.5)?1:(f<3)?2:(f<7)?5:10; } else { nf=(f<=1)?1:(f<=2)?2:(f<=5)?5:10; }
  return nf*Math.pow(10,e);
}
function niceTicks(min,max,maxTicks=6){
  if(!isFinite(min)||!isFinite(max)||min===max){ min-=1; max+=1; }
  const range=niceNum(max-min,false);
  const step=niceNum(range/(maxTicks-1),true);
  const niceMin=Math.floor(min/step)*step, niceMax=Math.ceil(max/step)*step;
  const ticks=[]; for(let v=niceMin; v<=niceMax+0.5*step; v+=step) ticks.push(v);
  return {ticks,niceMin,niceMax,step};
}
function timeTicks(minTs,maxTs){
  const span=maxTs-minTs, ticks=[];
  if(span<=3*3600){ const step=15*60; let t=Math.floor(minTs/step)*step; for(;t<=maxTs;t+=step) ticks.push(t); }
  else if(span<=36*3600){ const step=60*60; let t=Math.floor(minTs/step)*step; for(;t<=maxTs;t+=step) ticks.push(t); }
  else{ const step=24*3600; let t=Math.floor(minTs/step)*step; for(;t<=maxTs;t+=step) ticks.push(t); }
  return ticks;
}
function fmtTime(ts,span){
  const d=new Date(ts*1000);
  return (span<=36*3600) ? d.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit'})
                         : d.toLocaleDateString([],{month:'short',day:'2-digit'});
}
// deterministic colour per UID
function colorFor(uid){
  const palette=['#0a66c2','#dc2626','#16a34a','#9333ea','#fb923c','#0891b2','#ca8a04','#ef4444','#22c55e','#3b82f6'];
  const i = Math.abs(Number(uid)) % palette.length; return palette[i];
}
function drawMulti(ctx, seriesList, yLabel){
  const {W,H}=sizeCanvas(ctx); ctx.clearRect(0,0,W,H);
  const padL=56,padR=16,padT=16,padB=36;

  let xsAll=[], ysAll=[];
  seriesList.forEach(s=>{ xsAll = xsAll.concat(s.xs||[]); ysAll = ysAll.concat(s.ys.filter(v=>v!==null&&isFinite(v))); });
  if(xsAll.length===0 || ysAll.length===0){ ctx.fillStyle='#555'; ctx.fillText('Waiting for data…',10,20); return; }

  const minTs=Math.min(...xsAll), maxTs=Math.max(...xsAll);
  const minY=Math.min(...ysAll),  maxY=Math.max(...ysAll);
  const spanTs=Math.max(1,maxTs-minTs);
  const yinfo=niceTicks(minY,maxY,6);
  const xt=timeTicks(minTs,maxTs);
  const X=x=> padL+(W-padL-padR)*(x-minTs)/spanTs;
  const Y=y=> H-padB-(H-padT-padB)*(y-yinfo.niceMin)/Math.max(1e-9,(yinfo.niceMax-yinfo.niceMin));

  // grid
  ctx.strokeStyle='#e5e7eb'; ctx.lineWidth=1; ctx.beginPath();
  yinfo.ticks.forEach(t=>{const y=Y(t); ctx.moveTo(padL,y); ctx.lineTo(W-padR,y);});
  xt.forEach(t=>{const x=X(t); ctx.moveTo(x,padT); ctx.lineTo(x,H-padB);}); ctx.stroke();

  // axes
  ctx.strokeStyle='#222'; ctx.lineWidth=1.25; ctx.beginPath();
  ctx.moveTo(padL,padT); ctx.lineTo(padL,H-padB); ctx.lineTo(W-padR,H-padB); ctx.stroke();

  // y labels
  ctx.fillStyle='#111'; ctx.textAlign='right'; ctx.textBaseline='middle';
  yinfo.ticks.forEach(t=>{ ctx.fillText((yinfo.step<1)?t.toFixed(1):t.toFixed(0), padL-8, Y(t)); });

  // y axis label
  ctx.save(); ctx.translate(16,(H-padB+padT)/2); ctx.rotate(-Math.PI/2);
  ctx.textAlign='center'; ctx.textBaseline='middle'; ctx.fillText(yLabel,0,0); ctx.restore();

  // x labels
  ctx.textAlign='center'; ctx.textBaseline='top'; const span=maxTs-minTs;
  xt.forEach(t=>{ ctx.fillText(fmtTime(t,span), X(t), H-padB+6); });

  // draw each series
  const MAXM=1500;
  seriesList.forEach(s=>{
    const xs=s.xs, ys=s.ys, col=s.color||'#0a66c2';
    ctx.strokeStyle=col; ctx.lineWidth=2; ctx.beginPath();
    let pen=false;
    for(let i=0;i<xs.length;i++){
      const v=ys[i]; if(v===null||!isFinite(v)){ pen=false; continue; }
      const x=X(xs[i]), y=Y(v);
      if(!pen){ ctx.moveTo(x,y); pen=true; } else { ctx.lineTo(x,y); }
    }
    ctx.stroke();
    // markers
    const step=Math.max(1, Math.ceil(xs.length/MAXM));
    ctx.fillStyle=col;
    for(let i=0;i<xs.length;i+=step){
      const v=ys[i]; if(v===null||!isFinite(v)) continue;
      const x=X(xs[i]), y=Y(v); ctx.beginPath(); ctx.arc(x,y,2.4,0,Math.PI*2); ctx.fill();
    }
  });
}
function uidHex(uid){ return '0x'+Number(uid).toString(16).toUpperCase().padStart(8,'0'); }
function buildLegend(el, seriesList){
  el.innerHTML=''; seriesList.forEach(s=>{ const span=document.createElement('span'); const sw=document.createElement('i'); sw.className='sw'; sw.style.background=s.color; span.appendChild(sw); span.appendChild(document.createTextNode(uidHex(s.uid))); el.appendChild(span); });
}
async function fetchAll(){
  const r=await fetch('/data_all'); const j=await r.json();
  lastEl.textContent = j.last_ts ? (new Date(j.last_ts*1000)).toLocaleString() : '—';
  nodeCountEl.textContent = j.series.length;
  const tempSeries=[], pressSeries=[], battSeries=[];
  j.series.forEach(s=>{
    const color = colorFor(s.uid);
    const xs = s.samples.map(p=>p.ts);
    tempSeries.push({uid:s.uid, color, xs, ys: s.samples.map(p=>p.t_c)});
    pressSeries.push({uid:s.uid, color, xs, ys: s.samples.map(p=>p.p_hpa)});
    battSeries.push({uid:s.uid, color, xs, ys: s.samples.map(p=>p.b_v===null?null:p.b_v)});
  });
  drawMulti(tempCtx,  tempSeries,  'Temp (°C)');
  drawMulti(pressCtx, pressSeries, 'Pressure (hPa)');
  drawMulti(battCtx,  battSeries,  'Battery (V)');
  setTimeout(fetchAll, 4000);
}
document.getElementById('clearBtn').addEventListener('click', async ()=>{ await fetch('/clear',{method:'POST'}); });
fetchAll();
window.addEventListener('resize', ()=>fetchAll());
</script>
</div></body></html>
)HTML";

// ---------------- Helpers ----------------
static uint32_t unixNow() {
  time_t now = time(nullptr);
  return (now > 100000) ? (uint32_t)now : (uint32_t)(millis()/1000);
}

static void freeNode(NodeBuf& n) {
  if (n.buf) { heap_caps_free(n.buf); n.buf = nullptr; }
  n.used = false; n.uid = 0; n.cap = n.head = n.count = 0;
}

static NodeBuf* allocNode(uint32_t uid) {
  NodeBuf* slot = nullptr;
  for (int i=0;i<MAX_NODES;i++) if (!nodes[i].used) { slot=&nodes[i]; break; }
  if (!slot) slot = &nodes[0];

  uint16_t cap   = TARGET_CAP;                 // ~5040
  size_t   bytes = cap * sizeof(Sample);

  Sample* buf = (Sample*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf) {
    cap   = min<uint16_t>(1500, TARGET_CAP);   // DRAM fallback
    bytes = cap * sizeof(Sample);
    buf   = (Sample*)heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
  }
  if (!buf) return nullptr;

  if (slot->buf) freeNode(*slot);

  slot->uid  = uid;
  slot->buf  = buf;
  slot->cap  = cap;
  slot->head = 0;
  slot->count= 0;
  slot->used = true;
  return slot;
}

static NodeBuf* getOrCreateNode(uint32_t uid) {
  for (int i=0;i<MAX_NODES;i++) if (nodes[i].used && nodes[i].uid == uid) return &nodes[i];
  return allocNode(uid);
}

static void pushSample(NodeBuf* nb, const Sample& s) {
  if (!nb || !nb->buf || nb->cap==0) return;
  nb->buf[nb->head] = s;
  nb->head  = (nb->head + 1) % nb->cap;
  if (nb->count < nb->cap) nb->count++;
}

// ---------------- ESPNOW receive ----------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
static void onNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
static void onNowRecv(const uint8_t* /*mac*/, const uint8_t* data, int len) {
#endif
  uint32_t rx_ts = unixNow();

  auto handle_single = [&](uint32_t uid, float t_c, float p_hpa, float b_v, uint32_t ts){
    NodeBuf* nb = getOrCreateNode(uid); if (!nb) return;
    Sample s; s.ts = ts ? ts : rx_ts; s.t_c = t_c; s.p_hpa = p_hpa; s.b_v = b_v;
    pushSample(nb, s);
  };

  bool handled = false;

  // ---- Try V3 batch first (variable length) ----
  if (len >= (int)sizeof(SensorPktV3_Header)) {
    const SensorPktV3_Header* h = reinterpret_cast<const SensorPktV3_Header*>(data);
    uint16_t n = h->count;
    if (n > 0 && n <= 10) {
      size_t expect = sizeof(SensorPktV3_Header) + n * sizeof(SensorPktV3_Sample);
      if ((size_t)len == expect) {
        const SensorPktV3_Sample* s = reinterpret_cast<const SensorPktV3_Sample*>(data + sizeof(SensorPktV3_Header));

        // Determine anchor time from the last sample or use receive time
        uint32_t last_ts = rx_ts;
        bool use_sender_last = false;

        if (s[n-1].ts != 0) {
          int32_t dt = (int32_t)((int64_t)s[n-1].ts - (int64_t)rx_ts);
          if (abs(dt) <= 600) { last_ts = s[n-1].ts; use_sender_last = true; }
        }

        // Check if per-sample timestamps are all zero or all equal
        bool all_equal_or_zero = true;
        for (uint16_t i=0; i+1<n; ++i) {
          if (s[i].ts != 0 && s[i].ts != s[n-1].ts) { all_equal_or_zero = false; break; }
        }

        NodeBuf* nb = getOrCreateNode(h->node_uid);
        if (nb) {
          for (uint16_t i=0; i<n; ++i) {
            Sample out;
            if (!all_equal_or_zero && s[i].ts != 0 && use_sender_last) {
              out.ts = s[i].ts;
            } else {
              // Back-fill from the anchor at fixed cadence
              out.ts = last_ts - (uint32_t)((n - 1 - i) * CADENCE_SEC);
            }
            out.t_c   = s[i].t_c;
            out.p_hpa = s[i].p_hpa;
            out.b_v   = s[i].b_v;
            pushSample(nb, out);
          }
        }

        handled = true;
      }
    }
  }

  // ---- V2 (fixed length) ----
  if (!handled && len == (int)sizeof(SensorPktV2)) {
    const SensorPktV2* p = reinterpret_cast<const SensorPktV2*>(data);
    handle_single(p->node_uid, p->temperature_c, p->pressure_hpa, p->battery_v, 0);
    handled = true;
  }

  // ---- V1 (fixed length) ----
  if (!handled && len == (int)sizeof(SensorPktV1)) {
    const SensorPktV1* p = reinterpret_cast<const SensorPktV1*>(data);
    handle_single(p->node_uid, p->temperature_c, p->pressure_hpa, NAN, 0);
    handled = true;
  }

  if (!handled) return; // unknown payload

  // LED: brief white flash
  ledFlashUntilMs = millis() + 80;

  // Low-battery status from last sample of each node
  bool low = false;
  for (int i=0;i<MAX_NODES;i++) if (nodes[i].used && nodes[i].count>0) {
    int lastIdx = ((nodes[i].head - 1) % nodes[i].cap + nodes[i].cap) % nodes[i].cap;
    float bv = nodes[i].buf[lastIdx].b_v;
    if (!isnan(bv) && bv < LOW_BATT_V) { low = true; break; }
  }
  anyLowBattery = low;
}

// ---------------- Wi-Fi / ESPNOW / NTP ----------------
static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(250);
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.print("Channel: "); Serial.println(WiFi.channel());
}
static void initNTP()   { configTime(0,0,"pool.ntp.org","time.nist.gov"); }
static void initEspNow(){
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed, reboot"); delay(800); ESP.restart(); }
  esp_now_register_recv_cb(onNowRecv);
}

// ---------------- HTTP ----------------
static void handleRoot(){ server.send(200, "text/html", INDEX_HTML); }

static void handleDataAll(){
  uint32_t last_ts = 0;
  for (int i=0;i<MAX_NODES;i++) if (nodes[i].used && nodes[i].count>0) {
    int lastIdx = ((nodes[i].head - 1) % nodes[i].cap + nodes[i].cap) % nodes[i].cap;
    last_ts = max(last_ts, nodes[i].buf[lastIdx].ts);
  }
  String out;
  out.reserve(1024);
  out += "{\"last_ts\":" + String(last_ts) + ",\"series\":[";
  bool firstSeries = true;
  for (int i=0;i<MAX_NODES;i++) if (nodes[i].used) {
    if (!firstSeries) out += ",";
    firstSeries = false;
    out += "{\"uid\":" + String(nodes[i].uid) + ",\"samples\":[";
    int n = nodes[i].count, cap = nodes[i].cap;
    int start = ((nodes[i].head - n) % cap + cap) % cap;
    for (int k=0;k<n;k++){
      int idx = (start + k) % cap;
      if (k) out += ",";
      out += "{\"ts\":" + String(nodes[i].buf[idx].ts)
          +  ",\"t_c\":" + String(nodes[i].buf[idx].t_c, 2)
          +  ",\"p_hpa\":" + String(nodes[i].buf[idx].p_hpa, 2)
          +  ",\"b_v\":";
      if (isnan(nodes[i].buf[idx].b_v)) out += "null";
      else out += String(nodes[i].buf[idx].b_v, 3);
      out += "}";
    }
    out += "]}";
  }
  out += "]}";
  server.send(200, "application/json", out);
}

static void handleClear(){
  for (int i=0;i<MAX_NODES;i++) freeNode(nodes[i]);
  anyLowBattery = false;
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------- Arduino ----------------
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP-NOW Base — starting…");
  Serial.printf("PSRAM size: %u bytes\n", ESP.getPsramSize());

  pinMode(NEOPIXEL_PIN, OUTPUT);
  pixel.begin(); pixel.setBrightness(40); pixel.clear(); pixel.show();

  connectWiFi();
  initNTP();
  initEspNow();

  server.on("/",         handleRoot);
  server.on("/data_all", handleDataAll);
  server.on("/clear",    HTTP_POST, handleClear);
  server.begin();
  Serial.println("HTTP server started");
}

void loop(){
  server.handleClient();

  // LED state machine
  uint32_t now = millis();
  if (ledFlashUntilMs > now) {
    pixel.setPixelColor(0, pixel.Color(180, 180, 180)); pixel.show();
  } else if (anyLowBattery) {
    float phase = (now % 1200) / 1200.0f;
    float level = 0.15f + 0.85f * 0.5f * (1.0f + sinf(phase * 6.2831853f));
    uint8_t r = (uint8_t)(level * 255);
    pixel.setPixelColor(0, pixel.Color(r, 0, 0)); pixel.show();
  } else {
    if (pixel.getPixelColor(0) != 0) { pixel.clear(); pixel.show(); }
  }
}
