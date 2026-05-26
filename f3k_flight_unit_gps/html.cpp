#include "include/html.h"

// ============================================================
//  Pilot data collection page (AP mode)
// ============================================================
const char PILOT_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>F3K Data Collection</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0;}
    body{font-family:'Segoe UI',Arial,sans-serif;background:#0d1117;color:#e6edf3;
         max-width:500px;margin:0 auto;}
    .tab-bar{display:flex;background:#161b22;border-bottom:1px solid #30363d;
             position:sticky;top:0;z-index:100;}
    .tab{flex:1;padding:0.75rem;text-align:center;font-size:0.9rem;font-weight:600;
         color:#8b949e;cursor:pointer;border-bottom:2px solid transparent;transition:all 0.15s;}
    .tab.active{color:#58a6ff;border-bottom-color:#58a6ff;}
    .tab-content{display:none;padding:1rem;}
    .tab-content.active{display:block;}
    iframe{width:100%;height:calc(100vh - 45px);border:none;background:#0d1117;}
    h1{font-size:1.4rem;color:#58a6ff;margin-bottom:0.2rem;}
    .sub{font-size:0.75rem;color:#8b949e;margin-bottom:1rem;}
    .badge{display:inline-block;padding:0.3rem 1rem;border-radius:20px;
           font-size:0.9rem;font-weight:700;margin:0.5rem 0 1rem;
           background:#161b22;border:1px solid #30363d;color:#8b949e;}
    .badge.GROUND{border-color:#484f58;color:#8b949e;}
    .badge.LAUNCH_WIN{border-color:#d29922;color:#d29922;}
    .badge.FLIGHT{border-color:#3fb950;color:#3fb950;}
    .badge.LANDED{border-color:#58a6ff;color:#58a6ff;}
    .badge.CALIBRATING{border-color:#d29922;color:#d29922;}
    .section{font-size:0.65rem;text-transform:uppercase;letter-spacing:0.1em;
             color:#484f58;margin:1rem 0 0.4rem;}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:0.6rem;margin-bottom:0.8rem;}
    .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:0.7rem 0.9rem;}
    .card.blue{border-color:#58a6ff;}.card.blue .val{color:#58a6ff;}
    .card.green{border-color:#3fb950;}.card.green .val{color:#3fb950;}
    .card.amber{border-color:#d29922;}.card.amber .val{color:#d29922;}
    .lbl{font-size:0.62rem;text-transform:uppercase;letter-spacing:0.07em;
         color:#8b949e;margin-bottom:0.2rem;}
    .val{font-size:1.6rem;font-weight:700;color:#f0f6fc;line-height:1.1;}
    .unit{font-size:0.72rem;color:#8b949e;margin-top:0.1rem;}
    .win-bar{background:#161b22;border:1px solid #30363d;border-radius:8px;
             padding:0.8rem;margin-bottom:0.8rem;}
    .win-bar label{font-size:0.75rem;color:#8b949e;display:block;margin-bottom:0.3rem;}
    .win-bar select,.win-bar input{background:#0d1117;color:#e6edf3;
             border:1px solid #30363d;border-radius:6px;padding:0.4rem 0.7rem;
             font-size:0.9rem;width:100%;margin-bottom:0.5rem;}
    .btn{width:100%;padding:0.7rem;border-radius:8px;border:none;font-size:1rem;
         font-weight:700;cursor:pointer;margin-bottom:0.5rem;}
    .btn-start{background:#238636;color:#fff;}
    .btn-stop{background:#b91c1c;color:#fff;}
    .btn-dl{background:#1f6feb;color:#fff;text-decoration:none;
            display:block;text-align:center;padding:0.7rem;border-radius:8px;
            font-size:1rem;font-weight:700;margin-bottom:0.5rem;}
    .progress-wrap{background:#21262d;border-radius:4px;height:8px;
                   margin-bottom:0.8rem;overflow:hidden;}
    .progress-bar{height:100%;background:#238636;border-radius:4px;
                  transition:width 1s linear;}
    .flight-table{width:100%;border-collapse:collapse;font-size:0.82rem;}
    .flight-table th{text-align:left;color:#8b949e;font-weight:600;
                     border-bottom:1px solid #30363d;padding:0.3rem 0.5rem;}
    .flight-table td{padding:0.3rem 0.5rem;border-bottom:1px solid #21262d;}
    .flight-table tr:last-child td{border-bottom:none;}
    .best{color:#3fb950;font-weight:700;}
    .status{font-size:0.68rem;color:#484f58;margin-top:0.8rem;text-align:center;
            padding-bottom:0.8rem;}
    #dot{display:inline-block;width:7px;height:7px;border-radius:50%;
         background:#3fb950;margin-right:4px;vertical-align:middle;}
  </style>
</head>
<body>
  <div class="tab-bar">
    <div class="tab active" id="tab-flight" onclick="showTab('flight')">&#x2708; Flight</div>
    <div class="tab" id="tab-telem" onclick="showTab('telem')">&#x1F4E1; Telemetry</div>
    <div class="tab" id="tab-logs" onclick="showTab('logs')">&#x1F4C1; Logs</div>
    <div class="tab" id="tab-gps" onclick="showTab('gps')">&#x1F6F0; GPS</div>
  </div>

  <div class="tab-content active" id="content-flight">
    <div style="display:flex;align-items:center;justify-content:space-between;
                margin-bottom:1rem;flex-wrap:wrap;gap:0.4rem;">
      <h1 style="margin:0;">F3K Unit <span id="uid">--</span></h1>
      <div class="badge GROUND" id="state-badge" style="margin:0;">--</div>
    </div>

    
    <div class="section">Window Control</div>
    <div class="win-bar" id="ctrl-panel">
      <label>Window duration</label>
      <select id="dur-select">
        <option value="60">1 minute</option>
        <option value="120">2 minutes</option>
        <option value="180">3 minutes</option>
        <option value="300" selected>5 minutes</option>
        <option value="600">10 minutes</option>
        <option value="900">15 minutes</option>
        <option value="custom">Custom...</option>
      </select>
      <input type="number" id="dur-custom" placeholder="Seconds" min="30" max="3600"
             style="display:none;">
      <button class="btn btn-start" onclick="startWindow()">&#9654; Start Window</button>
    </div>
    
    <div id="prep-msg" style="display:none;text-align:center;
         background:#161b22;border:2px solid #58a6ff;border-radius:8px;
         padding:1rem;margin-bottom:0.8rem;">
      <div style="font-size:0.75rem;text-transform:uppercase;letter-spacing:0.1em;
                  color:#58a6ff;margin-bottom:0.3rem;">Window opens in</div>
      <div style="font-size:2.5rem;font-weight:700;color:#58a6ff;line-height:1;"
           id="prep-countdown">--</div>
      <div style="font-size:0.75rem;color:#8b949e;margin-top:0.3rem;">seconds</div>
    </div>

    <div id="wifi-off-msg" style="display:none;text-align:center;
         background:#161b22;border:2px solid #d29922;border-radius:8px;
         padding:1rem;margin-bottom:0.8rem;">
      <div style="font-size:1.5rem;margin-bottom:0.3rem;">&#x26A1;</div>
      <div style="font-size:1rem;font-weight:700;color:#d29922;">WiFi disabled during window</div>
      <div style="font-size:0.75rem;color:#8b949e;margin-top:0.3rem;">
        Reconnect to F3K-Unit-<span id="wifi-off-unit">--</span> after window ends to download log
      </div>
    </div>

    <div id="countdown-panel" style="display:none;text-align:center;
         background:#161b22;border:2px solid #d29922;border-radius:8px;
         padding:1.5rem;margin-bottom:0.8rem;">
      <div style="font-size:0.8rem;color:#d29922;margin-bottom:0.5rem;">
        WINDOW OPENS IN
      </div>
      <div id="countdown-num" style="font-size:4rem;font-weight:700;
           color:#d29922;line-height:1;">5</div>
      <button class="btn" onclick="stopWindow()"
              style="margin-top:0.8rem;background:#21262d;color:#8b949e;
                     border:1px solid #30363d;">Cancel</button>
    </div>

    <div id="win-active" style="display:none;">
      <div class="progress-wrap">
        <div class="progress-bar" id="prog-bar" style="width:100%"></div>
      </div>
      <div style="display:flex;gap:0.5rem;margin-bottom:0.8rem;">
        <div class="card" style="flex:1;text-align:center;">
          <div class="lbl">Window remaining</div>
          <div class="val" id="win-remain" style="font-size:2rem;">--:--</div>
        </div>
      </div>
      <button class="btn btn-stop" onclick="stopWindow()">&#9209; Stop Window</button>
    </div>
    <div id="dl-panel" style="display:none;margin-bottom:0.8rem;">
      <a class="btn-dl" id="dl-link" href="#" onclick="downloadClicked()"
         style="margin-bottom:0.5rem;display:none;">
        &#x2B07; Download Sensor Log
      </a>
      <p style="font-size:0.7rem;color:#8b949e;text-align:center;">
        Sensor log kept after download &middot; Summary auto-saved on unit
      </p>
    </div>

    <div class="section">Current Flight</div>
    <div class="grid">
      <div class="card blue">
        <div class="lbl">Flight #</div>
        <div class="val" id="flight-num">--</div>
        <div class="unit">this window</div>
      </div>
      <div class="card green">
        <div class="lbl">Flight time</div>
        <div class="val" id="flight-t">--</div>
        <div class="unit">seconds</div>
      </div>
      <div class="card amber">
        <div class="lbl">Throw height</div>
        <div class="val" id="throw-ht">--</div>
        <div class="unit">feet</div>
      </div>
      <div class="card">
        <div class="lbl">Altitude</div>
        <div class="val" id="alt">--</div>
        <div class="unit">feet tared</div>
      </div>
    </div>

    <div class="section">Flight History</div>
    <div style="background:#161b22;border:1px solid #30363d;border-radius:8px;
                padding:0.6rem;overflow-x:auto;">
      <table class="flight-table">
        <thead><tr>
          <th>#</th><th>Time (s)</th><th>Throw (ft)</th><th>Peak (ft)</th><th>Score</th>
        </tr></thead>
        <tbody id="flight-hist"></tbody>
      </table>
      
      <div id="total-score" style="display:none;text-align:right;color:#58a6ff;
           font-size:0.85rem;font-weight:700;padding:0.4rem 0.5rem 0;"></div>

      <p id="no-flights" style="font-size:0.8rem;color:#484f58;text-align:center;
         padding:0.5rem;">No flights yet this window</p>
    </div>
    <div id="wifi-banner" style="display:none;text-align:center;padding:0.5rem 0.8rem;
         border-radius:6px;font-size:0.78rem;font-weight:600;margin-bottom:0.5rem;"></div>
    <div class="status"><span id="dot"></span><span id="status-txt">Connecting...</span></div>
  </div>

  <div class="tab-content" id="content-telem">
    <div style="padding:0.8rem 1rem 0;">
      <div style="display:flex;align-items:center;gap:0.8rem;background:#161b22;
                  border:1px solid #30363d;border-radius:8px;padding:0.8rem;
                  margin-bottom:0.8rem;">
        <div style="flex:1;">
          <div class="lbl">Input Mode</div>
          <div class="val" id="mode-val" style="font-size:1.1rem;">Normal</div>
        </div>
        <div style="display:flex;flex-direction:column;gap:0.3rem;">
          <button id="btn-mode0" onclick="setSimMode(0)"
                  style="padding:0.35rem 0.7rem;border-radius:6px;border:1px solid #30363d;
                         font-size:0.78rem;font-weight:700;cursor:pointer;
                         background:#21262d;color:#8b949e;">
            Normal
          </button>
          <button id="btn-mode1" onclick="setSimMode(1)"
                  style="padding:0.35rem 0.7rem;border-radius:6px;border:1px solid #30363d;
                         font-size:0.78rem;font-weight:700;cursor:pointer;
                         background:#21262d;color:#8b949e;">
            Tilt Sim
          </button>
          <button id="btn-mode2" onclick="setSimMode(2)"
                  style="padding:0.35rem 0.7rem;border-radius:6px;border:1px solid #30363d;
                         font-size:0.78rem;font-weight:700;cursor:pointer;
                         background:#21262d;color:#8b949e;">
            Parabola
          </button>
        </div>
      </div>
      <div style="display:flex;align-items:center;gap:0.8rem;background:#161b22;
                  border:1px solid #30363d;border-radius:8px;padding:0.8rem;
                  margin-bottom:0.8rem;">
        <div style="flex:1;">
          <div class="lbl">Scoring Formula</div>
          <div class="val" id="score-mode-val" style="font-size:1.0rem;">Secs-Ft</div>
        </div>
        <button id="btn-score" onclick="toggleScore()"
                style="padding:0.5rem 1rem;border-radius:8px;border:none;
                       font-size:0.85rem;font-weight:700;cursor:pointer;
                       background:#21262d;color:#8b949e;border:1px solid #30363d;">
          JoeD V1
        </button>
      </div>
    </div>
    <iframe src="about:blank" id="telem-frame" title="Telemetry"
            style="height:calc(100vh - 120px);"></iframe>
  </div>

  <div class="tab-content" id="content-logs">
    <iframe src="about:blank" id="logs-frame" title="Logs"></iframe>
  </div>

  <div class="tab-content" id="content-gps">
    <div style="padding:0.6rem 0 0.4rem;">

      <div style="display:flex;align-items:center;gap:0.6rem;margin-bottom:0.8rem;">
        <div id="gps-fix-badge"
             style="padding:0.35rem 0.9rem;border-radius:20px;font-size:0.78rem;
                    font-weight:700;letter-spacing:0.06em;background:#21262d;
                    color:#8b949e;border:1px solid #30363d;">NO FIX</div>
        <div style="font-size:0.78rem;color:#8b949e;" id="gps-sats-txt">-- sats</div>
        <div style="font-size:0.78rem;color:#8b949e;" id="gps-hdop-txt">HDOP --</div>
        <div style="font-size:0.78rem;color:#8b949e;margin-left:auto;" id="gps-age-txt"></div>
      </div>

      <div class="section-label" style="font-size:0.7rem;text-transform:uppercase;
           letter-spacing:0.1em;color:#8b949e;margin-bottom:0.5rem;">Position</div>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:0.5rem;margin-bottom:0.8rem;">
        <div class="card">
          <div class="lbl">Latitude</div>
          <div class="val" id="gps-lat" style="font-size:1.1rem;font-family:monospace;">--</div>
          <div class="unit">degrees</div>
        </div>
        <div class="card">
          <div class="lbl">Longitude</div>
          <div class="val" id="gps-lon" style="font-size:1.1rem;font-family:monospace;">--</div>
          <div class="unit">degrees</div>
        </div>
        <div class="card blue">
          <div class="lbl">MSL Altitude</div>
          <div class="val" id="gps-alt-m">--</div>
          <div class="unit">metres</div>
        </div>
        <div class="card blue">
          <div class="lbl">MSL Altitude</div>
          <div class="val" id="gps-alt-ft">--</div>
          <div class="unit">feet</div>
        </div>
      </div>

      <div class="section-label" style="font-size:0.7rem;text-transform:uppercase;
           letter-spacing:0.1em;color:#8b949e;margin-bottom:0.5rem;">Fix Quality</div>
      <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:0.5rem;margin-bottom:0.8rem;">
        <div class="card">
          <div class="lbl">Fix Type</div>
          <div class="val" id="gps-fixtype" style="font-size:1.1rem;">--</div>
        </div>
        <div class="card">
          <div class="lbl">Satellites</div>
          <div class="val" id="gps-sats">--</div>
        </div>
        <div class="card">
          <div class="lbl">HDOP</div>
          <div class="val" id="gps-hdop">--</div>
          <div class="unit">lower = better</div>
        </div>
      </div>

      <div class="section-label" style="font-size:0.7rem;text-transform:uppercase;
           letter-spacing:0.1em;color:#8b949e;margin-bottom:0.5rem;">Raw NMEA / Debug</div>
      <div style="background:#0d1117;border:1px solid #30363d;border-radius:8px;
                  padding:0.6rem;font-family:monospace;font-size:0.72rem;
                  color:#8b949e;min-height:4.5rem;line-height:1.6;"
           id="gps-raw">Waiting for data...</div>

      <div style="display:flex;align-items:center;gap:0.6rem;margin-top:0.8rem;">
        <div class="lbl" style="margin:0;">Module</div>
        <div id="gps-module-badge"
             style="padding:0.25rem 0.7rem;border-radius:12px;font-size:0.75rem;
                    font-weight:600;background:#21262d;color:#8b949e;
                    border:1px solid #30363d;">Checking...</div>
      </div>

    </div>
  </div>

<script>
  let winSecs=0,winStart=0,winActive=false,bestTime=0,isTiltMode=false;  // winStart=0 means not yet set

  function showTab(name){
    ['flight','telem','logs','gps'].forEach(t=>{
      document.getElementById('tab-'+t).classList.toggle('active',t===name);
      document.getElementById('content-'+t).classList.toggle('active',t===name);
    });
    if(name==='telem'){
      const f=document.getElementById('telem-frame');
      if(f.src==='about:blank')f.src='/debug';
    }
    if(name==='logs'){
      document.getElementById('logs-frame').src='/logs';
    }
    if(name==='gps'){
      startGpsPolling();
    }
  }

  function toggleScore(){
    const cur=parseInt(document.getElementById('score-mode-val').getAttribute('data-mode')||'0');
    const newMode=cur===0?1:0;
    fetch('/setscore?m='+newMode).then(r=>r.json()).then(d=>{
      updateScoreDisplay(d.score_mode);
    });
  }

  function updateScoreDisplay(mode){
    const el=document.getElementById('score-mode-val');
    const btn=document.getElementById('btn-score');
    el.setAttribute('data-mode',mode);
    if(mode===1){
      el.textContent='JoeD V1';
      el.style.color='#58a6ff';
      btn.textContent='Secs-Ft';
    }else{
      el.textContent='Secs-Ft';
      el.style.color='#8b949e';
      btn.textContent='JoeD V1';
    }
  }

  function setSimMode(m){
    fetch('/settilt?v='+m).then(r=>r.json()).then(d=>{
      updateModeDisplay(d.sim_mode);
    });
  }

  function updateModeDisplay(m){
    const modeNames  = ['Normal',   'Tilt Sim', 'Parabola'];
    const modeColors = ['#3fb950',  '#d29922',  '#58a6ff'];
    const el  = document.getElementById('mode-val');
    el.textContent  = modeNames[m]  || 'Normal';
    el.style.color  = modeColors[m] || '#3fb950';
    // Highlight the active button, dim the others
    [0,1,2].forEach(function(i){
      const btn = document.getElementById('btn-mode'+i);
      if (!btn) return;
      if (i === m) {
        btn.style.background   = modeColors[i];
        btn.style.color        = i === 0 ? '#000' : '#000';
        btn.style.borderColor  = modeColors[i];
      } else {
        btn.style.background   = '#21262d';
        btn.style.color        = '#8b949e';
        btn.style.borderColor  = '#30363d';
      }
    });
    isTiltMode = (m > 0);
  }

  document.getElementById('dur-select').addEventListener('change',function(){
    document.getElementById('dur-custom').style.display=
      this.value==='custom'?'block':'none';
  });

  function fmtTime(s){
    const m=Math.floor(s/60),sec=Math.floor(s%60);
    return m+':'+String(sec).padStart(2,'0');
  }
  function startWindow(){
    let sel=document.getElementById('dur-select').value;
    let secs=sel==='custom'
      ?parseInt(document.getElementById('dur-custom').value)||300
      :parseInt(sel);
    fetch('/pstart?secs='+secs);
  }
  function stopWindow(){fetch('/pstop');}
  function downloadClicked(){
    setTimeout(()=>{
      document.getElementById('dl-link').style.display='none';
      document.getElementById('dl-panel').style.display='none';
    },3000);
  }

  function poll(){
    fetch('/pstatus').then(r=>r.json()).then(d=>{
      document.getElementById('uid').textContent='#'+d.unit_id;
      const badge=document.getElementById('state-badge');
      badge.textContent=d.state_name;badge.className='badge '+d.state_name;
      document.getElementById('flight-num').textContent=d.flight_num||'--';
      document.getElementById('flight-t').textContent=d.flight_t>0?d.flight_t.toFixed(1):'--';
      document.getElementById('throw-ht').textContent=d.throw_ht>0?d.throw_ht.toFixed(1):'--';
      document.getElementById('alt').textContent=d.alt_tared.toFixed(1);

      // Sim mode sync — update buttons if server state changed
      if(typeof d.sim_mode !== 'undefined' && d.sim_mode !== window._lastSimMode){
        window._lastSimMode = d.sim_mode;
        updateModeDisplay(d.sim_mode);
      }
      updateScoreDisplay(d.score_mode);

      // Countdown display
      if(d.countdown){
        document.getElementById('ctrl-panel').style.display='none';
        document.getElementById('win-active').style.display='none';
        document.getElementById('countdown-panel').style.display='block';
        document.getElementById('countdown-num').textContent=d.countdown_remain;
        // Show WiFi-off warning during countdown so pilot sees it before connection drops
        document.getElementById('wifi-off-msg').style.display='block';
        document.getElementById('wifi-off-unit').textContent=d.unit_id;
      } else {
        document.getElementById('countdown-panel').style.display='none';
        document.getElementById('wifi-off-msg').style.display='none';
        // Window state — evaluate every poll, not just on transition
        winActive = d.win_active;
        const inCountdown = d.countdown;
        document.getElementById('ctrl-panel').style.display =
          (!winActive && !inCountdown) ? 'block' : 'none';
        document.getElementById('win-active').style.display =
          winActive ? 'block' : 'none';

        // Download link — show whenever log ready, regardless of how window ended
        if(d.log_ready){
          document.getElementById('dl-link').href='/log?n='+d.log_num;
          document.getElementById('dl-link').style.display='block';
          document.getElementById('dl-panel').style.display='block';
        }
        if(winActive){
          if(!winStart) winStart=Date.now();
          winSecs=d.win_secs;
          document.getElementById('dl-link').style.display='none';
          document.getElementById('dl-panel').style.display='none';
        } else {
          winStart=0;
        }
      }

      if(winActive){
        const remain=Math.max(0,winSecs-(Date.now()-winStart)/1000);
        const pct=(remain/winSecs)*100;
        document.getElementById('win-remain').textContent=fmtTime(remain);
        document.getElementById('prog-bar').style.width=pct+'%';
        document.getElementById('prog-bar').style.background=
          pct>30?'#238636':pct>10?'#d29922':'#f85149';
      }

      const tbody=document.getElementById('flight-hist');
      const nf=document.getElementById('no-flights');
      const totalEl=document.getElementById('total-score');
      if(d.flights&&d.flights.length>0){
        nf.style.display='none';
        bestTime=Math.max(...d.flights.map(f=>f.dur));
        tbody.innerHTML=d.flights.map(f=>
          '<tr><td>'+f.num+'</td><td class="'+(f.dur===bestTime?'best':'')+'">'
          +f.dur.toFixed(1)+'</td><td>'+f.throw.toFixed(1)+'</td><td>'
          +f.peak.toFixed(1)+'</td><td style="color:#58a6ff;font-weight:700;">'
          +f.score.toFixed(1)+'</td></tr>'
        ).join('');
        totalEl.textContent=(d.score_mode===1?'Avg: ':'Total: ')+d.total_score.toFixed(1);
        totalEl.style.display='block';
        // Prominent display when window is complete
        if(!d.win_active && !d.countdown){
          totalEl.style.fontSize='1.6rem';
          totalEl.style.padding='0.6rem 0.5rem';
          totalEl.style.background='#1f2937';
          totalEl.style.borderRadius='6px';
          totalEl.style.marginTop='0.4rem';
          totalEl.style.color='#58a6ff';
        } else {
          totalEl.style.fontSize='0.85rem';
          totalEl.style.padding='0.4rem 0.5rem 0';
          totalEl.style.background='';
          totalEl.style.borderRadius='';
          totalEl.style.marginTop='';
        }
      }else{
        nf.style.display='block';tbody.innerHTML='';
        totalEl.style.display='none';
      }

      // WiFi status banner
      // Prep countdown banner
      const prepMsg=document.getElementById('prep-msg');
      if(d.prep_active && !d.win_active){
        prepMsg.style.display='block';
        document.getElementById('prep-countdown').textContent=d.prep_remain;
      } else {
        prepMsg.style.display='none';
      }

      const banner=document.getElementById('wifi-banner');
      if(d.win_active){
        banner.style.display='block';
        banner.textContent='⚡ WiFi off — logging in progress';
        banner.style.background='#21262d';banner.style.color='#8b949e';
      }else if(!d.wifi_active){
        banner.style.display='block';
        banner.textContent='↻ WiFi restarting… download link coming shortly';
        banner.style.background='#1c2a1c';banner.style.color='#3fb950';
      }else{
        banner.style.display='none';
      }
      document.getElementById('status-txt').textContent='Live · '+new Date().toLocaleTimeString();
      document.getElementById('dot').style.background='#3fb950';
    }).catch(()=>{
      // During window WiFi is intentionally off — suppress alarming error
      if(winActive){
        const b=document.getElementById('wifi-banner');
        b.style.display='block';
        b.textContent='⚡ WiFi off — logging in progress';
        b.style.background='#21262d';b.style.color='#8b949e';
      }
      document.getElementById('status-txt').textContent='Connection lost — retrying...';
      document.getElementById('dot').style.background='#f85149';
    });
  }
  setInterval(poll,1000);
  poll();

  // ── GPS tab polling ──────────────────────────────────────────
  let gpsTimer=null, gpsLastFixMs=null;
  const FIX_NAMES={0:'No fix',1:'GPS',2:'DGPS',3:'PPS',4:'RTK',5:'Float RTK',6:'Estimated'};

  function startGpsPolling(){
    if(gpsTimer) return;   // already running
    pollGps();
    gpsTimer=setInterval(pollGps,1000);
  }

  function pollGps(){
    fetch('/pgps').then(r=>r.json()).then(d=>{
      // Module presence badge
      const modBadge=document.getElementById('gps-module-badge');
      if(d.present){
        modBadge.textContent='PA1010D OK';
        modBadge.style.background='#1a3a2a';
        modBadge.style.color='#3fb950';
        modBadge.style.borderColor='#3fb950';
      } else {
        modBadge.textContent='Not found';
        modBadge.style.background='#3a1a1a';
        modBadge.style.color='#f85149';
        modBadge.style.borderColor='#f85149';
      }

      // Fix badge
      const fixBadge=document.getElementById('gps-fix-badge');
      if(d.fix){
        gpsLastFixMs=Date.now();
        fixBadge.textContent=(FIX_NAMES[d.fix_quality]||'Fix').toUpperCase();
        fixBadge.style.background='#1a3a2a';
        fixBadge.style.color='#3fb950';
        fixBadge.style.borderColor='#3fb950';
      } else {
        fixBadge.textContent='NO FIX';
        fixBadge.style.background='#21262d';
        fixBadge.style.color='#8b949e';
        fixBadge.style.borderColor='#30363d';
      }

      // Age indicator
      const ageTxt=document.getElementById('gps-age-txt');
      if(gpsLastFixMs){
        const sec=Math.round((Date.now()-gpsLastFixMs)/1000);
        ageTxt.textContent='Last fix '+sec+'s ago';
        ageTxt.style.color=sec>10?'#d29922':'#8b949e';
      }

      // Sat / HDOP header row
      document.getElementById('gps-sats-txt').textContent=
        d.fix ? d.satellites+' sats' : '-- sats';
      document.getElementById('gps-hdop-txt').textContent=
        d.fix ? 'HDOP '+d.hdop.toFixed(1) : 'HDOP --';

      // Position cards
      document.getElementById('gps-lat').textContent=
        d.fix ? d.lat.toFixed(6) : '--';
      document.getElementById('gps-lon').textContent=
        d.fix ? d.lon.toFixed(6) : '--';
      document.getElementById('gps-alt-m').textContent=
        d.fix ? d.alt_m.toFixed(1) : '--';
      document.getElementById('gps-alt-ft').textContent=
        d.fix ? (d.alt_m*3.28084).toFixed(0) : '--';

      // Fix quality cards
      document.getElementById('gps-fixtype').textContent=
        FIX_NAMES[d.fix_quality]||('Type '+d.fix_quality);
      document.getElementById('gps-sats').textContent=
        d.fix ? d.satellites : '--';
      document.getElementById('gps-hdop').textContent=
        d.fix ? d.hdop.toFixed(1) : '--';

      // Raw debug line
      const raw=document.getElementById('gps-raw');
      let lines=[];
      lines.push('present='+d.present+'  fix='+d.fix+'  quality='+d.fix_quality);
      if(d.fix){
        lines.push('lat='+d.lat.toFixed(6)+'  lon='+d.lon.toFixed(6)
                   +'  alt='+d.alt_m.toFixed(1)+'m');
        lines.push('sats='+d.satellites+'  hdop='+d.hdop.toFixed(1));
      }
      raw.textContent=lines.join('\n');

    }).catch(()=>{
      document.getElementById('gps-raw').textContent='Poll error — WiFi may be off during window';
    });
  }
</script>
</body>
</html>)rawhtml";


// ============================================================
//  WSTATUS_HTML — window status page, served during active window
//  in AP mode. Pull-only: no polling, no timers. Single fetch per
//  Refresh tap. Data injected inline as window.__WS__ JSON object
//  by the /wstatus handler — no separate API call needed.
// ============================================================
const char WSTATUS_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
  <title>F3K Window Status</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0;}
    body{background:#0d1117;color:#e6edf3;font-family:-apple-system,BlinkMacSystemFont,
         'Segoe UI',sans-serif;padding:1rem;max-width:480px;margin:0 auto;}
    h1{font-size:1.1rem;font-weight:700;color:#58a6ff;margin-bottom:0.15rem;}
    .sub{font-size:0.75rem;color:#8b949e;margin-bottom:1rem;}
    .progress-wrap{background:#21262d;border-radius:4px;height:8px;
                   margin-bottom:1rem;overflow:hidden;}
    .progress-bar{height:8px;border-radius:4px;background:#58a6ff;}
    .row{display:flex;justify-content:space-between;align-items:baseline;
         padding:0.45rem 0;border-bottom:1px solid #21262d;font-size:0.88rem;}
    .row:last-of-type{border-bottom:none;}
    .lbl{color:#8b949e;}
    .val{font-weight:600;font-family:monospace;}
    .green{color:#3fb950;} .amber{color:#d29922;} .red{color:#f85149;}
    .section{font-size:0.7rem;text-transform:uppercase;letter-spacing:0.1em;
             color:#8b949e;margin:1rem 0 0.4rem;}
    table{width:100%;border-collapse:collapse;font-size:0.82rem;}
    th{text-align:left;color:#8b949e;font-weight:600;padding:0.3rem 0.4rem;
       border-bottom:1px solid #30363d;}
    td{padding:0.35rem 0.4rem;border-bottom:1px solid #1c2128;}
    tr:last-child td{border-bottom:none;}
    .btn{display:block;width:100%;padding:0.8rem;border:none;border-radius:8px;
         font-size:0.95rem;font-weight:700;cursor:pointer;margin-top:0.6rem;}
    .btn-refresh{background:#1f6feb;color:#fff;}
    .btn-close{background:#21262d;color:#f85149;border:1px solid #f85149;
               margin-top:0.4rem;}
    .no-flights{color:#484f58;font-size:0.8rem;text-align:center;padding:0.8rem;}
  </style>
</head>
<body>
  <h1 id="ttl">F3K Unit &#x1F4F6; Window Status</h1>
  <div class="sub" id="logfile">--</div>

  <div class="progress-wrap">
    <div class="progress-bar" id="prog" style="width:0%"></div>
  </div>

  <div class="row"><span class="lbl">Window elapsed</span>
    <span class="val" id="elapsed">--</span></div>
  <div class="row"><span class="lbl">Window remaining</span>
    <span class="val" id="remain">--</span></div>
  <div class="row"><span class="lbl">Log file size</span>
    <span class="val" id="logsize">--</span></div>
  <div class="row"><span class="lbl">Flights scored</span>
    <span class="val" id="nflights">--</span></div>
  <div class="row"><span class="lbl">GPS</span>
    <span class="val" id="gpsval">--</span></div>

  <div class="section">Flights this window</div>
  <table id="ftable" style="display:none;">
    <thead><tr>
      <th>#</th><th>Time (s)</th><th>Throw (ft)</th>
      <th>Peak (ft)</th><th>Score</th>
    </tr></thead>
    <tbody id="ftbody"></tbody>
  </table>
  <div class="no-flights" id="no-fl">No flights yet</div>

  <button class="btn btn-refresh" onclick="location.reload()">&#x21BB;&nbsp; Refresh</button>
  <button class="btn btn-close"   onclick="closeWin()">&#x25A0;&nbsp; Close Window</button>

<script>
  var d = window.__WS__;
  if (d) {
    document.getElementById('ttl').textContent =
      'F3K Unit ' + d.unit_id + '  \u{1F4F6} Window Status';
    document.getElementById('logfile').textContent =
      d.log_open ? d.log_name + '  \u00B7  ' + (d.log_bytes/1024).toFixed(1) + ' KB'
                 : 'Log not open';
    var pct = d.win_secs > 0
      ? Math.min(100, Math.round(d.elapsed_s / d.win_secs * 100)) : 0;
    document.getElementById('prog').style.width = pct + '%';
    function fmt(s) {
      var m=Math.floor(s/60), ss=Math.floor(s%60);
      return m+':'+(ss<10?'0':'')+ss;
    }
    document.getElementById('elapsed').textContent  = fmt(d.elapsed_s);
    document.getElementById('remain').textContent   =
      d.win_secs > 0 ? fmt(Math.max(0, d.win_secs - d.elapsed_s)) : '--';
    document.getElementById('logsize').textContent  =
      d.log_open ? (d.log_bytes/1024).toFixed(1)+' KB' : '--';
    document.getElementById('nflights').textContent = d.flight_count;
    var gEl = document.getElementById('gpsval');
    if (d.gps_present && d.gps_fix) {
      gEl.textContent = 'Fix \u00B7 '+d.gps_sats+' sats';
      gEl.className = 'val green';
    } else if (d.gps_present) {
      gEl.textContent = 'No fix';  gEl.className = 'val amber';
    } else {
      gEl.textContent = 'Not fitted';  gEl.className = 'val';
    }
    if (d.flights && d.flights.length > 0) {
      var tb = document.getElementById('ftbody');
      d.flights.forEach(function(f) {
        var tr = document.createElement('tr');
        tr.innerHTML = '<td>'+f.num+'</td><td>'+f.dur.toFixed(1)+'</td>'
          +'<td>'+f.throw_ft.toFixed(0)+'</td><td>'+f.peak_ft.toFixed(0)+'</td>'
          +'<td>'+f.score.toFixed(1)+'</td>';
        tb.appendChild(tr);
      });
      document.getElementById('ftable').style.display = 'table';
      document.getElementById('no-fl').style.display  = 'none';
    }
  }
  function closeWin() {
    if (!confirm('Close the window now?\nThis ends timing and saves the log.')) return;
    fetch('/pstop').then(function(){ location.replace('/pilot'); });
  }
</script>
</body>
</html>)rawhtml";


// ============================================================
const char PAGE_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>F3K Flight Unit</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', Arial, sans-serif;
      background: #0d1117; color: #e6edf3;
      display: flex; flex-direction: column; align-items: center;
      padding: 1.5rem 1rem; min-height: 100vh;
    }
    h1 { font-size: 1.5rem; font-weight: 600; color: #58a6ff; margin-bottom: 0.2rem; }
    .subtitle { font-size: 0.8rem; color: #8b949e; margin-bottom: 1rem; }
    .section-label {
      width: 100%; max-width: 750px;
      font-size: 0.7rem; text-transform: uppercase; letter-spacing: 0.1em;
      color: #484f58; margin: 0.8rem 0 0.4rem;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(148px, 1fr));
      gap: 0.7rem; width: 100%; max-width: 750px;
    }
    .card {
      background: #161b22; border: 1px solid #30363d;
      border-radius: 10px; padding: 0.9rem 1.1rem;
    }
    .card .label {
      font-size: 0.68rem; text-transform: uppercase;
      letter-spacing: 0.08em; color: #8b949e; margin-bottom: 0.25rem;
    }
    .card .value { font-size: 1.7rem; font-weight: 700; color: #f0f6fc; line-height: 1.1; }
    .card .unit  { font-size: 0.8rem; color: #8b949e; margin-top: 0.1rem; }
    .card.blue  { border-color: #58a6ff; } .card.blue  .value { color: #58a6ff; }
    .card.green { border-color: #3fb950; } .card.green .value { color: #3fb950; }
    .card.amber { border-color: #d29922; } .card.amber .value { color: #d29922; }
    .card.red   { border-color: #f85149; } .card.red   .value { color: #f85149; }
    .card.dim   .value { font-size: 1.1rem; color: #8b949e; }
    .state-badge {
      display: inline-block; padding: 0.3rem 0.9rem;
      border-radius: 20px; font-size: 0.85rem; font-weight: 700;
      letter-spacing: 0.05em; margin-bottom: 1rem;
      background: #161b22; border: 1px solid #30363d; color: #8b949e;
    }
    .state-badge.CALIBRATING { border-color:#d29922; color:#d29922; }
    .state-badge.GROUND      { border-color:#8b949e; color:#8b949e; }
    .state-badge.LAUNCH_WIN  { border-color:#d29922; color:#d29922; }
    .state-badge.FLIGHT      { border-color:#3fb950; color:#3fb950; }
    .state-badge.LANDED      { border-color:#58a6ff; color:#58a6ff; }
    .chart-wrap {
      width: 100%; max-width: 750px;
      background: #161b22; border: 1px solid #30363d;
      border-radius: 10px; padding: 1rem; margin-top: 0.8rem;
    }
    .chart-title {
      font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.08em;
      color: #8b949e; margin-bottom: 0.5rem;
      display: flex; justify-content: space-between;
    }
    .chart-title span { color: #484f58; font-size: 0.68rem; }
    canvas { width: 100% !important; display: block; }
    .btn-row { display: flex; gap: 0.8rem; margin-top: 0.8rem; flex-wrap: wrap; }
    button {
      padding: 0.55rem 1.3rem; border-radius: 8px; border: none;
      font-size: 0.88rem; font-weight: 600; cursor: pointer; transition: opacity 0.15s;
    }
    button:hover { opacity: 0.82; }
    #btn-tare  { background: #238636; color: #fff; }
    #btn-clear { background: #21262d; color: #8b949e; border: 1px solid #30363d; }
    .status { font-size: 0.7rem; color: #484f58; margin-top: 0.8rem; }
    #dot {
      display: inline-block; width: 8px; height: 8px;
      border-radius: 50%; background: #3fb950;
      margin-right: 5px; vertical-align: middle;
    }
  </style>
</head>
<body>
  <h1>F3K Flight Unit <span id="unit-id" style="color:#8b949e;font-size:1rem"></span></h1>
  <p class="subtitle">QT Py ESP32-S3 &nbsp;|&nbsp; 8 Hz baro · 26 Hz IMU · 5 Hz display · UDP 5 Hz</p>

  <div class="state-badge GROUND" id="state-badge">--</div>

  <div class="section-label">Altitude</div>
  <div class="grid">
    <div class="card blue">
      <div class="label">Absolute</div>
      <div class="value" id="alt-ft">--</div>
      <div class="unit">feet · mean</div>
    </div>
    <div class="card green">
      <div class="label">Tared</div>
      <div class="value" id="alt-tared">--</div>
      <div class="unit">feet from baseline</div>
    </div>
    <div class="card dim">
      <div class="label">Peak</div>
      <div class="value" id="peak-alt">--</div>
      <div class="unit">feet this flight</div>
    </div>
    <div class="card dim">
      <div class="label">Launch height</div>
      <div class="value" id="launch-ht">--</div>
      <div class="unit">feet</div>
    </div>
  </div>

  <div class="section-label">Flight</div>
  <div class="grid">
    <div class="card">
      <div class="label">Duration</div>
      <div class="value" id="duration">--</div>
      <div class="unit">seconds</div>
    </div>
    <div class="card dim">
      <div class="label">Noise p-p</div>
      <div class="value" id="noise">--</div>
      <div class="unit">feet this batch</div>
    </div>
    <div class="card">
      <div class="label">Pressure</div>
      <div class="value" id="pressure">--</div>
      <div class="unit">hPa</div>
    </div>
    <div class="card">
      <div class="label">Temperature</div>
      <div class="value" id="temp">--</div>
      <div class="unit">&deg;C</div>
    </div>
  </div>

  <div class="section-label">Diagnostics</div>
  <div class="grid">
    <div class="card amber" id="card-rssi">
      <div class="label">WiFi RSSI</div>
      <div class="value" id="rssi">--</div>
      <div class="unit" id="rssi-qual">dBm</div>
    </div>
    <div class="card dim">
      <div class="label">CPU Load</div>
      <div class="value" id="cpu">--</div>
      <div class="unit">% loop busy</div>
    </div>
    <div class="card dim">
      <div class="label">Loop avg / max</div>
      <div class="value" id="loop-avg">--</div>
      <div class="unit" id="loop-max">max -- µs</div>
    </div>
    <div class="card dim">
      <div class="label">Free Heap</div>
      <div class="value" id="heap">--</div>
      <div class="unit">kB</div>
    </div>
  </div>

  <div class="section-label">IMU — LSM6DSO32 <span id="imu-status" style="color:#f85149;font-size:0.65rem;margin-left:0.4rem">NOT FOUND</span></div>
  <div class="grid">
    <div class="card blue">
      <div class="label">G-Force</div>
      <div class="value" id="g-force">--</div>
      <div class="unit">G total</div>
    </div>
    <div class="card amber">
      <div class="label">Tilt</div>
      <div class="value" id="tilt">--</div>
      <div class="unit">degrees from vertical</div>
    </div>
    <div class="card dim">
      <div class="label">Accel X / Y / Z</div>
      <div class="value" id="accel-x">--</div>
      <div class="unit" id="accel-yz">Y:-- Z:-- m/s²</div>
    </div>
    <div class="card dim">
      <div class="label">Gyro X / Y / Z</div>
      <div class="value" id="gyro-x">--</div>
      <div class="unit" id="gyro-yz">Y:-- Z:-- rad/s</div>
    </div>
  </div>

  <div class="btn-row">
    <button id="btn-tare"  onclick="tare()">&#8853; Tare Altitude</button>
    <button id="btn-clear" onclick="clearGraph()">Clear Graph</button>
  </div>

  <div class="chart-wrap">
    <div class="chart-title">
      Altitude — tared (feet) · all 8 Hz samples
      <span id="point-count">0 pts</span>
    </div>
    <canvas id="chart" height="200"></canvas>
  </div>

  <div class="chart-wrap">
    <div class="chart-title">
      Accelerometer — X / Y / Z (G)
      <span id="accel-point-count">0 pts</span>
    </div>
    <canvas id="chart-accel" height="200"></canvas>
  </div>

  <div class="chart-wrap">
    <div class="chart-title">
      Gyroscope — X / Y / Z (deg/s)
      <span id="gyro-point-count">0 pts</span>
    </div>
    <canvas id="chart-gyro" height="200"></canvas>
  </div>

  <p class="status"><span id="dot"></span><span id="status-txt">Connecting...</span></p>

<script>
const MAX_POINTS = 480;   // ~60s at 8 Hz for altitude
const IMU_MAX    = 1560;  // ~60s at 26 Hz for IMU

// Altitude history (8 Hz, tared ft)
const history = [];

// IMU histories — one array per axis, all in display units
const accelHist = { x:[], y:[], z:[] };  // G
const gyroHist  = { x:[], y:[], z:[] };  // deg/s

// ── Canvas refs ───────────────────────────────────────────────
const canvas      = document.getElementById('chart');
const ctx         = canvas.getContext('2d');
const canvasAccel = document.getElementById('chart-accel');
const ctxAccel    = canvasAccel.getContext('2d');
const canvasGyro  = document.getElementById('chart-gyro');
const ctxGyro     = canvasGyro.getContext('2d');

function rssiQuality(dbm) {
  if (dbm >= -60) return { label:'Good',  cls:'green' };
  if (dbm >= -70) return { label:'Fair',  cls:'amber' };
  return              { label:'Poor',  cls:'red'   };
}

// ── Generic multi-series chart ────────────────────────────────
// series: [{data:[], color:'#hex', label:'X'}, ...]
// ptCountId: element id to update with point count
function drawMultiChart(cvs, context, series, ptCountId, yDecimals=2) {
  const W = cvs.offsetWidth, H = cvs.height;
  cvs.width = W;
  context.clearRect(0, 0, W, H);

  const allPts = series.flatMap(s => s.data);
  if (allPts.length < 2) {
    context.fillStyle='#484f58'; context.font='13px Segoe UI';
    context.textAlign='center';
    context.fillText('Waiting for data...', W/2, H/2); return;
  }

  let yMin = Math.min(...allPts), yMax = Math.max(...allPts);
  const yPad = Math.max(0.1, (yMax - yMin) * 0.2);
  yMin -= yPad; yMax += yPad;
  if (yMax - yMin < 0.2) { yMin -= 0.1; yMax += 0.1; }

  const pad = { top:10, right:10, bottom:28, left:52 };
  const cW = W - pad.left - pad.right;
  const cH = H - pad.top  - pad.bottom;
  const maxLen = Math.max(...series.map(s => s.data.length));
  const toX = i => pad.left + (i / (maxLen - 1)) * cW;
  const toY = v => pad.top  + (1 - (v - yMin) / (yMax - yMin)) * cH;

  // Grid lines
  for (let i = 0; i <= 4; i++) {
    const v = yMin + (yMax - yMin) * (i / 4), y = toY(v);
    context.strokeStyle='#21262d'; context.lineWidth=1;
    context.beginPath(); context.moveTo(pad.left,y); context.lineTo(pad.left+cW,y); context.stroke();
    context.fillStyle='#8b949e'; context.font='10px monospace'; context.textAlign='right';
    context.fillText(v.toFixed(yDecimals), pad.left-4, y+3);
  }

  // Zero line
  if (yMin < 0 && yMax > 0) {
    const y0 = toY(0);
    context.strokeStyle='#ffffff22'; context.lineWidth=1; context.setLineDash([4,4]);
    context.beginPath(); context.moveTo(pad.left,y0); context.lineTo(pad.left+cW,y0); context.stroke();
    context.setLineDash([]);
  }

  // Time axis
  context.fillStyle='#8b949e'; context.font='10px monospace'; context.textAlign='center';
  const totalSec = maxLen / (series[0].hz || 26);
  for (let i = 0; i <= 4; i++) {
    const x = pad.left + (i / 4) * cW;
    const sec = (totalSec * (1 - i / 4)).toFixed(0);
    context.fillText(sec > 0 ? '-'+sec+'s' : 'now', x, H-6);
  }

  // Legend — top right
  let lx = pad.left + cW;
  series.forEach(s => {
    const lw = context.measureText(s.label).width + 18;
    lx -= lw + 6;
    context.fillStyle = s.color;
    context.fillRect(lx, pad.top + 2, 12, 3);
    context.fillStyle = '#8b949e'; context.font='10px monospace'; context.textAlign='left';
    context.fillText(s.label, lx + 14, pad.top + 8);
  });

  // Draw each series
  series.forEach(s => {
    if (s.data.length < 2) return;
    const n = s.data.length;
    const xOff = pad.left + (maxLen - n) / (maxLen - 1) * cW;
    const toXs = i => pad.left + ((i + maxLen - n) / (maxLen - 1)) * cW;
    context.strokeStyle = s.color; context.lineWidth = 1.5; context.lineJoin='round';
    context.beginPath();
    s.data.forEach((v, i) => i === 0 ? context.moveTo(toXs(i), toY(v)) : context.lineTo(toXs(i), toY(v)));
    context.stroke();
  });

  if (ptCountId) document.getElementById(ptCountId).textContent = maxLen + ' pts';
}

// ── Single-series altitude chart (keeps dots + fill) ─────────
function drawChart() {
  const W = canvas.offsetWidth, H = canvas.height;
  canvas.width = W; ctx.clearRect(0, 0, W, H);
  if (history.length < 2) {
    ctx.fillStyle='#484f58'; ctx.font='13px Segoe UI'; ctx.textAlign='center';
    ctx.fillText('Waiting for data...', W/2, H/2); return;
  }
  let yMin=Math.min(...history), yMax=Math.max(...history);
  const yPad=Math.max(0.5,(yMax-yMin)*0.25);
  yMin-=yPad; yMax+=yPad;
  if(yMax-yMin<1){yMin-=0.5;yMax+=0.5;}
  const pad={top:10,right:10,bottom:28,left:52};
  const cW=W-pad.left-pad.right, cH=H-pad.top-pad.bottom;
  const toX=i=>pad.left+(i/(history.length-1))*cW;
  const toY=v=>pad.top+(1-(v-yMin)/(yMax-yMin))*cH;
  for(let i=0;i<=4;i++){
    const v=yMin+(yMax-yMin)*(i/4),y=toY(v);
    ctx.strokeStyle='#21262d';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(pad.left,y);ctx.lineTo(pad.left+cW,y);ctx.stroke();
    ctx.fillStyle='#8b949e';ctx.font='10px monospace';ctx.textAlign='right';
    ctx.fillText(v.toFixed(2),pad.left-4,y+3);
  }
  ctx.strokeStyle='#30363d';ctx.lineWidth=1;ctx.setLineDash([2,4]);
  for(let i=8;i<history.length;i+=8){
    const x=toX(i);
    ctx.beginPath();ctx.moveTo(x,pad.top);ctx.lineTo(x,pad.top+cH);ctx.stroke();
  }
  ctx.setLineDash([]);
  if(yMin<0&&yMax>0){
    const y0=toY(0);
    ctx.strokeStyle='#3fb95066';ctx.lineWidth=1;ctx.setLineDash([4,4]);
    ctx.beginPath();ctx.moveTo(pad.left,y0);ctx.lineTo(pad.left+cW,y0);ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.fillStyle='#8b949e';ctx.font='10px monospace';ctx.textAlign='center';
  const totalSec=history.length/8;
  for(let i=0;i<=4;i++){
    const x=pad.left+(i/4)*cW;
    const sec=(totalSec*(1-i/4)).toFixed(0);
    ctx.fillText(sec>0?'-'+sec+'s':'now',x,H-6);
  }
  ctx.strokeStyle='#58a6ff';ctx.lineWidth=1.5;ctx.lineJoin='round';
  ctx.beginPath();
  history.forEach((v,i)=>i===0?ctx.moveTo(toX(i),toY(v)):ctx.lineTo(toX(i),toY(v)));
  ctx.stroke();
  ctx.fillStyle='#58a6ffaa';
  history.forEach((v,i)=>{ctx.beginPath();ctx.arc(toX(i),toY(v),2,0,Math.PI*2);ctx.fill();});
  ctx.beginPath();
  history.forEach((v,i)=>i===0?ctx.moveTo(toX(i),toY(v)):ctx.lineTo(toX(i),toY(v)));
  ctx.lineTo(toX(history.length-1),pad.top+cH);
  ctx.lineTo(pad.left,pad.top+cH);
  ctx.closePath();ctx.fillStyle='#58a6ff12';ctx.fill();
  document.getElementById('point-count').textContent=history.length+' pts';
}

function drawAllCharts() {
  drawChart();
  drawMultiChart(canvasAccel, ctxAccel, [
    { data: accelHist.x, color:'#58a6ff', label:'X', hz:26 },
    { data: accelHist.y, color:'#3fb950', label:'Y', hz:26 },
    { data: accelHist.z, color:'#f85149', label:'Z', hz:26 },
  ], 'accel-point-count', 3);
  drawMultiChart(canvasGyro, ctxGyro, [
    { data: gyroHist.x,  color:'#58a6ff', label:'X', hz:26 },
    { data: gyroHist.y,  color:'#3fb950', label:'Y', hz:26 },
    { data: gyroHist.z,  color:'#f85149', label:'Z', hz:26 },
  ], 'gyro-point-count', 1);
}

function poll(){
  const t0=performance.now();
  fetch('/json').then(r=>r.json()).then(d=>{
    if(!d.ready)return;
    d.alt_tared.forEach(v=>history.push(v));
    while(history.length>MAX_POINTS)history.shift();

    document.getElementById('unit-id').textContent   = '#'+d.unit_id;
    const badge=document.getElementById('state-badge');
    badge.textContent=d.state_name;
    badge.className='state-badge '+d.state_name;

    document.getElementById('alt-ft').textContent    = d.mean_alt_ft.toFixed(1);
    document.getElementById('alt-tared').textContent = d.mean_alt_tared_ft.toFixed(1);
    document.getElementById('peak-alt').textContent  = d.peak_alt_ft.toFixed(1);
    document.getElementById('launch-ht').textContent = d.launch_height_ft.toFixed(1);
    document.getElementById('duration').textContent  = d.flight_duration_s.toFixed(1);

    const bMin=Math.min(...d.alt_tared),bMax=Math.max(...d.alt_tared);
    document.getElementById('noise').textContent=(bMax-bMin).toFixed(2);
    document.getElementById('pressure').textContent=d.pressure[d.pressure.length-1].toFixed(2);
    document.getElementById('temp').textContent=d.temp[d.temp.length-1].toFixed(2);

    document.getElementById('rssi').textContent     = d.rssi_dbm;
    document.getElementById('cpu').textContent      = d.cpu_load_pct.toFixed(1)+'%';
    document.getElementById('loop-avg').textContent = Math.round(d.loop_avg_us)+' µs';
    document.getElementById('loop-max').textContent = 'max '+Math.round(d.loop_max_us)+' µs';
    document.getElementById('heap').textContent     = (d.free_heap_b/1024).toFixed(1)+' kB';

    const q=rssiQuality(d.rssi_dbm);
    document.getElementById('rssi-qual').textContent='dBm · '+q.label;
    document.getElementById('card-rssi').className='card '+q.cls;

    // IMU
    const imuStatus = document.getElementById('imu-status');
    if (d.imu_present) {
      imuStatus.textContent = '26 Hz';
      imuStatus.style.color = '#3fb950';

      const RAD_TO_DEG = 180 / Math.PI;
      document.getElementById('g-force').textContent  = d.g_force.toFixed(2);
      document.getElementById('tilt').textContent     = d.tilt_deg.toFixed(1);
      document.getElementById('accel-x').textContent  = (d.accel_x / 9.80665).toFixed(3);
      document.getElementById('accel-yz').textContent =
        'Y:'+(d.accel_y/9.80665).toFixed(3)+' Z:'+(d.accel_z/9.80665).toFixed(3)+' G';
      document.getElementById('gyro-x').textContent   = (d.gyro_x * RAD_TO_DEG).toFixed(1);
      document.getElementById('gyro-yz').textContent  =
        'Y:'+(d.gyro_y*RAD_TO_DEG).toFixed(1)+' Z:'+(d.gyro_z*RAD_TO_DEG).toFixed(1)+' °/s';

      // Push to IMU histories (display units)
      accelHist.x.push(d.accel_x / 9.80665);
      accelHist.y.push(d.accel_y / 9.80665);
      accelHist.z.push(d.accel_z / 9.80665);
      gyroHist.x.push(d.gyro_x * RAD_TO_DEG);
      gyroHist.y.push(d.gyro_y * RAD_TO_DEG);
      gyroHist.z.push(d.gyro_z * RAD_TO_DEG);
      // Trim to max length
      ['x','y','z'].forEach(ax => {
        while (accelHist[ax].length > IMU_MAX) accelHist[ax].shift();
        while (gyroHist[ax].length  > IMU_MAX) gyroHist[ax].shift();
      });
    } else {
      imuStatus.textContent = 'NOT FOUND';
      imuStatus.style.color = '#f85149';
    }

    const rtt=Math.round(performance.now()-t0);
    document.getElementById('status-txt').textContent=
      'Live · RTT '+rtt+'ms · '+new Date().toLocaleTimeString();
    document.getElementById('dot').style.background='#3fb950';
    drawAllCharts();
  }).catch(()=>{
    document.getElementById('status-txt').textContent='Connection lost — retrying...';
    document.getElementById('dot').style.background='#f85149';
  });
}

function tare() {
  fetch('/tare').then(()=>{
    history.length=0;
    ['x','y','z'].forEach(ax=>{ accelHist[ax].length=0; gyroHist[ax].length=0; });
    drawAllCharts();
  });
}
function clearGraph() {
  history.length=0;
  ['x','y','z'].forEach(ax=>{ accelHist[ax].length=0; gyroHist[ax].length=0; });
  drawAllCharts();
}

window.addEventListener('resize', drawAllCharts);
setInterval(poll, 200);  // 5 Hz — matches device display rate
poll();
</script>
</body>
</html>)rawhtml";