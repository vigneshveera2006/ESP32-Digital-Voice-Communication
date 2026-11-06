// =====================================================================
//  dashboard_page.h  —  The phone dashboard served by the receiver.
//  Self-contained (no internet needed): the page draws its own graphs and
//  fetches /data (JSON) twice a second.
// =====================================================================
#pragma once
static const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Live Voice Receiver</title>
<style>
:root{--bg:#f4f6f8;--card:#fff;--ink:#14202b;--quiet:#5c6b78;--line:#d8dee4;--acc:#1f6fb2;--orig:#9aa7b3;--good:#1e8a4c;--warn:#b26b00;--bad:#c0392b}
@media (prefers-color-scheme:dark){:root{--bg:#0f151b;--card:#18212a;--ink:#e6edf3;--quiet:#93a1ad;--line:#2b3742;--acc:#5aa9e6;--orig:#5f6d79;--good:#3fbf74;--warn:#e0a03a;--bad:#ef6a5a}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:640px;margin:0 auto;padding:12px 16px 28px}
h1{font-size:18px;margin:4px 0 10px;display:flex;align-items:center;gap:10px}
.pill{font-size:12px;font-weight:600;padding:3px 9px;border-radius:99px;color:#fff;background:var(--quiet)}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:12px}
.k{font-size:12px;color:var(--quiet)}.big{font-size:20px;font-weight:650}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.grid div{min-width:0}.v{font-size:17px;font-weight:600;font-variant-numeric:tabular-nums}
canvas{width:100%;display:block}
.leg{font-size:12px;color:var(--quiet);margin-top:4px}.leg b{display:inline-block;width:14px;height:3px;vertical-align:middle;margin:0 4px 0 10px}
.hex{font:12px ui-monospace,Consolas,monospace;color:var(--quiet);word-break:break-all;margin-top:6px}
.row{display:flex;gap:8px;align-items:center}button{flex:1;padding:11px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--ink);font-size:15px}
.note{font-size:12px;color:var(--quiet);margin-top:6px}
</style></head><body><main>
<h1>Live Voice Receiver <span id="st" class="pill">…</span></h1>

<div class="card">
 <div class="k">Encoding technique</div>
 <div class="big" id="tech">–</div>
 <div class="k" id="rate" style="margin-top:4px">–</div>
</div>

<div class="card grid">
 <div><div class="k">Packets received</div><div class="v" id="rx">–</div></div>
 <div><div class="k">Packets lost</div><div class="v" id="lost">–</div></div>
 <div><div class="k">CRC errors</div><div class="v" id="crc">–</div></div>
 <div><div class="k">SNR (last 5 s)</div><div class="v" id="snr">–</div></div>
 <div><div class="k">MSE (last 5 s)</div><div class="v" id="mse">–</div></div>
 <div><div class="k">Decoder state check</div><div class="v" id="sc">–</div></div>
</div>

<div class="card">
 <div class="k">Original vs reconstructed — one 20 ms frame (<span id="wseq">–</span>)</div>
 <canvas id="wave" height="170"></canvas>
 <div class="leg"><b style="background:var(--orig)"></b>original (encoder input)<b style="background:var(--acc)"></b>reconstructed (decoded here)<b style="background:var(--bad)"></b>error</div>
</div>

<div class="card">
 <div class="k">Encoded data — start of the latest packet</div>
 <canvas id="bits" height="90"></canvas>
 <div class="hex" id="hex">–</div>
</div>

<div class="card">
 <div class="k">SNR history (dB)</div>
 <canvas id="hist" height="70"></canvas>
</div>

<div class="card">
 <div class="k">Reconstructed audio recording</div>
 <div id="recInfo" style="margin-top:2px">No recording yet — turn S3 ON at the transmitter and speak.</div>
 <div id="recPlayer" style="display:none;margin-top:8px">
  <audio id="recAudio" controls preload="none" style="width:100%"></audio>
  <div style="margin-top:6px"><a id="recDl" href="#" style="color:var(--acc)">Download WAV</a></div>
 </div>
 <div class="note">Decoded on the receiver from the received bits, with the same decoder as the speaker. Lost packets appear as short silences. A new transmission replaces it.</div>
</div>

<div class="card">
 <div class="k">Speaker volume <span id="vol">–</span></div>
 <div class="row" style="margin-top:8px"><button onclick="cmd('down')">Vol −</button><button onclick="cmd('up')">Vol +</button><button onclick="cmd('tone')">Test tone</button></div>
 <div class="note">Audio plays on the receiver's speaker. This page only shows measurements.</div>
</div>
<div class="note" id="upd">–</div>
</main><script>
const $=id=>document.getElementById(id), css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let hist=[], lastSeqForHist=-1;
function fit(c){const r=window.devicePixelRatio||1;if(!c.dataset.h){c.dataset.h=c.getAttribute('height');c.style.height=c.dataset.h+'px';}
 const w=c.clientWidth,h=+c.dataset.h,W=Math.round(w*r),H=Math.round(h*r);if(c.width!==W||c.height!==H){c.width=W;c.height=H;}
 const x=c.getContext('2d');x.setTransform(r,0,0,r,0,0);x.clearRect(0,0,w,h);return [x,w,h];}
function line(x,arr,w,h,col,lw){if(!arr||!arr.length)return;x.strokeStyle=col;x.lineWidth=lw;x.beginPath();arr.forEach((v,i)=>{const px=i*(w-1)/(arr.length-1),py=h/2-v/32768*(h/2-4);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();}
function drawWave(d){const[x,w,h]=fit($('wave'));x.strokeStyle=css('--line');x.lineWidth=1;x.beginPath();x.moveTo(0,h/2);x.lineTo(w,h/2);x.stroke();
 if(!d.orig||!d.orig.length){x.fillStyle=css('--quiet');x.font='13px system-ui';x.fillText('Waiting for a reference frame…',8,h/2-8);return;}
 const err=d.orig.map((v,i)=>v-d.recon[i]);line(x,d.orig,w,h,css('--orig'),2);line(x,d.recon,w,h,css('--acc'),1.5);line(x,err,w,h,css('--bad'),1);}
function drawBits(d){const[x,w,h]=fit($('bits'));const c=d.codes||[];if(!c.length)return;const top=8,bot=h-18,max=(1<<d.bits)-1;
 x.fillStyle=css('--quiet');x.font='11px system-ui';
 if(d.bits==1){const s=w/c.length;x.strokeStyle=css('--acc');x.lineWidth=2;x.beginPath();c.forEach((b,i)=>{const y=b?top:bot;i?x.lineTo(i*s,y):x.moveTo(0,y);x.lineTo((i+1)*s,y)});x.stroke();
  x.fillText('1',0,top+4);x.fillText('0',0,bot);x.fillText(c.length+' bits (1 bit per code)',w-150,h-2);}
 else{const s=w/c.length;x.fillStyle=css('--acc');c.forEach((v,i)=>{const bh=(bot-top)*v/max;x.fillRect(i*s+1,bot-bh,Math.max(1,s-2),bh)});
  x.fillStyle=css('--quiet');x.fillText(c.length+' codes, '+d.bits+' bits each (0–'+max+')',w-190,h-2);}}
function drawHist(){const[x,w,h]=fit($('hist'));if(hist.length<2)return;const lo=Math.min(0,...hist),hi=Math.max(35,...hist);
 x.strokeStyle=css('--acc');x.lineWidth=2;x.beginPath();hist.forEach((v,i)=>{const px=i*(w-1)/(hist.length-1),py=h-4-(v-lo)/(hi-lo)*(h-8);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
 x.fillStyle=css('--quiet');x.font='11px system-ui';x.fillText(hi.toFixed(0)+' dB',2,11);x.fillText(lo.toFixed(0)+' dB',2,h-2);}
let shownRec=-1;
function showRec(d){const info=$('recInfo'),pl=$('recPlayer');
 if(!d.recSec){info.textContent='No recording yet — turn S3 ON at the transmitter and speak.';pl.style.display='none';shownRec=-1;return;}
 const full=d.recFull?' — memory full, the first '+d.recSec.toFixed(1)+' s were kept':'';
 if(d.recLive){info.textContent='Recording… '+d.recSec.toFixed(1)+' s of '+d.recName+' (up to '+d.recMaxSec+' s). Turn S3 OFF to listen.'+full;pl.style.display='none';shownRec=-1;return;}
 info.textContent='Recorded '+d.recSec.toFixed(1)+' s of '+d.recName+full;
 if(d.recId!==shownRec){shownRec=d.recId;const a=$('recAudio');a.src='/rec.wav?id='+d.recId;$('recDl').href='/rec.wav?id='+d.recId+'&dl=1';}
 pl.style.display='block';}
function setPill(s){const p=$('st');p.textContent=s;p.style.background=s=='LIVE'?css('--good'):(s=='STOPPED'?css('--quiet'):css('--bad'));}
async function poll(){try{const d=await (await fetch('/data',{cache:'no-store'})).json();
 setPill(d.status);
 $('tech').textContent=d.codecName? d.codecName+' · '+d.bits+' bit'+(d.bits>1?'s':'')+' per code'+(d.osr>1?' · '+d.osr+'× oversampled':''):'–';
 $('rate').textContent=d.codecName? 'Audio bitrate '+d.kbps+' kbit/s · '+d.pps+' packets/s · '+d.payload+'-byte payload + 24-byte header':'–';
 $('rx').textContent=d.rx.toLocaleString();$('lost').textContent=d.lost+' ('+d.lossPct.toFixed(2)+'%)';$('crc').textContent=d.crc;
 $('snr').textContent=d.snrValid? d.snr.toFixed(1)+' dB':'–';$('mse').textContent=d.snrValid? d.mse.toExponential(2):'–';
 $('sc').textContent=d.stateMismatch==0?'OK (0)':d.stateMismatch+' mismatches';$('vol').textContent='('+d.volume+')';
 $('wseq').textContent=d.waveSeq>=0?'frame #'+d.waveSeq+', SNR '+d.frameSnr.toFixed(1)+' dB':'–';
 $('hex').textContent=d.hex||'–';
 drawWave(d);drawBits(d);
 if(d.snrValid&&d.waveSeq!==lastSeqForHist){hist.push(d.snr);if(hist.length>60)hist.shift();lastSeqForHist=d.waveSeq;}drawHist();
 showRec(d);
 $('upd').textContent='Updated '+new Date().toLocaleTimeString()+' · receiver up '+Math.floor(d.uptime/60)+' min';
}catch(e){setPill('OFFLINE');}}
async function cmd(c){try{await fetch('/cmd?c='+c,{cache:'no-store'});}catch(e){}}
async function loop(){await poll();setTimeout(loop,500);}loop();window.addEventListener('resize',()=>{drawHist();});
</script></body></html>)HTML";
