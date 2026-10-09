import {normalize,fromEuler} from './math.js';
import {DroneRenderer} from './renderer.js';

const $=id=>document.getElementById(id),DEG=180/Math.PI;
let renderer;
try { renderer=new DroneRenderer($('drone')); }
catch(error){$('scene-wait').innerHTML='<strong>3D görüntü kullanılamıyor</strong><small>WebGL destekli bir tarayıcı aç. Sensör değerleri yine gösterilir.</small>';console.error(error);}
let demo=false,paused=false,connected=false,latest=null,lastSequence=null,lastTimestamp=null,receivedAt=0;
let history=[],recording=[],count=0,generation=0;
const signed=(v,digits=2)=>(v>=0?'+':'')+v.toFixed(digits);
const colors=['#3e9d8b','#6382cf','#c69852'];

// Kart başarısız/gecikmiş ölçümü valid=false gönderir. Tarayıcı da biçimi doğrular.
export function validSample(s) {
  return s && s.valid===true && Number.isFinite(s.age_ms) && s.age_ms>=0 && s.age_ms<=500 &&
    Number.isSafeInteger(s.sequence) && Number.isSafeInteger(s.timestamp_us) &&
    ['angles_rad','accel_g','gyro_rad_s'].every(k=>Array.isArray(s[k])&&s[k].length===3&&s[k].every(Number.isFinite))&&
    Number.isFinite(s.temperature_c) && normalize(s.quaternion)!==null;
}
function status(text,state='waiting') {
  $('connection').textContent=text;$('status-dot').className='status-dot '+state;
}
function emptyValues(){
  for(const id of ['roll','pitch','yaw','ax','ay','az','gx','gy','gz'])$(id).textContent='—';
  for(const id of ['roll','pitch','yaw'])$(id+'-bar').style.left='50%';
  $('temperature').textContent='— °C';$('norm').textContent='— g';
  $('calibration').textContent='Ölçüm bekleniyor';$('calibration').className='calibration';
}
function waiting(text){
  connected=false;status(text);$('view-state').textContent=text;$('sample-age').textContent='— ms';
  if(renderer){$('scene-wait').hidden=false;$('scene-wait').querySelector('strong').textContent=text;}
  emptyValues();
}
function resetSession(){history=[];recording=[];count=0;lastSequence=null;lastTimestamp=null;latest=null;renderer?.setQuaternion([1,0,0,0]);$('export').disabled=true;$('sample-count').textContent='0 geçerli örnek';drawChart();}
function showSample(s){
  const q=normalize(s.quaternion);
  renderer?.setQuaternion(q);
  const angles=s.angles_rad.map(v=>v*DEG);
  ['roll','pitch','yaw'].forEach((id,i)=>{
    $(id).textContent=signed(angles[i]);
    $(id+'-bar').style.left=(50+Math.max(-1,Math.min(1,angles[i]/180))*45)+'%';
  });
  ['ax','ay','az'].forEach((id,i)=>$(id).textContent=signed(s.accel_g[i],4));
  ['gx','gy','gz'].forEach((id,i)=>$(id).textContent=signed(s.gyro_rad_s[i],4));
  $('norm').textContent=Math.hypot(...s.accel_g).toFixed(4)+' g';
  $('temperature').textContent=s.temperature_c.toFixed(1)+' °C';
  const calibrated=s.accel_calibrated&&s.gyro_calibrated;
  $('calibration').textContent=calibrated?(s.gyro_temperature_valid?'● Kalibrasyon hazır':'● Sıcaklık farkı yüksek'):'● Kalibrasyon eksik';
  $('calibration').className='calibration '+(calibrated&&s.gyro_temperature_valid?'good':'warning');
}
function accept(s){
  receivedAt=performance.now();
  if(!validSample(s)){waiting(s?.age_ms>500?'Ölçüm güncel değil':'Geçerli ölçüm bekleniyor');return;}
  // Kart yeniden başladığında önceki oturumu yeni referansla karıştırma.
  if(!demo&&lastTimestamp!==null&&s.timestamp_us<lastTimestamp)resetSession();
  latest=s;connected=true;
  status(demo?'Demo · simülasyon':'Sensör bağlı',demo?'demo':'live');
  $('view-state').textContent=paused?'Görünüm donduruldu':demo?'Simülasyon görüntüsü':'Canlı yönelim';
  $('sample-age').textContent=Math.round(s.age_ms)+' ms';
  if(renderer)$('scene-wait').hidden=true;
  if(!paused)showSample(s);
  // Aynı ölçümü her HTTP yanıtında yeniden kaydetme.
  if(s.sequence===lastSequence)return;
  lastSequence=s.sequence;lastTimestamp=s.timestamp_us;
  if(paused)return;
  const point={t:performance.now(),a:s.angles_rad.map(v=>v*DEG)};
  history.push(point);history=history.filter(p=>point.t-p.t<=12000);
  if(history.length>300)history.shift();
  recording.push([demo?'simulation':'sensor',s.timestamp_us,...point.a,...s.quaternion,...s.accel_g,...s.gyro_rad_s,s.temperature_c]);
  if(recording.length>3600)recording.shift();
  count++;$('sample-count').textContent=count.toLocaleString('tr-TR')+' geçerli örnek';$('export').disabled=false;
}
async function poll(){
  if(demo){setTimeout(poll,100);return;}
  const current=generation,controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2000),started=performance.now();
  try {
    const response=await fetch('/api/telemetry',{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw new Error('HTTP '+response.status);
    const sample=await response.json();if(current===generation&&!demo)accept(sample);
  } catch(error){if(current===generation&&!demo)waiting('Kart bağlantısı kesildi');}
  finally{clearTimeout(timer);setTimeout(poll,Math.max(0,(connected?50:750)-(performance.now()-started)));}
}
// Ağın sessizce takılması halinde eski örneği canlı olarak göstermeyiz.
setInterval(()=>{
  if(!demo&&connected&&performance.now()-receivedAt>650)waiting('Veri akışı durdu');
  if(demo){
    const t=performance.now()/1000,r=.3*Math.sin(t*.8),p=.38*Math.sin(t*.55),y=.7*Math.sin(t*.3),q=fromEuler(r,p,y);
    accept({valid:true,sequence:Math.floor(performance.now()/50),timestamp_us:Math.floor(performance.now()*1000),age_ms:0,
      quaternion:q,angles_rad:[r,p,y],accel_g:[2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2)],
      gyro_rad_s:[.24*Math.cos(t*.8),.209*Math.cos(t*.55),.21*Math.cos(t*.3)],temperature_c:30.4,
      accel_calibrated:true,gyro_calibrated:true,gyro_temperature_valid:true});
  }
  if(!paused)drawChart();
},100);
function drawChart(){
  const canvas=$('chart'),rect=canvas.getBoundingClientRect(),ratio=Math.min(devicePixelRatio||1,2);
  if(!rect.width||!rect.height)return;
  canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
  const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
  const width=rect.width,height=rect.height,left=32,right=9,top=8,bottom=18,plotW=width-left-right,plotH=height-top-bottom;
  ctx.font='8px Consolas,monospace';ctx.textBaseline='middle';
  for(const degree of [180,90,0,-90,-180]){
    const y=top+(180-degree)/360*plotH;ctx.strokeStyle=degree===0?'#d9e0d3':'#edf0e9';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-right,y);ctx.stroke();ctx.fillStyle='#adb6a5';ctx.fillText(degree+'°',0,y);
  }
  ctx.textBaseline='bottom';ctx.fillStyle='#b0b8aa';ctx.fillText('−12 s',left,height);ctx.textAlign='right';ctx.fillText('şimdi',width-right,height);ctx.textAlign='left';
  if(!history.length){ctx.textAlign='center';ctx.fillStyle='#a8b39e';ctx.fillText('İlk ölçümle birlikte grafik oluşacak.',width/2,height/2);return;}
  const now=performance.now();
  colors.forEach((color,i)=>{
    ctx.strokeStyle=color;ctx.lineWidth=1.6;ctx.beginPath();let previous=null;
    for(const point of history){const x=left+(1-(now-point.t)/12000)*plotW,y=top+(180-point.a[i])/360*plotH;if(x<left){previous=null;continue;}
      // Euler ±180° sarımını grafikte fiziksel bir dönüş gibi birleştirme.
      if(previous===null||Math.abs(point.a[i]-previous.a[i])>180||point.t-previous.t>500)ctx.moveTo(x,y);else ctx.lineTo(x,y);previous=point;}
    ctx.stroke();
  });
}
$('iso').onclick=()=>{renderer?.reset();$('iso').classList.add('selected');$('top').classList.remove('selected');};
$('top').onclick=()=>{renderer?.top();$('top').classList.add('selected');$('iso').classList.remove('selected');};
$('reset').onclick=()=>$('iso').click();
$('axes').onclick=()=>{const enabled=$('axes').getAttribute('aria-pressed')!=='true';$('axes').setAttribute('aria-pressed',String(enabled));if(renderer)renderer.axes=enabled;};
$('pause').onclick=()=>{paused=!paused;$('pause').setAttribute('aria-pressed',String(paused));$('pause').textContent=paused?'▶ Devam':'Ⅱ Dondur';if(!paused&&latest&&connected)showSample(latest);};
$('help').onclick=()=>$('help-dialog').showModal();$('close-help').onclick=()=>$('help-dialog').close();
$('help-dialog').addEventListener('click',e=>{if(e.target===$('help-dialog')){const r=e.target.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)e.target.close();}});
$('demo').onclick=()=>{
  demo=!demo;generation++;if(paused)$('pause').click();resetSession();$('demo-banner').hidden=!demo;
  $('demo').textContent=demo?'Canlı sensöre dön ↗':'Demoyu dene ↗';$('source').textContent=demo?'Simülasyon kaynağı':'Canlı sensör kaynağı';
  waiting(demo?'Demo hazırlanıyor':'Sensör bağlantısı bekleniyor');
};
$('export').onclick=()=>{
  const heading='source,timestamp_us,roll_deg,pitch_deg,yaw_deg,qw,qx,qy,qz,ax_g,ay_g,az_g,gx_rad_s,gy_rad_s,gz_rad_s,temperature_c';
  const blob=new Blob([heading+'\r\n'+recording.map(row=>row.join(',')).join('\r\n')],{type:'text/csv;charset=utf-8'});
  const url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download=demo?'flight-deck-DEMO.csv':'flight-deck-sensor.csv';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
};
window.addEventListener('resize',drawChart);drawChart();poll();
