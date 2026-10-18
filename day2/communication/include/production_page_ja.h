#pragma once

constexpr char productionPageJapanese[] = R"HTML(
<!doctype html>
<html lang="ja">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>水上ボート 統合運転</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#f4f5f7;color:#18202a;font:16px system-ui,sans-serif}
main{max-width:760px;margin:auto;padding:18px}h1{font-size:23px;margin:2px 0 14px}
.status,.panel{border:1px solid #d7dce2;border-radius:10px;background:#fff;padding:14px}
.status{margin-bottom:12px;background:#fff4ce}.status.ok{background:#dcf3e3}.status.bad{background:#fde0df}
.gnss-panel{margin-bottom:12px}.gnss-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.gnss-cell{background:#f2f5f8;border-radius:8px;padding:8px}.gnss-key{color:#59636e;font-size:12px;font-weight:700}.gnss-value{font-size:15px;font-weight:800;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
.readiness-panel{margin-bottom:12px}.requirements{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.requirement{border-radius:8px;padding:9px;font-weight:750;font-size:14px}.requirement.ok{background:#dcf3e3}.requirement.bad{background:#fde0df}.requirement.optional{background:#f2f5f8;color:#59636e}
.group{border-top:1px solid #e2e5e9;margin-top:15px;padding-top:13px}.group:first-child{border-top:0;margin-top:0;padding-top:0}
.group-title{font-weight:800;margin-bottom:9px}.switches{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.switch{display:flex;align-items:center;gap:9px;border:1px solid #d7dce2;border-radius:8px;padding:10px;font-weight:700}
select,input[type=number]{width:100%;min-height:42px;border:1px solid #c9cfd6;border-radius:8px;background:#fff;padding:8px;font:inherit}
label.output{display:block;font-weight:700;margin:13px 0 7px}.range{display:grid;grid-template-columns:1fr 62px;gap:10px;align-items:center}
input[type=range]{width:100%}.number{text-align:right;font-weight:700;font-variant-numeric:tabular-nums}
.buttons{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-top:20px}
button{min-height:48px;border:0;border-radius:8px;font:700 16px system-ui}button:disabled,input:disabled,select:disabled{opacity:.48}
.start{background:#24863d;color:#fff}.stop{background:#d9dee4}.estop{grid-column:1/-1;background:#c92532;color:#fff}.clear{background:#efb521;color:#18202a}
.apply{width:100%;margin-top:12px;background:#2563eb;color:#fff}
.message{margin-top:14px;min-height:46px}.detail,.hint{margin-top:10px;color:#59636e;font-size:14px;line-height:1.55}.hint{background:#f2f5f8;border-radius:8px;padding:9px}.hint.warn{background:#fff4ce;color:#7a4a00}
.tuning-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px 14px}.field label{display:block;font-size:13px;font-weight:750;margin:0 0 5px}
.servo-grid{display:grid;grid-template-columns:1.15fr repeat(3,1fr);gap:7px;align-items:center;margin-top:12px}.servo-grid .head{font-size:13px;font-weight:750;text-align:center}.servo-name{font-weight:750}
@media(max-width:560px){.tuning-grid{grid-template-columns:1fr}.servo-grid{grid-template-columns:1fr 1fr 1fr}.servo-name{grid-column:1/-1;margin-top:8px}.servo-grid .blank{display:none}}
</style>
<main>
  <div style="text-align:right"><a href="/track" style="color:#1859a8;font-weight:700">航跡・ウェイポイント</a>　<a href="/sensors" style="color:#1859a8;font-weight:700">センサー</a>　<a href="/diagnostics" style="color:#1859a8;font-weight:700">診断</a></div>
  <h1>統合運転</h1>
  <div id="connection" class="status">XIAOとGNSSを確認しています…</div>
  <section class="panel gnss-panel">
    <div class="group-title">GNSS 状態（約1秒ごとに更新）</div>
    <div id="gnss-state" class="hint">受信待ち</div>
    <div class="gnss-grid">
      <div class="gnss-cell"><div class="gnss-key">緯度</div><div id="gnss-lat" class="gnss-value">--</div></div>
      <div class="gnss-cell"><div class="gnss-key">経度</div><div id="gnss-lon" class="gnss-value">--</div></div>
      <div class="gnss-cell"><div class="gnss-key">速度</div><div id="gnss-speed" class="gnss-value">--</div></div>
      <div class="gnss-cell"><div class="gnss-key">衛星数</div><div id="gnss-satellites" class="gnss-value">--</div></div>
      <div class="gnss-cell"><div class="gnss-key">最終更新</div><div id="gnss-age" class="gnss-value">--</div></div>
      <div class="gnss-cell"><div class="gnss-key">受信文 / エラー</div><div id="gnss-count" class="gnss-value">--</div></div>
    </div>
  </section>
  <section class="panel readiness-panel">
    <div class="group-title">ヨー制御状態</div>
    <div id="yaw-state" class="hint">制御側XIAOからの状態待ち</div>
  </section>
  <section class="panel readiness-panel">
    <div class="group-title">接続・開始条件</div>
    <div id="required-items" class="requirements">状態を取得中</div>
  </section>
  <section class="panel">
    <div class="group">
      <div class="group-title">制御選択（停止中のみ変更可能）</div>
      <div class="switches">
        <label class="switch"><input id="waypoint-enable" type="checkbox">ウェイポイント</label>
        <label class="switch"><input id="attitude-enable" type="checkbox">姿勢制御</label>
        <label class="switch"><input id="sd-logging" type="checkbox">SDカードへ記録</label>
      </div>
      <label class="output" for="target">固定ウェイポイント</label>
      <select id="target">
        <option value="0">A — 35.45327, 136.07198</option>
        <option value="1">B — 35.44437, 136.07399</option>
        <option value="2">C — 35.43214, 136.07628</option>
        <option value="3">D — 35.42587, 136.09377</option>
        <option value="4">E — 35.42542, 136.12038</option>
        <option value="5">F — 35.43055, 136.14585</option>
        <option value="6">G — 35.44150, 136.12081</option>
        <option value="7">H — 35.44196, 136.09429</option>
        <option value="placed">配置したウェイポイント</option>
        <option value="course">大会コース — A → B → C → D → E → F → G → H → C → B → A</option>
      </select>
      <div id="mode-hint" class="hint">全出力を手動操作します。</div>
    </div>

    <div class="group" id="manual-section">
      <div class="group-title">手動入力</div>
      <label class="output" for="left">左前翼 CH0</label><div class="range"><input id="left" type="range" min="-1" max="1" step="0.01" value="0"><span id="left-number" class="number">0.00</span></div>
      <label class="output" for="right">右前翼 CH1</label><div class="range"><input id="right" type="range" min="-1" max="1" step="0.01" value="0"><span id="right-number" class="number">0.00</span></div>
      <label class="output" for="rear">後部ヨー CH2</label><div class="range"><input id="rear" type="range" min="-1" max="1" step="0.01" value="0"><span id="rear-number" class="number">0.00</span></div>
      <label class="output" for="propulsion">推進</label><div class="range"><input id="propulsion" type="range" min="0" max="1" step="0.01" value="0"><span id="propulsion-number" class="number">0.00</span></div>
    </div>

    <div class="group">
      <div class="group-title">制御・機体調整（DISARMED中のみ適用可能）</div>
      <div class="tuning-grid">
        <div class="field"><label for="kp-pitch">Pitch Kp</label><input id="kp-pitch" type="number" min="0" max="10" step="0.01"></div><div class="field"><label for="kd-pitch">Pitch Kd</label><input id="kd-pitch" type="number" min="0" max="5" step="0.01"></div>
        <div class="field"><label for="kp-roll">Roll Kp</label><input id="kp-roll" type="number" min="0" max="10" step="0.01"></div><div class="field"><label for="kd-roll">Roll Kd</label><input id="kd-roll" type="number" min="0" max="5" step="0.01"></div>
        <div class="field"><label for="pitch-cutoff">Pitch LPF カットオフ [Hz]</label><input id="pitch-cutoff" type="number" min="0" max="10" step="0.1"></div><div class="field"><label for="roll-cutoff">Roll LPF カットオフ [Hz]</label><input id="roll-cutoff" type="number" min="0" max="10" step="0.1"></div>
        <div class="field"><label for="kp-height">Height Kp</label><input id="kp-height" type="number" min="0" max="10" step="0.01"></div><div class="field"><label for="height-cutoff">Height LPF カットオフ [Hz]</label><input id="height-cutoff" type="number" min="0" max="10" step="0.1"></div>
        <div class="field"><label for="target-height">目標高さ [m]</label><input id="target-height" type="number" min="0.10" max="2.00" step="0.01"></div><div class="field"><label for="kp-yaw">Yaw Kp</label><input id="kp-yaw" type="number" min="0" max="10" step="0.01"></div>
        <div class="field"><label for="kd-yaw">Yaw Kd</label><input id="kd-yaw" type="number" min="0" max="5" step="0.01"></div><div class="field"><label for="yaw-cutoff">Yaw LPF カットオフ [Hz]</label><input id="yaw-cutoff" type="number" min="0" max="10" step="0.1"></div>
        <div class="field"><label for="max-duty">VESC 最大Duty [0〜1.00]</label><input id="max-duty" type="number" min="0" max="1.00" step="0.01"></div><div class="field"><label for="vesc-ramp-rise">VESC 最大Dutyまでの加速時間 [s]</label><input id="vesc-ramp-rise" type="number" min="0.1" max="20" step="0.1"></div>
        <div class="hint" style="margin:0">各LPFは 0 Hz で無効、初期値は 3 Hz です。VESCの加速時間の初期値は 2 秒です。停止時は従来どおり直ちにDutyを0にします。</div>
      </div>
      <div class="servo-grid"><div class="blank"></div><div class="head">上 / 左 [°]</div><div class="head">中 [°]</div><div class="head">下 / 右 [°]</div><div class="servo-name">左前翼 CH0</div><input id="left-up" type="number" min="0" max="180" step="0.5"><input id="left-middle" type="number" min="0" max="180" step="0.5"><input id="left-down" type="number" min="0" max="180" step="0.5"><div class="servo-name">右前翼 CH1</div><input id="right-up" type="number" min="0" max="180" step="0.5"><input id="right-middle" type="number" min="0" max="180" step="0.5"><input id="right-down" type="number" min="0" max="180" step="0.5"><div class="servo-name">後部ヨー CH2</div><input id="rear-left" type="number" min="0" max="180" step="0.5"><input id="rear-middle" type="number" min="0" max="180" step="0.5"><input id="rear-right" type="number" min="0" max="180" step="0.5"></div>
      <button id="apply-tuning" class="apply">調整値を適用</button>
      <div class="hint">初期値: 左前翼 上105° / 中90° / 下75°、右前翼 上75° / 中90° / 下105°、後部ヨー 左70° / 中90° / 右110°。上中下・左中右の数値順は自由で、各区間は直線補間されます。設定はRAM上のみです。</div>
    </div>

    <div class="buttons">
      <button id="start" class="start">開始</button>
      <button id="stop" class="stop">停止</button>
      <button id="estop" class="estop">緊急停止</button>
    </div>
    <div id="message" class="message">制御方法を選んで開始してください。</div>
    <div id="detail" class="detail">全出力OFF</div>
  </section>
</main>
<script>
const get=id=>document.getElementById(id);
let latest=null;let requestsInFlight=0;let selectionInitialized=false;
const outputIds=['left','right','rear','propulsion'];
const tuningIds=['kp-pitch','kd-pitch','kp-roll','kd-roll','pitch-cutoff','roll-cutoff','kp-height','height-cutoff','target-height','kp-yaw','kd-yaw','yaw-cutoff','max-duty','vesc-ramp-rise','left-up','left-middle','left-down','right-up','right-middle','right-down','rear-left','rear-middle','rear-right'];
function manualQuery(){return outputIds.map(id=>id+'='+encodeURIComponent(get(id).value)).join('&')}
function selectionQuery(){const course=get('target').value==='course';return 'waypoint='+(get('waypoint-enable').checked?1:0)+'&attitude='+(get('attitude-enable').checked?1:0)+'&competition_course='+(course?1:0)+'&target='+(course?'course':encodeURIComponent(get('target').value))}
function tuningQuery(){const names=['kp_pitch','kd_pitch','kp_roll','kd_roll','pitch_cutoff_hz','roll_cutoff_hz','kp_height','height_cutoff_hz','target_height','kp_yaw','kd_yaw','yaw_cutoff_hz','max_duty','vesc_ramp_rise_s','left_up','left_middle','left_down','right_up','right_middle','right_down','rear_left','rear_middle','rear_right'];return names.map((name,i)=>name+'='+encodeURIComponent(get(tuningIds[i]).value)).join('&')}
function modeHint(){
  const wp=get('waypoint-enable').checked,att=get('attitude-enable').checked;
  get('mode-hint').textContent=wp?(att?'固定点へLOS航行し、左右前翼で姿勢・高さも制御します。':'固定点へLOS航行します。前翼は中立で、姿勢補正は行いません。'):(att?'左右前翼を姿勢・高さ制御し、後部ヨーと推進は手動です。':'左右前翼・後部ヨー・推進をすべて手動操作します。');
  updateControlAvailability();
}
function updateControlAvailability(){
  const locked=latest&&!['idle','error'].includes(latest.operation),transition=locked&&latest.operation!=='running',wp=get('waypoint-enable').checked,att=get('attitude-enable').checked;
  get('waypoint-enable').disabled=locked;get('attitude-enable').disabled=locked;get('target').disabled=locked||!wp;
  get('left').disabled=transition||wp||att;get('right').disabled=transition||wp||att;get('rear').disabled=transition||wp;get('propulsion').disabled=transition||wp;
  get('sd-logging').disabled=!latest||!latest.storage.sd_ready||requestsInFlight>0;
  const tuneLocked=locked||!latest||!latest.connected||latest.control.safety!==1||requestsInFlight>0;tuningIds.forEach(id=>get(id).disabled=tuneLocked);get('apply-tuning').disabled=tuneLocked;
}
async function post(path){
  requestsInFlight++;
  updateControlAvailability();
  try{const response=await fetch(path,{method:'POST'});const result=await response.json();get('message').textContent=result.message||'指令を送りました。';return response.ok&&result.accepted!==false}
  catch(error){get('message').textContent='通信側XIAOへ指令を送れませんでした。';return false}
  finally{requestsInFlight--;renderButtons();updateControlAvailability()}
}
function setTuningValues(t){const values=[t.kp_pitch,t.kd_pitch,t.kp_roll,t.kd_roll,t.pitch_cutoff_hz,t.roll_cutoff_hz,t.kp_height,t.height_cutoff_hz,t.target_height,t.kp_yaw,t.kd_yaw,t.yaw_cutoff_hz,t.max_duty,t.vesc_ramp_rise_s,t.left.up,t.left.middle,t.left.down,t.right.up,t.right.middle,t.right.down,t.rear.left,t.rear.middle,t.rear.right];values.forEach((v,i)=>get(tuningIds[i]).value=Number(v))}
async function loadTuning(){try{const r=await fetch('/api/tuning',{cache:'no-store'});if(r.ok)setTuningValues(await r.json())}catch(error){}}
function validTuningClient(){const v=tuningIds.map(id=>Number(get(id).value));if(v.some(x=>!Number.isFinite(x)))return false;const positions=o=>v[o]>=0&&v[o]<=180&&v[o+1]>=0&&v[o+1]<=180&&v[o+2]>=0&&v[o+2]<=180;return v[0]>=0&&v[0]<=10&&v[1]>=0&&v[1]<=5&&v[2]>=0&&v[2]<=10&&v[3]>=0&&v[3]<=5&&v[4]>=0&&v[4]<=10&&v[5]>=0&&v[5]<=10&&v[6]>=0&&v[6]<=10&&v[7]>=0&&v[7]<=10&&v[8]>=.10&&v[8]<=2&&v[9]>=0&&v[9]<=10&&v[10]>=0&&v[10]<=5&&v[11]>=0&&v[11]<=10&&v[12]>=0&&v[12]<=1&&v[13]>=.1&&v[13]<=20&&positions(14)&&positions(17)&&positions(20)}
async function applyTuning(){if(!validTuningClient()){get('message').textContent='調整値を確認してください。各位置は 0〜180° の範囲で設定できます。';return}await post('/api/tuning?'+tuningQuery())}
function renderButtons(){
  if(!latest){get('start').disabled=true;return}
  const idle=['idle','error'].includes(latest.operation),wp=get('waypoint-enable').checked,att=get('attitude-enable').checked;
  const imuNeeded=att,gnssReady=latest.gnss.communication_valid&&latest.gnss.control_valid;
  get('start').disabled=requestsInFlight>0||!idle||latest.control.safety===4||!latest.connected||(imuNeeded&&!latest.sensors.imu_valid);
  get('stop').disabled=false;
}
function render(state){
  latest=state;const connection=get('connection');
  const pre=state.preflight;
  const required=[
    ['制御側XIAO通信',pre.control_link,true],['PCA9685',pre.pca,true],
    ['BNO08X',pre.imu,pre.attitude_required],
    ['GNSS（通信側）',pre.gnss_communication,pre.waypoint_required],
    ['GNSS（制御側）',pre.gnss_control,pre.waypoint_required],
    ['VESC電源状態',pre.power,pre.waypoint_required],
    ['VESCテレメトリ',pre.vesc,pre.waypoint_required],
    ['INA226（電圧・電流）',pre.ina,false],
    ['ToF（高さ制御用）',pre.tof,false]
  ];
  get('required-items').innerHTML=required.map(item=>'<div class="requirement '+(item[1]?'ok':(item[2]?'bad':'optional'))+'">'+(item[1]?'✓ ':'— ')+item[0]+'<br><small>'+(item[1]?'接続済み':(item[2]?'未接続・開始不可':'未接続・任意'))+'</small></div>').join('');
  const gnss=state.gnss,gnssOk=gnss.communication_valid&&gnss.control_valid;
  get('gnss-state').textContent=gnssOk?'測位有効（通信側・制御側ともに有効）':(gnss.sentences?'受信中・測位待ち':'GNSSデータ未受信');
  get('gnss-lat').textContent=gnss.communication_valid?Number(gnss.latitude).toFixed(8):'--';
  get('gnss-lon').textContent=gnss.communication_valid?Number(gnss.longitude).toFixed(8):'--';
  get('gnss-speed').textContent=gnss.communication_valid?Number(gnss.speed_mps).toFixed(2)+' m/s':'--';
  get('gnss-satellites').textContent=gnss.communication_valid?String(gnss.satellites):'--';
  get('gnss-age').textContent=gnss.age_ms===4294967295?'--':String(gnss.age_ms)+' ms';
  get('gnss-count').textContent=String(gnss.sentences)+' / '+String(gnss.checksum_errors);
  get('sd-logging').checked=Boolean(state.storage.logging_enabled);
  const yaw=state.yaw||{active:0,criteria_mask:0,vesc_low_erpm_warning:0,vesc_erpm:0};
  const criterion=[];if(yaw.criteria_mask&4)criterion.push('GNSS進行方向');if(yaw.criteria_mask&1)criterion.push('Duty指令');
  const yawState=get('yaw-state');yawState.className='hint'+(yaw.vesc_low_erpm_warning?' warn':'');
  yawState.textContent='ヨー制御: '+(yaw.active?'有効':'待機（舵は中立）')+' / 有効条件 '+(criterion.length?criterion.join('・'):'なし')+' / VESC '+Number(yaw.vesc_erpm).toFixed(0)+' ERPM（表示のみ）'+(yaw.vesc_low_erpm_warning?' / 注意: Duty指令に対してERPMが極端に低い状態が2秒以上継続しています':'');
  const locked=!['idle','error'].includes(state.operation);
  if(!selectionInitialized||locked){get('waypoint-enable').checked=Boolean(state.selection.waypoint);get('attitude-enable').checked=Boolean(state.selection.attitude);get('target').value=state.selection.competition_course?'course':(state.selection.placed_waypoints?'placed':String(state.selection.target_index));selectionInitialized=true;modeHint()}
  if(!state.connected){connection.className='status bad';connection.textContent=state.ever_received?'XIAOとの通信が切れています':'XIAOを待っています'}
  else if(!state.actuators.pca_ready){connection.className='status bad';connection.textContent='XIAO接続済み・PCA9685未接続'}
  else{const wpSelected=get('waypoint-enable').checked,fix=state.gnss.communication_valid&&state.gnss.control_valid;connection.className=!wpSelected||fix?'status ok':'status';connection.textContent='制御側XIAO '+state.control.safety_name+' / '+(wpSelected?('GNSS '+(fix?'FIX':'WAIT')+' / 衛星 '+state.gnss.satellites):('GNSS不要 / 衛星 '+state.gnss.satellites))+' / SD '+(state.storage.sd_ready?'OK':'未認識')}
  get('message').textContent=state.message;const running=state.operation==='running';
  const tofWarning=state.sensors.tof_valid?'':' / 警告: ToF未接続または無効（高さ制御なし）';
  get('detail').textContent=(running?state.selection.mode_name+' / PWM '+state.actuators.left_us+'・'+state.actuators.right_us+'・'+state.actuators.rear_us+' µs / Duty '+Number(state.actuators.applied_duty).toFixed(3)+' / 目標距離 '+Number(state.control.waypoint_distance_m).toFixed(1)+' m':'全出力OFF / 停止理由 '+state.control.stop_reason_name+' / PCAエラー '+state.actuators.pwm_errors)+tofWarning;
  const emergency=state.control.safety===4;get('estop').textContent=emergency?'緊急停止を解除':'緊急停止';get('estop').classList.toggle('clear',emergency);
  updateControlAvailability();renderButtons();
}
async function poll(){try{const response=await fetch('/api/status',{cache:'no-store'});if(!response.ok)throw new Error();render(await response.json())}catch(error){latest=null;get('connection').className='status bad';get('connection').textContent='通信側XIAOから状態を取得できません';get('start').disabled=true;updateControlAvailability()}}
outputIds.forEach(id=>get(id).addEventListener('input',()=>{get(id+'-number').textContent=Number(get(id).value).toFixed(2)}));
outputIds.forEach(id=>get(id).addEventListener('change',()=>{if(latest&&latest.operation==='running'&&!latest.selection.waypoint)post('/api/value?'+manualQuery())}));
get('waypoint-enable').addEventListener('change',modeHint);get('attitude-enable').addEventListener('change',modeHint);
get('sd-logging').addEventListener('change',()=>post('/api/logging?enabled='+(get('sd-logging').checked?1:0)));
get('apply-tuning').addEventListener('click',applyTuning);
get('start').addEventListener('click',()=>post('/api/start?'+manualQuery()+'&'+selectionQuery()));
get('stop').addEventListener('click',()=>post('/api/stop'));
get('estop').addEventListener('click',()=>post(latest&&latest.control.safety===4?'/api/clear-estop':'/api/estop'));
setInterval(poll,2000);modeHint();loadTuning();poll();
</script>
</html>
)HTML";
