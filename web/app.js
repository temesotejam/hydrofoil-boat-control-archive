'use strict';
const versions={day1:{label:'1日目',commit:'88bed77ee1779f9e76afa91d313e38ace85177a6',variant:'used-day1-v1'},day2:{label:'2日目',commit:'0820d1d1c2107609c839bf6cc5d8c42f9d56e982',variant:''}};
let buildInfo=null;
const byId=id=>document.getElementById(id);
function update(){
 const day=document.querySelector('input[name="day"]:checked').value;
 const version=versions[day];
 byId('selected-label').textContent=version.label;
 byId('download-all').textContent=version.label+'のプログラムZIP（2台分）';
 for(const [id,kind] of [['download-all','program'],['download-control','control'],['download-communication','communication']])byId(id).href='./downloads/hydrofoil-'+day+'-'+kind+'.zip';
 byId('source-commit').textContent='選択中の保存版：'+version.commit+(version.variant?' / '+version.variant:'');
 const container=byId('installers');container.replaceChildren();
 if(!buildInfo){container.hidden=true;return;}
 container.hidden=false;
 for(const [node,label,description] of [['control','制御側','姿勢・距離・電源の計測と、前翼・後翼・推進の制御を担当します。'],['communication','通信側','GNSSの取得、操作画面、制御側との通信を担当します。']]){
  const card=document.createElement('article');card.className='card';
  const badge=document.createElement('span');badge.className='node';badge.textContent=node.toUpperCase()+' · XIAO ESP32S3';
  const title=document.createElement('h3');title.textContent=version.label+' '+label;
  const body=document.createElement('p');body.textContent=description;
  const installer=document.createElement('esp-web-install-button');installer.setAttribute('manifest','./'+day+'/'+node+'.json');
  const button=document.createElement('button');button.slot='activate';button.textContent=label+'へ書き込む';installer.append(button);
  for(const [slot,text] of [['unsupported','PC版ChromeまたはEdgeで開いてください。'],['not-allowed','HTTPSの公開ページから開いてください。']]){const p=document.createElement('p');p.slot=slot;p.textContent=text;installer.append(p);}
  const links=document.createElement('div');links.className='binary-links';
  for(const [suffix,text] of [['.bin','書き込み用BIN'],['.bin.sha256','SHA-256']]){const a=document.createElement('a');a.href='./'+day+'/'+node+suffix;a.download='';a.textContent=text;links.append(a);}
  card.append(badge,title,body,installer,links);container.append(card);
 }
}
document.querySelectorAll('input[name="day"]').forEach(input=>input.addEventListener('change',update));update();
(async()=>{
 try{
  const response=await fetch('./build-info.json',{cache:'no-store'});if(!response.ok)throw Error('build-info');
  const info=await response.json();if(!info.ready)throw Error('not ready');
  for(const day of Object.keys(versions)){
   if(info.versions?.[day]?.commit!==versions[day].commit || (info.versions[day].variant||'')!==versions[day].variant)throw Error('version mismatch');
   for(const node of ['control','communication']){
    if(!/^[0-9a-f]{64}$/.test(info.versions[day].firmware?.[node]||''))throw Error('firmware hash');
    const manifestResponse=await fetch('./'+day+'/'+node+'.json');if(!manifestResponse.ok)throw Error('manifest');
    const manifest=await manifestResponse.json();if(manifest.version!==versions[day].commit.slice(0,7)+(versions[day].variant?'-'+versions[day].variant:''))throw Error('manifest version');
   }
  }
  buildInfo=info;byId('build-status').textContent='両日の書き込み用ファイルを確認しました。対象基板のボタンを選んでください。';update();
 }catch(error){byId('build-status').textContent='書き込み用ファイルを読み込めませんでした。ページを再読み込みするか、末尾のビルド状況を確認してください。';}
})();
