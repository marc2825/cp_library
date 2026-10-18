import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';

const browserIndex=process.argv.indexOf('--browser');
if(browserIndex<0)throw Error('--browser /path/to/chromium is required');
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'cp-b-16-20-browser-'));
const browser=spawn(process.argv[browserIndex+1],['--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage','--no-first-run','--disable-breakpad','--remote-debugging-port=0',`--user-data-dir=${temporary}`,'about:blank'],{stdio:['ignore','ignore','pipe']});
let socket,closeBrowser;
try{
  const endpoint=await new Promise((resolve,reject)=>{
    let error='';const timeout=setTimeout(()=>reject(Error(`Chromium startup timeout: ${error}`)),10000);
    browser.on('error',reject);browser.on('exit',code=>{clearTimeout(timeout);reject(Error(`Chromium exited ${code}: ${error}`));});
    browser.stderr.on('data',data=>{error+=data;const match=error.match(/DevTools listening on (ws:\/\/\S+)/);if(match){clearTimeout(timeout);resolve(match[1]);}});
  });
  socket=new WebSocket(endpoint);await new Promise((resolve,reject)=>{socket.addEventListener('open',resolve,{once:true});socket.addEventListener('error',reject,{once:true});});
  let id=0;const pending=new Map(),errors=[];
  socket.addEventListener('message',event=>{const m=JSON.parse(event.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails.text);if(m.id){const r=pending.get(m.id);pending.delete(m.id);m.error?r.reject(Error(JSON.stringify(m.error))):r.resolve(m.result);}});
  const send=(method,params={},sessionId)=>new Promise((resolve,reject)=>{const next=++id;pending.set(next,{resolve,reject});socket.send(JSON.stringify({id:next,method,params,...(sessionId?{sessionId}:{})}));});
  closeBrowser=()=>send('Browser.close');
  const {targetId}=await send('Target.createTarget',{url:'about:blank'}),{sessionId}=await send('Target.attachToTarget',{targetId,flatten:true});
  const command=(method,params={})=>send(method,params,sessionId);await command('Page.enable');await command('Runtime.enable');
  const evaluate=async expression=>{const r=await command('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
  const check=async expression=>assert.equal(await evaluate(expression),true,expression);
  const click=async id=>evaluate(`document.getElementById(${JSON.stringify(id)}).click()`);
  const wait=async expression=>{for(let i=0;i<100;i++){if(await evaluate(expression))return;await new Promise(resolve=>setTimeout(resolve,50));}throw Error(`timeout: ${expression}`);};
  const shot=async suffix=>{const i=process.argv.indexOf('--screenshot');if(i>=0){const screenshot=await command('Page.captureScreenshot',{format:'png'});fs.writeFileSync(process.argv[i+1].replace(/\.png$/,`-${suffix}.png`),Buffer.from(screenshot.data,'base64'));}};
  const unique=new Map();for(const e of JSON.parse(fs.readFileSync(path.join(root,'verify/b_16_20/manifest.json'),'utf8')).algorithms)if(!unique.has(e.mode))unique.set(e.mode,e);const entries=[...unique.values()];
  const algorithms=entries;
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1050,deviceScaleFactor:1,mobile:false});
  let revision=0;
  for(const {mode,title}of algorithms){
    const url=pathToFileURL(path.join(root,'visualizations/b_16_20/index.html')).href+`?case=${++revision}#${mode}`;
    await command('Page.navigate',{url});await wait(`document.readyState==='complete'&&document.getElementById('mode')?.value===${JSON.stringify(mode)}&&document.getElementById('rows')?.children.length>0`);
    assert.equal(await evaluate("document.getElementById('title').textContent"),title);
    await check("document.getElementById('input-error').textContent===''");
    await check("(()=>{document.getElementById('first').click();for(let step=0;step<200;step++){const s=document.getElementById('diagram'),v=s.viewBox.baseVal,b=s.getBBox();if(b.width&&b.height&&(b.x<v.x-2||b.y<v.y-2||b.x+b.width>v.x+v.width+2||b.y+b.height>v.y+v.height+2))throw Error(JSON.stringify({mode:document.getElementById('mode').value,step,x:b.x,y:b.y,w:b.width,h:b.height,vw:v.width,vh:v.height}));if(document.getElementById('next').disabled)return true;document.getElementById('next').click();}return false;})()");
    await click('finish');await check("document.getElementById('step-title').textContent==='最終結果'");
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link ${href}`);
    await click('first');await check("document.getElementById('previous').disabled");await click('next');await click('previous');await click('play');await new Promise(resolve=>setTimeout(resolve,500));await click('play');
    await evaluate("document.getElementById('input').value='{'");await click('run');await check("document.getElementById('input-error').textContent.length>0");await click('sample');await check("document.getElementById('input-error').textContent===''");
    if(['compositional_inverse','partial_fraction','minimal_polynomial','low_rank_update'].includes(mode)){await click('finish');await click('previous');await shot(mode);}
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check('document.documentElement.scrollWidth<=391');if(mode==='low_rank_update')await shot('mobile');
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1050,deviceScaleFactor:1,mobile:false});
  }
  await evaluate("document.getElementById('mode').value='set_power_series';document.getElementById('mode').dispatchEvent(new Event('change'))");await wait("document.getElementById('title').textContent==='SPSのexp / log / inv'");
  await evaluate("document.getElementById('input').value=JSON.stringify({mod:17,f:[0,2,3,5],operation:'exp'})");await click('run');assert.deepEqual(JSON.parse(await evaluate("document.getElementById('result').textContent")),[1,2,3,11]);
  await evaluate("document.getElementById('input').value=JSON.stringify({mod:15,f:[0,2],operation:'exp'})");await click('run');await check("document.getElementById('input-error').textContent.length>0");await click('sample');
  assert.deepEqual(errors,[]);console.log('31 browser modes: input editing, history, play/pause, errors, mode switch, local links and 390px layout passed');
}finally{
  if(closeBrowser&&socket?.readyState===WebSocket.OPEN){await Promise.race([closeBrowser(),new Promise(resolve=>setTimeout(resolve,1500))]).catch(()=>{});socket.close();}
  if(browser.exitCode===null)browser.kill('SIGKILL');
  await new Promise(resolve=>setTimeout(resolve,100));fs.rmSync(temporary,{recursive:true,force:true});
}
