import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';

const browserIndex=process.argv.indexOf('--browser');
if(browserIndex<0)throw Error('--browser /path/to/chromium is required');
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'cp-combinatorics-extensions-browser-'));
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
  const entries=JSON.parse(fs.readFileSync(path.join(root,'verify/combinatorics_extensions/manifest.json'),'utf8')).algorithms;
  const expected=JSON.parse(fs.readFileSync(path.join(root,'verify/combinatorics_extensions/visualization_expected.json'),'utf8'));
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  for(const {mode} of entries){
    await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/combinatorics_extensions/index.html')).href+'#'+mode});
    await wait(`document.readyState==='complete'&&document.getElementById('mode')?.value===${JSON.stringify(mode)}&&document.getElementById('step-title')?.textContent.length>0`);
    await check("document.getElementById('input-error').textContent===''");
    assert.deepEqual(JSON.parse(await evaluate("document.getElementById('result').textContent")),expected[mode]);
    await click('finish');await check("document.getElementById('next').disabled");
    await click('previous');await click('first');await check("document.getElementById('previous').disabled");
    await click('next');await check("!document.getElementById('previous').disabled");
    await click('play');await new Promise(resolve=>setTimeout(resolve,750));await click('play');
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),href);
    await evaluate("document.getElementById('input').value='{' ");await click('run');
    await check("document.getElementById('input-error').textContent.length>0");await click('sample');
    await check("document.getElementById('input-error').textContent===''");
    if(['bounded_monotone','rsk','hook_length'].includes(mode)){await click('first');for(let i=0;i<8&&!(await evaluate("document.getElementById('forest').children.length>0"));i++)await click('next');await check("document.getElementById('forest').children.length>0");await shot(mode);}
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check('document.documentElement.scrollWidth<=391');
    if(mode==='bounded_monotone'){await click('finish');await click('previous');await evaluate("document.querySelector('.viewer').scrollIntoView()");await shot(`${mode}-mobile`);}
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  }
  const edited = [
    ['q_binomial',{prime:3,q:0,n:6,k:2},['1','1','0']],
    ['bounded_monotone',{lower:[0,0],upper:[2,2],strict:true},['3']],
    ['stirling_small_prime',{prime:5,n:6,k:2},['4','1']],
    ['unlabeled_trees',{n:3},['1','1','2','1','1','1']],
    ['rsk',{word:[3,2,1]},['1','1','1','3','2','1']],
    ['young_tableaux',{shape:[2,1]},['2','2','1']],
    ['hook_length',{shape:[2,1],alphabet:2,modulus:2},['2','0','2']],
    ['combinatorial_transforms',{values:[0,1,0]},['0','1','2','1','1','1','0','1','0']]
  ];
  for(const [mode,input,want] of edited){
    await evaluate(`document.getElementById('mode').value=${JSON.stringify(mode)};document.getElementById('mode').dispatchEvent(new Event('change'));document.getElementById('input').value=${JSON.stringify(JSON.stringify(input))}`);
    await click('run');await check("document.getElementById('input-error').textContent===''");
    assert.deepEqual(JSON.parse(await evaluate("document.getElementById('result').textContent")),want);
  }
  assert.deepEqual(errors,[]);
  console.log('combinatorics extensions browser: 8 modes, 8 edited inputs, editable input, navigation, autoplay, errors, local links and 390px layout passed');
}finally{
  if(closeBrowser&&socket?.readyState===WebSocket.OPEN){await Promise.race([closeBrowser(),new Promise(resolve=>setTimeout(resolve,1500))]).catch(()=>{});socket.close();}
  if(browser.exitCode===null)browser.kill('SIGKILL');
  await new Promise(resolve=>setTimeout(resolve,100));fs.rmSync(temporary,{recursive:true,force:true});
}
