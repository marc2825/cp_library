import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';

const browserIndex=process.argv.indexOf('--browser');
if(browserIndex<0)throw Error('--browser /path/to/chromium is required');
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'cp-polynomial-extensions-browser-'));
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
  const entries=JSON.parse(fs.readFileSync(path.join(root,'verify/polynomial_extensions/manifest.json'),'utf8')).algorithms;
  const expected=JSON.parse(fs.readFileSync(path.join(root,'verify/polynomial_extensions/visualization_expected.json'),'utf8'));
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  for(const {mode} of entries){
    await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/polynomial_extensions/index.html')).href+'#'+mode});
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
    if(['factorization','composition','intersection'].includes(mode)){await click('first');for(let i=0;i<8&&!(await evaluate("document.getElementById('forest').children.length>0"));i++)await click('next');await check("document.getElementById('forest').children.length>0");await shot(mode);}
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check('document.documentElement.scrollWidth<=391');
    if(mode==='composition'){await click('finish');await click('previous');await evaluate("document.querySelector('.viewer').scrollIntoView()");await shot(`${mode}-mobile`);}
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  }
  const changes={extended_gcd:x=>x.a[0]++,resultant:x=>x.b[0]--,factorization:x=>x.a[1]=0,wrap_convolution:x=>x.a[0]='0',nimber_convolution:x=>x.a[0]='4',transpose:x=>x.kernel[0]++,doubling:x=>x.a[0]++,subset_sum:x=>x.weights[0]++,prefix_sum:x=>x.a[0]++,rader:x=>x.a[0][0]++,bluestein:x=>x.a[0][0]++,ntt64:x=>x.a[0]='1',avx2:x=>x.a[0]++,cyclic_tensor:x=>x.b[0]++,multiplicative:x=>x.a[1]++,projection:x=>x.count++,roots:x=>x.a[0]--,composition:x=>x.n++,fractional:x=>x.c++,multivariate:x=>x.a[1]=0,ode:x=>x.n++,polynomial_matrix:x=>x.count++,composed:x=>x.b[0]--,middle:x=>x.b[0]++,holonomic:x=>x.n--,reeds_sloane:x=>x.samples=[0,0,0],labelled:x=>x.operation='set',recurrence_slice:x=>x.start+=2,sps_projection:x=>x.count--,online_subset:x=>x.a[0]++,intersection:x=>x.b[0]++};
  const {createRequire}=await import('node:module');const require=createRequire(import.meta.url),model=require('../../visualizations/polynomial_extensions/model.js');
  const edited=model.modes.map(m=>{const input=structuredClone(m.sample);changes[m.id](input);return [m.id,input,model.trace(m.id,input).result];});
  for(const [mode,input,want] of edited){
    await evaluate(`document.getElementById('mode').value=${JSON.stringify(mode)};document.getElementById('mode').dispatchEvent(new Event('change'));document.getElementById('input').value=${JSON.stringify(JSON.stringify(input))}`);
    await click('run');await check("document.getElementById('input-error').textContent===''");
    assert.deepEqual(JSON.parse(await evaluate("document.getElementById('result').textContent")),want);
  }
  assert.deepEqual(errors,[]);
  console.log('polynomial extensions browser: 31 modes, 31 edited inputs, editable input, navigation, autoplay, errors, local links and 390px layout passed');
}finally{
  if(closeBrowser&&socket?.readyState===WebSocket.OPEN){await Promise.race([closeBrowser(),new Promise(resolve=>setTimeout(resolve,1500))]).catch(()=>{});socket.close();}
  if(browser.exitCode===null)browser.kill('SIGKILL');
  await new Promise(resolve=>setTimeout(resolve,100));fs.rmSync(temporary,{recursive:true,force:true});
}
