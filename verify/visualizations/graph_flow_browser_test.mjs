// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/graph_flow_browser_test.mjs --browser /path/to/chromium
import assert from "node:assert/strict";
import {spawn} from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import {fileURLToPath,pathToFileURL} from "node:url";

const browserIndex=process.argv.indexOf("--browser");
if (browserIndex<0 || !process.argv[browserIndex+1]) throw new Error("--browser /path/to/chromium is required");
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),"../..");
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),"cp-visualization-"));
const browser=spawn(process.argv[browserIndex+1],["--headless","--no-sandbox","--disable-gpu","--disable-dev-shm-usage","--no-first-run","--disable-breakpad","--remote-debugging-port=0",`--user-data-dir=${temporary}`,"about:blank"],{stdio:["ignore","ignore","pipe"]});
let socket, closeBrowser;
async function bounded(promise, milliseconds) {
  let timeout;
  try { return await Promise.race([promise, new Promise(resolve=>{timeout=setTimeout(resolve,milliseconds);})]); }
  finally { clearTimeout(timeout); }
}
try {
  const endpoint=await new Promise((resolve,reject)=>{
    let error="";
    const timeout=setTimeout(()=>reject(new Error(`Chromium startup timeout: ${error}`)),10000);
    browser.on("error",reject);
    browser.on("exit",code=>{clearTimeout(timeout);reject(new Error(`Chromium exited ${code}: ${error}`));});
    browser.stderr.on("data",data=>{error+=data;const match=error.match(/DevTools listening on (ws:\/\/\S+)/);if(match){clearTimeout(timeout);resolve(match[1]);}});
  });
  socket=new WebSocket(endpoint);
  await new Promise((resolve,reject)=>{socket.addEventListener("open",resolve,{once:true});socket.addEventListener("error",reject,{once:true});});
  let id=0;
  const pending=new Map(),errors=[];
  socket.addEventListener("message",event=>{
    const message=JSON.parse(event.data);
    if(message.method==="Runtime.exceptionThrown") errors.push(message.params.exceptionDetails.text);
    if(message.id){const request=pending.get(message.id);pending.delete(message.id);message.error?request.reject(new Error(JSON.stringify(message.error))):request.resolve(message.result);}
  });
  const send=(method,params={},sessionId)=>new Promise((resolve,reject)=>{const next=++id;pending.set(next,{resolve,reject});socket.send(JSON.stringify({id:next,method,params,...(sessionId?{sessionId}:{})}));});
  closeBrowser=()=>send("Browser.close");
  const {targetId}=await send("Target.createTarget",{url:"about:blank"});
  const {sessionId}=await send("Target.attachToTarget",{targetId,flatten:true});
  const command=(method,params={})=>send(method,params,sessionId);
  await command("Page.enable");await command("Runtime.enable");
  const evaluate=async expression=>{
    const result=await command("Runtime.evaluate",{expression,returnByValue:true,awaitPromise:true});
    if(result.exceptionDetails) throw new Error(JSON.stringify(result.exceptionDetails));
    return result.result.value;
  };
  const check=async expression=>assert.equal(await evaluate(expression),true,expression);
  const click=async id=>evaluate(`document.getElementById(${JSON.stringify(id)}).click()`);
  const set=async(id,value)=>evaluate(`document.getElementById(${JSON.stringify(id)}).value=${JSON.stringify(String(value))}`);



  const waitFor=async expression=>{for(let i=0;i<100;++i){if(await evaluate(expression))return;await new Promise(resolve=>setTimeout(resolve,50));}throw new Error(`timeout: ${expression}`);};
  const screenshotIndex=process.argv.indexOf("--screenshot");let revision=0;
  const shot=async suffix=>{if(screenshotIndex>=0){await evaluate("new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)))");const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,suffix+".png"),Buffer.from(screenshot.data,"base64"));}};
  const load=async mode=>{
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/graph_flow/index.html")).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);
  };
  const result=async value=>check(`document.getElementById('result').textContent===${JSON.stringify(value)}`);
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');await click('finish');};
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of ['dinic','cut','matching']){
    await load(mode);await click('finish');await result(mode==='matching'?'3 組':'2');
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>getComputedStyle(p).fill==='none')");
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot(`-${mode}`);
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('dinic');await click('augment');await check("document.getElementById('step-title').textContent==='増加路が見つかった'");await click('augment');
  await check("document.getElementById('path-summary').textContent.includes('#2 逆向き') && document.querySelectorAll('#diagram .reverse.active').length===1");await shot('-reverse');
  await click('next');assert.deepEqual(await evaluate("[...document.querySelectorAll('#values tr')].map(r=>Number(r.dataset.flow))"),[1,1,0,1,1,1,1]);
  await evaluate("document.getElementById('show-reverse').checked=false;document.getElementById('show-reverse').dispatchEvent(new Event('change'))");await check("document.querySelectorAll('#diagram .reverse').length===0");
  await scenario('capacities');await result('5');await check("document.getElementById('certificate').textContent==='5'");
  await scenario('limited');await result('1');await check("document.getElementById('certificate').textContent==='未確定' && document.getElementById('outputs').textContent.includes('未確定')");
  await set('limit',2);await click('run');await click('finish');await result('2');await check("document.getElementById('certificate').textContent==='2'");
  await set('limit',0);await click('run');await click('finish');await result('0');await check("document.getElementById('certificate').textContent==='未確定'");
  await load('cut');await set('limit',0);await click('run');await click('finish');await result('2');
  await check("document.getElementById('outputs').textContent.includes('カット辺 ID: 0, 1') && document.querySelectorAll('#diagram .cut').length===2");
  await scenario('parallel');await result('4');await check("document.getElementById('outputs').textContent.includes('カット辺 ID: 4')");
  await scenario('zero');await result('0');await check("document.getElementById('outputs').textContent.includes('カット辺 ID: 0, 1')");
  await scenario('disconnected');await result('0');await check("document.getElementById('outputs').textContent.includes('カット辺 ID: なし')");
  await scenario('empty');await result('0');
  await load('matching');await click('augment');await click('augment');await click('augment');
  await check("document.getElementById('path-summary').textContent.includes('− L0—R0') && document.getElementById('path-summary').textContent.includes('− L2—R1')");await shot('-alternating');
  await click('next');await result('3 組');await click('finish');assert.deepEqual(await evaluate("[...document.querySelectorAll('#values tr')].map(r=>Number(r.dataset.match))"),[1,0,2]);
  await scenario('parallel');await result('2 組');await scenario('unequal');await result('2 組');await scenario('disconnected');await result('2 組');await scenario('empty');await result('0 組');
  await set('left-size',0);await set('right-size',3);await set('edges','');await click('run');await click('finish');await result('0 組');
  await set('edges','0 0');await click('run');await check("document.getElementById('input-error').textContent!==''");
  await load('dinic');await set('sink',0);await click('run');await check("document.getElementById('input-error').textContent.includes('異なる')");
  await set('sink',5);await set('edges','0 1 -1');await click('run');await check("document.getElementById('input-error').textContent.includes('容量')");
  await set('edges','0 6 1');await click('run');await check("document.getElementById('input-error').textContent.includes('頂点')");
  await set('edges','0 1');await click('run');await check("document.getElementById('input-error').textContent.includes('3 個')");
  await set('edges',Array(19).fill('0 1 1').join('\n'));await click('run');await check("document.getElementById('input-error').textContent.includes('18 本')");
  await load('matching');await click('next');await set('left-size',1);await set('right-size',1);await set('edges','0 0');await click('run');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('finish');await result('1 組');
  await load('dinic');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=matching]').click()");await waitFor("document.querySelector('[data-mode=matching]').getAttribute('aria-pressed')==='true'");await click('finish');await result('3 組');
  await load('dinic');await click('augment');await click('augment');await shot('');
  await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await shot('-mobile');
  assert.deepEqual(errors,[]);
  console.log('flow browser: 3 modes, residual reversal, alternating paths, capacities, limited/exact/zero flow, cut IDs, parallel/self/empty graphs, replay, validation, links and desktop/mobile passed');
} finally {
  const exited=new Promise(resolve=>browser.once("exit",resolve));
  const running=()=>browser.pid && browser.exitCode===null && browser.signalCode===null;
  if(running() && closeBrowser && socket?.readyState===WebSocket.OPEN) {
    await bounded(closeBrowser().catch(()=>{}),3000);
    if(running()) await bounded(exited,3000);
  }
  if(running()){browser.kill("SIGTERM");await bounded(exited,2000);}
  if(running()){browser.kill("SIGKILL");await exited;}
  if(socket)socket.close();
  await fs.promises.rm(temporary,{recursive:true,force:true,maxRetries:10,retryDelay:100});
}
