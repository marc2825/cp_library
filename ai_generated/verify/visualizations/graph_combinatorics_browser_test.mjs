// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/graph_combinatorics_browser_test.mjs --browser /path/to/chromium
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
    await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/graph_combinatorics/index.html')).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);
  };
  const result=async value=>check(`document.getElementById('result').textContent===${JSON.stringify(value)}`);
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');await click('finish');};
  const toggle=async(id,value)=>evaluate(`document.getElementById(${JSON.stringify(id)}).checked=${value}`);
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of ['triangles','cliques','maximum','independent','coloring','trees']){
    await load(mode);await click('finish');await result({triangles:'2 個',cliques:'15 個',maximum:'3 頂点',independent:'3 頂点',coloring:'3 色',trees:'9'}[mode]);
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>getComputedStyle(p).fill==='none')");
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot(`-${mode}`);
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('triangles');await click('milestone');await check("document.getElementById('step-title').textContent==='一つ見つかった'");await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>getComputedStyle(p).markerEnd.includes('#arrow'))");
  await set('stop-after',1);await click('run');await click('finish');await result('1 個');await check("document.getElementById('phase-value').textContent==='打切り'");
  await scenario('bipartite');await result('0 個');await scenario('complete');await result('4 個');
  await load('cliques');await toggle('maximal',true);await click('run');await click('finish');await result('3 個');await check("document.querySelectorAll('#values tr').length===3");
  await toggle('maximal',false);await toggle('include-empty',true);await click('run');await click('finish');await result('16 個');
  await set('stop-after',2);await click('run');await click('finish');await result('2 個');await check("document.getElementById('outputs').textContent.includes('打切り')");
  await scenario('empty');await result('0 個');await toggle('include-empty',true);await click('run');await click('finish');await result('1 個');await toggle('maximal',true);await click('run');await click('finish');await result('1 個');
  await load('maximum');await seek('上界で枝を刈る');await check("document.getElementById('rule').textContent.includes('≤')");await shot('-prune');
  await scenario('complete');await result('4 頂点');await scenario('isolated');await result('1 頂点');
  await load('independent');await click('finish');const original=await evaluate("document.querySelectorAll('#diagram .graph-edge').length");await toggle('show-complement',true);await evaluate("document.getElementById('show-complement').dispatchEvent(new Event('change'))");assert.equal(original,7);await check("document.querySelectorAll('#diagram .complement').length===8");await shot('-complement');await scenario('complete');await result('1 頂点');await scenario('isolated');await result('5 頂点');
  await load('coloring');await click('finish');await evaluate("document.querySelector('.below').scrollIntoView({block:'start'})");await shot('-dp');await check("document.querySelectorAll('#values tr').length===64 && document.getElementById('outputs').textContent.includes('グループ:')");await scenario('cycle');await result('3 色');await scenario('bipartite');await result('2 色');await scenario('complete');await result('4 色');await scenario('isolated');await result('1 色');
  await load('trees');await scenario('multiple');await result('5');await toggle('weighted',true);await click('run');await click('finish');await result('65');
  await set('root',0);await click('run');await click('finish');await result('65');await check("document.querySelectorAll('#full-matrix .removed-cell').length===5");
  await scenario('swap');await result('4');await evaluate("document.getElementById('run').click()");await seek('行 1 と 0 を交換');await check("document.getElementById('phase-value').textContent==='行交換'");await shot('-swap');await evaluate("document.querySelector('.below').scrollIntoView({block:'start'})");await shot('-matrix');await click('finish');
  await scenario('zero');await result('0');await check("document.getElementById('outputs').textContent.includes('連結性の判定に使えません')");
  await set('vertices',1);await set('edges','0 0 7');await set('root','');await click('run');await click('finish');await result('1');
  await scenario('disconnected');await result('0');
  for(const mode of ['triangles','cliques','maximum','independent','coloring','trees']){await load(mode);await scenario('empty');await result({triangles:'0 個',cliques:'0 個',maximum:'0 頂点',independent:'0 頂点',coloring:'0 色',trees:'0'}[mode]);}
  await load('triangles');await set('edges','0 1\n1 0');await click('run');await check("document.getElementById('input-error').textContent.includes('重複')");await set('edges','0 0');await click('run');await check("document.getElementById('input-error').textContent.includes('自己ループ')");await set('edges','0 6');await click('run');await check("document.getElementById('input-error').textContent.includes('頂点')");await set('edges','0 1 2');await click('run');await check("document.getElementById('input-error').textContent.includes('2個')");
  await load('trees');await set('root',6);await click('run');await check("document.getElementById('input-error').textContent.includes('除く頂点')");await set('root','');await set('edges','0 1 100');await click('run');await check("document.getElementById('input-error').textContent.includes('重み')");
  await load('coloring');await click('next');await set('vertices',2);await set('edges','0 1');await click('run');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('finish');await result('2 色');
  await load('triangles');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=maximum]').click()");await waitFor("document.querySelector('[data-mode=maximum]').getAttribute('aria-pressed')==='true'");await click('finish');await result('3 頂点');
  await load('maximum');await seek('上界で枝を刈る');await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('combinatorics browser: 6 modes, enumeration limits/maximal/empty, color bounds, complement, DP restoration, weighted minors, row swaps, modular zero, replay, validation, links and desktop/mobile passed');
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
