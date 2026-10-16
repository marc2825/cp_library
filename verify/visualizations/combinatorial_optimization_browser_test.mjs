// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/combinatorial_optimization_browser_test.mjs --browser /path/to/chromium
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
  const screenshotIndex=process.argv.indexOf('--screenshot');let revision=0;
  const shot=async suffix=>{if(screenshotIndex>=0){await evaluate('new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)))');const screenshot=await command('Page.captureScreenshot',{format:'png'});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,suffix+'.png'),Buffer.from(screenshot.data,'base64'));}};
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/combinatorial_optimization/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram circle,#diagram rect,#diagram path')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const mobile=async()=>{await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await waitFor('document.documentElement.clientWidth===390');await check('document.documentElement.scrollWidth<=391');};
  const desktop=async()=>command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  const modes=['hungarian','assignment','matching','bipartite','flow'],expected=['総費用=5 / 列=[1, 0, 2]','総費用=5 / 列=[1, 0, 2]','最大組数=3 / 組=(0,1) (2,5) (3,4)','最大組数=3 / 組=(L0,R1) (L1,R0) (L2,R2)','流量=2/2 / 費用=2'];
  await desktop();
  for(let i=0;i<modes.length;++i){await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);await shot('-'+modes[i]);await mobile();await desktop();}
  await load('hungarian');await seek('slackを更新');await check("document.getElementById('formula').textContent.includes('Δ') && document.querySelector('#diagram .cell.active')!==null");await within();await shot('-slack');await seek('potentialを更新');await within();await shot('-potential');await click('finish');await result(expected[0]);
  await load('assignment');await scenario('rectangular');await click('finish');await result('総費用=9 / 列=[0, 2]');await scenario('maximize');await click('finish');await result('総重み=15 / 列=[1, 0]');await within();await shot('-maximize');await scenario('impossible');await click('finish');await result('割当なし');await scenario('negative');await click('finish');await result('総費用=-14 / 列=[0, 1]');await scenario('large');await click('finish');await check("optimizationState.data.result.cost===-2n");await within();await shot('-wide');
  await set('matrix',Array.from({length:5},(_,i)=>Array.from({length:6},(_,j)=>String(9223372036854775807n-BigInt(i*j))).join(' ')).join('\n'));await click('run');await seek('slackを更新');await within();await click('finish');await within();await mobile();await desktop();
  await load('matching');await seek('Blossomを縮約');await check("document.querySelectorAll('#diagram .base-ring').length===3");await within();await shot('-blossom');await scenario('nested');await seek('Blossomを縮約');await within();await shot('-nested');await click('finish');await check("optimizationState.data.result.cardinality===4");await scenario('odd');await click('finish');await check("optimizationState.data.result.cardinality===2");await scenario('disconnected');await click('finish');await check("optimizationState.data.result.cardinality===2");
  await load('bipartite');await scenario('augment');await seek('二部の交互路を反転');await click('next');await seek('二部の交互路を反転');await check("optimizationState.data.frames[optimizationState.index].path.length===2");await within();await shot('-alternating');await click('finish');await result('最大組数=2 / 組=(L0,R1) (L1,R0)');
  await load('flow');await evaluate("while(!document.getElementById('next').disabled){const f=optimizationState.data.frames[optimizationState.index];if(f.stage==='path'&&f.path.some(id=>id%2))break;document.getElementById('next').click();}");await check("optimizationState.data.frames[optimizationState.index].path.some(id=>id%2) && document.querySelector('#diagram .reverse.active')!==null");await within();await shot('-reverse');await click('finish');await result(expected[4]);await check("document.querySelectorAll('#details tr').length===3");await scenario('shortage');await click('finish');await result('流量=2/3 / 費用=2');await scenario('negative');await click('finish');await result('流量=2/2 / 費用=-2');await scenario('negative-cycle');await click('finish');await result('負閉路を検出');await check("optimizationState.data.frames.every(f=>f.flow.every(x=>x===0))");await within();await shot('-negative-cycle');await scenario('parallel');await click('finish');await result('流量=2/2 / 費用=2');await check("optimizationState.data.result.slope.length===2");await within();await scenario('zero');await click('finish');await result('流量=0/0 / 費用=0');
  for(const mode of ['hungarian','assignment','matching','bipartite']){await load(mode);await scenario('empty');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();}
  for(const [mode,id,value,message]of [['hungarian','matrix','1\n1 2','列数'],['assignment','matrix','0 0 0 0 0 0 0','列は6'],['hungarian','matrix','0.5','費用'],['hungarian','matrix','9223372036854775808','64ビット'],['matching','n','11','0..10'],['matching','edges','0 9','頂点'],['matching','edges','0 1 2','u v'],['flow','edges','0 1 -1 0','容量'],['flow','edges','0 1 1','capacity'],['flow','limit','21','0..20']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('hungarian');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");await evaluate("document.querySelector('[data-mode=matching]').click()");await seek('Blossomを縮約');await within();await shot('');await mobile();await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('optimization browser: 5 modes, Hungarian slack/dual, rectangular/forbidden/min/max/wide costs, Blossom and nested cycles, bipartite alternating paths, reverse rerouting, negative costs/cycles, slopes/shortage/zero, empty/rejection/replay/links and desktop/mobile bounds passed');
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
