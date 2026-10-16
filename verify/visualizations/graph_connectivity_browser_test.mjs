// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/graph_connectivity_browser_test.mjs --browser /path/to/chromium
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
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/graph_connectivity/index.html")).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);
  };
  const result=async value=>check(`document.getElementById('result').textContent===${JSON.stringify(value)}`);
  const change=async(id,value)=>{await set(id,value);await evaluate(`document.getElementById(${JSON.stringify(id)}).dispatchEvent(new Event('change',{bubbles:true}))`);};
  const scenario=async type=>{await change("scenario-type",type);await click("scenario");await click("finish");};
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of ["scc","condensation","lowlink","bridges","articulation","bipartite","components"]){
    await load(mode);await click("finish");await result({scc:"4 成分",condensation:"2",lowlink:"2 橋 / 2 関節",bridges:"2 本",articulation:"2 個",bipartite:"二部",components:"2 成分"}[mode]);
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>getComputedStyle(p).fill==='none')");
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>document.getElementById('kind').value==='有向'?getComputedStyle(p).markerEnd.includes('#arrow'):getComputedStyle(p).markerEnd==='none')");
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot(`-${mode}`);
    await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("window.innerWidth===390 && document.documentElement.scrollWidth<=391");
    await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load("scc");const forward=await evaluate("document.querySelector('#diagram .graph-edge').getAttribute('d')");await click("second-pass");
  await check("document.getElementById('step-title').textContent.includes('逆グラフ')");assert.notEqual(await evaluate("document.querySelector('#diagram .graph-edge').getAttribute('d')"),forward);
  await click("finish");assert.deepEqual(await evaluate("[...document.querySelectorAll('#values tr')].map(r=>Number(r.dataset.component))"),[1,1,1,2,2,3,0]);
  await scenario("oneway");await result("3 成分");
  await load("condensation");await click("finish");await check("document.querySelectorAll('#dag-values tr').length===2");
  assert.deepEqual(await evaluate("[...document.querySelectorAll('#dag-values tr')].map(r=>Number(r.children[3].textContent))"),[4,7]);
  await evaluate("document.getElementById('deduplicate').checked=false");await click("run");await click("finish");await result("3");
  assert.deepEqual(await evaluate("[...document.querySelectorAll('#dag-values tr')].map(r=>Number(r.children[3].textContent))"),[4,3,7]);
  await load("lowlink");await click("finish");
  await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.startsWith('子 4 から親 3'))document.getElementById('previous').click()");
  await check("!document.getElementById('rule').hidden && document.getElementById('rule').textContent.includes('false') && document.getElementById('rule').textContent.includes('true')");await shot("-comparison");
  await load("bridges");await click("inspect");await click("finish");await result("2→3");await check("document.querySelectorAll('#diagram .graph-edge.removed').length===1");
  await set("deletion-id",4);await click("inspect");await click("finish");await result("2→2");
  await set("deletion-id",100);await click("inspect");await check("document.getElementById('input-error').textContent.includes('範囲外')");
  await load("articulation");await click("inspect");await click("finish");await result("2→4");await check("document.querySelectorAll('#diagram .cross').length===1");await shot("-deletion");
  await scenario("root");await result("1 個");await check("document.getElementById('outputs').textContent.includes('関節点: 0')");
  await click("inspect");await click("finish");await result("1→3");
  await set("vertices",1);await set("edges","");await set("deletion-id",0);await click("run");await click("finish");await result("0 個");
  await click("inspect");await click("finish");await result("1→0");
  await load("bipartite");await click("finish");assert.deepEqual(await evaluate("[...document.querySelectorAll('#values tr')].map(r=>Number(r.dataset.color))"),[0,1,0,1,0,1,0]);
  await scenario("odd");await result("非二部");await check("document.querySelectorAll('#diagram .graph-edge.critical').length===1");
  await scenario("parallel");await result("二部");await scenario("loop");await result("非二部");
  for(const mode of ["scc","condensation","lowlink","bridges","articulation","bipartite","components"]){await load(mode);await scenario("empty");await check("document.getElementById('input-error').textContent==='' && document.querySelectorAll('#values tr').length===0");}
  await load("lowlink");await click("next");await set("vertices",2);await set("edges","0 1");await click("run");await check("document.getElementById('progress').textContent.startsWith('1 /')");await click("finish");await result("1 橋 / 0 関節");
  await set("edges","0 2");await click("run");await check("document.getElementById('input-error').textContent.includes('頂点数未満')");
  await set("edges","0 1 2");await click("run");await check("document.getElementById('input-error').textContent.includes('2 個')");
  await load("scc");await click("next");await click("previous");await check("document.getElementById('progress').textContent.startsWith('1 /')");
  await click("play");await new Promise(resolve=>setTimeout(resolve,650));await click("play");await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=bridges]').click()");await waitFor("document.querySelector('[data-mode=bridges]').getAttribute('aria-pressed')==='true'");await click("finish");await result("2 本");
  await load("lowlink");await click("finish");await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.startsWith('子 4 から親 3'))document.getElementById('previous').click()");await shot("");
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await shot("-mobile");
  assert.deepEqual(errors,[]);
  console.log("connectivity browser: 7 modes, reverse DFS, DAG dedup/mapping, strict vs inclusive rule, root/parallel/self cases, deletion, coloring, empty input, replay, links and desktop/mobile passed");
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
