// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/offline_query_browser_test.mjs --browser /path/to/chromium
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
  let revision=0;
  const load=async mode=>{
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/offline_query/index.html")).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);
  };
  const answers=async expected=>assert.deepEqual(await evaluate("[...document.querySelectorAll('#answers tr')].map(r=>Number(r.dataset.answer))"),expected);
  const screenshotIndex=process.argv.indexOf("--screenshot");
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  await load("mo");await click("finish");await answers([2,4,0,4,4]);
  await set("array","7 7 2");await set("queries","0 2\n1 3\n3 3");await set("width",1);await click("run");await click("finish");await answers([1,2,0]);
  await set("queries","0 4");await click("run");await check("document.getElementById('input-error').textContent.includes('配列長')");
  await set("array","");await set("queries","0 0");await click("run");await click("finish");await answers([0]);
  await evaluate("document.querySelector('[data-mode=pbs]').click()");await waitFor("document.querySelector('[data-mode=pbs]').getAttribute('aria-pressed')==='true'");
  await click("finish");await answers([0,3,4,5]);
  await set("edges","");await set("queries","0 0\n0 1");await click("run");await click("finish");await answers([0,1]);
  await set("queries","0 6");await click("run");await check("document.getElementById('input-error').textContent.includes('頂点数未満')");
  await load("sweep");await click("finish");await answers([3,12,0]);
  await set("queries","1 2 3 3\n3 3 1 2\n1 2 1 3");await click("run");await click("finish");await answers([3,0,0]);
  await set("points","");await click("run");await click("finish");await answers([0,0,0]);
  await set("points","1 2");await click("run");await check("document.getElementById('input-error').textContent.includes('3 個')");
  await load("cdq");await click("scenario");await click("finish");await answers([0,4,3]);
  await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.includes('巻き戻す'))document.getElementById('previous').click()");
  await check("document.getElementById('step-description').textContent.includes('逆元')");
  await set("points","");await set("queries","sum 0 0 2 2\nadd 1 1 9\nsum 0 0 2 2");await click("run");
  await check("document.getElementById('progress').textContent.startsWith('1 /')");await click("finish");await answers([0,9]);
  await set("queries","delete 1 1 9");await click("run");await check("document.getElementById('input-error').textContent.includes('add x y w')");
  for(const mode of ["mo","pbs","sweep","cdq"]){
    await load(mode);await click("scenario");await click("finish");await answers({mo:[2,4,0,4,4],pbs:[0,3,4,5],sweep:[3,12,0],cdq:[0,4,3]}[mode]);
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("window.innerWidth===390 && document.documentElement.scrollWidth<=391");
    await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
    await check("[...document.querySelectorAll('#diagram .axis, #diagram .sweep')].every(p=>getComputedStyle(p).fill==='none')");
    if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,`-${mode}.png`),Buffer.from(screenshot.data,"base64"));}
    await set("queries","");await click("run");await click("finish");await answers([]);
  }
  await load("cdq");await click("scenario");await click("next");await click("previous");
  await check("document.getElementById('progress').textContent.startsWith('1 /')");
  await click("play");await new Promise(resolve=>setTimeout(resolve,650));await click("play");
  await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await click("finish");await answers([0,4,3]);
  await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.startsWith('右の Q'))document.getElementById('previous').click()");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1],Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-mobile.png"),Buffer.from(screenshot.data,"base64"));}
  assert.deepEqual(errors,[]);
  console.log("offline browser: 4 modes, editable input, boundaries, initial/never-true, time order, rollback, replay, invalid input, links and desktop/mobile passed");
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
