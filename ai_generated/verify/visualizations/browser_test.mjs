// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/browser_test.mjs --browser /path/to/chromium
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
  const finish=async()=>click("finish");
  const load=async hash=>{
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/union_find/index.html")).href+hash});
    for(let i=0;i<100;++i){if(await evaluate("document.readyState==='complete' && !!document.querySelector('#forest .node')"))return;await new Promise(resolve=>setTimeout(resolve,50));}
    throw new Error("visualization did not initialize");
  };
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await load("#ordinary");
  await check("document.querySelectorAll('#forest .node').length===8");
  await click("scenario");await finish();
  await check("document.getElementById('component-count').textContent==='1'");
  await evaluate("document.querySelector('[data-vertex=\"7\"]').dispatchEvent(new MouseEvent('click',{bubbles:true}))");
  await check("document.getElementById('vertex-u').value==='7'");
  await click("merge");await finish();
  await check("document.getElementById('component-count').textContent==='1'");
  await click("previous");await click("next");
  await set("vertex-count",4);await click("reset");await click("merge");await finish();
  await check("document.getElementById('component-count').textContent==='3'");
  await set("vertex-u",1);await set("vertex-v",2);await click("merge");
  const progress=await evaluate("document.getElementById('progress').textContent");
  await click("play");await new Promise(resolve=>setTimeout(resolve,950));await click("play");
  assert.notEqual(await evaluate("document.getElementById('progress').textContent"),progress);
  await finish();await check("document.getElementById('component-count').textContent==='2'");
  await click("potential-tab");
  await check("location.hash==='#potential' && !document.getElementById('weight-field').hidden");
  async function merge(u,v,w){await set("vertex-u",u);await set("vertex-v",v);await set("difference",w);await click("merge");await finish();}
  await merge(0,1,5);await merge(1,2,-2);
  await set("vertex-u",0);await set("vertex-v",2);await click("diff");await finish();
  await check("document.getElementById('step-title').textContent==='diff(0, 2) = 3'");
  await merge(0,2,3);
  await check("document.getElementById('step-description').textContent.includes('AlreadyConsistent')");
  await merge(0,2,4);
  await check("document.getElementById('explanation').dataset.tone==='danger' && document.getElementById('component-count').textContent==='4'");
  await set("difference","");await click("merge");
  await check("document.getElementById('input-error').textContent.includes('整数')");
  await click("scenario");await finish();
  await check("document.getElementById('step-title').textContent.includes('まだ分かりません')");
  await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.includes('矛盾')) document.getElementById('previous').click()");
  await check("document.getElementById('step-title').textContent.includes('矛盾')");
  for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)")) {
    assert.ok(fs.existsSync(fileURLToPath(href)),`broken local link: ${href}`);
  }
  const screenshotIndex=process.argv.indexOf("--screenshot");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1],Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await check("window.innerWidth===390");
  await check("document.documentElement.scrollWidth<=window.innerWidth+1");
  await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
  await check("document.querySelectorAll('#values tr').length===6");
  await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-mobile.png"),Buffer.from(screenshot.data,"base64"));}
  assert.deepEqual(errors,[]);
  console.log("browser: mode switch, operations, replay, contradiction, inputs, links, desktop/mobile passed");
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
