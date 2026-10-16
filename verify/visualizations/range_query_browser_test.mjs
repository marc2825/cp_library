// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/range_query_browser_test.mjs --browser /path/to/chromium
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
  const finish=async()=>click("finish");
  const load=async(mode,label)=>{
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/range_query/index.html")).href+"#"+mode});
    await waitFor(`document.readyState==='complete' && document.getElementById('mode-label')?.textContent===${JSON.stringify(label)} && !!document.querySelector('#diagram rect')`);
  };
  const numberAt=async index=>evaluate(`Number(document.querySelectorAll('#values tr')[${index}].children[1].textContent)`);
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await load("fenwick","Fenwick");
  await check("document.querySelectorAll('#values tr').length===8");
  await click("update");await finish();assert.equal(await numberAt(3),9);
  await click("query");await finish();await check("document.getElementById('result').textContent==='25'");
  await click("search");await finish();await check("document.getElementById('step-title').textContent==='lower_bound(15) = 3'");
  await set("value",-100);await click("update");await finish();
  await click("search");await check("document.getElementById('input-error').textContent.includes('非負')");
  await click("scenario");await finish();await check("document.getElementById('result').textContent==='3'");
  await click("previous");await click("next");
  await click("play");await new Promise(resolve=>setTimeout(resolve,850));await click("play");
  await check("document.getElementById('progress').textContent.startsWith('2 /')");await finish();
  await set("target","");await click("search");await check("document.getElementById('input-error').textContent.includes('整数')");
  for(const [mode,label] of [["segtree","Segtree"],["lazy","Lazy"],["dual","Dual"],["range-bit","2-BIT"]]){
    await load(mode,label);await click("scenario");await finish();
    const expected={segtree:"32",lazy:"27",dual:"2","range-bit":"45"}[mode];
    await check(`document.getElementById('result').textContent===${JSON.stringify(expected)}`);
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)")) assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("window.innerWidth===390 && document.documentElement.scrollWidth<=391");
    await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  }
  await load("lazy","Lazy");
  // A query during a partial update must complete that update before continuing.
  await set("left",0);await set("right",8);await click("update");await click("next");
  await set("left",1);await set("right",7);await click("query");await finish();
  await check("document.getElementById('result').textContent==='57'");
  await set("left",6);await set("right",2);await click("update");await check("document.getElementById('input-error').textContent.includes('l ≤ r')");
  await set("initial","1 2.5");await click("reset");await check("document.getElementById('input-error').textContent.includes('整数')");
  await set("initial","1 2 3");await click("reset");await check("document.querySelectorAll('#values tr').length===3");
  await set("left",0);await set("right",3);await set("multiplier",0);await set("offset",2);await click("update");await finish();
  await set("multiplier",1);await set("offset",5);await click("update");await finish();
  await click("query");await finish();await check("document.getElementById('result').textContent==='21'");
  await evaluate("while(!document.getElementById('previous').disabled) document.getElementById('previous').click()");
  await set("point",1);await set("value",9);await click("set");await finish();
  await click("query");await finish();await check("document.getElementById('result').textContent==='13'");
  await load("lazy","Lazy");await click("scenario");
  // Show the initial lazy action at the root, before propagation.
  await click("next");
  const screenshotIndex=process.argv.indexOf("--screenshot");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1],Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await check("document.documentElement.scrollWidth<=391");
  await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-mobile.png"),Buffer.from(screenshot.data,"base64"));}
  assert.deepEqual(errors,[]);
  console.log("range browser: 5 modes, operations, pending updates, replay, branch, validation, links, desktop/mobile passed");
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
