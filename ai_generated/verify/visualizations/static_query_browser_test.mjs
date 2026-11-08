// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/static_query_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/static_query/index.html")).href+"#"+mode});
    await waitFor(`document.readyState==='complete' && !!document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && !!document.querySelector('#diagram text')`);
  };
  const change=async(id,value)=>{await set(id,value);await evaluate(`document.getElementById(${JSON.stringify(id)}).dispatchEvent(new Event('change',{bubbles:true}))`);};
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await load("sparse");await click("query");await finish();
  await check("document.getElementById('result').textContent==='1'");
  await check("document.querySelectorAll('#diagram .range-a,#diagram .range-b').length===2");
  await set("left",2);await set("right",2);await click("query");await finish();await check("document.getElementById('result').textContent==='∞'");
  await click("build");await click("next"); // A query during precomputation must finish the build first.
  await set("left",0);await set("right",8);await click("query");await finish();await check("document.getElementById('result').textContent==='1'");
  await set("left",7);await set("right",2);await click("query");await check("document.getElementById('input-error').textContent.includes('l ≤ r')");
  await load("stack");await set("initial","3 1 1 2");await click("reset");
  for(let i=0;i<3;++i){await click("step");await finish();}
  await check("document.querySelectorAll('#values tr')[2].children[2].textContent==='-1'");
  await change("strict","equal");
  for(let i=0;i<3;++i){await click("step");await finish();}
  await check("document.querySelectorAll('#values tr')[2].children[2].textContent==='1'");
  await change("direction","next");
  for(let i=0;i<4;++i){await click("step");await finish();}
  await check("document.querySelectorAll('#values tr')[1].children[2].textContent==='2' && document.getElementById('step').disabled");
  await change("strict","strict");await change("direction","previous");
  await load("queue");await click("pop");await check("document.getElementById('input-error').textContent.includes('空')");
  for(let i=0;i<3;++i){await click("step");await finish();}
  await click("pop");await finish();await click("query");await finish();
  await check("document.getElementById('result').textContent==='1'"); // Input inherited from the stack test: 3 1 1 2.
  await check("document.querySelectorAll('#values tr')[0].children[0].textContent==='1'");
  await click("pop");await finish();await click("query");await finish();
  await check("document.getElementById('step-title').textContent.includes('index = 2')");
  await click("scenario");await finish();await check("document.getElementById('result').textContent==='1'");
  await load("window");await click("scenario");await finish();
  await check("document.querySelectorAll('#values tr').length===6 && document.querySelectorAll('#values tr')[4].children[2].textContent==='4' && document.querySelectorAll('#values tr')[5].children[2].textContent==='6'");
  await set("width",9);await click("reset");for(let i=0;i<8;++i){await click("step");await finish();}
  await check("document.querySelectorAll('#values tr').length===0");
  await set("width",0);await click("reset");await check("document.getElementById('input-error').textContent.includes('正の整数')");
  await set("width",3);await load("swag");await click("query");await finish();await check("document.getElementById('result').textContent==='ε'");
  await click("scenario");await finish();await check("document.getElementById('result').textContent==='2251'");
  await change("aggregation","sum");await click("scenario");await finish();await check("document.getElementById('result').textContent==='10'");
  await change("aggregation","concat");await click("reset");
  for(let i=0;i<3;++i){await click("step");await finish();}
  await click("pop");await click("next"); // Query while one element has transferred.
  await check("document.getElementById('table-note').textContent.includes('現在の積: 422')");
  await click("query");await finish();await check("document.getElementById('result').textContent==='22'");
  await evaluate("while(!document.getElementById('previous').disabled)document.getElementById('previous').click()");
  await set("value",9);await click("push");await finish();await click("query");await finish();await check("document.getElementById('result').textContent==='9'");
  await set("value","");await click("push");await check("document.getElementById('input-error').textContent.includes('整数')");
  await load("swag_deque");await click("query");await finish();await check("document.getElementById('result').textContent==='ε'");
  for(const id of ["pop","pop-back"]){await click(id);await check("document.getElementById('input-error').textContent.includes('空')");}
  await set("value",2);await click("push-front");await finish();
  await set("value",3);await click("push");await finish();await click("query");await finish();
  await check("document.getElementById('result').textContent==='23'");
  await click("clear");await finish();await click("query");await finish();await check("document.getElementById('result').textContent==='ε'");
  await set("initial","1 2 3 4 5");await click("reset");
  for(let i=0;i<5;++i){await click("step");await finish();}
  await click("pop");await click("next");
  await check("document.getElementById('table-note').textContent.includes('元の論理列') && document.getElementById('table-note').textContent.includes('12345')");
  await click("query");await finish();await check("document.getElementById('result').textContent==='2345'");
  for(let i=0;i<3;++i){await click("pop-back");await finish();}
  await click("query");await finish();await check("document.getElementById('result').textContent==='2'");
  await evaluate("while(!document.getElementById('previous').disabled)document.getElementById('previous').click()");
  await set("value",9);await click("push-front");await finish();await click("query");await finish();await check("document.getElementById('result').textContent==='9'");
  await click("scenario");await finish();await check("document.getElementById('result').textContent==='97'");
  await change("aggregation","sum");await click("scenario");await finish();await check("document.getElementById('result').textContent==='16'");
  await change("aggregation","concat");await set("value",1);
  for(let i=0;i<16;++i){await click("push-front");await finish();}
  await click("push-front");await check("document.getElementById('input-error').textContent.includes('16') && document.getElementById('count').textContent==='16'");
  for(const mode of ["sparse","stack","queue","window","swag","swag_deque"]){
    await load(mode);await click("scenario");await finish();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("window.innerWidth===390 && document.documentElement.scrollWidth<=391");
    await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  }
  await click("scenario");await finish();await click("previous");await click("next");
  await click("play");await new Promise(resolve=>setTimeout(resolve,850));await click("play");
  await check("document.getElementById('progress').textContent.startsWith('2 /')");await finish();
  await set("initial","1 2.5");await click("reset");await check("document.getElementById('input-error').textContent.includes('整数')");
  const screenshotIndex=process.argv.indexOf("--screenshot");
  await load("swag_deque");await click("scenario");
  await evaluate("while(!document.getElementById('next').disabled && document.getElementById('step-title').textContent!=='再構築完了')document.getElementById('next').click()");
  await check("document.getElementById('step-title').textContent==='再構築完了'");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-deque.png"),Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-deque-mobile.png"),Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await load("swag");
  await set("initial","4 2 2 5 1 3 1 6");await click("scenario");
  await evaluate("while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes('back → front'))document.getElementById('next').click()");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1],Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-mobile.png"),Buffer.from(screenshot.data,"base64"));}
  assert.deepEqual(errors,[]);
  console.log("static browser: 6 modes, SWAG Deque ends/rebalancing/clear, replay/branch, validation, links and desktop/mobile passed");
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
