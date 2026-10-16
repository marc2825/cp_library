// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/ordered_structure_browser_test.mjs --browser /path/to/chromium
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
  const finish=async()=>click("finish");let revision=0;
  const load=async mode=>{
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/ordered_structure/index.html")).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && !!document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && !!document.querySelector('#diagram text')`);
  };
  const change=async(id,value)=>{await set(id,value);await evaluate(`document.getElementById(${JSON.stringify(id)}).dispatchEvent(new Event('change',{bubbles:true}))`);};
  const result=async text=>check(`document.getElementById('result').textContent===${JSON.stringify(text)}`);
  const query=async type=>{await change("query-type",type);await click("query");await finish();};
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await load("set");await check("document.getElementById('size').textContent==='3'");
  await click("insert");await finish();await result("false");
  await set("key",7);await click("insert");await finish();await result("true");
  await set("key",20);await query("predecessor");await result("7");
  await evaluate("document.getElementById('inclusive').checked=true");await query("predecessor");await result("20");
  await evaluate("document.getElementById('inclusive').checked=false");await query("successor");await result("100");
  await click("pop-first");await finish();await result("-5");await click("pop-last");await finish();await result("100");
  await evaluate("document.querySelector('[data-mode=multiset]').click()");await waitFor("document.querySelector('[data-mode=multiset]').getAttribute('aria-pressed')==='true'");
  await click("erase");await finish();await query("count");await result("1");
  await click("erase-all");await finish();await result("1");await query("count");await result("0");
  await set("initial","");await click("reset");await click("pop-first");await finish();await result("該当なし");
  await load("map");await set("key",20);await set("mapped",7);await click("insert");await finish();await result("false");
  await check("document.querySelectorAll('#values tr')[1].children[1].textContent==='200'");
  await click("assign");await finish();await result("updated");
  await check("document.querySelectorAll('#values tr')[1].children[1].textContent==='7'");
  await set("key",7);await query("lookup");await result("該当なし");await check("document.getElementById('size').textContent==='3'");
  await query("bracket");await result("0");await check("document.getElementById('size').textContent==='4'");
  await set("key",8);await query("predecessor");await result("7 → 0");
  await load("compression");await click("build");await finish();await result("3");
  await check("[...document.querySelectorAll('#values tr')].map(r=>r.children[2].textContent).join(',')==='2,0,1,1'");
  await set("key",7);await query("index");await result("該当なし");await query("lower");await result("1");
  await set("key",20);await query("upper");await result("2");
  await change("order","descending");await set("key",7);await query("lower");await result("2");
  await set("initial","");await click("reset");await query("lower");await result("0");
  await load("statistics");await query("kth");await result("20");
  await set("key",7);await query("rank");await result("1");await click("insert");await finish();await result("false");
  await check("document.getElementById('explanation').dataset.tone==='danger' && document.getElementById('size').textContent==='4'");
  await set("key",20);await set("amount",3);await click("erase");await finish();await result("false");
  await check("document.getElementById('size').textContent==='4'");await query("rank_le");await result("3");
  await set("key",0);await set("high",100);await query("range");await result("2");
  await set("key",-5);await set("amount",2);await click("insert");
  await evaluate("while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.startsWith('BIT[1]'))document.getElementById('next').click()");
  await set("rank",3);await query("kth");await result("20"); // Must finish the BIT update before querying.
  await click("clear");await finish();await query("kth");await result("該当なし");
  await check("document.getElementById('size').textContent==='0' && document.getElementById('distinct').textContent==='3'");
  await evaluate("while(!document.getElementById('previous').disabled)document.getElementById('previous').click()");
  await set("key",100);await set("amount",1);await click("insert");await finish();await set("rank",4);await query("kth");await result("100");
  await set("key","");await click("insert");await check("document.getElementById('input-error').textContent.includes('整数')");
  await set("initial","7");await click("reset");await check("document.getElementById('input-error').textContent.includes('登録候補')");
  for(const mode of ["set","multiset","map","compression","statistics"]){
    await load(mode);await click("scenario");await finish();await result({set:"20",multiset:"0",map:"0",compression:"1",statistics:"20"}[mode]);
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("window.innerWidth===390 && document.documentElement.scrollWidth<=391");
    await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  }
  await click("previous");await click("next");await click("play");await new Promise(resolve=>setTimeout(resolve,850));await click("play");
  await check("document.getElementById('progress').textContent.startsWith('2 /')");await finish();
  await load("set");await set("initial",Array.from({length:16},(_,i)=>i).join(' '));await click("reset");await set("key",16);await click("insert");
  await check("document.getElementById('input-error').textContent.includes('16 個')");
  await load("statistics");await click("scenario");await finish();
  await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.startsWith('幅'))document.getElementById('previous').click()");
  const screenshotIndex=process.argv.indexOf("--screenshot");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1],Buffer.from(screenshot.data,"base64"));}
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");
  if(screenshotIndex>=0){const screenshot=await command("Page.captureScreenshot",{format:"png"});fs.writeFileSync(process.argv[screenshotIndex+1].replace(/\.png$/,"-mobile.png"),Buffer.from(screenshot.data,"base64"));}
  assert.deepEqual(errors,[]);
  console.log("ordered browser: 5 modes, duplicates, map side effects, compression bounds, ranks, BIT completion, replay/branch, invalid input, links and desktop/mobile passed");
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
