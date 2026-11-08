// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/graph_basic_browser_test.mjs --browser /path/to/chromium
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
    await command("Page.navigate",{url:pathToFileURL(path.join(root,"visualizations/graph_basic/index.html")).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);
  };
  const result=async value=>check(`document.getElementById('result').textContent===${JSON.stringify(value)}`);
  const change=async(id,value)=>{await set(id,value);await evaluate(`document.getElementById(${JSON.stringify(id)}).dispatchEvent(new Event('change',{bubbles:true}))`);};
  await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of ["bfs","dfs","zero-one","dijkstra","bellman","floyd","topological","cycle","kruskal","prim","path"]){
    await load(mode);await click("finish");
    await result({bfs:"0→2→3",dfs:"0→1→2→3","zero-one":"0→2→3",dijkstra:"0→2→1→3→4",bellman:"0→1→2→3",floyd:"0→1→2→3",topological:"DAG",cycle:"3 辺",kruskal:"6",prim:"6",path:"0→2→1→3→4"}[mode]);
    if(["kruskal","prim"].includes(mode)){
      await check("document.getElementById('state-value').textContent==='2'");
      await check("document.querySelectorAll('#values tr[data-selected=true]').length===4");
    }
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>getComputedStyle(p).fill==='none')");
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>document.getElementById('kind').value==='directed'?getComputedStyle(p).markerEnd.includes('#arrow'):getComputedStyle(p).markerEnd==='none')");
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot(`-${mode}`);
    if(mode==="floyd"){await evaluate("document.getElementById('matrix-wrap').scrollIntoView({block:'center'})");await shot("-matrix");await evaluate("window.scrollTo(0,0)");}
    await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("window.innerWidth===390 && document.documentElement.scrollWidth<=391");
    await check("document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command("Emulation.setDeviceMetricsOverride",{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load("bfs");await change("scenario-type","disconnected");await click("scenario");await click("finish");await result("空列");
  await check("document.querySelectorAll('#values tr')[5].dataset.distance==='inf'");
  await load("dijkstra");await click("next");await set("vertices",2);await set("edges","0 1 3");await set("target",1);await click("run");
  await check("document.getElementById('progress').textContent.startsWith('1 /')");await click("restore");
  await check("document.getElementById('step-title').textContent==='親配列から経路を復元する'");await click("finish");await result("0→1");
  await check("document.querySelectorAll('#values tr')[1].dataset.distance==='3'");
  await set("edges","0 1 -1");await click("run");await check("document.getElementById('input-error').textContent.includes('非負')");
  await load("zero-one");await set("edges","0 1 2");await click("run");await check("document.getElementById('input-error').textContent.includes('0 または 1')");
  for(const mode of ["bellman","floyd"]){
    await load(mode);await change("scenario-type","negative");await click("scenario");await click("finish");await result("空列");
    assert.deepEqual(await evaluate("[...document.querySelectorAll('#values tr')].map(r=>r.dataset.distance)"),["0","-inf","-inf","-inf","inf","inf"]);
    if(mode==="floyd")await check("document.querySelectorAll('#matrix tr')[4].children[6].textContent==='−∞'");
  }
  await load("topological");await change("scenario-type","cycle");await click("scenario");await click("finish");await result("閉路あり");
  await check("document.getElementById('step-description').textContent.includes('残りすべてが閉路上とは限りません')");
  await load("cycle");await set("vertices",4);await set("edges","0 1\n2 3\n3 2");await click("run");await click("finish");await result("2 辺");
  await evaluate("document.getElementById('scope').checked=true");await click("run");await click("finish");await result("なし");
  await set("source",2);await click("run");await click("finish");await result("2 辺");
  await change("scenario-type","parallel");await click("scenario");await click("finish");await result("2 辺");
  await set("vertices",1);await set("edges","0 0");await click("run");await click("finish");await result("1 辺");
  for(const mode of ["dfs","floyd","topological","cycle","kruskal","prim"]){await load(mode);await set("vertices",0);await set("edges","");await click("run");await click("finish");await check("document.getElementById('input-error').textContent===''");}
  await load("bfs");await set("vertices",0);await set("edges","");await click("run");await check("document.getElementById('input-error').textContent.includes('1 個以上')");
  await set("vertices",3);await set("edges","0 3");await click("run");await check("document.getElementById('input-error').textContent.includes('頂点数未満')");
  await load("dijkstra");await click("next");await click("previous");await check("document.getElementById('progress').textContent.startsWith('1 /')");
  await click("play");await new Promise(resolve=>setTimeout(resolve,650));await click("play");await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=prim]').click()");await waitFor("document.querySelector('[data-mode=prim]').getAttribute('aria-pressed')==='true'");await click("finish");await result("6");
  await load("dijkstra");await click("finish");
  await evaluate("while(!document.getElementById('previous').disabled && !document.getElementById('step-title').textContent.startsWith('辺 #'))document.getElementById('previous').click()");
  await shot("");
  await command("Emulation.setDeviceMetricsOverride",{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await shot("-mobile");
  assert.deepEqual(errors,[]);
  console.log("graph browser: 11 modes, graph editing, direction, negative influence, unreachable/self/parallel edges, scoped cycle, empty graphs, restoration, replay, links and desktop/mobile passed");
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
