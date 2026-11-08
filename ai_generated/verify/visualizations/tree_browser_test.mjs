// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/tree_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{
    await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/tree/index.html')).href+`?case=${++revision}#${mode}`});
    await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);
  };
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of ['rooted','lca','euler','diameter','rerooting','lifting','dp']){
    await load(mode);await click('finish');await result({rooted:'1 成分',lca:0,euler:'13 項',diameter:10,rerooting:'18, 20, 19, 35, 20, 39, 29',lifting:0,dp:41}[mode]);
    await check("[...document.querySelectorAll('#diagram .graph-edge')].every(p=>getComputedStyle(p).fill==='none')");
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot(`-${mode}`);
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('rooted');await set('root',3);await click('run');await click('finish');await check("document.querySelector('#values tr:nth-child(4) td:nth-child(2)').textContent==='-1' && document.querySelector('#values tr:nth-child(4) td:nth-child(4)').textContent==='7'");
  await scenario('forest');await click('finish');await result('4 成分');
  await set('vertices',1);await set('edges','');await click('run');await click('finish');await result('1 成分');
  await load('euler');await seek('3 から 1 に戻る');await check("document.querySelector('#sequences').textContent.includes('walk')");await shot('-return');
  await scenario('forest');await click('finish');await result('10 項');
  await load('lca');await set('source',3);await set('target',4);await set('steps',2);await click('run');await click('finish');await result(1);await check("document.getElementById('step-description').textContent.includes('番目は 4')");
  await scenario('forest');await click('finish');await result(-1);
  await load('diameter');await scenario('zero');await click('finish');await result(0);
  await scenario('negative');await check("document.getElementById('input-error').textContent.includes('非負')");
  await scenario('forest');await check("document.getElementById('input-error').textContent.includes('連結')");
  await load('rerooting');await seek('子 1 を除いて渡す');await check("!document.getElementById('prefix-wrap').hidden && document.querySelectorAll('#prefix-values tr').length===3 && document.getElementById('rule').textContent.includes('prefix[0]')");await shot('-exclude');
  await evaluate("document.getElementById('prefix-wrap').scrollIntoView({block:'start'})");await shot('-prefix');
  await scenario('star');await click('finish');await result('6, 11, 11, 11, 11, 11, 11');
  await scenario('negative');await click('finish');await check("document.getElementById('result').textContent.includes(', ')");
  await load('lifting');await scenario('cycle');await click('finish');await result(1);await check("document.querySelectorAll('#values tr').length===64");
  await set('steps',0);await click('run');await click('finish');await result(3);
  await set('successors','-1 -1 -1 -1 -1 -1 -1');await set('steps',1);await click('run');await click('finish');await result(-1);
  await set('steps','18446744073709551616');await click('run');await check("document.getElementById('input-error').textContent.includes('uint64')");
  await set('steps','-1');await click('run');await check("document.getElementById('input-error').textContent.includes('0以上')");
  await set('steps',2);await set('successors','1 2');await click('run');await check("document.getElementById('input-error').textContent.includes('個数')");
  await scenario('star');await set('steps',1);await click('run');await click('finish');await result(0);
  await scenario('forest');await click('finish');await result(-1);
  await check("document.querySelector('#scenario-type option[value=negative]').disabled && !document.querySelector('#scenario-type option[value=cycle]').disabled");
  await load('dp');await scenario('star');await click('finish');await result(65);
  for(const mode of ['rooted','lca','euler','diameter','rerooting','lifting','dp']){await load(mode);await scenario('empty');await click('finish');await result(mode==='diameter'?0:'空');}
  await load('rooted');for(const [edges,message] of [['0 0','閉路'],['0 1\n1 0','閉路'],['0 7','頂点'],['0 1 2 3','2個'],['0 1 51','重み']]){await set('edges',edges);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)})`);}
  await load('lca');await set('root',7);await click('run');await check("document.getElementById('input-error').textContent.includes('根')");
  await load('rooted');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('phase-value').textContent==='行きがけ'");
  await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=dp]').click()");await waitFor("document.querySelector('[data-mode=dp]').getAttribute('aria-pressed')==='true'");await click('finish');await result(41);
  await load('rerooting');await seek('子 1 を除いて渡す');await shot('');
  await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('tree browser: 7 modes; root changes, forest/empty, tours, LCA/jump, zero diameter, prefix/suffix, 64-bit transitions, replay, input errors, links and desktop/mobile passed');
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
