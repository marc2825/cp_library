// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/dp_optimization_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/dp_optimization/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const toggle=async(id,value)=>evaluate(`document.getElementById(${JSON.stringify(id)}).checked=${value}`);
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const subset=async kind=>{await set('subset-kind',kind);await evaluate("document.getElementById('subset-kind').dispatchEvent(new Event('change'))");};
  const modes=['lis','lcs','edit','zero-one','unbounded','bounded','bitset','interval','digit','subset','rectangle','grid'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){await load(mode);await click('finish');await result({lis:3,lcs:4,edit:3,'zero-one':9,unbounded:10,bounded:10,bitset:7,interval:190,digit:104,subset:16,rectangle:8,grid:4}[mode]);
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot(`-${mode}`);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('lis');await scenario('ties');await click('finish');await result(1);await toggle('strict',false);await click('run');await click('finish');await result(4);await set('input-a','4 3 2 1');await toggle('decreasing',true);await click('run');await click('finish');await result(4);
  await load('lcs');await set('input-a','');await set('input-b','abc');await click('run');await click('finish');await result(0);
  await load('edit');await set('input-a','');await set('input-b','abc');await click('run');await click('finish');await result(3);await check("document.getElementById('details').textContent.includes('Insert')");
  await load('zero-one');await set('items','2 3');await set('capacity',4);await click('run');await click('finish');await result(3);await check("document.querySelector('#values tr:nth-child(5) td:nth-child(3)').textContent==='—'");
  await scenario('zero-weight');await click('finish');await result(7);
  await load('unbounded');await set('items','2 3');await set('capacity',4);await click('run');await click('finish');await result(6);
  await load('bounded');await set('items','2 3 2');await set('capacity',8);await click('run');await seek('期限切れ');await check("document.getElementById('phase-value').textContent==='候補キュー'");await shot('-queue');await click('finish');await result(6);
  await scenario('zero-weight');await click('finish');await result(14);
  await load('bitset');await scenario('word-boundary');await click('finish');await result(7);await check("document.querySelectorAll('#values tr').length===3 && document.getElementById('details').textContent.includes('128')");await shot('-words');await set('capacity',192);await click('run');await click('finish');await check("[...document.querySelectorAll('#diagram rect')].every(r=>Number(r.getAttribute('y'))+Number(r.getAttribute('height'))<=540)");
  await load('interval');await set('input-a','10 20 30');await click('run');await click('finish');await result(90);await check("document.getElementById('details').textContent.includes('[0,1) + [1,2)')");
  await load('digit');await scenario('empty');await click('finish');await result(0);await toggle('include-zero',true);await click('run');await click('finish');await result(1);await set('upper','00100');await set('divisor',1);await click('run');await click('finish');await result(101);await scenario('basic');await seek('桁 2:');await shot('-tight');await evaluate("document.querySelector('.below').scrollIntoView({block:'start'})");await shot('-digit-table');
  await load('subset');await scenario('no-tour');await click('finish');await result('なし');await subset('submasks');await click('finish');await result(8);await check("document.querySelectorAll('#values tr').length===8");await subset('zeta');await click('finish');await result('1, 3, 4, 10');await subset('mobius');await click('finish');await result('1, 2, 3, 4');await set('input-a',Array(32).fill(1).join(' '));await click('run');await click('finish');await shot('-transform');
  await load('rectangle');await scenario('ties');await click('finish');await result(8);
  await load('grid');await scenario('blocked');await click('finish');await result(0);await set('grid','000\n000');await click('run');await click('finish');await result(6);
  for(const mode of modes){await load(mode);await scenario('empty');await click('finish');await result(mode==='subset'?0:mode==='bitset'?1:0);}
  for(const [mode,id,value,message]of [['lis','input-a','x','整数'],['lcs','input-a','あ','ASCII'],['bounded','items','1 2','3個'],['unbounded','items','0 1','重さ'],['bitset','capacity',193,'容量'],['digit','upper','-1','十進'],['digit','divisor',0,'法'],['subset','matrix','1 2','正方'],['rectangle','input-a','-1','配列'],['grid','grid','00\n0','列数']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)})`);}
  await load('subset');await subset('zeta');await set('input-a','1 2 3');await click('run');await check("document.getElementById('input-error').textContent.includes('2冪')");
  await load('lis');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('phase-value').textContent==='更新'");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=edit]').click()");await waitFor("document.querySelector('[data-mode=edit]').getAttribute('aria-pressed')==='true'");await click('finish');await result(3);
  await load('bounded');await seek('剰余 0');await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('DP browser: 12 modes, strictness/direction, alignment, knapsack scan/limits/zero, 64-bit words, interval reconstruction, tight/zero, 4 subset operations, blocked grids, replay, validation, links and desktop/mobile passed');
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
