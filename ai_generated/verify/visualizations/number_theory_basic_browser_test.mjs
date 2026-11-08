// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/number_theory_basic_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/number_theory_basic/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const modes=['gcd','extended','pow','inverse','sieve','spf','factor','divisors'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){await load(mode);await click('finish');await result({gcd:6,extended:6,pow:808,inverse:7,sieve:12,spf:'2^2 × 3^2',factor:'2^3 × 3^2 × 5^1',divisors:9}[mode]);await within();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+mode);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('gcd');await scenario('negative');await click('finish');await result(6);await check("document.getElementById('extra').textContent==='LCM = 90'");await scenario('boundary');await click('finish');await result(1);await check("document.getElementById('extra').textContent.includes('範囲超過')");await within();await shot('-overflow');
  await load('extended');await scenario('boundary');await click('finish');await within();await check("document.getElementById('extra').textContent==='x = -1, y = -1'");await shot('-coefficients');
  await load('pow');await scenario('boundary');await click('finish');await result('9223372036854775806');await within();await check("document.querySelectorAll('#values tr').length===63");await shot('-bits');
  await load('inverse');await scenario('non-coprime');await click('finish');await result('なし');await scenario('negative');await click('finish');await result(4);await scenario('boundary');await click('finish');await result('9223372036854775806');await within();
  await load('sieve');await seek('素数 3');await check("document.querySelectorAll('#diagram rect.active').length>0");await shot('-sieve-step');await scenario('boundary');await click('finish');await result(22);await within();
  await load('spf');await seek('最小素因数 2');await check("document.getElementById('phase-value').textContent==='分解' && document.getElementById('details').textContent.includes('36 ÷ 2^2 → 9')");await shot('-spf-route');await scenario('boundary');await click('finish');await result('2^3 × 3^2');await within();
  await load('factor');await set('n',2310);await click('run');await click('finish');await result('2^1 × 3^1 × 5^1 × 7^1 × 11^1');await within();await shot('-factors');
  await load('divisors');await scenario('boundary');await click('finish');await result(3);await check("document.getElementById('details').textContent==='昇順: 1, 97, 9409'");await within();await set('n',9240);await click('run');await click('finish');await result(64);await within();await shot('-many-divisors');
  for(const mode of modes){await load(mode);await scenario('zero');await click('finish');await result({gcd:0,extended:0,pow:0,inverse:0,sieve:0,spf:'分解なし',factor:'1',divisors:1}[mode]);await within();}
  for(const [mode,id,value,message]of [['gcd','a','x','十進'],['gcd','a','9223372036854775808','範囲'],['pow','exponent','-1','範囲'],['pow','modulus','0','範囲'],['inverse','modulus','-1','範囲'],['sieve','limit','81','範囲'],['spf','query','41','範囲'],['factor','n','0','範囲'],['divisors','n','10001','範囲']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)})`);}
  await load('gcd');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await result(6);await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=pow]').click()");await waitFor("document.querySelector('[data-mode=pow]').getAttribute('aria-pressed')==='true'");await click('finish');await result(808);
  await load('spf');await seek('i = 12,');await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('number theory browser: 8 modes, signed/zero/64-bit inputs, overflow, missing inverse, 63 exponent bits, sieve marking, SPF route, divisor pairs, replay, validation, links, SVG bounds and desktop/mobile passed');
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
