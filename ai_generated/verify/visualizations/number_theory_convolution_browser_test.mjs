// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/number_theory_convolution_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/number_theory_convolution/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const definitions=async()=>check("document.getElementById('term-total').textContent.split('：')[1].split(' ')[0]===document.getElementById('result').textContent");
  const modes=['divisor-zeta','divisor-mobius','multiple-zeta','multiple-mobius','dirichlet','gcd','lcm'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){await load(mode);await click('finish');await result({'divisor-zeta':12,'divisor-mobius':2,'multiple-zeta':6,'multiple-mobius':6,dirichlet:4,gcd:6,lcm:125}[mode]);await definitions();await within();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+mode);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('divisor-zeta');await scenario('prime-power');await seek('p=2, k=4:');await check("document.getElementById('description').textContent==='f[8] = 0 + 1 = 1'");await within();await shot('-powers');await click('finish');await result(1);await check("document.getElementById('extra').textContent.includes('f[0] = 99')");
  await load('divisor-mobius');await seek('p=3, k=2:');await check("document.getElementById('description').textContent==='f[6] = 6 − 2 = 4'");await within();await shot('-subtraction');
  await load('multiple-zeta');await set('focus',1);await click('run');await seek('p=2, k=3:');await check("document.getElementById('description').textContent==='f[3] = 3 + 6 = 9'");await click('finish');await result(21);await definitions();
  await load('multiple-mobius');await set('focus',1);await click('run');await click('finish');await result(1);await check("document.getElementById('extra').textContent.includes('f[0] = 99')");
  for(const mode of ['dirichlet','gcd','lcm']){await load(mode);await scenario('truncate');await set('focus',1);await click('run');await click('finish');await result(mode==='gcd'?1:0);await definitions();await scenario('padded');await click('finish');await result(mode==='gcd'?0:1);await definitions();}
  await load('gcd');await seek('添字 6 の要素積');await check("document.getElementById('phase-value').textContent==='要素積' && document.getElementById('result').textContent==='未確定'");await within();await shot('-product');await seek('倍数メビウスを始める');await check("document.querySelectorAll('#diagram .pipeline.active').length===1");await click('finish');await result(6);
  for(const mode of modes){await load(mode);await scenario('composite');await click('finish');await definitions();await within();await set('modulus',1);await click('run');await click('finish');await result(0);await definitions();await scenario('empty');await click('finish');await result('空');await within();await check("document.querySelectorAll('#values tr').length===1");}
  await load('gcd');await set('a','1 2 3');await set('b','4 5');await set('focus',1);await click('run');await click('finish');await result(44);await definitions();await check("document.querySelector('#values tr:nth-child(4) td:nth-child(3)').textContent==='0'");
  await load('lcm');await set('a',Array(12).fill(1).join(' '));await set('b',Array(12).fill(1).join(' '));await set('focus',12);await click('run');await click('finish');await result(15);await definitions();await within();await shot('-twelve');
  for(const mode of modes){await load(mode);await scenario('boundary');await click('finish');await definitions();await within();}await shot('-integer-boundary');
  for(const [mode,id,value,message]of [['divisor-zeta','a','x','十進'],['gcd','b',Array(13).fill(1).join(' '),'12要素'],['lcm','a','9223372036854775808','範囲'],['dirichlet','focus',0,'範囲']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)})`);}
  await load('divisor-zeta');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent.includes('始める')");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=lcm]').click()");await waitFor("document.querySelector('[data-mode=lcm]').getAttribute('aria-pressed')==='true'");await click('finish');await result(125);
  await load('gcd');await seek('p=2, k=2:');await shot('');await evaluate("document.querySelector('.below').scrollIntoView({block:'start'})");await shot('-tables');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('arithmetic convolution browser: 7 modes, prime-power propagation, subtraction order, stages, composite moduli/mod1, empty/mismatched inputs, zero index, truncation/padding, 64-bit coefficients, definitions, replay, validation, links, SVG bounds and desktop/mobile passed');
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
