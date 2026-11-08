// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/fps_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/fps/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline,#diagram circle,#diagram path,#diagram line')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");

  const modes=['polynomial','calculus','inv','log','exp','pow','sqrt','division','multipoint','interpolation','taylor','bostan'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){
    await load(mode);await click('finish');
    await check("document.getElementById('input-error').textContent==='' && document.getElementById('result').textContent!=='結果: 計算途中'");await within();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+mode);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('polynomial');await set('operation','add');await click('run');await click('finish');await result('[5, 3, 3]');await set('operation','subtract');await click('run');await click('finish');await result('[14, 1, 3]');
  await load('calculus');await set('calculus','integral');await click('run');await click('finish');await result('[0, 1, 1, 1]');await set('calculus','truncate');await click('run');await click('finish');await result('[1, 2, 3, 0, 0, 0]');
  await load('inv');await seek('4項が確定');await check("document.getElementById('precision-value').textContent==='4項' && document.querySelector('#diagram').textContent.includes('f*g')");await within();await shot('-inverse-doubling');await click('finish');await result('[1, 1, 1, 1, 1, 1]');await scenario('invalid-constant');await check("document.getElementById('input-error').textContent.includes('非0') && document.getElementById('next').disabled");await scenario('precision-zero');await click('finish');await result('[]');
  await load('exp');await seek('精度4への補正');await check("document.querySelector('#diagram').textContent.includes('log(g)')");await within();await shot('-exp-correction');await click('finish');await result('[1, 1, 9, 3, 5, 1]');
  await load('pow');await seek('先頭の非0項を分離');await within();await shot('-valuation');await click('finish');await result('[0, 0, 0, 8, 12, 6]');await scenario('large-index');await click('finish');await result('[0, 0, 0, 0, 0, 0]');
  await load('sqrt');await seek('精度4への補正');await within();await shot('-sqrt-newton');await click('finish');await result('[0, 2, 1, 0, 0, 0]');await scenario('nonresidue');await click('finish');await check("document.getElementById('result').textContent==='結果: 平方根なし'");await set('a','0 1');await click('run');await click('finish');await check("document.getElementById('result').textContent==='結果: 平方根なし'");
  await load('division');await set('a','1 2 3 4');await click('run');await seek('商の1次項');await within();await shot('-division-partial');await click('finish');await check("document.getElementById('result').textContent==='商: [3, 16, 4] / 剰余: [15]'");await scenario('zero-divisor');await check("document.getElementById('input-error').textContent.includes('零多項式')");
  await load('multipoint');await seek('節点M2');await check("document.querySelector('#diagram .tree-node.active')!==null && !document.getElementById('tree-details').hidden");await within();await shot('-remainder-tree');await click('finish');await result('[1, 6, 0]');await scenario('duplicate');await click('finish');await result('[6, 6, 0]');
  await load('interpolation');await seek('導関数と重み');await within();await shot('-interpolation-weights');await click('finish');await result('[1, 2, 3]');await scenario('duplicate');await check("document.getElementById('input-error').textContent.includes('相異なる')");
  await load('taylor');await seek('畳み込み');await within();await shot('-taylor-product');await click('finish');await result('[0, 14, 3]');await set('shift','-2');await click('run');await click('finish');await result('[9, 7, 3]');
  await load('bostan');await seek('偶奇の係数を抽出');await within();await shot('-bostan-parity');await click('finish');await result('[4]');await scenario('large-index');await click('finish');await result('[15]');await within();await shot('-bostan-uint64');
  for(const mode of modes){await load(mode);await scenario('large-modulus');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();}
  await load('polynomial');await set('a','2147483648');await click('run');await check("document.getElementById('input-error').textContent.includes('範囲外')");await load('pow');await set('exponent','-1');await click('run');await check("document.getElementById('input-error').textContent.includes('範囲外')");await load('inv');await set('precision','9');await click('run');await check("document.getElementById('input-error').textContent.includes('精度')");
  await load('inv');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent.includes('項が確定')");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=multipoint]').click()");await waitFor("document.querySelector('[data-mode=multipoint]').getAttribute('aria-pressed')==='true'");await seek('節点M2');await within();await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('fps browser: 12 modes, Newton precision, leading zeros, no root, division, product/remainder tree, interpolation weights, Taylor convolution, Bostan parity/uint64, validation, playback, links and desktop/mobile bounds passed');
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
