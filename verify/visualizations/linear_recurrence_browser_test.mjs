// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/linear_recurrence_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/linear_recurrence/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline,#diagram circle,#diagram path,#diagram line')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");

  const modes=['matrix','kitamasa','berlekamp','bostan'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){
    await load(mode);await click('finish');await check("document.getElementById('input-error').textContent===''");
    if(mode==='berlekamp')await check("document.getElementById('result').textContent==='係数c: [1, 1] / a[10]=4'");else await result('[4]');
    await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+mode);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('matrix');await seek('bit 3を採用');await within();await shot('-matrix-bit');await scenario('affine');await seek('状態と遷移');await check("!document.getElementById('matrix-field').hidden && document.getElementById('b-field').hidden");await within();await shot('-affine');await click('finish');await result('[5, 1]');
  await scenario('asymmetric');await click('finish');await result('[15]');await scenario('n-zero');await click('finish');await result('[0]');await scenario('huge');await click('finish');await result('[15]');
  await load('kitamasa');await seek('X^2を還元');await check("document.querySelector('#diagram .cell.active')!==null");await within();await shot('-kitamasa-reduction');await click('finish');await result('[4]');await scenario('composite');await click('finish');await result('[7]');await set('modulus','1');await click('run');await click('finish');await result('[0]');await scenario('asymmetric');await click('finish');await result('[15]');
  await load('berlekamp');await seek('観測1の不一致量');await check("document.getElementById('bm-state').textContent.includes('delta=1') && !document.getElementById('bm-details').hidden");await within();await shot('-bm-discrepancy');await seek('次数を増やして補正');await within();await shot('-bm-order');await scenario('late');await click('finish');await check("document.getElementById('result').textContent==='係数c: [0, 0, 1] / a[10]=0'");await within();await shot('-bm-late');
  await scenario('trailing-zero');await click('finish');await check("document.getElementById('result').textContent==='係数c: [0] / a[10]=0'");await scenario('empty');await click('finish');await check("document.getElementById('result').textContent==='係数c: [] / a[10]=0'");await scenario('huge');await click('finish');await check("document.getElementById('result').textContent==='係数c: [1, 1] / a[18446744073709551615]=15'");
  await load('bostan');await seek('P/Qへ変換');await within();await shot('-generating-function');await seek('偶奇を抽出');await within();await shot('-bostan-parity');await click('finish');await result('[4]');await scenario('improper');await click('finish');await result('[2]');await scenario('q-zero');await check("document.getElementById('input-error').textContent.includes('非0')");await scenario('huge');await click('finish');await result('[15]');await within();await shot('-uint64');
  for(const mode of modes){await load(mode);await scenario('zero');await click('finish');await check(mode==='berlekamp'?"document.getElementById('result').textContent==='係数c: [] / a[10]=0'":"document.getElementById('result').textContent==='結果: [0]'");await within();}
  for(const mode of ['matrix','kitamasa','berlekamp']){await load(mode);await scenario('prime-large');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();if(mode==='berlekamp')await check("document.getElementById('result').textContent.includes('a[10]=55')");else await result('[55]');}
  for(const [mode,id,value,message]of [['matrix','a','1','同じ'],['kitamasa','n','-1','範囲外'],['berlekamp','modulus','12','素数'],['bostan','modulus','1000000007','法は'],['matrix','a','2147483648 1','範囲外'],['berlekamp','a',Array(21).fill('1').join(' '),'20項']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('matrix');await set('source','custom');await set('matrix','1 2\n3');await click('run');await check("document.getElementById('input-error').textContent.includes('正方')");
  await load('berlekamp');await set('a',Array(19).fill('0').concat('1').join(' '));await click('run');await click('finish');await within();await shot('-twenty-observations');
  await load('kitamasa');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('progress').textContent.startsWith('2 /')");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=matrix]').click()");await waitFor("document.querySelector('[data-mode=matrix]').getAttribute('aria-pressed')==='true'");await seek('状態と遷移');await within();await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('linear recurrence browser: 4 modes, column-state powers, affine/composite states, Kitamasa reduction, BM discrepancy/order/trailing zeros, generating P/Q, parity, uint64, rejection, replay, links and desktop/mobile bounds passed');
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
