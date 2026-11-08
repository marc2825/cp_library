// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/linear_algebra_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/linear_algebra/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline,#diagram circle,#diagram path,#diagram line')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");

  const modes=['matrix','product','power','gauss','xor'];
  const expected=['3×2 [1, 4] [2, 5] [3, 6]','2×3 [66, 72, 78] [156, 171, 186]','2×2 [89, 55] [55, 34]','自由変数あり / rank=1 / 特解=[4, 0, 0]','7'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(let i=0;i<modes.length;++i){await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);await shot('-'+modes[i]);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('matrix');await seek('A(0,1)を転置');await check("document.querySelector('#diagram .cell.source')!==null && document.querySelector('#diagram .cell.active')!==null");await within();await shot('-transpose');await scenario('empty');await click('finish');await result('3×0 [] [] []');await within();
  await load('product');await seek('k=1の積を加える');await check("document.querySelectorAll('#details tr').length===3 && document.querySelectorAll('#diagram .cell.source').length===2 && document.querySelector('#diagram .cell.active')!==null");await within();await shot('-dot');await scenario('mod12');await click('finish');await result('2×3 [6, 0, 6] [0, 3, 6]');await scenario('mod1');await click('finish');await result('2×3 [0, 0, 0] [0, 0, 0]');
  await load('power');await seek('bit 1を採用');await check("document.getElementById('formula').textContent==='A^10 = A^2 × (A^2)^4'");await within();await shot('-power-bit');await scenario('exponent-zero');await click('finish');await result('2×2 [1, 0] [0, 1]');await scenario('huge');await click('finish');await result('2×2 [0, 1] [1, 0]');await within();await shot('-uint64-exponent');await scenario('one');await set('a','9223372036854775807');await set('exponent','1');await click('run');await click('finish');await result('1×1 [9223372036854775807]');await within();
  await load('gauss');await scenario('swap');await seek('行を交換');await within();await shot('-row-swap');await seek('pivotを1');await within();await seek('行1からpivot行');await check("document.querySelector('#diagram .cell.source')!==null && document.querySelector('#diagram .cell.active')!==null");await within();await shot('-elimination');await click('finish');await result('一意解 / rank=2 / 特解=[1, 2]');
  await load('gauss');await click('finish');await check("!document.getElementById('solution').hidden && document.querySelectorAll('#solution-vectors tr').length===3");await within();await shot('-nullspace');await evaluate("document.getElementById('solution').scrollIntoView({block:'center'})");await shot('-solution-basis');await scenario('inconsistent');await click('finish');await result('解なし / rank=1 / 特解=[]');await within();await shot('-inconsistent');await scenario('mod17');await click('finish');await result('自由変数あり / rank=1 / 特解=[4, 0, 0]');await check("document.querySelector('#solution-vectors').textContent.includes('[15, 1, 0]') && document.querySelector('#solution-vectors').textContent.includes('[14, 0, 1]')");await within();await shot('-prime-field');
  await scenario('tiny');await seek('列0のpivot');await check("document.getElementById('description').textContent.includes('絶対値が最大')");await click('finish');await result('一意解 / rank=2 / 特解=[1, 2]');await scenario('empty');await click('finish');await result('自由変数あり / rank=0 / 特解=[0, 0, 0]');await check("document.querySelectorAll('#solution-vectors tr').length===4");await within();
  await load('xor');await seek('従属なベクトル');await check("document.getElementById('metric').textContent==='2'");await within();await shot('-dependent');await scenario('dependent');await click('finish');await result('true');await scenario('outside');await click('finish');await result('false');await set('operation','min');await click('run');await click('finish');await result('1');await scenario('highest');await seek('bit 63を新しい');await within();await shot('-xor-64');await click('finish');await result('18446744073709551615');await set('operation','min');await click('run');await click('finish');await result('9223372036854775806');await within();
  for(const mode of modes){await load(mode);await scenario('empty');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();}
  for(const [mode,id,value,message]of [['matrix','rows','6','範囲外'],['product','b','1 2','9項'],['power','exponent','-1','範囲外'],['power','exponent','18446744073709551616','範囲外'],['gauss','rhs','1','2項'],['gauss','epsilon','0','epsilon'],['gauss','a','1 2 3 4 5 Infinity','有限'],['xor','vectors','-1','範囲外'],['xor','vectors',Array(13).fill('1').join(' '),'12個']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('gauss');await set('modulus','12');await click('run');await check("document.getElementById('input-error').textContent.includes('素数')");for(const mode of ['product','power']){await load(mode);await scenario('overflow');await check("document.getElementById('input-error').textContent.includes('long long')");}
  await load('product');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent==='C(0,0)が確定'");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");await evaluate("document.querySelector('[data-mode=gauss]').click()");await waitFor("document.querySelector('[data-mode=gauss]').getAttribute('aria-pressed')==='true'");await click('finish');await result('自由変数あり / rank=1 / 特解=[4, 0, 0]');
  await load('product');await seek('k=1の積を加える');await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('linear algebra browser: 5 modes, transpose/dot/power invariants, uint64 exponents, pivot/swap/elimination/nullspace/inconsistency, prime fields, 64-bit XOR, rejection/replay/links and desktop/mobile bounds passed');
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
