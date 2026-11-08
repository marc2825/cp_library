// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/convolution_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/convolution/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline,#diagram circle,#diagram path,#diagram line')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");

  const modes=['ntt','convolution','integer','fft-real','fft-complex','convolution-fft'];
  const expected={ntt:'[10, 6, 15, 7]',convolution:'[4, 13, 5, 14, 3]',integer:'[4, 13, 22, 31, 20]','fft-real':'[10+0i, -2-2i, -2+0i, -2+2i]','fft-complex':'[10+2i, 1-1i, -2+0i, -5+3i]','convolution-fft':'[1, -0.5, 1, 6]'};
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){await load(mode);await click('finish');await result(expected[mode]);await within();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+mode);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('ntt');await seek('添字 1 と 2');await check("document.querySelectorAll('#diagram rect.active').length===2");await within();await shot('-bit-reversal');await seek('長さ4 / (1, 3)');await check("document.getElementById('step-title').textContent.includes('(1, 3)') && document.querySelectorAll('#diagram .wire.active').length===4");await within();await shot('-ntt-butterfly');
  await scenario('inverse');await click('finish');await result('[1, 2, 3, 4]');await scenario('mod-large');await click('finish');await within();
  await load('convolution');await scenario('padding');await seek('結果長5');await check("document.getElementById('size').textContent==='8' && document.querySelectorAll('#values tr').length===8");await within();await shot('-padding');await seek('各点積: C[2]');await within();await shot('-pointwise');await click('finish');await result('[4, 13, 11, 10, 1]');await check("document.getElementById('comparison').textContent.includes('= 11')");
  await load('integer');await scenario('integer-limit');await seek('係数0をCRTで復元');await check("document.querySelector('#diagram').textContent.includes('混合基数の桁') && document.querySelector('#diagram').textContent.includes('P を引いて')");await within();await shot('-crt-limit');await click('finish');await result('[-9223372036854775808]');await check("document.getElementById('extra').textContent.includes('9223372036854775808')");await scenario('overflow');await check("document.getElementById('input-error').textContent.includes('2^63')");
  await set('a','-9223372036854775808');await set('b','-1');await click('run');await check("document.getElementById('input-error').textContent.includes('long long')");
  await load('fft-real');await scenario('impulse');await seek('長さ4 / (1, 3)');await check("document.querySelectorAll('#diagram circle.root').length===4 && document.querySelectorAll('#diagram .vector').length===1");await within();await shot('-roots');await click('finish');await result('[1+0i, 0+1i, -1+0i, 0-1i]');await check("document.getElementById('comparison').textContent.includes('conj')");
  await load('fft-complex');await scenario('inverse');await seek('nで割る');await click('finish');await result('[1+0i, 2+0i, 3+0i, 4+0i]');await set('a','1,1 0,0 0,0 0,0');await set('direction','forward');await click('run');await click('finish');await result('[1+1i, 1+1i, 1+1i, 1+1i]');
  await load('convolution-fft');await scenario('complex');await seek('長さ4 / (1, 3)');await within();await shot('-complex-butterfly');await click('finish');await result('[-1+1i, 2+3i, 0-1i, 0+2i]');await check("document.querySelectorAll('#detail-values tr').length===2 && document.getElementById('comparison').textContent.includes('差の大きさ')");
  for(const mode of modes){await load(mode);await scenario('empty');await click('finish');await result('[]');await within();await scenario('one');await click('finish');await result(['convolution','integer'].includes(mode)?(mode==='convolution'?'[11]':'[-6]'):mode==='convolution-fft'?'[-6]':mode==='ntt'?'[2]':mode==='fft-complex'?'[2-1i]':'[2+0i]');await within();}
  for(const [mode,id,value,message]of [['ntt','a','1 2 3','2冪'],['fft-real','a','NaN','実数'],['fft-complex','a','1,Infinity','実部'],['integer','a','9223372036854775808','64ビット'],['convolution','b',Array(9).fill('1').join(' '),'8要素'],['fft-real','a',Array(17).fill('1').join(' '),'16要素']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)})`);}
  await load('ntt');await set('a',Array.from({length:16},(_,i)=>i-8).join(' '));await click('run');await seek('長さ16 / (7, 15)');await within();await shot('-sixteen');await click('finish');await within();
  await load('ntt');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent.includes('順変換の開始')");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=fft-real]').click()");await waitFor("document.querySelector('[data-mode=fft-real]').getAttribute('aria-pressed')==='true'");await seek('長さ4 / (1, 3)');await within();await shot('');await evaluate("document.querySelector('.below').scrollIntoView({block:'start'})");await shot('-tables');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('convolution browser: 6 modes, bit reversal, butterflies, unit roots, zero padding, pointwise products, inverse normalization, CRT/LLONG_MIN, bounds, real/complex inputs, direct terms, replay, validation, links, SVG bounds and desktop/mobile passed');
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
