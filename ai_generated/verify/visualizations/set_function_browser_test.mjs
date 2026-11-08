// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/set_function_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/set_function/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline,#diagram circle,#diagram path,#diagram line')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");

  const modes=['submask','supermask','sos','subset-zeta','subset-mobius','superset-zeta','superset-mobius'];
  const expected=['[5, 4, 1, 0]','[2, 3, 6, 7]','[1, 3, 4, 10, 6, 14, 16, 36]','[1, 3, 4, 10, 6, 14, 16, 36]','[1, 2, 3, 4, 5, 6, 7, 8]','[36, 20, 22, 12, 26, 14, 15, 8]','[1, 2, 3, 4, 5, 6, 7, 8]'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(let i=0;i<modes.length;++i){
    await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+modes[i]);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  }
  await load('submask');await seek('次の集合へ');await check("document.getElementById('formula').textContent==='next = (current-1) & mask'");await within();await shot('-submask-next');
  await scenario('zero');await click('finish');await result('[0]');await evaluate("document.getElementById('includeEmpty').checked=false");await click('run');await click('finish');await result('[]');await scenario('no-empty');await click('finish');await result('[5, 4, 1]');await scenario('one');await click('finish');await result('[0]');await within();
  await scenario('highest');await seek('次の集合へ');await within();await shot('-submask-64');await click('finish');await result('[9223372036854775809, 9223372036854775808, 1, 0]');
  await load('supermask');await seek('次の集合へ');await check("document.getElementById('formula').textContent==='next = (current+1) | mask'");await within();await scenario('full');await click('finish');await result('[7]');await scenario('zero');await click('finish');await result('[0, 1, 2, 3, 4, 5, 6, 7]');await scenario('highest');await seek('次の集合へ');await within();await shot('-supermask-64');await click('finish');await result('[18446744073709551612, 18446744073709551613, 18446744073709551614, 18446744073709551615]');
  for(const [mode,title,formula]of [['subset-zeta','f[1]を更新','f[1] += f[0]'],['superset-zeta','f[0]を更新','f[0] += f[1]'],['subset-mobius','f[4]を更新','f[4] -= f[0]'],['superset-mobius','f[0]を更新','f[0] -= f[4]']]){
    await load(mode);await click('next');await check(`document.getElementById('metric').textContent==='${mode.endsWith('mobius')?'2':'0'}'`);await seek(title);await check(`document.getElementById('formula').textContent===${JSON.stringify(formula)}`);await check("document.querySelector('#diagram .cell.source')!==null && document.querySelector('#diagram .cell.active')!==null && document.querySelector('#diagram .wire')!==null");await within();await shot('-'+mode+'-update');await click('finish');await check(`document.querySelectorAll('#terms tr').length===${mode.startsWith('subset')?4:2}`);
  }
  await load('subset-zeta');await set('focus','1');await click('run');await seek('f[1]を更新');await check("document.querySelectorAll('#terms tr').length===2 && document.getElementById('focus-note').textContent.includes('現在の値=3')");await scenario('mod12');await click('finish');await result('[1, 3, 4, 10, 6, 2, 4, 0]');await scenario('mod1');await click('finish');await result('[0, 0, 0, 0, 0, 0, 0, 0]');await scenario('negative');await click('finish');await result('[-1, 1, -4, 2, -6, 2, -16, 4]');
  await set('bits','4');await set('values-input',Array(16).fill('1').join(' '));await click('run');await click('finish');await within();await check("document.querySelectorAll('#values tr').length===16");await shot('-four-bits');
  await load('subset-mobius');await click('finish');await check("[...document.querySelectorAll('#terms tr')].filter(tr=>tr.lastElementChild.textContent==='−').length===2");
  await load('sos');for(const [op,answer]of [['min','[1, 1, 1, 1, 1, 1, 1, 1]'],['max','[1, 2, 3, 4, 5, 6, 7, 8]'],['or','[1, 3, 3, 7, 5, 7, 7, 15]']]){await set('operation',op);await click('run');await click('finish');await result(answer);await check("document.getElementById('modulus-field').hidden");await within();}
  for(const mode of modes){await load(mode);await scenario('one');await click('finish');await result(mode==='submask'||mode==='supermask'?'[0]':'[7]');await within();}
  for(const [mode,id,value,message]of [['submask','mask','8','収まり'],['submask','mask','18446744073709551616','uint64'],['submask','mask','-1','非負'],['submask','bits','-1','範囲外'],['supermask','bits','9','自由なビット'],['subset-zeta','bits','5','4ビット'],['subset-zeta','values-input','1 2 3','8項'],['subset-mobius','focus','8','範囲外'],['subset-zeta','values-input','1 9223372036854775807 0 0 0 0 0 0','途中'],['subset-mobius','values-input','1 0 0 0 -9223372036854775808 0 0 0','途中']]){await load(mode);if(mode==='supermask')await set('mask','0');await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('subset-zeta');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent==='ビット0の段階が完了'");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=supermask]').click()");await waitFor("document.querySelector('[data-mode=supermask]').getAttribute('aria-pressed')==='true'");await click('finish');await result('[2, 3, 6, 7]');
  await load('subset-zeta');await seek('f[5]を更新');await within();await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('set function browser: 7 modes, enumeration order/next/64-bit boundaries, four butterfly directions/inverse bit order, contributions/signs, generic SOS, composite modulus/one, shape/overflow rejection, replay, links and desktop/mobile bounds passed');
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
