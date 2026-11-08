// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/combinatorics_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/combinatorics/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const modes=['factorial','binomial','permutation','combination','catalan','multiset','inclusion'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(const mode of modes){await load(mode);await click('finish');await result({factorial:120,binomial:10,permutation:60,combination:10,catalan:14,multiset:15,inclusion:7}[mode]);await within();
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);
    await shot('-'+mode);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391 && document.querySelector('.drawing').scrollWidth>document.querySelector('.drawing').clientWidth");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('factorial');await seek('逆階乗 5 を作る');await check("document.getElementById('phase-value').textContent==='逆階乗' && document.querySelector('#values tr:nth-child(6) td:nth-child(3)').textContent==='808333339'");await within();await shot('-inverse');await scenario('nonunit');await click('finish');await result(1);await check("document.getElementById('phase-value').textContent==='逆元なし' && document.getElementById('extra').textContent.includes('逆元なし')");
  await load('binomial');await scenario('nonunit');await check("document.getElementById('input-error').textContent.includes('逆元')");await set('backend','pascal');await click('run');await click('finish');await result(3);await within();await shot('-small-prime');
  await scenario('composite');await set('n',6);await set('k',3);await click('run');await click('finish');await result(8);await within();await set('bound',24);await set('n',12);await set('k',6);await click('run');await seek('Pascalの第24行');await within();await shot('-wide-pascal');await click('finish');await result(0);
  await load('permutation');await check("document.getElementById('backend-field').hidden");await click('finish');await check("document.getElementById('samples').textContent.includes('(0, 1, 2)')");await set('k',0);await click('run');await click('finish');await result(1);
  await load('combination');await click('finish');await check("document.getElementById('samples').textContent.includes('{0, 1, 2}')");await set('backend','pascal');await click('run');await click('finish');await result(10);
  await load('catalan');await seek('違反する配置');await check("document.querySelectorAll('#diagram polyline').length===3 && !!document.querySelector('#diagram .reflected')");await within();await shot('-reflection');await scenario('composite');await click('finish');await result(2);await set('n',12);await set('bound',24);await click('run');await click('finish');await result(4);await within();
  await load('multiset');await scenario('zero-types');await click('finish');await result(0);await set('k',0);await click('run');await click('finish');await result(1);await set('n',12);await set('k',12);await set('bound',24);await click('run');await click('finish');await result(1352078);await within();await shot('-stars');
  await load('inclusion');await set('target','complement');await click('run');await click('finish');await result(1);await scenario('overlap');await seek('mask=2:');await check("document.querySelectorAll('#diagram .negative').length===3 && document.getElementById('extra').textContent.includes('-1, -1, -1')");await within();await shot('-negative');await click('finish');await result(0);await set('modulus',12);await click('run');await seek('mask=2:');await check("document.querySelector('#detail-values tr:nth-child(3) td:nth-child(5)').textContent==='9'");
  await set('sets','');await set('target','complement');await click('run');await click('finish');await result(3);await set('target','union');await click('run');await click('finish');await result(0);await check("document.querySelectorAll('#detail-values tr').length===0");
  await set('universe',12);await set('sets','0 1\n2 3\n4 5\n6 7\n8 9\n10 11');await set('modulus',0);await click('run');await click('finish');await result(12);await within();await check("document.querySelectorAll('#detail-values tr').length===63");
  for(const mode of modes){await load(mode);await scenario('empty');await click('finish');await result(mode==='inclusion'?0:1);await within();await scenario('mod-one');await click('finish');await result(0);await within();await scenario('large-mod');await click('finish');await within();await shot('-large-'+mode);}
  for(const [mode,id,value,message]of [['factorial','n','-1','階乗'],['binomial','modulus','0','範囲'],['catalan','bound','2','2*n'],['multiset','bound','1','n+k-1'],['inclusion','sets','8','範囲'],['inclusion','sets','x','整数'],['inclusion','sets',Array(7).fill('-').join('\n'),'6個'],['binomial','modulus','9223372036854775808','範囲']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)})`);}
  await load('factorial');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent==='0! = 1'");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,650));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");
  await evaluate("document.querySelector('[data-mode=catalan]').click()");await waitFor("document.querySelector('[data-mode=catalan]').getAttribute('aria-pressed')==='true'");await click('finish');await result(14);
  await load('inclusion');await seek('mask=3:');await shot('');await evaluate("document.querySelector('.below').scrollIntoView({block:'start'})");await shot('-tables');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('combinatorics browser: 7 modes, factorial/inverse direction, nonunit rejection, Pascal/composite/mod1, ordered/unordered examples, Catalan reflection, stars/bars, empty subsets, negative intermediate weights, 64-bit moduli, replay, validation, links, SVG bounds and desktop/mobile passed');
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
