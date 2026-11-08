// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/string_algorithm_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/string_algorithm/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram rect,#diagram polyline,#diagram circle,#diagram path,#diagram line')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");

  const modes=['z','prefix','kmp','hash','trie','suffix','lcp','manacher'];
  const expected=['[12, 1, 0, 0, 3, 1, 0, 0, 2, 2, 1, 0]','[0, 0, 1, 2, 3, 0, 1]','[4]','hash=[490740096, 746433278] / equal=false / LCP=4','size=3','[5, 3, 1, 0, 4, 2]','[1, 3, 0, 0, 2]','odd=[1, 1, 1, 1, 1, 2, 1] / even=[0, 0, 2, 0, 2, 0, 0] / palindrome=true'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(let i=0;i<modes.length;++i){await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);await shot('-'+modes[i]);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('z');await seek('i=5の初期値');await check("document.getElementById('description').textContent.includes('コピー') && document.getElementById('diagram').textContent.includes('[4, 7)')");await within();await shot('-z-reuse');await seek('先頭とsuffixを比較');await check("document.querySelector('#diagram .cell.source')!==null && document.querySelector('#diagram .cell.active')!==null");await within();
  await load('prefix');await seek('3→1');await check("document.getElementById('description').textContent.includes('border')");await within();await shot('-fallback');await click('finish');await result(expected[1]);
  await load('kmp');await scenario('overlap');await click('finish');await result('[0, 2, 4]');await set('s','ab');await set('pattern','');await click('run');await click('finish');await result('[0, 1, 2]');await scenario('no-match');await click('finish');await result('[]');await scenario('fallback');await seek('本文とpatternを比較');await within();await shot('-kmp-compare');
  await load('hash');await seek('区間の先頭分を引く');await check("document.getElementById('formula').textContent.includes('H[r]')");await within();await shot('-hash-subtract');await seek('LCPの長さ');await within();await shot('-hash-probe');await scenario('small-base');await click('finish');await check("document.getElementById('result').textContent.endsWith('equal=false / LCP=4')");await scenario('large-base');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();
  await load('trie');await click('finish');await check("document.querySelectorAll('#details tr').length===9 && document.getElementById('metric').textContent==='3' && document.getElementById('state').textContent==='6'");await within();await shot('-trie-counts');await scenario('trie-erase');await seek('操作8を開始');await within();await shot('-trie-erased');await click('finish');await result('size=0');await check("document.getElementById('state').textContent==='1'");
  await load('suffix');await seek('長さ1の順位');await within();await shot('-suffix-pairs');await seek('順位の組でsuffix');await check("document.getElementById('diagram').textContent.includes('-1')");await within();await shot('-suffix-sort');await click('finish');await result(expected[5]);
  await load('lcp');await seek('suffix 4と2');await check("document.getElementById('description').textContent.includes('再利用')");await within();await shot('-lcp-reuse');await click('finish');await result(expected[6]);
  await load('manacher');await scenario('odd');await seek('中心4の鏡像');await check("document.getElementById('description').textContent.includes('対称') && document.getElementById('formula').textContent.includes('鏡像=2')");await within();await shot('-mirror');await scenario('even');await seek('中心2の半径2');await within();await shot('-even');await click('finish');await result('odd=[1, 1, 1, 1] / even=[0, 0, 2, 0] / palindrome=true');
  for(const mode of modes){await load(mode);await scenario('empty');await click('finish');await check("document.getElementById('input-error').textContent===''");if(mode==='kmp')await result('[0]');else if(mode==='trie')await result('size=0');else if(['z','prefix','suffix','lcp'].includes(mode))await result('[]');await within();await scenario('bytes');await click('finish');await check("document.getElementById('input-error').textContent==='' && document.getElementById('diagram').textContent.includes('FF')");await within();}
  await load('z');await scenario('utf8');await click('finish');await result('[9, 0, 0, 2, 0, 0, 3, 0, 0]');await within();await shot('-utf8');await load('kmp');await scenario('utf8');await click('finish');await result('[0, 6]');
  for(const mode of ['z','suffix','lcp','manacher','hash']){await load(mode);await scenario('maximum');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  for(const [mode,id,value,message]of [['z','s','a'.repeat(33),'32バイト'],['kmp','pattern','a'.repeat(17),'16バイト'],['hash','base','256','base'],['hash','right','12','範囲外'],['hash','left','5','l≤r'],['manacher','right','8','範囲外'],['trie','operations','bad','操作'],['trie','operations','+ aaaaaaaaa','8バイト'],['trie','operations',Array(17).fill('+ a').join('\n'),'16件']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('z');await set('format','hex');await set('s','f');await click('run');await check("document.getElementById('input-error').textContent.includes('hex')");
  await load('z');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('milestone');await check("document.getElementById('step-title').textContent==='z[1]=1が確定'");await click('run');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");await evaluate("document.querySelector('[data-mode=manacher]').click()");await waitFor("document.querySelector('[data-mode=manacher]').getAttribute('aria-pressed')==='true'");await click('finish');await result(expected[7]);
  await load('prefix');await seek('3→1');await within();await shot('');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('string algorithm browser: 8 modes, Z reuse/border fallback/overlap, hash prefixes/subtraction/probes, trie duplicates/erase, doubling ranks/LCP carry, odd/even mirrors, NUL/FF/UTF-8/empty, rejection/replay/links and desktop/mobile bounds passed');
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
