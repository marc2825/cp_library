// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/game_theory_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/game_theory/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram circle,#diagram rect,#diagram path')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const mobile=async()=>{await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await waitFor('document.documentElement.clientWidth===390');await check('document.documentElement.scrollWidth<=391');};
  const desktop=async()=>command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  const modes=['nim','misere','grundy','dag'],expected=['XOR=2 / 手番側の勝ち / 山0を1へ','手番側の勝ち / 山2を1へ','Grundy=[0, 1, 2, 0, 3, 1]','XOR=1 / 手番側の勝ち / 成分1: 4→2'];
  await desktop();
  for(let i=0;i<modes.length;++i){await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);await shot('-'+modes[i]);await mobile();await desktop();}
  await load('nim');await seek('山 1をXOR');await check("document.getElementById('metric').textContent==='7'");await within();await shot('-xor-prefix');await click('finish');await click('winning-move');await click('finish');await result('XOR=0 / 手番側の負け');await check("document.getElementById('piles').value==='1 4 5' && document.getElementById('winning-move').disabled");
  for(const [type,win,target]of [['ones-even',true,0],['ones-odd',false,null],['one-large-even',true,1],['one-large-odd',true,0]]){await load('misere');await scenario(type);await click('finish');await check(`gameTheoryState.data.result.winning===${win}`);if(target!==null)await check(`gameTheoryState.data.result.move.remaining===${target}n`);await within();}await load('misere');await scenario('one-large-even');await seek('大きい山が一つ');await within();await shot('-endgame');await click('finish');await click('winning-move');await click('finish');await check("document.getElementById('piles').value==='1 1 1' && gameTheoryState.data.result.winning===false");
  await scenario('single');await click('finish');await check("gameTheoryState.data.result.winning===false");await set('move-index','0');await set('move-target','0');await click('apply-move');await click('finish');await result('手番側の勝ち / 合法手なし');await check("document.getElementById('move-note').textContent.includes('最後の石') && document.getElementById('apply-move').disabled");
  await load('nim');await scenario('high-bit');await click('finish');await check("gameTheoryState.data.result.xor===9223372036854775806n && gameTheoryState.data.result.move.remaining===9223372036854775809n");await set('bit-window','48');await evaluate("document.getElementById('bit-window').dispatchEvent(new Event('change'))");await check("document.getElementById('diagram').textContent.includes('63') && document.getElementById('diagram').textContent.includes('18446744073709551615')");await within();await shot('-high-bit');await click('winning-move');await click('finish');await result('XOR=0 / 手番側の負け');
  await load('grundy');await seek('入次数0を集める');await within();await shot('-topology');await seek('値 2は選択肢にある');await check("gameTheoryState.data.frames[gameTheoryState.index].vertex===4");await within();await shot('-mex');await click('finish');await result(expected[2]);await scenario('parallel');await click('finish');await result(expected[2]);await scenario('dense');await click('finish');await result('Grundy=[0, 1, 2, 3, 4, 5, 6, 7]');await within();
  await load('dag');await seek('成分 1のGrundy数をXOR');await check("document.getElementById('description').textContent.includes('一手で一成分')");await within();await shot('-independent-sum');await seek('XORを0にする移動先');await check("document.querySelector('#diagram .wire.active')!==null");await within();await shot('-winning-edge');await click('finish');await click('winning-move');await click('finish');await result('XOR=0 / 手番側の負け');await check("document.getElementById('positions').value==='2 2'");await scenario('duplicate-tokens');await click('finish');await check("gameTheoryState.data.result.winning===false && document.querySelector('#diagram .token')!==null");await scenario('empty-sum');await click('finish');await result('XOR=0 / 手番側の負け');await check("document.getElementById('apply-move').disabled");
  for(const mode of ['grundy','dag'])for(const type of ['cycle','unreachable-cycle']){await load(mode);await scenario(type);await click('finish');await result('閉路を検出：DAGの計算を拒否');await check("document.querySelectorAll('#diagram .blocked').length>=2 && document.getElementById('apply-move').disabled");await within();}await shot('-cycle');
  for(const mode of modes){await load(mode);await scenario('empty');await click('finish');await check("document.getElementById('input-error').textContent===''");await within();await scenario('single');await click('finish');await within();}
  await load('nim');await set('piles','1 2 3 4 5 6 7 8 9 10 11 18446744073709551615');await click('run');await click('finish');await within();await mobile();await desktop();await load('dag');await set('positions',Array(12).fill('4').join(' '));await click('run');await click('finish');await within();await mobile();await desktop();
  for(const [mode,id,value,message]of [['nim','piles','-1','非負'],['nim','piles','0.5','整数'],['nim','piles','18446744073709551616','uint64'],['misere','piles','1 '.repeat(13),'12個'],['grundy','n','11','範囲外'],['grundy','edges','0 6','移動先'],['grundy','edges','0 1 2','from to'],['dag','positions','6','範囲外'],['dag','positions','0 '.repeat(13),'12個']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('nim');await click('finish');await set('move-index','0');await set('move-target','3');await click('apply-move');await check("document.getElementById('move-error').textContent.includes('減ら') && document.getElementById('piles').value==='3 4 5'");await load('dag');await click('finish');await set('move-index','0');await set('move-target','5');await click('apply-move');await check("document.getElementById('move-error').textContent.includes('辺') && document.getElementById('positions').value==='2 4'");
  await load('nim');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");await evaluate("document.querySelector('[data-mode=grundy]').click()");await seek('値 2は選択肢にある');await within();await shot('');await mobile();await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check('document.documentElement.scrollWidth<=391');await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('game theory browser: 4 modes, full 64-bit XOR/window, misere odd/even/last stone/empty, Kahn/mex/DAG sums, legal and winning move application, duplicate tokens/edges, disconnected cycles, rejection/replay/links and desktop/mobile bounds passed');
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
