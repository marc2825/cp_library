// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/heuristic_search_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/heuristic_search/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+value);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram circle,#diagram rect,#diagram path')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const mobile=async()=>{await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await waitFor('document.documentElement.clientWidth===390');await check('document.documentElement.scrollWidth<=391');};
  const desktop=async()=>command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  const modes=['random','timer','annealing','beam','local'],expected=['5, 5, 0, 2, 6, 4','elapsed=2.25, expired=true','best=7 (score=0), iteration_limit','score=5, depth=5, depth_limit','best=2 (score=2), local_optimum'];
  await desktop();
  for(let i=0;i<modes.length;++i){await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);await shot('-'+modes[i]);await mobile();await desktop();}
  await load('random');await set('seed','0');await set('steps','1');await click('run');await click('finish');await check("heuristicSearchState.data.result.outputs[0].raw===16294208416658607535n && document.getElementById('diagram').textContent.includes('16294208416658607535')");await within();await shot('-known-vector');
  await click('boundary');await click('finish');await check("heuristicSearchState.data.input.seed===18446744073709551615n && heuristicSearchState.data.result.outputs.every(x=>x.value<9223372036854775809n) && heuristicSearchState.data.result.draws>4");await within();await evaluate("while(!heuristicSearchState.data.frames[heuristicSearchState.index].rejected)document.getElementById('previous').click()");await check("document.getElementById('step-title').textContent.includes('棄却')");await within();await shot('-rejection');
  await load('timer');await evaluate("while(!heuristicSearchState.data.frames[heuristicSearchState.index].expired)document.getElementById('next').click()");await check("document.getElementById('state').textContent==='0'");await within();await click('boundary');await click('finish');await result('elapsed=0.1, expired=true');await set('limit','2');await set('ticks','1 1');await click('run');await click('finish');await check("heuristicSearchState.data.result.elapsed===2 && heuristicSearchState.data.result.progress===1 && heuristicSearchState.data.result.expired");
  await load('annealing');await evaluate("while(!(heuristicSearchState.data.frames[heuristicSearchState.index].gain<0 && heuristicSearchState.data.frames[heuristicSearchState.index].take))document.getElementById('next').click()");await check("heuristicSearchState.data.frames[heuristicSearchState.index].current!==heuristicSearchState.data.frames[heuristicSearchState.index].best");await within();await shot('-worsening');
  await evaluate("while(!(heuristicSearchState.data.frames[heuristicSearchState.index].gain<0 && !heuristicSearchState.data.frames[heuristicSearchState.index].take))document.getElementById('next').click()");await within();await shot('-sa-reject');await click('finish');await result(expected[2]);await click('run');const replay=await evaluate('JSON.stringify(heuristicSearchState.data.frames)');await click('run');assert.equal(await evaluate('JSON.stringify(heuristicSearchState.data.frames)'),replay);
  await set('goal','maximize');await set('cooling','linear');await click('run');await click('finish');await check("heuristicSearchState.data.input.goal==='maximize' && heuristicSearchState.data.result.best>=0");await within();await click('boundary');await click('finish');await check("heuristicSearchState.data.result.iterations===0 && heuristicSearchState.data.result.evaluations===1");
  await load('beam');await click('next');await check("heuristicSearchState.data.frames[heuristicSearchState.index].candidates.length===2");await within();await click('finish');await check("heuristicSearchState.data.result.frontier.length===3 && new Set(heuristicSearchState.data.result.frontier.map(x=>x.pos)).size<3 && document.querySelector('#diagram .discarded')!==null");await within();await shot('-beam-pruning');
  await set('steps','0');await click('run');await click('finish');await check("heuristicSearchState.data.result.depth===0 && heuristicSearchState.data.result.evaluations===1");await click('boundary');await click('finish');await result('score=5, depth=0, exhausted');await within();
  await load('local');await click('finish');await check("heuristicSearchState.data.result.iterations===3 && heuristicSearchState.data.result.accepted===2");await set('scores','0 3 1');await set('start','1');await set('steps','1');await set('localMode','first');await click('run');await click('finish');await check("heuristicSearchState.data.result.current===0 && heuristicSearchState.data.result.evaluations===2");await set('localMode','best');await click('run');await click('finish');await check("heuristicSearchState.data.result.current===0 && heuristicSearchState.data.result.evaluations===3");await click('boundary');await click('finish');await check("heuristicSearchState.data.result.accepted===0 && heuristicSearchState.data.result.termination==='local_optimum'");await within();
  await load('local');await set('scores',Array.from({length:21},(_,i)=>i%2?-100:100).join(' '));await set('start','20');await click('run');await click('finish');await within();await mobile();await desktop();await load('beam');await set('width','6');await set('steps','8');await click('run');await click('finish');await within();await mobile();await desktop();
  for(const[mode,id,value,message]of [['random','bound','0','正'],['random','seed','18446744073709551616','uint64'],['random','steps','17','範囲外'],['timer','ticks','-1','範囲外'],['timer','ticks','0 '.repeat(21),'20個'],['annealing','startTemperature','0','範囲外'],['annealing','scores','101','範囲外'],['beam','width','0','範囲外'],['beam','steps','9','範囲外'],['local','scores','','1〜21'],['local','start','21','範囲外']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);await click('basic');await check("document.getElementById('input-error').textContent===''");}
  await load('annealing');await click('next');await click('previous');await check('heuristicSearchState.index===0');await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check('heuristicSearchState.index===1');await evaluate("document.querySelector('[data-mode=beam]').click()");await click('finish');await within();await shot('');await mobile();await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check('document.documentElement.scrollWidth<=391');await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('heuristic search browser: 5 modes, 64-bit RNG/rejection, exact/zero deadlines, SA accept/reject/best/replay/goals/cooling, beam duplicates/pruning/exhaustion, local modes/plateaus, invalid inputs, links, SVG and mobile bounds passed');
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
