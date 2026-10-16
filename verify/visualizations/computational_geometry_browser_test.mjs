// 追加パッケージ不要。Chromium と Node.js 22 以降で UI を確認する。
// node verify/visualizations/computational_geometry_browser_test.mjs --browser /path/to/chromium
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
  const load=async mode=>{await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/computational_geometry/index.html')).href+`?case=${++revision}#${mode}`});await waitFor(`document.readyState==='complete' && document.querySelector('[data-mode="${mode}"][aria-pressed="true"]') && document.querySelector('#diagram text')`);};
  const result=async value=>assert.equal(await evaluate("document.getElementById('result').textContent"),'結果: '+String(value));
  const scenario=async type=>{await set('scenario-type',type);await click('scenario');};
  const seek=async title=>evaluate(`while(!document.getElementById('next').disabled && !document.getElementById('step-title').textContent.includes(${JSON.stringify(title)}))document.getElementById('next').click()`);
  const within=async()=>check("[...document.querySelectorAll('#diagram text,#diagram circle,#diagram polygon,#diagram polyline')].every(e=>{const b=e.getBBox(),v=document.getElementById('diagram').viewBox.baseVal;return b.x>=0&&b.y>=0&&b.x+b.width<=v.width+1&&b.y+b.height<=v.height+1})");
  const modes=['vectors','segments','lines','projection','hull','angle','area','inside','convex','cut','calipers','closest'];
  const expected=['{"dot":"6","cross":"2","ccw":1}','true','{"relation":"unique","point":["3/4","3/4"]}','{"point":["9/5","2/5"],"lineSquared":"2/5","segmentSquared":"2/5"}','[["0","0"],["4","0"],["4","3"],["0","3"]]','[["0","0"],["4","0"],["2","0"],["4","3"],["1","1"],["0","3"]]','{"signedArea2":"24","area":"12"}','inside','inside','{"points":[["3","0"],["4","0"],["4","3"],["0","3"],["0","1"]],"area":"21/2"}','{"squaredDistance":"25","pair":[0,2],"hull":[["0","0"],["4","0"],["4","3"],["0","3"]]}','{"squaredDistance":"2","pair":[0,4]}'];
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
  for(let i=0;i<modes.length;++i){await load(modes[i]);await click('finish');await check("document.getElementById('input-error').textContent===''");await result(expected[i]);await within();for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),`broken link: ${href}`);await shot('-'+modes[i]);await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await waitFor('document.documentElement.clientWidth===390');await check("document.documentElement.scrollWidth<=391");await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});}
  await load('vectors');await scenario('near');await click('finish');await check("geometryState.data.result.ccw===0");await set('epsilon','0');await click('run');await click('finish');await check("geometryState.data.result.ccw===1");await within();await shot('-epsilon');
  await load('segments');await scenario('point-segment');await click('finish');await result('true');await set('line','3 3 3 3');await click('run');await click('finish');await result('false');
  for(const type of ['parallel','coincident']){await load('lines');await scenario(type);await click('finish');await result(JSON.stringify({relation:type}));await within();}
  await load('projection');await set('line','0 0 2 1');await set('query','1 2');await click('run');await seek('直線への垂足');await check("document.getElementById('formula').textContent.includes('9/5')");await within();await shot('-rational-projection');await click('finish');await result('{"point":["8/5","4/5"],"lineSquared":"9/5","segmentSquared":"9/5"}');
  await load('hull');await seek('末尾を除く');await within();await shot('-hull-pop');await evaluate("document.getElementById('keep').checked=true");await click('run');await click('finish');await check("geometryState.data.result.length===5");await scenario('collinear');await click('finish');await check("geometryState.data.result.length===2");
  for(const mode of ['inside','convex'])for(const [type,want]of [['boundary','boundary'],['outside','outside']]){await load(mode);await scenario(type);await click('finish');await result(want);}
  await load('inside');await scenario('concave');await set('query','2 2');await click('run');await click('finish');await result('outside');await load('convex');await seek('二分探索');await within();await shot('-binary-search');
  await load('area');await set('points','0 3\n4 3\n4 0\n0 0');await click('run');await seek('三角形 2');await within();await shot('-signed-area');await click('finish');await result('{"signedArea2":"-24","area":"12"}');
  await load('cut');await scenario('fractional');await seek('切断結果');await check("document.getElementById('formula').textContent.includes('8/3')");await within();await shot('-fractional-cut');await click('finish');await check("geometryState.data.result.area==='8/3'");
  await load('calipers');await seek('対蹠点');await within();await shot('-antipodal');await load('closest');await set('points','0 0\n3 3\n4 3\n1 2\n2 1\n8 8\n6 7\n7 6');await click('run');await seek('境界の帯');await within();await shot('-strip');await click('finish');await check("geometryState.data.result.squaredDistance==='1'");await scenario('near');await click('finish');await check("Number(geometryState.data.result.squaredDistance)>0");
  for(const mode of ['hull','angle','area','inside','convex','cut','calipers','closest'])for(const type of ['empty','single','duplicate']){await load(mode);await scenario(type);await click('finish');await check("document.getElementById('input-error').textContent===''");await within();}
  for(const [mode,id,value,message]of [['hull','points','0.5 0','整数'],['hull','points','1 2 3','x y'],['hull','points','1000001 0','1000000'],['hull','points','0 0\n'.repeat(21),'20個'],['lines','line','0 0 0 0','異なる'],['projection','line','0 0 0 0','異なる'],['cut','line','0 0 0 0','異なる'],['vectors','epsilon','-1','EPS']]){await load(mode);await set(id,value);await click('run');await check(`document.getElementById('input-error').textContent.includes(${JSON.stringify(message)}) && document.getElementById('next').disabled`);}
  await load('hull');await click('next');await click('previous');await check("document.getElementById('progress').textContent.startsWith('1 /')");await click('play');await new Promise(resolve=>setTimeout(resolve,550));await click('play');await check("document.getElementById('progress').textContent.startsWith('2 /')");await evaluate("document.querySelector('[data-mode=cut]').click()");await click('finish');await result(expected[9]);await scenario('fractional');await click('finish');await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});await waitFor('document.documentElement.clientWidth===390');await evaluate("document.querySelector('.stats').scrollIntoView({block:'start'})");await check("document.documentElement.scrollWidth<=391");await within();await shot('-mobile');
  assert.deepEqual(errors,[]);console.log('computational geometry browser: 12 modes, exact fractions / EPS, degenerate segments, parallel/coincident lines, hull pop, area/winding/binary fan, rational cut, antipodal/strip states, empty/single/duplicates, rejections/replay/links and desktop/mobile bounds passed');
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
