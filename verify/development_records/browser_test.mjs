import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';

const index=process.argv.indexOf('--browser');
if(index<0)throw Error('--browser /path/to/chromium is required');
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const report=JSON.parse(fs.readFileSync(path.join(root,'build/catalog/catalog.json'),'utf8'));
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'cp-catalog-browser-'));
const browser=spawn(process.argv[index+1],['--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage','--no-first-run','--remote-debugging-port=0',`--user-data-dir=${temporary}`,'about:blank'],{stdio:['ignore','ignore','pipe']});
let socket,closeBrowser;
try{
  const endpoint=await new Promise((resolve,reject)=>{
    let error='';const timeout=setTimeout(()=>reject(Error(`startup timeout: ${error}`)),10000);
    browser.on('error',reject);browser.on('exit',code=>{clearTimeout(timeout);reject(Error(`Chromium exited ${code}: ${error}`));});
    browser.stderr.on('data',data=>{error+=data;const match=error.match(/DevTools listening on (ws:\/\/\S+)/);if(match){clearTimeout(timeout);resolve(match[1]);}});
  });
  socket=new WebSocket(endpoint);await new Promise((resolve,reject)=>{socket.addEventListener('open',resolve,{once:true});socket.addEventListener('error',reject,{once:true});});
  let id=0;const pending=new Map(),errors=[];
  socket.addEventListener('message',event=>{const m=JSON.parse(event.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails.text);if(m.id){const p=pending.get(m.id);pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}});
  const send=(method,params={},sessionId)=>new Promise((resolve,reject)=>{const next=++id;pending.set(next,{resolve,reject});socket.send(JSON.stringify({id:next,method,params,...(sessionId?{sessionId}:{})}));});
  closeBrowser=()=>send('Browser.close');
  const {targetId}=await send('Target.createTarget',{url:'about:blank'}),{sessionId}=await send('Target.attachToTarget',{targetId,flatten:true});
  const command=(method,params={})=>send(method,params,sessionId);
  await command('Page.enable');await command('Runtime.enable');
  const evaluate=async expression=>{const result=await command('Runtime.evaluate',{expression,returnByValue:true});if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;};
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  await command('Page.navigate',{url:pathToFileURL(path.join(root,'build/catalog/index.html')).href});
  for(let i=0;i<100;i++){if(await evaluate("document.querySelector('#count')?.textContent.includes('件表示')"))break;await new Promise(resolve=>setTimeout(resolve,50));}
  const count=()=>evaluate("document.querySelectorAll('tbody tr:not([hidden])').length");
  assert.equal(await count(),report.features.length);
  for(const status of ['unverified','local-only','online-ac']){
    await evaluate(`document.querySelector('#status').value=${JSON.stringify(status)};document.querySelector('#status').dispatchEvent(new Event('change'))`);
    assert.equal(await count(),report.summary[status]||0);
  }
  await evaluate("document.querySelector('#status').value='';document.querySelector('#status').dispatchEvent(new Event('change'));document.querySelector('#query').value='string_algorithm/suffix_array.hpp';document.querySelector('#query').dispatchEvent(new Event('input'))");
  assert.equal(await count(),1);
  await evaluate("document.querySelector('#query').value='this-path-does-not-exist';document.querySelector('#query').dispatchEvent(new Event('input'))");
  assert.equal(await count(),0);
  await evaluate("document.querySelector('#query').value='range_query/';document.querySelector('#query').dispatchEvent(new Event('input'))");
  const shot=async name=>{const data=await command('Page.captureScreenshot',{format:'png'});fs.writeFileSync(path.join(os.tmpdir(),`cp-catalog-${name}.png`),Buffer.from(data.data,'base64'));};
  await shot('desktop');
  await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
  await evaluate("document.querySelector('#query').value='basic_utility/big_integer.hpp';document.querySelector('#query').dispatchEvent(new Event('input'))");
  assert.equal(await count(),1);
  assert.equal(await evaluate('document.documentElement.scrollWidth<=391'),true);
  await shot('mobile');assert.deepEqual(errors,[]);
  console.log('catalog browser: all counts, three statuses, search, zero matches, 390px layout passed');
}finally{
  if(closeBrowser&&socket?.readyState===WebSocket.OPEN){await Promise.race([closeBrowser(),new Promise(resolve=>setTimeout(resolve,1500))]).catch(()=>{});socket.close();}
  if(browser.exitCode===null)browser.kill('SIGKILL');
  await new Promise(resolve=>setTimeout(resolve,100));fs.rmSync(temporary,{recursive:true,force:true});
}
