import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';

const browserIndex=process.argv.indexOf('--browser');
if(browserIndex<0)throw Error('--browser /path/to/chromium is required');
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../..');
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'cp-utility-search-browser-'));
const browser=spawn(process.argv[browserIndex+1],['--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage','--no-first-run','--disable-breakpad','--remote-debugging-port=0',`--user-data-dir=${temporary}`,'about:blank'],{stdio:['ignore','ignore','pipe']});
let socket,closeBrowser;
try{
  const endpoint=await new Promise((resolve,reject)=>{
    let error='';const timeout=setTimeout(()=>reject(Error(`Chromium startup timeout: ${error}`)),10000);
    browser.on('error',reject);browser.on('exit',code=>{clearTimeout(timeout);reject(Error(`Chromium exited ${code}: ${error}`));});
    browser.stderr.on('data',data=>{error+=data;const match=error.match(/DevTools listening on (ws:\/\/\S+)/);if(match){clearTimeout(timeout);resolve(match[1]);}});
  });
  socket=new WebSocket(endpoint);await new Promise((resolve,reject)=>{socket.addEventListener('open',resolve,{once:true});socket.addEventListener('error',reject,{once:true});});
  let id=0;const pending=new Map(),errors=[];
  socket.addEventListener('message',event=>{const m=JSON.parse(event.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails.text);if(m.id){const r=pending.get(m.id);pending.delete(m.id);m.error?r.reject(Error(JSON.stringify(m.error))):r.resolve(m.result);}});
  const send=(method,params={},sessionId)=>new Promise((resolve,reject)=>{const next=++id;pending.set(next,{resolve,reject});socket.send(JSON.stringify({id:next,method,params,...(sessionId?{sessionId}:{})}));});
  closeBrowser=()=>send('Browser.close');
  const {targetId}=await send('Target.createTarget',{url:'about:blank'}),{sessionId}=await send('Target.attachToTarget',{targetId,flatten:true});
  const command=(method,params={})=>send(method,params,sessionId);await command('Page.enable');await command('Runtime.enable');
  const evaluate=async expression=>{const r=await command('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
  const check=async expression=>assert.equal(await evaluate(expression),true,expression);
  const click=async id=>evaluate(`document.getElementById(${JSON.stringify(id)}).click()`);
  const wait=async expression=>{for(let i=0;i<100;i++){if(await evaluate(expression))return;await new Promise(resolve=>setTimeout(resolve,50));}throw Error(`timeout: ${expression}`);};
  const shot=async suffix=>{const i=process.argv.indexOf('--screenshot');if(i>=0){const screenshot=await command('Page.captureScreenshot',{format:'png'});fs.writeFileSync(process.argv[i+1].replace(/\.png$/,`-${suffix}.png`),Buffer.from(screenshot.data,'base64'));}};
  const entries=JSON.parse(fs.readFileSync(path.join(root,'verify/utility_search/manifest.json'),'utf8')).algorithms;
  const expected=JSON.parse(fs.readFileSync(path.join(root,'verify/utility_search/visualization_expected.json'),'utf8'));
  await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  for(const {mode} of entries){
    await command('Page.navigate',{url:pathToFileURL(path.join(root,'visualizations/utility_search/index.html')).href+'#'+mode});
    await wait(`document.readyState==='complete'&&document.getElementById('mode')?.value===${JSON.stringify(mode)}&&document.getElementById('values')?.children.length>0`);
    await check("document.getElementById('input-error').textContent===''");
    assert.deepEqual(JSON.parse(await evaluate("document.getElementById('result').textContent")),expected[mode]);
    await click('finish');await check("document.getElementById('step-title').textContent==='最終結果'");
    await click('previous');await click('first');await check("document.getElementById('previous').disabled");
    await click('next');await check("!document.getElementById('previous').disabled");
    await click('play');await new Promise(resolve=>setTimeout(resolve,750));await click('play');
    for(const href of await evaluate("[...document.querySelectorAll('a[href]')].map(a=>a.href)"))assert.ok(fs.existsSync(fileURLToPath(href)),href);
    await evaluate("document.getElementById('input').value='{' ");await click('run');
    await check("document.getElementById('input-error').textContent.length>0");await click('sample');
    await check("document.getElementById('input-error').textContent===''");
    if(['two_pointers','permutation_tree','parentheses','retroactive_operations','special_integer_enumeration'].includes(mode)){await click('first');for(let i=0;i<8&&!(await evaluate("document.getElementById('diagram').children.length>0"));i++)await click('next');await check("document.getElementById('diagram').children.length>0");await shot(mode);}
    await command('Emulation.setDeviceMetricsOverride',{width:390,height:844,deviceScaleFactor:1,mobile:true});
    await check('document.documentElement.scrollWidth<=391');
    if(mode==='partial_persistent_search'||mode==='retroactive_operations'){await click('finish');await evaluate("document.querySelector('.viewer').scrollIntoView()");await shot(`${mode}-mobile`);}
    await command('Emulation.setDeviceMetricsOverride',{width:1280,height:1000,deviceScaleFactor:1,mobile:false});
  }
  for(const [mode,input,want] of [
    ['two_pointers',{values:[0,0],limit:0},{right:[2,2],longest:[0,2],count:3}],
    ['maximum_subarray',{values:[-5,-2],allowEmpty:false},{sum:-2,left:1,right:2}],
    ['dynamic_mex',{operations:[['add',0],['add',0],['erase',0]]},{mex:1,size:1,history:[1,1,1]}],
    ['meet_in_the_middle',{values:[],target:-1},{best:null,countEqual:0,countLeq:0}],
    ['permutation',{values:[1,0],exponent:0},{inverse:[1,0],cycles:[[0,1]],sign:-1,power:[0,1]}],
    ['unimodal_search',{method:'fibonacci',left:4,right:5,center:0},{position:4,value:4}],
    ['collection_hash',{multi:false,seed:'42',operations:[['add','3'],['erase','3']]},{seed:'42',size:0,first:'0',second:'0'}],
    ['permutation_tree',{values:[]},{count:0,nodes:[]}],
    ['parentheses',{pairs:0,rank:'0'},{count:'1',text:'',rank:'0',children:[[]]}],
    ['integer_partitions',{n:0,limit:2},{count:'1',partitions:[[]],more:false}],
    ['set_partitions',{n:0,limit:2},{count:'1',partitions:[[]],more:false}],
    ['partial_persistent_search',{initial:[0],updates:[],index:0,threshold:1},{time:0,first:null,history:[[0]]}],
    ['retroactive_operations',{times:[1,2],initial:3,query:1,operations:[['set',2,2,0],['set',1,0,7]]},{size:2,before:'3',through:'7',current:'14',history:['6','14']}],
    ['special_integer_enumeration',{kind:'palindrome',first:'18446744066044764481',last:'18446744073709551615',limit:10},{values:['18446744066044764481'],more:false}],
    ['special_integer_enumeration',{kind:'digits',first:0,last:25,digits:[2,0,2],limit:3},{values:['0','2','20'],more:true}],
    ['special_integer_enumeration',{kind:'products',first:0,last:20,factors:[2,3,4],limit:100},{values:['1','2','3','4','6','8','9','12','16','18'],more:false}]
  ]){
    await evaluate(`document.getElementById('mode').value=${JSON.stringify(mode)};document.getElementById('mode').dispatchEvent(new Event('change'))`);
    await evaluate(`document.getElementById('input').value=${JSON.stringify(JSON.stringify(input))}`);await click('run');
    await check("document.getElementById('input-error').textContent===''");
    assert.deepEqual(JSON.parse(await evaluate("document.getElementById('result').textContent")),want);
  }
  await evaluate("document.getElementById('mode').value='parentheses';document.getElementById('mode').dispatchEvent(new Event('change'));document.getElementById('input').value=JSON.stringify({pairs:2,rank:'2'})");await click('run');
  await check("document.getElementById('input-error').textContent.length>0");
  assert.deepEqual(errors,[]);
  console.log('utility search browser: 14 modes, 16 edited inputs, editable input, navigation, autoplay, errors, local links and 390px layout passed');
}finally{
  if(closeBrowser&&socket?.readyState===WebSocket.OPEN){await Promise.race([closeBrowser(),new Promise(resolve=>setTimeout(resolve,1500))]).catch(()=>{});socket.close();}
  if(browser.exitCode===null)browser.kill('SIGKILL');
  await new Promise(resolve=>setTimeout(resolve,100));fs.rmSync(temporary,{recursive:true,force:true});
}
