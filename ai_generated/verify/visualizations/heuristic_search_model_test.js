'use strict';
const assert=require('node:assert/strict');
const M=require('../../visualizations/heuristic_search/model.js');
let cases=0,snapshots=0,rejections=0;
function check(c){const d=M.run(c);cases++;snapshots+=d.frames.length;assert.ok(d.frames.length>=2);const first=structuredClone(d.frames[0]),last=structuredClone(d.frames.at(-1));
 if(d.frames[0].frontier)d.frames[0].frontier[0].path.push(999);if(d.frames[0].outputs)d.frames[0].outputs.push({raw:999n});assert.deepEqual(d.frames.at(-1),last,'snapshots own their arrays');d.frames[0]=first;return d;}
function stream(seed){let state=BigInt(seed);return()=>{state=BigInt.asUintN(64,state+11400714819323198485n);let z=state;z=BigInt.asUintN(64,(z^(z>>30n))*13787848793156543929n);z=BigInt.asUintN(64,(z^(z>>27n))*10723151780598845931n);return z^(z>>31n);};}
function rng(seed){const raw=stream(seed);return{raw,uniform(bound){const cutoff=(2n**64n)%bound;for(;;){const n=raw();if(n>=cutoff)return n%bound;}},real(){return Number(raw()>>11n)*2**-53;}};}
function rawTests(){const known=stream(0);assert.equal(known(),16294208416658607535n);assert.equal(known(),7960286522194355700n);
 for(let i=0;i<200;i++){const seed=(BigInt(i)*999999999999999999n)%2n**64n,bound=[1n,7n,2n**63n+1n,2n**64n-1n][i%4],next=stream(seed),count=i%17;
  const d=check({...M.defaults('random'),seed:String(seed),bound:String(bound),steps:String(count)});let draws=0;const want=[];
  while(want.length<count){const value=next();draws++;if(value>=(2n**64n)%bound)want.push({raw:value,value:value%bound,real:Number(value>>11n)*2**-53});}
  assert.deepEqual(d.result,{outputs:want,draws});for(const f of d.frames){assert.ok(f.value>=0n&&f.value<2n**64n);assert.ok(f.outputs.every(x=>x.value>=0n&&x.value<bound&&x.real>=0&&x.real<1));}
 }
}
function timerTests(){for(let budget=0;budget<=20;budget++)for(let t=0;t<5;t++){const ticks=Array.from({length:10},(_,i)=>(t+i)%5/4),d=check({...M.defaults('timer'),limit:String(budget/4),ticks:ticks.join(' ')});let elapsed=0;
 for(let i=0;i<ticks.length;i++){elapsed+=ticks[i];const f=d.frames[i+1];assert.equal(f.elapsed,elapsed);assert.equal(f.remaining,Math.max(0,budget/4-elapsed));assert.equal(f.expired,elapsed>=budget/4);assert.equal(f.progress,budget===0?1:Math.min(1,elapsed/(budget/4)));}
 assert.equal(d.result.elapsed,elapsed);}}
function localTests(){for(let n=1;n<=6;n++)for(let code=0;code<3**n;code++){let x=code;const scores=Array.from({length:n},()=>{const s=x%3;x=Math.floor(x/3);return s;});
 for(const goal of ['minimize','maximize'])for(const localMode of ['first','best'])for(let start=0;start<n;start++){
  const cap=code%7;let current=start,iterations=0,accepted=0,evaluations=1,termination='iteration_limit';const good=(a,b)=>goal==='minimize'?a<b:a>b;
  for(let i=0;i<cap;i++){let chosen=current;for(const v of [current-1,current+1].filter(v=>v>=0&&v<n)){evaluations++;if(good(scores[v],scores[chosen])){chosen=v;if(localMode==='first')break;}}iterations++;if(chosen===current){termination='local_optimum';break;}current=chosen;accepted++;}
  const d=check({...M.defaults('local'),scores:scores.join(' '),start:String(start),steps:String(cap),goal,localMode});
  assert.deepEqual(d.result,{current,best:current,iterations,accepted,evaluations,termination});let prev=scores[start];for(const f of d.frames.slice(1,-1)){assert.ok(!good(prev,scores[f.current]));prev=scores[f.current];}
 }
}}
function annealingTests(){for(let seed=0;seed<180;seed++)for(const goal of ['minimize','maximize'])for(const cooling of ['linear','geometric']){
 const scores=[4,-1,8,2,-4,3,3,0],start=seed%8,steps=seed%50,random=rng(seed);let current=start,best=start,accepted=0;const want=[];
 for(let i=0;i<steps;i++){const candidate=Math.max(0,Math.min(7,current+(random.uniform(2n)===0n?-1:1))),p=steps>1?i/(steps-1):0,temp=cooling==='linear'?6*(1-p)+0.125*p:Math.exp((1-p)*Math.log(6)+p*Math.log(0.125));
  const gain=(scores[candidate]-scores[current])*(goal==='minimize'?-1:1),draw=gain<0?random.real():null,take=gain>=0||draw<Math.exp(gain/temp);if(take){current=candidate;accepted++;if((scores[current]-scores[best])*(goal==='minimize'?-1:1)>0)best=current;}want.push({current,best,candidate,take,draw,accepted});}
 const d=check({...M.defaults('annealing'),scores:scores.join(' '),start:String(start),seed:String(seed),steps:String(steps),goal,cooling,startTemperature:'6',endTemperature:'0.125'});
 assert.deepEqual(d.result,{current,best,accepted,iterations:steps,evaluations:steps+1,termination:'iteration_limit'});for(let i=0;i<steps;i++){for(const[k,v]of Object.entries(want[i]))assert.equal(d.frames[i+1][k],v);assert.ok(d.frames[i+1].temperature>0);}
}}
function beamTests(){for(let code=0;code<160;code++)for(const goal of ['minimize','maximize'])for(const width of [1,2,4,6]){
 const n=1+code%7,scores=Array.from({length:n},(_,i)=>(code*17+i*i*5)%11-5),start=code%n,steps=code%9;let frontier=[{pos:start,path:[start],score:scores[start]}],depth=0,evaluations=1,termination='depth_limit';const good=(a,b)=>goal==='minimize'?a<b:a>b;
 for(let d=0;d<steps;d++){const candidates=[];for(const parent of frontier)for(const pos of [parent.pos-1,parent.pos+1].filter(x=>x>=0&&x<n))candidates.push({pos,path:[...parent.path,pos],score:scores[pos]});evaluations+=candidates.length;if(!candidates.length){termination='exhausted';break;}
  const next=[];while(candidates.length&&next.length<width){let k=0;for(let i=1;i<candidates.length;i++)if(good(candidates[i].score,candidates[k].score))k=i;next.push(candidates[k]);candidates.splice(k,1);}frontier=next;depth++;}
 const d=check({...M.defaults('beam'),scores:scores.join(' '),start:String(start),steps:String(steps),goal,width:String(width)});assert.equal(d.result.depth,depth);assert.equal(d.result.evaluations,evaluations);assert.equal(d.result.termination,termination);assert.deepEqual(d.result.frontier.map(({pos,path,score})=>({pos,path,score})),frontier);
 for(const f of d.frames){assert.ok(f.frontier.length<=width);assert.ok(f.frontier.length>0);assert.equal(f.candidates.filter(x=>x.kept).length,f.candidates.length?f.frontier.length:0);}
}}
function invalid(){const inputs=[['random','seed','-1'],['random','seed','18446744073709551616'],['random','bound','0'],['random','bound','18446744073709551616'],['random','steps','17'],['random','steps','0.1'],['timer','limit','-1'],['timer','limit','NaN'],['timer','ticks','1 -1'],['timer','ticks','0 '.repeat(21)],['local','scores',''],['local','scores','0 '.repeat(22)],['local','scores','101'],['local','start','21'],['local','goal','bad'],['local','localMode','bad'],['beam','width','0'],['beam','width','7'],['beam','steps','9'],['annealing','seed','xyz'],['annealing','startTemperature','0'],['annealing','endTemperature','Infinity'],['annealing','cooling','bad'],['annealing','steps','101']];
 for(const[mode,key,value]of inputs){assert.throws(()=>M.run({...M.defaults(mode),[key]:value}));rejections++;}assert.throws(()=>M.run({mode:'unknown'}));rejections++;
}
rawTests();timerTests();localTests();annealingTests();beamTests();invalid();
const demo=check(M.defaults('annealing'));assert.equal(demo.result.best,7);assert.ok(demo.frames.some(f=>f.gain<0&&f.take));assert.ok(demo.frames.some(f=>f.gain<0&&!f.take));assert.equal(check(M.defaults('local')).result.best,2);
console.log(`heuristic search model: ${cases} inputs, ${snapshots} snapshots, ${rejections} invalid inputs; independent RNG/timer/SA/beam/local oracles and immutable frames passed`);
