'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {modes,trace}=require('../../visualizations/foundation_structures/model.js');
const expected=JSON.parse(fs.readFileSync(path.join(__dirname,'../foundation_structures/visualization_expected.json'),'utf8'));
for(const mode of modes){const run=trace(mode.id,mode.sample);assert.deepEqual(run.result,expected[mode.id]);assert.equal(run.steps.at(-1).title,'最終結果');}
let random=123456;
const rng=n=>{random=(Math.imul(random,1664525)+1013904223)>>>0;return random%n;};
for(let trial=0;trial<300;trial++){
  const operations=[],map=new Map(),queries=[];
  for(let i=0;i<30;i++){
    const op=['set','get','erase'][rng(3)],key=String(rng(19)-9),value=String(rng(99));
    operations.push(op==='set'?[op,key,value]:[op,key]);
    if(op==='set')map.set(key,value);else if(op==='erase')map.delete(key);else queries.push(map.has(key)?map.get(key):'none');
  }
  const run=trace('hash_map',{seed:String(trial),constantHash:trial%2===0,operations});
  assert.deepEqual(run.result,{entries:[...map].sort((a,b)=>Number(a[0])-Number(b[0])),queries});
  assert.ok(run.steps.every(s=>!s.slots||s.slots.filter(Boolean).length<=s.slots.length/2));
  const n=rng(8),edges=[];if(n)for(let i=0;i<20;i++)edges.push([rng(n),rng(n),rng(99)-49]);
  const directed=trial%2===0,g=trace('csr_graph',{n,directed,edges}).result;
  for(let v=0;v<n;v++){
    const want=[];edges.forEach(([u,w,cost],id)=>{if(u===v)want.push([v,w,cost,id]);if(!directed&&w===v)want.push([v,u,cost,id]);});
    assert.deepEqual(g.arcs.slice(g.offsets[v],g.offsets[v+1]).map(a=>a.slice(0,4)),want);
  }
  g.arcs.forEach((a,i)=>{if(directed)assert.equal(a[4],-1);else{const b=g.arcs[a[4]];assert.equal(b[4],i);assert.equal(b[0],a[1]);assert.equal(b[1],a[0]);}});
  const values=Array.from({length:n},()=>String(rng(21)-10)),coefficients=Array.from({length:3},()=>String(rng(9)-4));
  let l=rng(n+1),r=rng(n+1);if(l>r)[l,r]=[r,l];
  const want=values.map(Number);for(let i=l;i<r;i++)want[i]+=Number(coefficients[0])+Number(coefficients[1])*i+Number(coefficients[2])*i*i;
  assert.deepEqual(trace('polynomial_add_sum',{values,coefficients,l,r}).result,{values:want.map(String),sum:String(want.reduce((a,b)=>a+b,0))});
  let best=n?Number.NEGATIVE_INFINITY:0;for(let i=0;i<n;i++){let sum=0;for(let j=i;j<n;j++){sum+=Number(values[j]);best=Math.max(best,sum);}}
  assert.equal(trace('monoid_catalog',{kind:'subarray',values}).result.best,String(best));
}
for(const [kind,values,want]of[
  ['min',['3','1','1'],{value:'1',count:2}],['max',['3','1','3'],{value:'3',count:2}],
  ['matrix',[['1','1','0','1'],['1','0','2','1']],['3','1','2','1']],
  ['hash',['1','2','3'],{hash:'66',power:'40',length:3}]
])assert.deepEqual(trace('monoid_catalog',{kind,values}).result,want);
assert.equal(trace('monoid_catalog',{kind:'fractional',values:[['0','1','1','0']],x:'0'}).result.atX,'infinity');
assert.equal(trace('monoid_catalog',{kind:'fractional',values:[['0','1','1','0']],x:null}).result.atX,'0');
assert.throws(()=>trace('monoid_catalog',{kind:'fractional',values:[['1','2','2','4']],x:'1'}));
assert.throws(()=>trace('csr_graph',{n:1,directed:false,edges:[[0,1,0]]}));
assert.throws(()=>trace('hash_map',{seed:'0',constantHash:false,operations:[['oops','1']]}));
assert.throws(()=>trace('polynomial_add_sum',{values:['1'],coefficients:['0'],l:1,r:0}));
assert.throws(()=>trace('seeded_hash',{seed:'-1',key:'2'}));
console.log('foundation structures model: 5 C++ reference cases, 1200 independent cases, 7 monoid kinds and invalid input passed');
