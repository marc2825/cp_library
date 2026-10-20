'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const model=require('../../visualizations/utility_search/model.js');
const expected=require('../utility_search/visualization_expected.json');
const result=(mode,input)=>model.trace(mode,input).result;
for(const mode of model.modes){const trace=model.trace(mode.id,mode.sample);assert.deepEqual(trace.result,expected[mode.id]);assert.equal(trace.steps.at(-1).title,'最終結果');assert.ok(trace.steps.length>=2);assert.ok(fs.existsSync(path.resolve(__dirname,'../../docs',mode.doc)));}
let seed=20261020;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
for(let t=0;t<250;t++){
  const a=Array.from({length:random(12)},()=>random(8)),limit=random(20),right=[],longest=[0,0];let count=0;
  for(let l=0;l<a.length;l++){let sum=0,r=l;for(let end=l;end<a.length;end++){sum+=a[end];if(sum<=limit){r=end+1;++count;}}right.push(r);if(r-l>longest[1]-longest[0]){longest[0]=l;longest[1]=r;}}
  assert.deepEqual(result('two_pointers',{values:a,limit}),{right,longest,count});
  const b=a.map(x=>x-4);for(const allowEmpty of [true,false]){let best=allowEmpty?{sum:0,left:0,right:0}:null;for(let l=0;l<b.length;l++)for(let r=l+1;r<=b.length;r++){const sum=b.slice(l,r).reduce((x,y)=>x+y,0);if(!best||sum>best.sum)best={sum,left:l,right:r};}assert.deepEqual(result('maximum_subarray',{values:b,allowEmpty}),best);}
  const target=random(15)-7;let best=null,countEqual=0,countLeq=0;for(let mask=0;mask<2**b.length;mask++){const sum=b.reduce((s,v,i)=>s+((mask>>i&1)?v:0),0);if(sum===target)++countEqual;if(sum<=target){++countLeq;if(!best||sum>best.sum)best={sum,mask};}}
  assert.deepEqual(result('meet_in_the_middle',{values:b,target}),{best,countEqual,countLeq});
}
assert.deepEqual(result('dynamic_mex',{operations:[['erase',0],['add',0],['add',0],['erase',0],['add',1]]}),{mex:2,size:2,history:[0,1,1,1,2]});
for(let n=1;n<=35;n++)for(let center=-1;center<=n;center++){const got=result('unimodal_search',{method:'fibonacci',left:0,right:n,center});const position=Math.max(0,Math.min(n-1,center));assert.deepEqual(got,{position,value:Math.abs(position-center)});}
for(const center of [-12,-3.5,0,2.5,12]){const got=result('unimodal_search',{method:'golden',left:-10,right:10,center,iterations:80});assert.ok(Math.abs(got.position-Math.max(-10,Math.min(10,center)))<1e-10);}
for(let n=0;n<=7;n++){
  const all=[];for(let mask=0;mask<2**(2*n);mask++){let balance=0,valid=true,text='';for(let i=0;i<2*n;i++){const open=mask>>i&1;balance+=open?1:-1;text+=open?'(':')';if(balance<0)valid=false;}if(valid&&!balance)all.push(text);}all.sort();
  for(let rank=0;rank<all.length;rank++){const got=result('parentheses',{pairs:n,rank:String(rank)});assert.equal(got.text,all[rank]);assert.equal(got.count,String(all.length));assert.equal(got.children.length,n+1);}
}
for(let n=0;n<=12;n++){
  const expected=[],path=[];function generate(rem,cap){if(!rem){expected.push([...path]);return;}for(let i=Math.min(rem,cap);i;i--){path.push(i);generate(rem-i,i);path.pop();}}generate(n,n);
  const got=result('integer_partitions',{n,limit:100});assert.deepEqual(got.partitions,expected.slice(0,100));assert.equal(got.count,String(expected.length));assert.equal(got.more,expected.length>100);
}
for(let n=0;n<=6;n++){
  const expected=[],path=[];function generate(i,blocks){if(i===n){expected.push([...path]);return;}for(let b=0;b<=blocks;b++){path.push(b);generate(i+1,Math.max(blocks,b+1));path.pop();}}generate(0,0);
  const got=result('set_partitions',{n,limit:100});assert.deepEqual(got.partitions,expected.slice(0,100));assert.equal(got.count,String(expected.length));assert.equal(got.more,expected.length>100);
}
for(const multi of [true,false]){
  const operations=[['add','-9223372036854775808'],['add','9223372036854775807'],['add','0'],['add','0']];
  assert.deepEqual(result('collection_hash',{multi,seed:'18446744073709551615',operations}),result('collection_hash',{multi,seed:'18446744073709551615',operations:[...operations].reverse()}));
  const empty=result('collection_hash',{multi,seed:'42',operations:[['add','7'],['erase','7'],['erase','7']]});assert.equal(empty.size,0);assert.equal(empty.first,'0');assert.equal(empty.second,'0');
}
assert.deepEqual(result('permutation',{values:[1,2,0],exponent:'-9223372036854775808'}).power,[1,2,0]);
assert.deepEqual(result('permutation_tree',{values:[]}),{count:0,nodes:[]});
const inc=result('permutation_tree',{values:[0,1,2,3]});assert.equal(inc.count,10);assert.ok(inc.nodes.some(n=>n[0]===0&&n[1]===4&&n[2]==='increasing'));
assert.deepEqual(result('partial_persistent_search',{initial:[0],updates:[[0,2],[0,2]],index:0,threshold:3}),{time:2,first:null,history:[[0],[2],[2]]});
for(const [mode,input] of [
 ['two_pointers',{values:[-1],limit:2}],['maximum_subarray',{values:[],allowEmpty:'yes'}],['dynamic_mex',{operations:[['clear',0]]}],
 ['meet_in_the_middle',{values:Array(17).fill(0),target:0}],['permutation',{values:[0,0],exponent:2}],['unimodal_search',{method:'fibonacci',left:1,right:1,center:0}],
 ['collection_hash',{multi:true,seed:'-1',operations:[]}],['permutation_tree',{values:[1]}],['parentheses',{pairs:2,rank:'2'}],
 ['integer_partitions',{n:-1,limit:2}],['set_partitions',{n:3,limit:0}],['partial_persistent_search',{initial:[0],updates:[[0,5],[0,0]],index:0,threshold:3}]
])assert.throws(()=>result(mode,input));
for(let trial=0;trial<120;trial++){
  const times=[-2,0,3,5],operations=[],states=new Map(),history=[],initial=random(10),query=random(10)-3;
  const replay=predicate=>{let x=BigInt(initial);for(const t of times)if(predicate(t)&&states.has(t)){const[a,b]=states.get(t);x=BigInt(a)*x+BigInt(b);}return x.toString();};
  for(let i=0;i<30;i++){const t=times[random(times.length)];if(random(3)){const a=random(5)-2,b=random(9)-4;operations.push(['set',t,a,b]);states.set(t,[a,b]);}else{operations.push(['erase',t]);states.delete(t);}history.push(replay(()=>true));}
  assert.deepEqual(result('retroactive_operations',{times:[...times,0],initial,query,operations}),{size:states.size,before:replay(t=>t<query),through:replay(t=>t<=query),current:replay(()=>true),history});
}
for(let mask=0;mask<1024;mask++){
  const digits=Array.from({length:10},(_,i)=>i).filter(i=>mask>>i&1),first=mask%40,last=first+70,want=[];
  for(let x=first;x<last;x++)if([...String(x)].every(d=>digits.includes(Number(d))))want.push(String(x));
  assert.deepEqual(result('special_integer_enumeration',{kind:'digits',first,last,digits,limit:30}),{values:want.slice(0,30),more:want.length>30});
}
for(let trial=0;trial<256;trial++){
  const first=trial*37,last=first+100,want=[];
  for(let x=first;x<last;x++)if(String(x)===[...String(x)].reverse().join(''))want.push(String(x));
  assert.deepEqual(result('special_integer_enumeration',{kind:'palindrome',first,last,limit:100}),{values:want,more:false});
  const factors=Array.from({length:8},(_,i)=>i+2).filter((f,i)=>trial>>i&1),reachable=Array(200).fill(false),products=[];reachable[1]=true;
  for(let x=1;x<200;x++){if(factors.some(f=>x%f===0&&reachable[x/f]))reachable[x]=true;if(reachable[x])products.push(String(x));}
  assert.deepEqual(result('special_integer_enumeration',{kind:'products',first:0,last:200,factors,limit:30}),{values:products.slice(0,30),more:products.length>30});
}
assert.deepEqual(result('special_integer_enumeration',{kind:'palindrome',first:'18446744066044764481',last:'18446744073709551615',limit:10}),{values:['18446744066044764481'],more:false});
assert.deepEqual(result('special_integer_enumeration',{kind:'digits',first:'18446744073709551614',last:'18446744073709551615',digits:[0,1,2,3,4,5,6,7,8,9],limit:10}),{values:['18446744073709551614'],more:false});
for(const [mode,input] of [
 ['retroactive_operations',{times:[1],initial:0,query:0,operations:[['set',0,1,2]]}],
 ['special_integer_enumeration',{kind:'digits',first:0,last:0,digits:[10],limit:1}],
 ['special_integer_enumeration',{kind:'products',first:0,last:0,factors:[1],limit:1}],
 ['special_integer_enumeration',{kind:'palindrome',first:2,last:1,limit:1}],
 ['special_integer_enumeration',{kind:'palindrome',first:0,last:'18446744073709551616',limit:1}]
])assert.throws(()=>result(mode,input));
console.log('utility search model: 14 C++ fixtures, exhaustive enumeration, randomized comparisons, uint64 bounds and validation passed');
