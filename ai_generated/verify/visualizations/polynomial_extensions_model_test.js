'use strict';
const assert=require('node:assert/strict');
const model=require('../../visualizations/polynomial_extensions/model.js');
const expected=require('../polynomial_extensions/visualization_expected.json');
for(const mode of model.modes){const input=JSON.stringify(mode.sample),state=model.trace(mode.id,mode.sample);assert.deepEqual(state.result,expected[mode.id],mode.id);assert.equal(JSON.stringify(mode.sample),input);assert.ok(state.steps.length>=2);assert.notEqual(state.steps[0],state.steps.at(-1));assert.throws(()=>model.trace(mode.id,null));}
let seed=1;const rand=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const P=998244353n,norm=x=>(x%P+P)%P;
const run=(mode,input)=>model.trace(mode,input).result.map(BigInt);
for(let test=0;test<150;test++){
 const a=Array.from({length:1+rand(6)},()=>rand(20)-10),b=Array.from({length:1+rand(a.length)},()=>rand(20)-10);
 const w=run('middle',{a,b});for(let i=0;i<w.length;i++){let sum=0n;for(let j=0;j<b.length;j++)sum+=BigInt(a[i+j])*BigInt(b[j]);assert.equal(w[i],norm(sum));}
 const weights=Array.from({length:rand(10)},()=>rand(8)),counts=Array(16).fill(0n);for(let mask=0;mask<(1<<weights.length);mask++){let sum=0;weights.forEach((w,j)=>{if(mask>>j&1)sum+=w;});if(sum<16)counts[sum]++;}assert.deepEqual(run('subset_sum',{weights,limit:15}),counts);
 const signed=Array.from({length:rand(7)},()=>rand(20)-10);const prefix=run('prefix_sum',{a:signed,prime:17}),evalp=(f,x)=>f.reduceRight((s,c)=>(s*x+BigInt(c))%17n,0n);for(let x=0n;x<17n;x++)assert.equal((evalp(prefix,x+1n)-evalp(prefix,x)+34n)%17n,(evalp(signed,x)+17n)%17n);
 const modulus=1n<<64n,A=a.map(x=>String(BigInt(x)+modulus>>1n)),B=b.map(x=>String(BigInt(x)+modulus>>1n));const wrap=run('wrap_convolution',{a:A,b:B});for(let k=0;k<wrap.length;k++){let v=0n;A.forEach((x,i)=>{if(k-i>=0&&k-i<B.length)v+=BigInt(x)*BigInt(B[k-i]);});assert.equal(wrap[k],v%modulus);}
}
for(let bits=0;bits<=5;bits++)for(let t=0;t<10;t++){
 const a=Array.from({length:1<<bits},()=>rand(20)),b=a.map(()=>rand(20));const layers=run('intersection',{a,b}),online=run('online_subset',{a,b});assert.deepEqual(layers.slice(0,a.length),online);
 const overall=layers.reduce((s,x)=>(s+x)%P,0n),sum=x=>x.reduce((s,y)=>s+BigInt(y),0n);assert.equal(overall,sum(a)*sum(b)%P);
 const moments=run('sps_projection',{a,weights:b,count:bits+3});assert.equal(moments[0],BigInt(b[0]));assert.equal(moments[1],a.reduce((s,x,i)=>(s+BigInt(x*b[i]))%P,0n));
}
for(const [id,input] of [['composition',{outer:[1],inner:[1],n:2}],['rader',{a:[[0,0],[0,0],[0,0],[0,0]]}],['roots',{a:[0],prime:17}],['wrap_convolution',{a:['18446744073709551616'],b:['1']}],['intersection',{a:[1,2,3],b:[1,2,3]}]])assert.throws(()=>model.trace(id,input));
console.log('polynomial extensions model: 31 C++ fixtures, 150 randomized batches, 60 set comparisons and invalid inputs passed');
