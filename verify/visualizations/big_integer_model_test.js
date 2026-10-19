'use strict';
const assert=require('node:assert/strict'),{trace,modes}=require('../../visualizations/big_integer/model.js');
let state=111026;const rng=()=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return BigInt(state);};
for(let i=0;i<600;i++){
  const a=(rng()*rng()*rng())*(i%2?-1n:1n),b=(rng()*rng()+1n)*(i%3?-1n:1n);
  for(const op of ['+','-','*']){
    const result=trace('arithmetic',{a:String(a),b:String(b),op});
    assert.equal(result.result.value,String(op==='+'?a+b:op==='-'?a-b:a*b));
    const snapshots=result.steps.filter(s=>s.limbs);if(snapshots.length>1){const last=JSON.stringify(snapshots.at(-1));snapshots[0].limbs[0]=-1;assert.equal(JSON.stringify(snapshots.at(-1)),last);}
  }
  assert.deepEqual(trace('division',{a:String(a),b:String(b)}).result,{quotient:String(a/b),remainder:String(a%b)});
  let x=a<0n?-a:a,y=b<0n?-b:b;while(y)[x,y]=[y,x%y];
  assert.equal(trace('gcd',{a:String(a),b:String(b)}).result.gcd,String(x));
  for(const base of [10,16]){
    const text=(a<0n?'-':'+')+(base===16?'0X':'')+(a<0n?-a:a).toString(base);
    assert.deepEqual(trace('parse',{text,base}).result,{decimal:String(a),hex:(a<0n?'-':'')+(a<0n?-a:a).toString(16)});
  }
}
for(const a of ['0','-0','1','-1','1000000','-1000000'])for(const b of ['0','-1','1','999999','-999999'])for(const op of ['+','-','*']){
  const x=BigInt(a),y=BigInt(b);assert.equal(trace('arithmetic',{a,b,op}).result.value,String(op==='+'?x+y:op==='-'?x-y:x*y));
}
assert.throws(()=>trace('division',{a:'1',b:'0'}));assert.throws(()=>trace('parse',{text:'0x10',base:10}));
assert.throws(()=>trace('parse',{text:'0x',base:16}));assert.throws(()=>trace('arithmetic',{a:'1',b:'2',op:'/'}));
const expected=require('../big_integer/visualization_expected.json');
for(const mode of modes)assert.deepEqual(trace(mode.id,mode.sample).result,expected[mode.id]);
console.log('BigInt model: 4200 random comparisons, 90 signed boundary cases, 4 C++ references and errors passed');
