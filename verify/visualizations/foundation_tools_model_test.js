'use strict';
const assert=require('node:assert/strict'),{trace,modes}=require('../../visualizations/foundation_tools/model.js');
const expected=require('../foundation_tools/visualization_expected.json');
for(const mode of modes)assert.deepEqual(trace(mode.id,mode.sample).result,expected[mode.id]);
let seed=121726;const rng=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const canonical=(n,d)=>{if(d<0){n=-n;d=-d;}let a=Math.abs(n),b=d;while(b)[a,b]=[b,a%b];return `${n/a}/${d/a}`;};
for(let i=0;i<300;i++){
 const a=rng(41)-20,b=rng(20)+1,c=rng(41)-20,d=rng(20)+1;
 for(const [op,n,den]of [['+',a*d+c*b,b*d],['-',a*d-c*b,b*d],['*',a*c,b*d],...(c?[['/',a*d,b*c]]:[])])assert.equal(trace('big_rational',{a:`${a}/${b}`,b:`${c}/${d}`,op}).result.value,canonical(n,den));
 const values=Array.from({length:rng(30)},()=>String(rng(201)-100));
 const result=trace('radix_sort',{values,bits:16,signed:true}).result,indices=values.map((_,i)=>i).sort((i,j)=>Number(values[i])-Number(values[j])||i-j);
 assert.deepEqual(result,{values:indices.map(i=>values[i]),indices});
 const h=1+rng(6),w=1+rng(6),grid=Array.from({length:h},(_,r)=>Array.from({length:w},(_,c)=>String.fromCharCode(65+r*w+c)).join('')),turns=rng(19)-9,t=(turns%4+4)%4;
 const out=Array.from({length:t%2?w:h},()=>Array(t%2?h:w).fill(''));
 for(let r=0;r<h;r++)for(let c=0;c<w;c++){let x,y;if(t===0)[x,y]=[r,c];if(t===1)[x,y]=[c,h-1-r];if(t===2)[x,y]=[h-1-r,w-1-c];if(t===3)[x,y]=[w-1-c,r];out[x][y]=grid[r][c];}
 const transformed=trace('grid',{grid,operation:'rotate',turns,origin:null}).result;assert.deepEqual(transformed.transformed,out.map(row=>row.join('')));
 const ops=[],state=[1,2,3],saved=[];let history=0;
 for(let j=0;j<30;j++){
  if(rng(4)===0&&saved.length){const k=rng(saved.length);ops.push(['rollback',saved[k].name]);state.splice(0,3,...saved[k].values);history=saved[k].history;saved.length=k+1;}
  else{const p=rng(3),v=rng(100);ops.push(['set',p,v]);state[p]=v;history++;}
  if(j%5===0){ops.push(['save',String(j)]);saved.push({name:String(j),values:[...state],history});}
 }
 assert.deepEqual(trace('rollback_array',{values:[1,2,3],operations:ops}).result,{values:state,history});
 const nodes=Array.from({length:rng(30)},(_,id)=>['add',rng(100),id?rng(id):null]),blockSize=1+rng(8),arena=trace('arena',{blockSize,operations:nodes});
 assert.deepEqual(arena.result,{nodes:nodes.map((x,id)=>({id,block:Math.floor(id/blockSize),slot:id%blockSize,value:x[1],previous:x[2]})),capacity:Math.ceil(nodes.length/blockSize)*blockSize});
 const withRows=arena.steps.filter(s=>s.rows.length);if(withRows.length>1){const last=JSON.stringify(withRows.at(-1));withRows[0].rows[0][0]='changed';assert.equal(JSON.stringify(withRows.at(-1)),last);}
}
for(let n=0;n<=9;n++)for(let k=0;k<=n;k++){
 const masks=[];for(let x=0;x<2**n;x++)if(x.toString(2).split('1').length-1===k)masks.push(String(x));
 assert.deepEqual(trace('bit_operations',{n,k,limit:128}).result,{masks,more:false});
}
assert.equal(trace('bit_operations',{n:64,k:64,limit:1}).result.masks[0],'18446744073709551615');
assert.equal(trace('bit_operations',{n:64,k:32,limit:3}).result.more,true);
assert.deepEqual(trace('grid',{grid:[],operation:'transpose',origin:null}).result.transformed,[]);
for(const [mode,input]of [
 ['big_rational',{a:'1/0',b:'1',op:'+'}],['arena',{blockSize:2,operations:[['add',1,9]]}],
 ['bit_operations',{n:64,k:65,limit:1}],['grid',{grid:['aa','b'],operation:'transpose',origin:null}],
 ['radix_sort',{values:['128'],bits:8,signed:true}],
 ['rollback_array',{values:[0],operations:[['save','root'],['set',0,1],['save','old'],['rollback','root'],['set',0,2],['rollback','old']]}]
])assert.throws(()=>trace(mode,input));
console.log('foundation tools model: 6 C++ references, 300 randomized batches, 55 exhaustive mask classes, boundaries and errors passed');
