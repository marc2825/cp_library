'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const model=require('../../visualizations/data_structure_advanced/model.js');
const expected=JSON.parse(fs.readFileSync(path.join(__dirname,'../data_structure_advanced/visualization_expected.json'),'utf8'));
let checks=0;
for(const mode of model.modes){const state=model.trace(mode.id,mode.sample);assert.deepEqual(state.result,expected[mode.id],mode.id);assert.ok(state.steps.length);checks++;}
let seed=3535;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
for(let trial=0;trial<500;trial++){
 const values=Array.from({length:random(12)},()=>random(10)-4),n=values.length,l=random(n+1),r=l+random(n-l+1),part=values.slice(l,r),queries=[[l,r]];
 const inv=part.reduce((sum,x,i)=>sum+part.slice(i+1).filter(y=>x>y).length,0);
 assert.deepEqual(model.trace('static_range_inversions',{values,queries}).result,[String(inv)]);checks++;
 // Enumerate subsequences independently of the visual tails model.
 let lis=0;for(let mask=0;mask<2**part.length;mask++){let previous=-Infinity,length=0,valid=true;for(let i=0;i<part.length;i++)if(mask>>i&1){valid&&=previous<part[i];previous=part[i];length++;}if(valid)lis=Math.max(lis,length);}
 assert.deepEqual(model.trace('static_range_lis',{values,queries}).result,[String(lis)]);checks++;
 const candidates=[...new Set(part)].map(x=>[x,part.filter(y=>x===y).length]).sort((x,y)=>y[1]-x[1]||x[0]-y[0]);
 assert.deepEqual(model.trace('static_range_mode',{values,queries}).result,candidates.length?candidates[0].map(String):['empty']);checks++;
 const positive=values.map(x=>x+4),divisor=random(8)+1,add=random(5),a=positive.map((x,i)=>i>=l&&i<r?Math.floor(x/divisor)+add:x);
 assert.deepEqual(model.trace('division_segment_tree_beats',{values:positive,ops:[['divide',l,r,divisor],['add',l,r,add]]}).result,[a.reduce((s,x)=>s+x,0),a.length?Math.min(...a):'empty',a.length?Math.max(...a):0].map(String));checks++;
 const after=values.map((x,i)=>i>=l&&i<r?x+add:x);after.reverse();
 assert.deepEqual(model.trace('persistent_lazy_rbst',{values,apply:[l,r,add],reverse:[0,n],query:[l,r]}).result,[values.reduce((s,x)=>s+x,0),after.reduce((s,x)=>s+x,0),after.slice(l,r).reduce((s,x)=>s+x,0)].map(String));checks++;
 const letters=values.map(x=>String(x)),tail=['x','y'],split=random(n+2),joined=letters.concat(tail);
 assert.deepEqual(model.trace('finger_tree',{values:letters,tail,split}).result,[joined.slice(0,split).join(''),joined.slice(split).join(''),joined.slice(split+1).join(''),letters.join('')]);checks++;
}
for(const mode of model.modes)assert.throws(()=>model.trace(mode.id,null));
assert.throws(()=>model.trace('dynamic_wavelet_matrix',{values:[8],bits:3,ops:[]}));
assert.throws(()=>model.trace('division_segment_tree_beats',{values:[1],ops:[['divide',0,1,0]]}));
console.log(`advanced visualization: ${checks} comparisons passed`);
