'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const model=require('../../visualizations/development/model.js');
const cases=JSON.parse(fs.readFileSync(path.join(__dirname,'../development/visualization_expected.json'),'utf8'));
for(const {input,result} of cases)assert.deepEqual(model.generate(input),result);
assert.equal(model.trace('ci',{online:false,local:'success'}).result.status,'success');
assert.equal(model.trace('ci',{online:true,local:'success',unionfind:'success',range_sum:'waiting',suffix_array:'success'}).result.status,'waiting');
const sample=model.modes.find(x=>x.id==='stress').sample;
assert.deepEqual(model.trace('stress',sample).result,{status:'mismatch',seed:41,executed:2});
assert.equal(model.trace('stress',{compare:'tokens',cases:[{seed:0,input:'',fast:'1\t2\n',naive:'1 2'}]}).result.status,'ok');
assert.equal(model.trace('stress',{compare:'exact',cases:[{seed:0,input:'',fast:'1\n',naive:'1'}]}).result.status,'mismatch');
assert.equal(model.trace('stress',{compare:'tokens',cases:[{seed:0,input:'',fast:'1',naive:'1',fastStatus:'timeout'}]}).result.status,'fast_failed');
assert.deepEqual(model.trace('bundle',{entry:'a',files:{a:['b','b'],b:['a']}}).result,{order:['b','a'],files:2});
for(const [mode,input] of [['bundle',{entry:'x',files:{}}],['stress',{compare:'float',cases:[]}],['random',{seed:'0',kind:'tree',n:-1}],['ci',{online:false,local:'wrong'}]])assert.throws(()=>model.trace(mode,input));
for(const mode of model.modes){const trace=model.trace(mode.id,mode.sample);assert.ok(trace.steps.length>1);assert.equal(trace.steps.at(-1).title,'最終結果');}
console.log('development model: 200 Python generator comparisons, 4 modes, failure/timeout/duplicate/cycle/boundary cases passed');
