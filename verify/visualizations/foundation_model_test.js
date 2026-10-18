'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {modes,trace}=require('../../visualizations/foundation/model.js');
const expected=JSON.parse(fs.readFileSync(path.join(__dirname,'../foundation/visualization_expected.json'),'utf8'));
for(const mode of modes){
  const observed=trace(mode.id,mode.sample);
  assert.deepEqual(observed.result,expected[mode.id]);
  assert.equal(observed.steps.at(-1).title,'最終結果');
  assert.ok(observed.steps.every(x=>x.values.length&&x.values.every(v=>typeof v.value==='string')));
}
let seed=20261019n;
const random=()=>{seed=(seed*6364136223846793005n+1442695040888963407n)&((1n<<64n)-1n);return seed;};
const norm=(x,m)=>(x%m+m)%m;
for(let i=0;i<1500;i++){
  const x=random(),a=random()-(1n<<63n),b=random()-(1n<<63n),m=(random()&((1n<<32n)-1n))||1n;
  assert.equal(trace('barrett_reduction',{mod:String(m),value:String(x)}).result,String(x%m));
  const n=(random()>>1n)|1n;
  assert.equal(trace('montgomery_modint',{mod:String(n),a:String(a),b:String(b)}).result,String(norm(a*b,n)));
  assert.equal(trace('modint61',{a:String(a),b:String(b)}).result,String(norm(a*b,(1n<<61n)-1n)));
  const r=trace('static_modint',{mod:String(m),a:String(a),b:String(b)}).result;
  assert.equal(r.product,String(norm(a*b,m)));assert.equal(r.sum,String(norm(a+b,m)));
}
for(const a of [-128,-127,-1,0,1,126,127])for(const b of [-128,-1,0,1,3,127]){
  const r=trace('safe_integer',{a:String(a),b:String(b),bits:8,signed:true}).result;
  const fit=x=>x>=-128&&x<=127?String(x):'none';
  assert.equal(r.sum,fit(a+b));assert.equal(r.product,fit(a*b));
  if(b!==0&&!(a===-128&&b===-1)){
    assert.equal(r.floor,String(Math.floor(a/b)));assert.equal(r.ceil,String(Math.ceil(a/b)));
  }
}
assert.deepEqual(trace('fast_io',{text:'-0 255 256 x  +7',bits:8,signed:false,buffer:1}).result,['invalid','255','overflow','invalid','7']);
assert.deepEqual(trace('fast_io',{text:'',bits:128,signed:true,buffer:1}).result,[]);
for(const mode of ['static_modint','barrett_reduction','montgomery_modint'])assert.throws(()=>trace(mode,{mod:'0',value:'1',a:'1',b:'1'}));
assert.throws(()=>trace('montgomery_modint',{mod:'4',a:'1',b:'2'}));
assert.throws(()=>trace('barrett_reduction',{mod:'3',value:String(1n<<64n)}));
assert.throws(()=>trace('fast_io',{text:'a'.repeat(121),bits:8,signed:true,buffer:3}));
assert.throws(()=>trace('modint61',{a:'1e2',b:'3'}));
assert.throws(()=>trace('safe_integer',{a:'128',b:'1',bits:8,signed:true}));
assert.equal(trace('montgomery_modint',{mod:'1',a:'8',b:'4'}).result,'0');
assert.equal(trace('dynamic_modint',{mod:'12',value:'5',nextMod:'12'}).result.status,'stale');
console.log('foundation model: 7 C++ reference cases, 6000 modular comparisons, rounding, parsing and invalid inputs passed');
