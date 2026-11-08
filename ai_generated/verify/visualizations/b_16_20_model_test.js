'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
require('../../visualizations/b_16_20/specs.js');
const {build,specs}=require('../../visualizations/b_16_20/model.js');
const expected=JSON.parse(fs.readFileSync(path.join(__dirname,'../b_16_20/visualization_expected.json'),'utf8'));
const normalize=(mode,x)=>mode==='gcd_lcm_convolution'?[x.gcd,x.lcm]:mode==='multidimensional_convolution'?x.coefficients:mode==='subproduct_tree'?[x.product,x.values]:mode==='partial_fraction'?[x.polynomial_part,...x.coefficients]:mode==='infer_nth'?x.value:mode==='set_partition'?x.by_blocks:mode==='black_box'?x.value:mode==='low_rank_update'?x.inverse:x;
let cases=0,states=0,rejections=0;
function run(mode,input){const r=build(mode,input);assert.ok(r.frames.length>=2);assert.equal(r.frames[0].title,'入力と係数規約');for(const f of r.frames){assert.equal(typeof f.note,'string');assert.ok(Array.isArray(f.grid)&&Array.isArray(f.rows));assert.ok(!JSON.stringify(f).includes('null,null,null'));}cases++;states+=r.frames.length;return r;}
for(const[mode,[,input]]of Object.entries(specs))assert.deepEqual(normalize(mode,run(mode,input).result),expected[mode],mode);
let seed=316202026;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const mod=(x,p)=>((x%p)+p)%p;
const multiply=(a,b,n,p)=>{const c=Array(n).fill(0);for(let i=0;i<a.length;i++)for(let j=0;j<b.length&&i+j<n;j++)c[i+j]=mod(c[i+j]+a[i]*b[j],p);return c;};
const evaluate=(a,x,p)=>{let result=0,power=1;for(const c of a){result=mod(result+c*power,p);power=mod(power*x,p);}return result;};
const submul=(a,b,p)=>{const c=Array(a.length).fill(0);for(let s=0;s<c.length;s++)for(let t=0;t<=s;t++)if((t&s)===t)c[s]=mod(c[s]+a[t]*b[s^t],p);return c;};
for(let trial=0;trial<120;trial++){
 const p=97,n=1+random(12),m=1+random(n),a=Array.from({length:n},()=>random(p)),b=Array.from({length:n},()=>random(p));
 const c=multiply(a,b,2*n-1,p);
 assert.deepEqual(run('relaxed_convolution',{mod:p,a,b}).result,c.slice(0,n));
 assert.deepEqual(run('convolution_large',{mod:p,a,b,block_size:2}).result,c);
 const small=b.slice(0,m),inner=run('middle_product',{mod:p,a,b:small}).result;
 assert.deepEqual(inner,Array.from({length:n-m+1},(_,k)=>mod(small.reduce((v,x,j)=>v+x*a[k+j],0),p)));
 const start=random(p),ratio=trial%5?random(p):0;
 assert.deepEqual(run('chirp_z',{mod:p,f:a,start,ratio,count:n}).result,Array.from({length:n},(_,k)=>{let x=start;for(let j=0;j<k;j++)x=mod(x*ratio,p);return evaluate(a,x,p);}));
 const result=run('composition',{mod:p,f:a,g:b,n}).result;
 let term=Array(n).fill(0);term[0]=1;const composed=Array(n).fill(0);for(const x of a){for(let j=0;j<n;j++)composed[j]=mod(composed[j]+x*term[j],p);term=multiply(term,b,n,p);}assert.deepEqual(result,composed);
 a[0]=1;const inverse=run('online_fps',{mod:p,a,operation:'inv'}),unit=Array(n).fill(0);unit[0]=1;assert.deepEqual(multiply(a,inverse.result,n,p),unit);
 for(const f of inverse.frames.filter(f=>f.title.startsWith('逆数の'))){const degree=Number(f.title.match(/\d+/)[0]);assert.deepEqual(multiply(a,f.grid[1],degree+1,p),unit.slice(0,degree+1));}
}
for(const p of [2,3,17,97])for(let trial=0;trial<65;trial++){
 const bits=random(5),n=1<<bits,a=Array.from({length:n},()=>random(p)),b=Array.from({length:n},()=>random(p));
 assert.deepEqual(run('fast_subset_convolution',{mod:p,a,b}).result,submul(a,b,p));
 a[0]=0;const e=run('set_power_series',{mod:p,f:a,operation:'exp'}).result;
 // restricted-growth strings: enumerate unordered partitions for each mask.
 const partition=Array(n).fill(0);partition[0]=1;
 for(let mask=1;mask<n;mask++){
  const elements=[];for(let j=0;j<bits;j++)if(mask&(1<<j))elements.push(j);
  function visit(at,blocks){if(at===elements.length){partition[mask]=mod(partition[mask]+blocks.reduce((v,s)=>mod(v*a[s],p),1),p);return;}const bit=1<<elements[at];for(let j=0;j<blocks.length;j++){const next=blocks.slice();next[j]|=bit;visit(at+1,next);}visit(at+1,[...blocks,bit]);}visit(0,[]);
 }
 assert.deepEqual(e,partition);
 assert.deepEqual(run('set_power_series',{mod:p,f:e,operation:'log'}).result,a);
 const dp=run('set_partition',{mod:p,weights:a}).result;assert.deepEqual(dp.total,partition);
 b[0]=1;const inv=run('set_power_series',{mod:p,f:b,operation:'inv'}).result;const unit=Array(n).fill(0);unit[0]=1;assert.deepEqual(submul(b,inv,p),unit);
 const outer=[1,2,3,4,5,6],composition=run('sps_composition',{mod:p,outer,inner:b}).result;let term=unit.slice(),wanted=Array(n).fill(0);for(const c of outer){for(let s=0;s<n;s++)wanted[s]=mod(wanted[s]+c*term[s],p);term=submul(term,b,p);}assert.deepEqual(composition,wanted);
}
function matmul(a,b,p){return a.map(row=>b[0].map((_,j)=>mod(row.reduce((v,x,k)=>v+x*b[k][j],0),p)));}
function annihilates(f,a,p){let result=a.map(row=>row.map(()=>0));for(let i=f.length-1;i>=0;i--){result=matmul(result,a,p);for(let j=0;j<a.length;j++)result[j][j]=mod(result[j][j]+f[i],p);}return result.flat().every(x=>x===0);}
for(let code=0;code<81;code++){
 let value=code;const a=Array.from({length:2},()=>Array.from({length:2},()=>{const x=value%3;value=Math.floor(value/3);return x;}));
 const f=run('minimal_polynomial',{mod:3,a}).result;assert.ok(annihilates(f,a,3));
 for(let d=0;d<f.length-1;d++)for(let x=0;x<3**d;x++){let val=x;const candidate=Array.from({length:d},()=>{const y=val%3;val=Math.floor(val/3);return y;});candidate.push(1);assert.ok(!annihilates(candidate,a,3));}
}
assert.deepEqual(run('low_rank_update',{mod:17,a:[[1,0],[0,1]],u:[[],[]],v:[[],[]]}).result,{inverse:[[1,0],[0,1]],ratio:1});
assert.equal(run('low_rank_update',{mod:17,a:[[1]],u:[[1]],v:[[-1]]}).result.inverse,null);
const bad=[['composition',{mod:12,f:[1],g:[0],n:1}],['composition',{mod:17,f:[1],g:[0],n:100}],['relaxed_convolution',{a:[1],b:[]}],['middle_product',{a:[1],b:[]}],['multidimensional_convolution',{a:[1],a_shape:[2],b:[1],b_shape:[1]}],['convolution_large',{a:[1],b:[1],block_size:3}],['min_plus_convolution',{a:[0,2,1],b:[1]}],['compositional_inverse',{f:[1,1],n:4}],['sparse_fps',{terms:[[0,0]],n:4,operation:'inv'}],['online_fps',{a:[1],operation:'exp'}],['online_fps',{a:[1],operation:'log'}],['polynomial_mod_pow',{a:[1],modulus:[0],exponent:'2'}],['polynomial_mod_pow',{a:[1],modulus:[1,1],exponent:'18446744073709551616'}],['partial_fraction',{p:[1],roots:[1,1],multiplicities:[1,1]}],['p_recursive',{initial:[1],p:[[1],[0]],count:4}],['coefficient_extraction',{p:[1],q:[0,1],n:4}],['set_power_series',{f:[0,1,1],operation:'exp'}],['set_power_series',{f:[0,1],operation:'inv'}],['set_power_series',{f:[2,1],operation:'log'}],['sparse_matrix',{rows:1,cols:1,entries:[[1,0,1]],v:[0]}],['wiedemann',{a:[[1,2]],left:[1],right:[1]}],['low_rank_update',{a:[[0]],u:[[1]],v:[[1]]}]];
for(const[mode,input]of bad){assert.throws(()=>build(mode,input),undefined,mode);rejections++;}
console.log(`${cases} model inputs, ${states} states, ${rejections} rejections and all 31 C++ reference cases passed`);
