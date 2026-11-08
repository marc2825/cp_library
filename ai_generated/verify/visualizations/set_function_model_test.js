'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/set_function/model.js');
let seed=319019,cases=0,snapshots=0,rejections=0;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const popcount=x=>{let n=0;for(;x;x&=x-1)n++;return n;};
function expected(a,s,processed,subset,inverse,operation,p){
 const terms=[];for(let t=0;t<a.length;++t)if((subset?(s&t)===t:(s&t)===s)&&((s^t)&~processed)===0)terms.push(inverse&&popcount(s^t)%2?-a[t]:a[t]);
 let value;if(operation==='min')value=terms.reduce((x,y)=>x<y?x:y);else if(operation==='max')value=terms.reduce((x,y)=>x>y?x:y);else if(operation==='or')value=terms.reduce((x,y)=>x|y);else value=terms.reduce((x,y)=>x+y,0n);return p?(value%p+p)%p:value;
}
function check(raw){
 const data=M.run(raw),c=data.config;cases++;snapshots+=data.frames.length;
 if(c.enumeration){
  const positions=[];for(let i=0;i<c.bits;++i)if(c.mode==='submask'?Boolean(c.mask>>BigInt(i)&1n):!(c.mask>>BigInt(i)&1n))positions.push(i);
  const all=[];for(let code=0;code<2**positions.length;++code){let value=0n;for(let j=0;j<positions.length;++j)if(code>>j&1)value|=1n<<BigInt(positions[j]);if(c.mode==='supermask')value|=c.mask;if(c.mode==='supermask'||value||c.includeEmpty)all.push(value);}all.sort((a,b)=>a<b?-1:a>b?1:0);if(c.mode==='submask')all.reverse();assert.deepEqual(data.result,all);
  for(const f of data.frames){assert.deepEqual(f.visited,all.slice(0,f.visited.length));if(f.stage==='next'){if(c.mode==='submask')assert.ok(f.next<f.current&&(f.next&c.mask)===f.next);else assert.ok(f.next>f.current&&(f.next&c.mask)===c.mask&&(f.next&~f.full)===0n);}}
 }else{
  const subset=!c.mode.startsWith('superset'),inverse=c.mode.endsWith('mobius'),operation=c.mode==='sos'?c.operation:'sum';
  assert.deepEqual(data.result,c.values.map((_,s)=>expected(c.values,s,c.values.length-1,subset,inverse,operation,c.p)));
  for(const f of data.frames){
   for(let s=0;s<c.values.length;++s){const processed=f.completed|((f.updated||[]).includes(s)?1<<(f.bit??0):0);assert.equal(f.values[s],expected(c.values,s,processed,subset,inverse,operation,c.p));}
   assert.deepEqual(f.columns.at(-1).values,f.values);
   if(f.stage==='update'){assert.equal(f.target^f.source,1<<f.bit);assert.equal(Boolean(f.target&(1<<f.bit)),subset);assert.equal(f.read,f.columns.at(-2).values[f.source]);assert.equal(f.after,f.values[f.target]);}
  }
 }
 if(data.frames.length>2){if(c.enumeration){const first=data.frames[0].visited.length;data.frames.at(-1).visited.push(1n);assert.equal(data.frames[0].visited.length,first);}else{const first=data.frames[0].values[0];data.frames.at(-1).values[0]=100000000n;assert.equal(data.frames[0].values[0],first);}}
 return data;
}
for(const mode of M.MODES)check(M.defaults(mode));
for(let bits=0;bits<=7;++bits)for(let mask=0;mask<2**bits;++mask){for(const includeEmpty of [false,true])check({...M.defaults('submask'),bits:String(bits),mask:String(mask),includeEmpty});check({...M.defaults('supermask'),bits:String(bits),mask:String(mask)});}
const transforms=M.MODES.filter(mode=>!['submask','supermask'].includes(mode));
for(const size of [1,2,4])for(let code=0;code<3**size;++code){let x=code;const a=Array.from({length:size},()=>{const y=x%3-1;x=Math.floor(x/3);return y;});for(const mode of transforms)check({...M.defaults(mode),bits:String(Math.log2(size)),values:a.join(' '),focus:'0'});}
for(let trial=0;trial<200;++trial){const bits=random(5),a=Array.from({length:2**bits},()=>random(19)-9),modulus=['integer','12','998244353','1'][trial%4];for(const mode of transforms){const raw={...M.defaults(mode),bits:String(bits),values:a.join(' '),modulus,focus:'0'};if(mode==='sos')for(const operation of ['sum','min','max','or'])check({...raw,operation});else check(raw);}}
for(const mode of M.MODES){check(M.scenario(mode,'one'));check(M.scenario(mode,'zero'));}
for(const mode of ['submask','supermask'])check(M.scenario(mode,'highest'));
for(const [mode,change,message]of [
 ['submask',{bits:'65'},'範囲外'],['supermask',{bits:'-1'},'範囲外'],['supermask',{mask:'8'},'収まり'],['submask',{mask:'-1'},'非負'],['submask',{mask:'18446744073709551616'},'uint64'],['submask',{bits:'9',mask:'511'},'自由'],['supermask',{bits:'9',mask:'0'},'自由'],['sos',{bits:'5'},'4ビット'],['subset-zeta',{values:''},'項が必要'],['superset-zeta',{values:'1 2'},'項が必要'],['subset-mobius',{values:'NaN'},'整数'],['sos',{focus:'8'},'範囲外'],['sos',{values:'9223372036854775808'},'範囲外'],['subset-zeta',{values:'1 9223372036854775807 0 0 0 0 0 0'},'long long'],['subset-mobius',{values:'1 0 0 0 -9223372036854775808 0 0 0'},'long long']
]){assert.throws(()=>M.run({...M.defaults(mode),...change}),e=>e.message.includes(message));rejections++;}
console.log(`set function model: ${cases} inputs / ${snapshots} snapshots, ${rejections} rejections; bit-position enumeration and every intermediate transform/aggregate from restricted-set definitions passed`);
