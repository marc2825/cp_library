'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/combinatorics/model.js');
let batches=0,snapshots=0,rejections=0;
const B=BigInt,facts=[1n],chooseRows=[];
for(let n=0;n<=24;++n){if(n)facts.push(facts[n-1]*B(n));const row=[];for(let k=0;k<=n;++k){let value=1n;for(let j=1;j<=k;++j)value=value*B(n-j+1)/B(j);row.push(value);}chooseRows.push(row);}
const popcount=n=>{let bits=0;for(;n;n&=n-1)++bits;return bits;};
for(let n=0;n<=12;++n){const counts=Array(n+1).fill(0n);for(let mask=0;mask<(1<<n);++mask)++counts[popcount(mask)];assert.deepEqual(chooseRows[n],counts);}
const ordered=[];for(let n=0;n<=8;++n){const counts=Array(n+1).fill(0n);function visit(mask,depth){++counts[depth];for(let x=0;x<n;++x)if(!(mask>>x&1))visit(mask|(1<<x),depth+1);}visit(0,0);ordered.push(counts);assert.equal(counts[n],facts[n]);}
const catalans=[1n];for(let n=1;n<=12;++n)catalans.push(Array.from({length:n},(_,i)=>catalans[i]*catalans[n-1-i]).reduce((a,b)=>a+b,0n));
for(let n=0;n<=8;++n){let count=0n;for(let mask=0;mask<(1<<(2*n));++mask){let level=0,valid=true;for(let i=0;i<2*n;++i){level+=(mask>>i&1)?1:-1;if(level<0){valid=false;break;}}if(valid&&level===0)++count;}assert.equal(count,catalans[n]);}
const weak=[Array(13).fill(0n)];weak[0][0]=1n;for(let types=1;types<=12;++types){const row=weak.at(-1).slice();for(let k=1;k<=12;++k)row[k]+=row[k-1];weak.push(row);}
function prime(p){for(let d=2;d<p;++d)if(p%d===0)return false;return p>=2;}
function unit(bound,m){for(let p=2;p<=bound;++p)if(prime(p)&&m%B(p)===0n)return false;return true;}
function expected(c,m){const {n,k,bound}=c;if(c.mode==='factorial')return facts[n]%m;if(c.mode==='catalan')return n<0?0n:catalans[n]%m;if(c.mode==='multiset'){if(n<0||k<0)return 0n;return weak[n][k]%m;}if(n<0||k<0||k>n)return 0n;return (c.mode==='permutation'?(n<=8?ordered[n][k]:facts[n]/facts[n-k]):chooseRows[n][k])%m;}
function check(input){const c=M.normalize(input),m=B(c.modulus),reduce=x=>m?(x%m+m)%m:x;let reject=false;
 if(c.mode!=='inclusion'){
  reject=c.mode!=='factorial'&&c.backend==='factorial'&&!unit(c.bound,m);
  if(c.mode==='catalan'&&c.n>=0&&2*c.n>c.bound)reject=true;
  if(['binomial','combination','permutation'].includes(c.mode)&&c.n>=0&&c.k>=0&&c.k<=c.n&&c.n>c.bound)reject=true;
  if(c.mode==='multiset'&&c.n>0&&c.k>0&&c.n+c.k-1>c.bound)reject=true;
 }
 if(reject){assert.throws(()=>M.run(input));++rejections;return;}
 const batch=M.run(input),{frames}=batch;++batches;snapshots+=frames.length;const end=frames.at(-1);assert.equal(end.done,true);
 if(c.mode==='inclusion'){
  const start=c.target==='union'?1:0,endMask=1<<c.sets.length;
  for(const f of frames){const prefix=f.terms.length?f.terms.at(-1).mask:-1;const weights=[];
   // 集合の共通部分からではなく、各要素が属する集合の部分集合から係数を直接数える。
   for(let x=0;x<c.universe;++x){const memberships=c.sets.flatMap((s,i)=>s.includes(x)?[i]:[]);let weight=0;
    function visit(at,mask,selected){if(at===memberships.length){if(mask>=start&&mask<=prefix)weight+=(selected%2===1)===(c.target==='union')?1:-1;return;}visit(at+1,mask,selected);visit(at+1,mask|(1<<memberships[at]),selected+1);}visit(0,0,0);weights.push(weight);
   }
   assert.deepEqual(f.weights,weights);assert.equal(B(f.sum),reduce(B(weights.reduce((a,b)=>a+b,0))));
   assert.deepEqual(f.terms.map(t=>t.mask),Array.from({length:f.terms.length},(_,i)=>i+start));
   if(f.mask!==null){const intersection=Array.from({length:c.universe},(_,x)=>x).filter(x=>c.sets.every((s,i)=>!(f.mask>>i&1)||s.includes(x)));assert.deepEqual(f.intersection,intersection);}
  }
  assert.equal(end.terms.length,endMask-start);const count=Array.from({length:c.universe},(_,x)=>x).filter(x=>c.target==='union'?c.sets.some(s=>s.includes(x)):c.sets.every(s=>!s.includes(x))).length;assert.equal(B(end.result),reduce(B(count)));assert.ok(end.weights.every(w=>w===0||w===1));
 }else{
  for(const f of frames){for(let i=0;i<f.fact.length;++i)assert.equal(B(f.fact[i]),facts[i]%m);for(let i=0;i<f.invfact.length;++i)if(f.invfact[i]!==null){const inv=B(f.invfact[i]);assert.ok(inv>=0n&&inv<m);assert.equal(inv*facts[i]%m,1n%m);}
   for(let n=0;n<f.pascal.length;++n)assert.deepEqual(f.pascal[n],chooseRows[n].map(x=>String(x%m)));
   if(f.accumulator!==null){let value;if(c.mode==='catalan')value=(chooseRows[2*c.n][c.n]-(chooseRows[2*c.n][c.n+1]??0n))%m;else value=f.terms.reduce((a,t)=>a*B(t.value)%m,1n%m);assert.equal(B(f.accumulator),reduce(value));}
   if(f.stage==='inverse-factorial'){assert.notEqual(f.invfact[f.i-1],null);for(let j=0;j<f.i-1;++j)assert.equal(f.invfact[j],null);}
  }
  assert.equal(B(end.result),expected(c,m));if(c.backend==='factorial'){assert.equal(end.fact.length,c.bound+1);const available=unit(c.bound,m);assert.equal(end.invfact.every(x=>x!==null),available);if(!available)assert.equal(end.inverseAvailable,false);}else assert.equal(end.pascal.length,c.bound+1);
 }
 return batch;
}
for(let bound=0;bound<=12;++bound)for(const modulus of [1,2,7,12,17,1000000007,'9223372036854775807'])check({mode:'factorial',n:bound,bound,modulus});
for(const mode of ['binomial','permutation','combination'])for(let n=0;n<=8;++n)for(let k=-1;k<=n+1;++k)check({mode,n,k,bound:n,modulus:1000000007});
for(let n=0;n<=8;++n)for(let k=0;k<=8;++k)for(const backend of ['factorial','pascal'])check({mode:'multiset',n,k,bound:n&&k?n+k-1:0,backend,modulus:backend==='pascal'?12:1000000007});
for(let n=-1;n<=12;++n)for(const backend of ['factorial','pascal'])check({mode:'catalan',n,bound:Math.max(0,2*n),backend,modulus:backend==='pascal'?12:1000000007});
for(let count=0,ways=1;count<=3;++count,ways*=8)for(let code=0;code<ways;++code){let state=code;const sets=[];for(let i=0;i<count;++i){const mask=state%8;state=Math.floor(state/8);sets.push(Array.from({length:3},(_,x)=>x).filter(x=>mask>>x&1));}for(const target of ['union','complement'])for(const modulus of [0,12])check({mode:'inclusion',sets,universe:3,target,modulus});}
let seed=315016;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
for(let trial=0;trial<240;++trial){const mode=M.MODES[random(M.MODES.length)],base=M.defaults(mode);if(mode==='inclusion'){base.universe=random(13);base.sets=Array.from({length:random(7)},()=>Array.from({length:base.universe},(_,x)=>x).filter(()=>random(2)));base.target=random(2)?'union':'complement';base.modulus=[0,1,12,17][random(4)];}else{base.n=random(14)-1;base.k=random(14)-1;base.bound=random(25);base.backend=random(2)?'factorial':'pascal';base.modulus=[1,2,7,12,17,1000000007][random(6)];if(mode==='factorial')base.n=Math.min(Math.max(0,base.n),base.bound);}check(base);}
for(const mode of M.MODES)for(const kind of ['basic','empty','composite','nonunit','mod-one','large-mod','zero-types'])check(M.scenario(mode,kind));
check(M.scenario('inclusion','overlap'));
for(const input of [{mode:'?'},{mode:'factorial',n:-1},{mode:'binomial',modulus:0},{mode:'catalan',bound:25},{mode:'inclusion',sets:Array(7).fill([])},{mode:'inclusion',universe:0,sets:[[0]]},{mode:'inclusion',sets:'x'},{mode:'binomial',modulus:'9223372036854775808'},{mode:'permutation',n:1.5},{mode:'binomial',backend:'?'},{mode:'inclusion',target:'?'}])assert.throws(()=>M.run(input));
for(const mode of M.MODES){const batch=check(M.defaults(mode)),last=JSON.stringify(batch.frames.at(-1));batch.frames[0].terms.push('mutated');batch.frames[0].fact.push('mutated');batch.frames[0].description='changed';assert.equal(JSON.stringify(batch.frames.at(-1)),last);assert.deepEqual(M.run(M.defaults(mode)),M.run(M.defaults(mode)));}
console.log(`combinatorics models: ${batches} batches / ${snapshots} snapshots / ${rejections} rejected table/query conditions; subsets, ordered selections, bracket enumeration, weak compositions, per-element inclusion coefficients, modulus boundaries and snapshot independence passed`);
