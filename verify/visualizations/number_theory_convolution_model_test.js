'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/number_theory_convolution/model.js');
let batches=0,snapshots=0;
const B=BigInt,S=a=>a.map(String);
function prime(n){if(n<2)return false;for(let d=2;d<n;++d)if(n%d===0)return false;return true;}
function primeFactors(n){const out=[];for(let d=2;d<=n;++d)if(prime(d)&&n%d===0)out.push(d);return out;}
function mu(n){for(let d=2;d*d<=n;++d)if(n%(d*d)===0)return 0;return primeFactors(n).length%2?-1:1;}
function gcd(a,b){for(let d=Math.min(a,b);d>=1;--d)if(a%d===0&&b%d===0)return d;}
function lcm(a,b){for(let v=Math.max(a,b);;v+=Math.max(a,b))if(v%a===0&&v%b===0)return v;}
function check(input){const batch=M.run(input),{config:c,frames}=batch,n=c.n,mod=B(c.modulus),reduce=x=>mod?(x%mod+mod)%mod:x,conv=['dirichlet','gcd','lcm'].includes(c.mode),ps=Array.from({length:n+1},(_,i)=>i).filter(prime);++batches;snapshots+=frames.length;
 const originalA=Array(n+1).fill(0n),originalB=Array(n+1).fill(0n),zero=Array(n+1).fill(0n);c.a.forEach((x,i)=>originalA[i+1]=B(x));c.b.forEach((x,i)=>originalB[i+1]=B(x));if(!conv)originalA[0]=B(c.zero);
 // 各素数の処理途中も、素因数の集合に制限した定義式から独立に求める。
 function transformed(base,divisor,inverse,processed=ps,current=null,k=null){const result=[base[0]];for(let i=1;i<=n;++i){const allowed=new Set(processed);if(current!==null){const completed=divisor?(inverse?i>=k*current:i<=k*current):(inverse?i<=k:i>=k);if(completed)allowed.add(current);}let sum=0n;for(let j=1;j<=n;++j){if(!(divisor?i%j===0:j%i===0))continue;const ratio=divisor?i/j:j/i;if(primeFactors(ratio).every(p=>allowed.has(p)))sum+=base[j]*B(inverse?mu(ratio):1);}result.push(reduce(sum));}return result;}
 const divisor=conv?c.mode==='lcm':c.mode.startsWith('divisor'),inverse=c.mode.endsWith('mobius');
 const za=conv?transformed(originalA,divisor,false):null,zb=conv?transformed(originalB,divisor,false):null,product=conv?za.map((x,i)=>i?reduce(x*zb[i]):0n):null;
 const directPairs=[];const final=conv?zero.slice():transformed(originalA,divisor,inverse);
 if(conv)for(let i=1;i<=c.a.length;++i)for(let j=1;j<=c.b.length;++j){const target=c.mode==='dirichlet'?i*j:c.mode==='gcd'?gcd(i,j):lcm(i,j);if(target<=n){final[target]=reduce(final[target]+originalA[i]*originalB[j]);if(c.mode==='dirichlet')directPairs.push([i,j,target]);}}
 for(const f of frames){let a=originalA,b=originalB,h=zero;
  if(!conv){if(f.stage==='transform')a=transformed(originalA,divisor,inverse,f.processed,f.p,f.k);else if(f.stage==='done')a=final;}
  else if(c.mode==='dirichlet'){
   if(f.stage==='direct'||f.stage==='done'){h=zero.slice();for(const[i,j,target]of directPairs.slice(0,f.history.length))h[target]=reduce(h[target]+originalA[i]*originalB[j]);}
  }else{
   if(f.stage==='a-zeta')a=transformed(originalA,divisor,false,f.processed,f.p,f.k);
   if(['b-zeta','product','h-mobius','done'].includes(f.stage))a=za;
   if(f.stage==='b-zeta')b=transformed(originalB,divisor,false,f.processed,f.p,f.k);
   if(['product','h-mobius','done'].includes(f.stage))b=zb;
   if(f.stage==='product'){const count=f.history.filter(x=>x.stage==='product').length;h=product.map((x,i)=>i<=count?x:0n);}
   if(f.stage==='h-mobius')h=transformed(product,divisor,true,f.processed,f.p,f.k);
   if(f.stage==='done')h=final;
  }
  assert.deepEqual(f.a,S(a),[c.mode,f.title,'A']);assert.deepEqual(f.b,S(b),[c.mode,f.title,'B']);assert.deepEqual(f.h,S(h),[c.mode,f.title,'H']);assert.deepEqual(f.output,S(conv?h:a));
  if(['transform','a-zeta','b-zeta','h-mobius'].includes(f.stage)){const inv=f.stage==='transform'?inverse:f.stage==='h-mobius',order=inv?[...ps].reverse():ps;assert.deepEqual(f.processed,order.slice(0,f.processed.length));if(f.p!==null)assert.equal(f.p,order[f.processed.length]);}
  if(f.operation){const op=f.operation;if(op.kind==='add'||op.kind==='subtract'){const row=f.active==='a'?a:f.active==='b'?b:h;assert.equal(B(op.value),row[op.source]);assert.equal(B(op.after),reduce(B(op.before)+(op.kind==='subtract'?-B(op.value):B(op.value))));assert.equal(B(op.after),row[op.target]);}}
 }
 const end=frames.at(-1);assert.equal(end.done,true);assert.deepEqual(end.output,S(final));assert.equal(end.result,n?String(final[c.focus]):'空');assert.equal(final[0],conv?0n:B(c.zero));
 // 素数とkの方向・全更新数を独立した添字列で確認する。
 const segments=!conv?[[divisor,inverse,'transform']]:c.mode==='dirichlet'?[]:[[divisor,false,'a-zeta'],[divisor,false,'b-zeta'],[divisor,true,'h-mobius']];
 for(const[d,inv,stage]of segments){const expected=[];for(const p of inv?[...ps].reverse():ps){const limit=Math.floor(n/p),ks=Array.from({length:limit},(_,i)=>i+1);if(d===inv)ks.reverse();for(const k of ks)expected.push([p,k,d?k:k*p,d?k*p:k]);}assert.deepEqual(end.history.filter(h=>h.stage===stage).map(h=>[h.p,h.k,h.source,h.target]),expected);}
 return batch;
}
for(let n=0,ways=1;n<=5;++n,ways*=3)for(let code=0;code<ways;++code){let state=code;const a=[];for(let i=0;i<n;++i){a.push(state%3-1);state=Math.floor(state/3);}for(const mode of M.MODES.slice(0,4))check({mode,a,zero:'-9223372036854775808',focus:Math.max(1,n)});}
for(let n=0,ways=1;n<=4;++n,ways*=2)for(let x=0;x<ways;++x)for(let y=0;y<ways;++y){const a=Array.from({length:n},(_,i)=>(x>>i)&1),b=Array.from({length:n},(_,i)=>(y>>i)&1);for(const mode of ['dirichlet','gcd','lcm'])check({mode,a,b,focus:Math.max(1,n)});}
let seed=314016;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
for(let trial=0;trial<220;++trial){const a=Array.from({length:random(13)},()=>random(21)-10),b=Array.from({length:random(13)},()=>random(21)-10),modulus=[0,1,12,998244353][random(4)];for(const mode of M.MODES)check({mode,a,b,modulus,focus:random(12)+1,zero:99});}
for(const mode of M.MODES)for(const kind of ['basic','empty','prime-power','signed','composite','truncate','padded','boundary'])check(M.scenario(mode,kind));
for(const input of [{mode:'?'},{mode:'gcd',a:'1 x',b:'2'},{mode:'lcm',a:Array(13).fill(1),b:[]},{mode:'divisor-zeta',a:'9223372036854775808'},{mode:'gcd',a:[Number.MAX_SAFE_INTEGER+1],b:[]},{mode:'multiple-zeta',modulus:-1},{mode:'divisor-mobius',focus:0},{mode:'dirichlet',a:'1.5',b:'2'}])assert.throws(()=>M.run(input));
for(const mode of M.MODES){const batch=check(M.defaults(mode)),last=JSON.stringify(batch.frames.at(-1));batch.frames[0].a.push('mutated');batch.frames[0].history.push('mutated');batch.frames[0].description='changed';assert.equal(JSON.stringify(batch.frames.at(-1)),last);assert.deepEqual(M.run(M.defaults(mode)),M.run(M.defaults(mode)));}
console.log(`arithmetic convolution models: ${batches} batches / ${snapshots} snapshots; every intermediate row checked by prime-factor-restricted definition, mu sums, pair enumeration, direction, zero, truncation, composite moduli and snapshot independence passed`);
