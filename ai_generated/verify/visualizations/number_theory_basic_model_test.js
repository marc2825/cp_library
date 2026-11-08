'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/number_theory_basic/model.js');
let batches=0,snapshots=0;
const B=BigInt,abs=x=>x<0n?-x:x;
function least(n){for(let d=2;d<=n;++d)if(n%d===0)return d;return n;}
function prime(n){return n>=2&&least(n)===n;}
function ds(n){return Array.from({length:n},(_,i)=>i+1).filter(d=>n%d===0);}
function factors(n){let remaining=n;const out=[];for(let p=2;p<=n;++p){let e=0;while(remaining%p===0){remaining/=p;++e;}if(e)out.push([p,e]);if(remaining===1)break;}return out;}
function product(fs){return fs.reduce((p,[v,e])=>p*B(v)**B(e),1n);}
function slowGcd(a,b){if(!a&&!b)return 0n;const limit=Number(a>b?a:b);let g=1;for(let d=1;d<=limit;++d)if(a%B(d)===0n&&b%B(d)===0n)g=d;return B(g);}
// 反復乗算と、剰余列の周期で巨大指数も確認する。繰り返し二乗は使わない。
function power(a,e,m){a=(a%m+m)%m;let x=1n%m,step=0n;const seen=new Map(),values=[];while(step<e){if(seen.has(x)){const begin=seen.get(x),period=step-begin;return values[Number(begin+(e-begin)%period)];}seen.set(x,step);values.push(x);x=x*a%m;++step;}return x;}
function check(input,known){
 const {config:c,frames}=M.run(input);++batches;snapshots+=frames.length;assert.ok(frames.length>=2);const end=frames.at(-1);assert.equal(end.phase,'完了');
 if(['gcd','extended','inverse'].includes(c.mode)){
  const a=B(c.a),b=B(c.mode==='inverse'?c.modulus:c.b),g=known?.g??slowGcd(abs(a),abs(b));
  for(const f of frames){assert.equal(a*B(f.x0)+b*B(f.y0),B(f.r0));assert.equal(a*B(f.x1)+b*B(f.y1),B(f.r1));assert.ok(B(f.r0)>=0n&&B(f.r1)>=0n);
   for(const h of f.history){assert.equal(B(h.dividend),B(h.divisor)*B(h.q)+B(h.remainder));assert.ok(B(h.remainder)>=0n&&B(h.remainder)<B(h.divisor));}}
  assert.equal(B(end.r0),g);assert.equal(end.r1,'0');
  if(c.mode==='inverse'){
   let inv=known?.inverse;
   if(inv===undefined){inv='なし';for(let x=0n;x<b;++x)if((a*x%b+b)%b===1n%b){inv=String(x);break;}}
   assert.equal(end.result,inv);
  }else{
   assert.equal(B(end.result),g);const l=!a||!b?0n:abs(a)/g*abs(b);assert.equal(end.lcm,l<1n<<64n?String(l):null);
   assert.ok(B(end.x0)>=-(1n<<63n)&&B(end.x0)<1n<<63n);assert.ok(B(end.y0)>=-(1n<<63n)&&B(end.y0)<1n<<63n);
  }
 }else if(c.mode==='pow'){
  const a=B(c.a),e=B(c.exponent),m=B(c.modulus);
  for(const f of frames){assert.equal(B(f.acc),known?.power===undefined?power(a,B(f.used),m):(B(f.used)%2n?m-1n:1n%m));
   assert.equal(B(f.used)+B(f.remaining)*B(f.weight),e);
   assert.ok(B(f.acc)>=0n&&B(f.acc)<m&&B(f.value)>=0n&&B(f.value)<m);
   for(const bit of f.bits)assert.equal(B(bit.acc),bit.bit==='1'?B(bit.previous)*B(bit.base)%m:B(bit.previous));}
  assert.equal(B(end.result),known?.power??power(a,e,m));
 }else if(c.mode==='sieve'){
  let removed=new Set();
  for(const f of frames){for(let n=0;n<=c.limit;++n)assert.equal(f.table[n],n>=2&&(f.phase==='完了'?prime(n):prime(n)||least(n)>(f.p??0)));
   for(const n of f.marked){assert.ok(!removed.has(n)&&!prime(n)&&!f.table[n]&&n%f.p===0);removed.add(n);}}
  const expected=Array.from({length:c.limit+1},(_,i)=>i).filter(prime);assert.deepEqual(end.primes,expected);assert.equal(Number(end.result),expected.length);
 }else if(c.mode==='spf'){
  const assigned=new Set();for(const f of frames){for(let n=0;n<=c.limit;++n){if(n<=1)assert.equal(f.table[n],n);else if(f.table[n])assert.equal(f.table[n],least(n));}
   for(const p of f.products){assert.ok(!assigned.has(p.value));assigned.add(p.value);assert.equal(p.value,f.i*p.p);assert.equal(p.p,least(p.value));assert.ok(p.p<=least(f.i));}
   const bound=f.i??(f.phase==='線形篩'?0:c.limit);assert.deepEqual(f.primes,Array.from({length:bound+1},(_,i)=>i).filter(prime));
   if(f.remaining!==null&&c.query>0)assert.equal(product(f.factors)*B(f.remaining),B(c.query));
   for(const r of f.route)assert.equal(r.before,r.p**r.exponent*r.after);
  }
  for(let n=2;n<=c.limit;++n)assert.equal(end.table[n],least(n));
  if(c.query>0)assert.deepEqual(end.factors,factors(c.query));else assert.equal(end.result,'分解なし');
 }else if(c.mode==='factor'){
  const n=Number(c.n);for(const f of frames){assert.equal(product(f.factors)*B(f.remaining),B(n));for(const[p,e]of f.factors)assert.ok(prime(Number(p))&&e>0);
   for(const t of f.trials)assert.equal(B(t.before),B(t.p)**B(t.exponent)*B(t.after));}
  assert.deepEqual(end.factors.map(([p,e])=>[Number(p),e]),factors(n));
 }else{
  const n=Number(c.n);for(const f of frames){const limit=Number(f.d??0),small=[],large=[];for(let d=1;d<=limit;++d)if(n%d===0){small.push(String(d));if(d!==n/d)large.push(String(n/d));}
   if(f.phase!=='完了'){assert.deepEqual(f.small,small);assert.deepEqual(f.large,large);}
   assert.equal(new Set([...f.small,...f.large]).size,f.small.length+f.large.length);
   assert.ok([...f.small,...f.large].every(d=>n%Number(d)===0));}
  assert.deepEqual(end.divisors,ds(n).map(String));assert.equal(Number(end.result),ds(n).length);
 }
 return {config:c,frames};
}
for(const mode of ['gcd','extended'])for(let a=-20;a<=20;++a)for(let b=-20;b<=20;++b)check({mode,a,b});
for(let a=-20;a<=20;++a)for(let m=1;m<=25;++m)check({mode:'inverse',a,modulus:m});
for(let a=-7;a<=7;++a)for(let e=0;e<=15;++e)for(let m=1;m<=12;++m)check({mode:'pow',a,exponent:e,modulus:m});
for(let n=0;n<=80;++n){check({mode:'sieve',limit:n});for(const query of new Set([0,n,Math.floor(n/2),Math.min(1,n)]))check({mode:'spf',limit:n,query});}
for(let n=1;n<=400;++n){check({mode:'factor',n});check({mode:'divisors',n});}
for(const mode of M.MODES)for(const kind of ['basic','zero','negative','non-coprime','prime'])check(M.scenario(mode,kind));
const MIN=-(1n<<63n),MAX=(1n<<63n)-1n;
for(const mode of ['gcd','extended'])for(const [a,b,g]of [[MIN,MAX,1n],[MIN,MIN,-MIN],[MIN,0n,-MIN],[0n,MIN,-MIN],[MAX,2n,1n]])check({mode,a:String(a),b:String(b)},{g});
check(M.scenario('inverse','boundary'),{g:1n,inverse:String(MAX-1n)});check(M.scenario('pow','boundary'),{power:MAX-1n});
for(const mode of ['sieve','spf','factor','divisors'])check(M.scenario(mode,'boundary'));
for(const input of [{mode:'pow',a:1,exponent:-1,modulus:5},{mode:'inverse',a:1,modulus:0},{mode:'gcd',a:'',b:1},{mode:'gcd',a:'9223372036854775808',b:1},{mode:'gcd',a:Number(MAX),b:1},{mode:'sieve',limit:81},{mode:'spf',limit:5,query:6},{mode:'factor',n:0},{mode:'divisors',n:10001},{mode:'?' }])assert.throws(()=>M.run(input));
for(const mode of M.MODES){const batch=check(M.defaults(mode)),again=M.run(M.defaults(mode)),end=JSON.stringify(batch.frames.at(-1));batch.frames[0].description='mutated';for(const key of ['table','history','bits','factors','small'])if(Array.isArray(batch.frames[0][key]))batch.frames[0][key].push('mutated');assert.equal(JSON.stringify(batch.frames.at(-1)),end);assert.deepEqual(again,M.run(M.defaults(mode)));}
console.log(`number theory models: ${batches} batches / ${snapshots} snapshots; independent divisors/primes/factors, modular cycles, Bezout identities, SPF uniqueness, 64-bit boundaries and snapshot independence passed`);
