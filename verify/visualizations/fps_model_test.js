'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/fps/model.js');
let seed=317017,cases=0,snapshots=0,rejections=0;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const norm=(v,p)=>(v%p+p)%p,pre=(a,n)=>Array.from({length:n},(_,i)=>a[i]||0);
function mul(a,b,p,n){if(n===undefined)n=a.length&&b.length?a.length+b.length-1:0;const c=Array(n).fill(0);for(let i=0;i<a.length&&i<n;++i)for(let j=0;j<b.length&&i+j<n;++j)c[i+j]=norm(c[i+j]+a[i]*b[j],p);return c;}
function inv(x,p){for(let y=1;y<p;++y)if(x*y%p===1)return y;throw Error('no inverse');}
const values=a=>a.map(Number);
const polynomial=(a,x,p)=>a.reduceRight((r,v)=>(r*x+v)%p,0);
function expDefinition(a,n,p){let result=pre([1],n),term=pre([1],n),fact=1;for(let j=1;j<n;++j){term=mul(term,a,p,n);fact=fact*j%p;result=result.map((x,i)=>(x+term[i]*inv(fact,p))%p);}return result;}
function logDefinition(a,n,p){if(!n)return [];const u=pre(a,n);u[0]=0;let result=Array(n).fill(0),term=pre([1],n);for(let j=1;j<n;++j){term=mul(term,u,p,n);result=result.map((x,i)=>norm(x+(j%2?1:-1)*term[i]*inv(j,p),p));}return result;}
function rational(a,b,n,p){const result=[];for(let i=0;i<n;++i){let x=a[i]||0;for(let j=1;j<=i&&j<b.length;++j)x=norm(x-b[j]*result[i-j],p);result.push(x*inv(b[0],p)%p);}return result;}
function pow(a,k,n,p){let result=pre([1],n);for(let i=0;i<k;++i)result=mul(result,a,p,n);return result;}
function binomial(n,k){let r=1;for(let i=1;i<=k;++i)r=r*(n-i+1)/i;return Math.round(r);}
function shifted(a,c,p){return a.map((_,i)=>{let sum=0;for(let j=i;j<a.length;++j){let power=1;for(let k=0;k<j-i;++k)power=power*c%p;sum=norm(sum+a[j]*binomial(j,i)*power,p);}return sum;});}
function check(raw){
 const data=M.run(raw),c=data.config,p=Number(c.p),a=values(c.a),b=values(c.b),n=c.n,k=Number(c.k),result=data.result===null?null:values(data.result);
 cases++;snapshots+=data.frames.length;
 if(c.mode==='polynomial'){
  const expected=c.operation==='multiply'?mul(a,b,p):Array.from({length:Math.max(a.length,b.length)},(_,i)=>norm((a[i]||0)+(c.operation==='subtract'?-1:1)*(b[i]||0),p));assert.deepEqual(result,expected);
 }else if(c.mode==='calculus')assert.deepEqual(result,c.calculus==='truncate'?pre(a,n):c.calculus==='derivative'?a.slice(1).map((x,i)=>x*(i+1)%p):[0,...a.map((x,i)=>x*inv(i+1,p)%p)]);
 else if(c.mode==='inv')assert.deepEqual(result,rational([1],a,n,p));
 else if(c.mode==='log')assert.deepEqual(result,logDefinition(a,n,p));
 else if(c.mode==='exp')assert.deepEqual(result,expDefinition(a,n,p));
 else if(c.mode==='pow')assert.deepEqual(result,pow(a,k,n,p));
 else if(c.mode==='sqrt'){
  const first=pre(a,n).findIndex(Boolean),hasRoot=first<0||(!(first%2)&&Array.from({length:p},(_,i)=>i*i%p).includes(a[first]));assert.equal(result!==null,hasRoot);
  if(result){assert.equal(result.length,n);assert.deepEqual(mul(result,result,p,n),pre(a,n));if(first>=0)assert.ok(result[first/2]<=p/2);}
 }else if(c.mode==='division'){
  const q=values(data.result),r=values(data.remainder),prod=mul(q,b,p);assert.deepEqual(pre(prod,Math.max(a.length,prod.length,r.length)).map((x,i)=>norm(x+(r[i]||0),p)),pre(a,Math.max(a.length,prod.length,r.length)));
  const degree=b.findLastIndex(Boolean);assert.ok(r.length<=degree);assert.ok(!q.length||q.at(-1));assert.ok(!r.length||r.at(-1));
 }else if(c.mode==='multipoint')assert.deepEqual(result,b.map(x=>polynomial(a,x,p)));
 else if(c.mode==='interpolation'){assert.equal(result.length,a.length);assert.deepEqual(a.map(x=>polynomial(result,x,p)),b);}
 else if(c.mode==='taylor')assert.deepEqual(result,shifted(a,Number(c.c),p));
 else if(c.mode==='bostan')assert.deepEqual(result,[rational(a,b,k+1,p)[k]]);
 for(const frame of data.frames){
  const source=frame.source?values(frame.source):[],current=frame.current?values(frame.current):[];
  if(frame.check==='inverse')assert.deepEqual(mul(source,current,p,frame.precision),pre([1],frame.precision));
  if(frame.check==='exp')assert.deepEqual(current,expDefinition(source,frame.precision,p));
  if(frame.check==='sqrt')assert.deepEqual(mul(current,current,p,frame.precision),pre(source,frame.precision));
  if(frame.check==='division'){
   const q=values(frame.quotient),r=values(frame.remainder),d=values(frame.divisor),count=Math.max(source.length,q.length+d.length-1,r.length),prod=mul(q,d,p,count);assert.deepEqual(prod.map((x,i)=>norm(x+(r[i]||0),p)),pre(source,count));
  }
  if(frame.check==='bostan-product')assert.ok(values(frame.denominatorProduct).every((x,i)=>!(i%2)||x===0));
  if(frame.check==='bostan-reduced')assert.equal(rational(values(frame.numerator),values(frame.denominator),Number(frame.k)+1,p)[Number(frame.k)],result[0]);
  if(frame.tree){const {base,count,nodes}=frame.tree,xs=c.mode==='multipoint'?b:a;
   for(let i=1;i<nodes.length;++i)if(nodes[i].length){let first=i,last=i;while(first<base){first*=2;last=2*last+1;}let expected=[1];for(let j=first-base;j<=last-base&&j<count;++j)expected=mul(expected,[norm(-xs[j],p),1],p);assert.deepEqual(values(nodes[i]),expected);}
   if(frame.remainders)for(let i=base;i<base+count;++i)if(frame.remainders[i].length)assert.equal(Number(frame.remainders[i][0]),polynomial(a,xs[i-base],p));
  }
  for(const row of frame.rows)for(const x of row.values)assert.ok(x>=0n&&x<c.p);
 }
 if(data.frames.length>2&&data.frames[0].rows[0].values.length){const original=data.frames[0].rows[0].values[0];data.frames.at(-1).rows[0].values[0]=123456789n;assert.equal(data.frames[0].rows[0].values[0],original);}
 return data;
}
for(const mode of M.MODES)check(M.defaults(mode));
for(const p of [17,97])for(let trial=0;trial<70;++trial){
 const n=random(8),a=Array.from({length:random(8)},()=>random(p*2)-p),b=Array.from({length:1+random(7)},()=>random(p*2)-p);
 for(const mode of M.MODES){const raw={...M.defaults(mode),modulus:String(p),a:a.join(' '),b:b.join(' '),precision:String(n),exponent:String(random(6)),shift:String(random(9)-4)};
  if(['inv','log','exp'].includes(mode)){const f=a.length?[...a]:[0];f[0]=mode==='inv'?1+random(p-1):mode==='log'?1:0;raw.a=f.join(' ');}
  if(['division','bostan'].includes(mode)){const d=b.map(x=>norm(x,p));d[0]=1;raw.b=d.join(' ');}
  if(mode==='interpolation'){const xs=Array.from({length:a.length},(_,i)=>i);raw.a=xs.join(' ');raw.b=a.join(' ');}
  if(mode==='bostan')raw.exponent=String(random(60));
  if(mode==='polynomial')for(const operation of ['add','subtract','multiply'])check({...raw,operation});
  else if(mode==='calculus')for(const calculus of ['derivative','integral','truncate'])check({...raw,calculus});
  else check(raw);
 }
}
for(let length=0,count=1;length<=4;++length,count*=3)for(let code=0;code<count;++code){let t=code;const a=Array.from({length},()=>{const v=t%3-1;t=Math.floor(t/3);return v;});for(let n=0;n<=8;++n){check({...M.defaults('sqrt'),a:a.join(' '),precision:String(n)});if(!n||a[0])check({...M.defaults('inv'),a:a.join(' '),precision:String(n)});}}
for(const mode of M.MODES){const raw=M.scenario(mode,'zero');if(mode==='inv'||mode==='log'){raw.precision='0';}check(raw);}
for(const [mode,change,message]of [
 ['inv',{a:'0'},'非0'],['log',{a:'2'},'定数項1'],['exp',{a:'1'},'定数項0'],['division',{b:'0 0'},'零多項式'],['bostan',{b:'0 1'},'非0'],['interpolation',{a:'1 18',b:'2 2'},'相異なる'],['interpolation',{a:'1',b:''},'長さ'],['inv',{precision:'9'},'精度'],['multipoint',{b:'0 1 2 3 4 5 6 7'},'点数'],['taylor',{a:'1 2 3 4 5 6 7 8 9'},'長さ'],['polynomial',{a:'NaN'},'整数'],['pow',{exponent:'18446744073709551616'},'範囲外'],['taylor',{shift:'9223372036854775808'},'範囲外'],['polynomial',{a:'2147483648'},'範囲外'],['calculus',{modulus:'97',a:Array(17).fill('1').join(' ')},'16要素']
]){assert.throws(()=>M.run({...M.defaults(mode),...change}),e=>e.message.includes(message));rejections++;}
const huge=M.run(M.scenario('pow','large-index'));assert.deepEqual(huge.result,Array(6).fill(0n));cases++;snapshots+=huge.frames.length;
// Fibonacci period modulo17 is36; uint64 maximum is15 modulo36, F15=15 mod17.
const distant=M.run(M.scenario('bostan','large-index'));assert.deepEqual(distant.result,[15n]);cases++;snapshots+=distant.frames.length;
console.log(`fps model: ${cases} inputs / ${snapshots} snapshots, ${rejections} rejected conditions; definitions of log/exp, brute roots, all tree products, division/precision invariants and uint64 limits passed`);
