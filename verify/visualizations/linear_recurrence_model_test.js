'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/linear_recurrence/model.js');
let seed=318018,cases=0,snapshots=0,rejections=0;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const norm=(x,p)=>(x%p+p)%p,num=a=>a.map(Number),pre=(a,n)=>Array.from({length:n},(_,i)=>a[i]||0);
function inverse(x,p){for(let y=1;y<p;++y)if(x*y%p===1)return y;throw Error('inverse missing');}
function sequence(initial,c,n,p){const a=initial.map(x=>norm(x,p));for(let i=c.length;i<n;++i)a.push(c.reduce((r,x,j)=>norm(r+x*a[i-1-j],p),0));return a;}
function fits(a,c,p){return a.every((x,i)=>i<c.length||x===c.reduce((r,y,j)=>norm(r+y*a[i-1-j],p),0));}
function minimum(a,p){for(let d=0;d<=a.length;++d){const rows=a.slice(d).map((x,i)=>[...Array.from({length:d},(_,j)=>a[i+d-1-j]),x]);let rank=0;for(let col=0;col<d;++col){const pivot=rows.findIndex((row,i)=>i>=rank&&row[col]);if(pivot<0)continue;[rows[rank],rows[pivot]]=[rows[pivot],rows[rank]];const scale=inverse(rows[rank][col],p);rows[rank]=rows[rank].map(x=>x*scale%p);for(let i=rank+1;i<rows.length;++i){const factor=rows[i][col];rows[i]=rows[i].map((x,j)=>norm(x-factor*rows[rank][j],p));}rank++;}if(rows.every(row=>row.slice(0,d).some(Boolean)||!row[d]))return d;}throw Error('order');}
function multiply(a,b,p){if(!a.length||!b.length)return [];const c=Array(a.length+b.length-1).fill(0);for(let i=0;i<a.length;++i)for(let j=0;j<b.length;++j)c[i+j]=norm(c[i+j]+a[i]*b[j],p);return c;}
function matrixApply(a,s,p){return a.map(row=>row.reduce((r,x,i)=>norm(r+x*s[i],p),0));}
function step(a,s,k,p){for(let i=0;i<k;++i)s=matrixApply(a,s,p);return s;}
function weights(c,n,p){return c.map((_,i)=>sequence(c.map((__,j)=>i===j?1%p:0),c,n+1,p)[n]);}
function polynomialWeights(a,c,p){return c.map((_,j)=>{const s=sequence(c.map((__,i)=>i===j?1%p:0),c,a.length,p);return a.reduce((r,x,i)=>norm(r+x*s[i],p),0);});}
function rational(a,b,k,p){const s=[];for(let i=0;i<=k;++i){let x=a[i]||0;for(let j=1;j<b.length&&j<=i;++j)x=norm(x-b[j]*s[i-j],p);s.push(x*inverse(b[0],p)%p);}return s[k];}
function check(raw){
 const data=M.run(raw),c=data.config,a=num(c.a),b=num(c.b),p=Number(c.p),n=Number(c.n),result=num(data.result);cases++;snapshots+=data.frames.length;
 if(c.mode==='berlekamp'){
  assert.equal(result.length,minimum(a,p));assert.ok(fits(a,result,p));const value=n<a.length?a[n]:result.length?sequence(a.slice(0,result.length),result,n+1,p)[n]:0;assert.equal(Number(data.target),value);
 }else if(c.mode==='matrix'&&c.source==='custom')assert.deepEqual(result,step(c.matrix.map(num),a,n,p));
 else if(c.mode==='bostan'&&c.source==='rational')assert.deepEqual(result,[rational(a,b,n,p)]);
 else assert.deepEqual(result,[sequence(a,b,Math.max(a.length,n+1),p)[n]]);
 for(const f of data.frames){
  if(f.matrix&&f.matrix.length){const transition=f.transition.map(num),matrix=f.matrix.map(num),steps=2**f.bit;
   const expected=transition.map((_,i)=>step(transition,transition.map((__,j)=>i===j?1%p:0),steps,p));
   assert.deepEqual(matrix.map((_,i)=>matrix.map(row=>row[i])),expected);
   if(f.stage==='matrix-apply')assert.deepEqual(num(f.state),step(transition,num(f.start),Number(f.processed),p));
  }
  if(f.stage==='weights')assert.deepEqual(num(f.weights),weights(b,Number(f.exponent),p));
  if(f.stage==='reduce'){
   const original=multiply(num(f.left),num(f.right),p),current=num(f.rows[0].values);assert.deepEqual(polynomialWeights(current,b,p),polynomialWeights(original,b,p));assert.equal(current[f.active],0);
  }
  if(f.stage==='discrepancy'){
   const C=num(f.C);let d=a[f.index];for(let j=1;j<=f.L;++j)d=norm(d+C[j]*a[f.index-j],p);assert.equal(Number(f.delta),d);for(const term of f.terms)assert.equal(Number(term.value),Number(term.c)*a[term.index]%p);
  }
  if(['bm-corrected','bm-stable'].includes(f.stage)){const coef=num(f.C).slice(1,f.L+1).map(x=>norm(-x,p)),prefix=a.slice(0,f.index+1);assert.ok(fits(prefix,coef,p));assert.equal(f.L,minimum(prefix,p));}
  if(f.stage==='generating'){
   const Q=num(f.Q),P=num(f.P),s=sequence(a,b,2*a.length+1,p),z=multiply(s,Q,p);assert.deepEqual(pre(P,a.length),z.slice(0,a.length));assert.ok(z.slice(a.length,s.length).every(x=>!x));
  }
  if(f.stage==='bostan-product')assert.ok(num(f.denominatorProduct).every((x,i)=>!(i%2)||!x));
  if(f.stage==='bostan-reduced')assert.equal(rational(num(f.P),num(f.Q),Number(f.k),p),result[0]);
  for(const row of f.rows)for(const x of row.values)assert.ok(x>=0n&&x<c.p);
 }
 if(data.frames.length>2&&data.frames[0].rows[0].values.length){const first=data.frames[0].rows[0].values[0];data.frames.at(-1).rows[0].values[0]=100000n;assert.equal(data.frames[0].rows[0].values[0],first);}
 return data;
}
for(const mode of M.MODES)check(M.defaults(mode));
for(const p of [1,2,12,17,97])for(let trial=0;trial<60;++trial){const d=1+random(6),a=Array.from({length:d},()=>random(2*p)-p),b=Array.from({length:d},()=>random(2*p)-p),n=random(48),raw={a:a.join(' '),b:b.join(' '),n:String(n),modulus:String(p)};
 for(const mode of ['matrix','kitamasa'])check({...M.defaults(mode),...raw});
 const dim=random(5),matrix=Array.from({length:dim},()=>Array.from({length:dim},()=>random(2*p)-p));check({...M.defaults('matrix'),source:'custom',a:Array(dim).fill(2).join(' '),matrix:matrix.map(row=>row.join(' ')).join('\n'),n:String(n),modulus:String(p)});
 if([2,17,97].includes(p))check({...M.defaults('berlekamp'),a:sequence(a.map(x=>norm(x,p)),b.map(x=>norm(x,p)),2*d,p).join(' '),n:String(n),modulus:String(p)});
 if([17,97].includes(p)){check({...M.defaults('bostan'),...raw});check({...M.defaults('bostan'),source:'rational',a:a.join(' '),b:[1,...b.slice(1)].join(' '),n:String(n),modulus:String(p)});}
}
for(const p of [2,3])for(let size=0,count=1;size<=(p===2?9:6);++size,count*=p)for(let code=0;code<count;++code){let value=code;const a=Array.from({length:size},()=>{const x=value%p;value=Math.floor(value/p);return x;});if(p===3){/* Map to mod17 still exhausts ternary observations independently. */}check({...M.defaults('berlekamp'),a:a.join(' '),modulus:p===2?'2':'17',n:String(size+3)});}
for(const [mode,change,message]of [
 ['matrix',{a:'',b:''},'非空'],['kitamasa',{a:'1',b:'1 2'},'同じ'],['matrix',{source:'custom',matrix:'1 2\n3',a:'1 2'},'正方'],['matrix',{source:'custom',matrix:'1',a:''},'一致'],['berlekamp',{modulus:'12'},'素数'],['berlekamp',{modulus:'1'},'素数'],['bostan',{modulus:'1000000007'},'法は'],['bostan',{source:'rational',a:'1',b:'0 1'},'非0'],['bostan',{source:'rational',a:Array(9).fill('1').join(' '),b:'1'},'有効長'],['kitamasa',{n:'-1'},'範囲外'],['kitamasa',{n:'18446744073709551616'},'範囲外'],['matrix',{a:'2147483648 1'},'範囲外'],['berlekamp',{a:'NaN'},'整数'],['berlekamp',{a:Array(21).fill('1').join(' ')},'20項'],['kitamasa',{a:Array(7).fill('1').join(' '),b:Array(7).fill('1').join(' ')},'6項']
]){assert.throws(()=>M.run({...M.defaults(mode),...change}),e=>e.message.includes(message));rejections++;}
for(const mode of M.MODES){const data=M.run(M.scenario(mode,'huge'));cases++;snapshots+=data.frames.length;assert.deepEqual(data.result,mode==='berlekamp'?[1n,1n]:[15n]);if(mode==='berlekamp')assert.equal(data.target,15n);}
for(const mode of ['matrix','kitamasa']){const raw={...M.defaults(mode),a:'-2147483648',b:'1',n:'18446744073709551615',modulus:'1000000007'};const data=M.run(raw);cases++;snapshots+=data.frames.length;assert.deepEqual(data.result,[852516373n]);}
console.log(`linear recurrence model: ${cases} inputs / ${snapshots} snapshots, ${rejections} rejected inputs; direct transitions, every polynomial reduction, Gaussian minimum orders, BM discrepancies and generating-function invariants passed`);
