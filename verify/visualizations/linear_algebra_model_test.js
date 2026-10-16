'use strict';
const assert=require('node:assert/strict'),M=require('../../visualizations/linear_algebra/model.js');
let seed=320020,inputs=0,snapshots=0,rejections=0;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const norm=(x,p)=>p?(x%p+p)%p:x;
function dot(a,b,p,real=false){const c={rows:a.rows,cols:b.cols,data:[]};for(let i=0;i<a.rows;++i){const row=[];for(let j=0;j<b.cols;++j){let sum=real?0:0n;for(let k=0;k<a.cols;++k)sum+=a.data[i][k]*b.data[k][j];row.push(real?sum:norm(sum,p));}c.data.push(row);}return c;}
function identity(n,p,real=false){return {rows:n,cols:n,data:Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>real?Number(i===j):norm(BigInt(i===j),p)))};}
function power(a,k,p){let result=identity(a.rows,p);for(let i=0n;i<k;++i)result=dot(result,a,p);return result;}
function span(a){let s=new Set([0n]);for(const x of a){const next=new Set(s);for(const y of s)next.add(x^y);s=next;}return [...s].sort((a,b)=>a<b?-1:a>b?1:0);}
function assignments(n,p){const out=[];for(let code=0;code<Number(p)**n;++code){let value=code;out.push(Array.from({length:n},()=>{const x=BigInt(value%Number(p));value=Math.floor(value/Number(p));return x;}));}return out;}
function close(a,b){assert.equal(a.rows,b.rows);assert.equal(a.cols,b.cols);for(let i=0;i<a.rows;++i)for(let j=0;j<a.cols;++j)if(typeof a.data[i][j]==='number')assert.ok(Math.abs(a.data[i][j]-b.data[i][j])<1e-7,`${a.data[i][j]} != ${b.data[i][j]}`);else assert.equal(a.data[i][j],b.data[i][j]);}
function check(raw,cycle=false){const d=M.run(raw),c=d.config;inputs++;snapshots+=d.frames.length;
 if(c.mode==='matrix'){
  const t={rows:c.a.cols,cols:c.a.rows,data:Array.from({length:c.a.cols},(_,j)=>Array.from({length:c.a.rows},(_,i)=>c.a.data[i][j]))};assert.deepEqual(d.result,t);
  for(const f of d.frames){const copied=f.cell?f.cell[0]*c.a.cols+f.cell[1]:f.stage==='done'?c.a.rows*c.a.cols-1:-1;for(let i=0;i<c.a.rows;++i)for(let j=0;j<c.a.cols;++j)assert.equal(f.target.data[j][i],i*c.a.cols+j<=copied?c.a.data[i][j]:0n);}
 }else if(c.mode==='product'){
  assert.deepEqual(d.result,dot(c.a,c.b,c.p));
  for(const f of d.frames)for(let i=0;i<c.rows;++i)for(let j=0;j<c.cols;++j){const pos=i*c.cols+j,current=f.cell?f.cell[0]*c.cols+f.cell[1]:-1;let terms=f.stage==='done'||pos<current?c.inner:pos>current?0:f.stage==='term'?f.k+1:f.stage==='complete'?c.inner:0;let value=0n;for(let k=0;k<terms;++k)value+=c.a.data[i][k]*c.b.data[k][j];assert.equal(f.out.data[i][j],norm(value,c.p));}
 }else if(c.mode==='power'){
  const expected=k=>cycle?{rows:c.rows,cols:c.rows,data:Array.from({length:c.rows},(_,i)=>Array.from({length:c.rows},(_,j)=>norm(BigInt((j-i+c.rows)%c.rows===Number(k%BigInt(c.rows))),c.p)))}:power(c.a,k,c.p);
  assert.deepEqual(d.result,expected(c.exponent));for(const f of d.frames){assert.equal(f.used+f.weight*f.remaining,c.exponent);assert.deepEqual(f.acc,expected(f.used));assert.deepEqual(f.base,expected(f.weight));}
 }else if(c.mode==='gauss'){
  const augmented={rows:c.rows,cols:c.cols+1,data:c.a.data.map((row,i)=>[...row,c.rhs[i]])};for(const f of d.frames)close(f.r,dot(f.transform,augmented,c.p,c.real));
  const r=d.result;
  if(!c.real&&c.p<=17n){const all=assignments(c.cols,c.p),fits=x=>c.a.data.every((row,i)=>norm(row.reduce((s,a,j)=>s+a*x[j],0n)-c.rhs[i],c.p)===0n),solutions=all.filter(fits),zero=all.filter(x=>c.a.data.every(row=>norm(row.reduce((s,a,j)=>s+a*x[j],0n),c.p)===0n));
   let nullity=0;for(let count=zero.length;count>1;count/=Number(c.p))nullity++;assert.equal(r.rank,c.cols-nullity);assert.equal(r.status,!solutions.length?'none':solutions.length===1?'unique':'multiple');
   if(solutions.length){assert.equal(r.basis.length,nullity);const generated=assignments(r.basis.length,c.p).map(scales=>r.particular.map((x,j)=>norm(x+r.basis.reduce((sum,v,i)=>sum+scales[i]*v[j],0n),c.p))).map(x=>x.join(',')).sort();assert.deepEqual(generated,solutions.map(x=>x.join(',')).sort());}
  }else if(r.status!=='none'){
   assert.equal(r.basis.length,c.cols-r.rank);const tolerance=c.real?1e-7:0;
   for(let i=0;i<c.rows;++i){const sum=c.a.data[i].reduce((s,a,j)=>s+a*r.particular[j],c.real?0:0n);if(c.real)assert.ok(Math.abs(sum-c.rhs[i])<tolerance);else assert.equal(norm(sum-c.rhs[i],c.p),0n);}
   for(const v of r.basis)for(const row of c.a.data){const sum=row.reduce((s,a,j)=>s+a*v[j],c.real?0:0n);if(c.real)assert.ok(Math.abs(sum)<tolerance);else assert.equal(norm(sum,c.p),0n);}
  }
 }else{
  const all=span(c.vectors),coset=all.map(x=>x^c.seed),low=coset.reduce((a,b)=>a<b?a:b),high=coset.reduce((a,b)=>a>b?a:b);assert.equal(d.result,c.operation==='contains'?all.includes(c.seed):c.operation==='min'?low:high);
  for(const f of d.frames){const basis=f.pivots.filter(x=>x);assert.equal(basis.length,f.rank);for(let bit=0;bit<64;++bit)if(f.pivots[bit])assert.equal(f.pivots[bit].toString(2).length-1,bit);const pending=f.phase==='insert'&&f.value!==undefined?[f.value]:[];assert.deepEqual(span([...basis,...pending]),span(f.prefix));if(f.stage==='query'){const processed=f.pivots.filter((x,bit)=>x&&bit>=f.bit),values=span(processed).map(x=>x^c.seed);assert.equal(f.value,values.reduce((a,b)=>c.operation==='max'?(a>b?a:b):(a<b?a:b)));}}
 }
 if(d.frames.length>1){const first=structuredClone(d.frames[0]);const last=d.frames.at(-1);if(c.mode==='xor')last.pivots[0]=99n;else if(c.mode==='gauss'&&last.r.rows&&last.r.cols)last.r.data[0][0]=999;else if(last.out?.rows&&last.out.cols)last.out.data[0][0]=999n;else if(last.target?.rows&&last.target.cols)last.target.data[0][0]=999n;else if(last.acc?.rows&&last.acc.cols)last.acc.data[0][0]=999n;assert.deepEqual(d.frames[0],first);}
 return d;
}
for(const mode of M.MODES)check(M.defaults(mode));
for(let n=0;n<=3;++n)for(let k=0;k<=3;++k)for(let m=0;m<=3;++m)for(const p of ['integer','12','1']){const a=Array.from({length:n*k},()=>random(9)-4),b=Array.from({length:k*m},()=>random(9)-4);check({...M.defaults('product'),rows:String(n),inner:String(k),cols:String(m),a:a.join(' '),b:b.join(' '),modulus:p});if(k===0)check({...M.defaults('matrix'),rows:String(n),cols:String(m),a:Array.from({length:n*m},()=>random(9)-4).join(' '),modulus:p});}
for(let trial=0;trial<120;++trial){const n=random(4),p=['integer','12','998244353','1'][trial%4],a=Array.from({length:n*n},()=>random(3)-1);check({...M.defaults('power'),rows:String(n),a:a.join(' '),exponent:String(random(13)),modulus:p});}
for(const p of ['integer','12','1'])check({...M.scenario('power','huge'),modulus:p},true);
for(const p of [2])for(let n=0;n<=2;++n)for(let m=0;m<=2;++m){const entries=n*(m+1);for(let code=0;code<p**entries;++code){let x=code;const a=[],b=[];for(let i=0;i<n;++i){for(let j=0;j<m;++j){a.push(x%p);x=Math.floor(x/p);}b.push(x%p);x=Math.floor(x/p);}check({...M.defaults('gauss'),rows:String(n),cols:String(m),a:a.join(' '),rhs:b.join(' '),modulus:String(p)});}}
for(let trial=0;trial<180;++trial){const n=random(4),m=random(4),a=Array.from({length:n*m},()=>random(7)-3),b=Array.from({length:n},()=>random(7)-3);check({...M.defaults('gauss'),rows:String(n),cols:String(m),a:a.join(' '),rhs:b.join(' '),modulus:trial%3===0?'real':trial%3===1?'17':'998244353'});}
for(const type of ['unique','inconsistent','swap','tiny','empty','zero'])check(M.scenario('gauss',type));
for(let n=0;n<=3;++n)for(let code=0;code<8**n;++code){let x=code;const a=Array.from({length:n},()=>{const value=x%8;x=Math.floor(x/8);return value;});for(const operation of ['min','max','contains'])check({...M.defaults('xor'),vectors:a.join(' '),seed:String(code%16),operation});}
for(const operation of ['min','max','contains'])check({...M.scenario('xor','highest'),operation});
for(const [mode,change,message]of [
 ['matrix',{rows:'6'},'範囲外'],['matrix',{cols:'-1'},'範囲外'],['matrix',{a:'1 2'},'6項'],['product',{b:'1 2'},'9項'],['matrix',{a:'1 2 3 4 5 NaN'},'整数'],['power',{exponent:'-1'},'範囲外'],['power',{exponent:'18446744073709551616'},'範囲外'],['power',{rows:'1',a:'9223372036854775807',exponent:'2'},'long long'],['product',{rows:'1',inner:'1',cols:'1',a:'9223372036854775807',b:'2'},'long long'],['gauss',{rhs:'1'},'2項'],['gauss',{modulus:'12'},'素数'],['gauss',{epsilon:'0'},'epsilon'],['gauss',{a:'1 2 3 4 5 Infinity'},'有限'],['gauss',{rows:'1',cols:'1',a:'1e-100',rhs:'1e300',epsilon:'1e-110'},'非有限'],['xor',{vectors:'-1'},'範囲外'],['xor',{vectors:Array(13).fill('1').join(' ')},'12個'],['xor',{seed:'18446744073709551616'},'範囲外']
]){assert.throws(()=>M.run({...M.defaults(mode),...change}),e=>e.message.includes(message));rejections++;}
console.log(`linear algebra model: ${inputs} inputs / ${snapshots} snapshots / ${rejections} rejections; direct dot prefixes/transposes/repeated powers, row-equivalence and complete solution sets, XOR subset spaces and greedy cosets passed`);
