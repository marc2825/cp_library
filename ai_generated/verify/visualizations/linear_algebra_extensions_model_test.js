'use strict';
const assert=require('node:assert/strict'),model=require('../../visualizations/linear_algebra_extensions/model.js'),expected=require('../linear_algebra_extensions/visualization_expected.json');
let seed=27;const rand=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const run=(id,input)=>model.trace(id,input),norm=(x,m)=>(x%m+m)%m;
for(const mode of model.modes){const before=JSON.stringify(mode.sample),t=run(mode.id,mode.sample);assert.deepEqual(t.result,expected[mode.id]);assert.equal(JSON.stringify(mode.sample),before);assert.ok(t.steps.length>=2);assert.notEqual(t.steps[0],t.steps.at(-1));assert.throws(()=>run(mode.id,null));}
function determinant(a,permanent=false){const n=a.length,used=Array(n).fill(false),p=[];let sum=0n;function go(){if(p.length===n){let term=1n,odd=false;for(let i=0;i<n;i++){term*=BigInt(a[i][p[i]]);for(let j=0;j<i;j++)if(p[j]>p[i])odd=!odd;}sum+=(odd&&!permanent)?-term:term;return;}for(let j=0;j<n;j++)if(!used[j]){used[j]=true;p.push(j);go();p.pop();used[j]=false;}}go();return sum;}
const multiply=(a,b)=>a.map(row=>Array.from({length:b[0]?.length||0},(_,j)=>row.reduce((v,x,k)=>v+BigInt(x)*BigInt(b[k][j]),0n)));
const bitProduct=(a,b)=>{let r=0n;for(;b;b>>=1n,a<<=1n)if(b&1n)r^=a;return r;};
for(let t=0;t<100;t++){
 const n=1+rand(4),a=Array.from({length:n},()=>Array.from({length:n},()=>rand(11)-5)),d=determinant(a),m=1n+BigInt(rand(10000));
 assert.deepEqual(run('det_integer',{a}).result,[String(d)]);assert.deepEqual(run('det_mod',{a,modulus:String(m)}).result,[String(norm(d,m))]);assert.deepEqual(run('permanent',{a}).result,[String(determinant(a,true))]);
 const charpoly=run('charpoly',{a,prime:101}).result.map(BigInt);assert.equal(charpoly.length,n+1);assert.equal(charpoly.at(-1),1n);
 for(let x=0n;x<=BigInt(n);x++){const matrix=a.map((row,i)=>row.map((y,j)=>(i===j?x:0n)-BigInt(y))),evaluation=charpoly.reduceRight((s,c)=>(s*x+c)%101n,0n);assert.equal(evaluation,norm(determinant(matrix),101n));}
 const h=run('hermite',{a}).meta;assert.deepEqual(multiply(h.u,a),h.a);assert.ok([1n,-1n].includes(determinant(h.u)));
 let last=-1;for(let i=0;i<n;i++){const col=h.a[i].findIndex(x=>x!==0n);if(col<0){for(let k=i;k<n;k++)assert.ok(h.a[k].every(x=>x===0n));break;}assert.ok(col>last);last=col;assert.ok(h.a[i][col]>0n);for(let r=0;r<i;r++)assert.ok(h.a[r][col]>=0n&&h.a[r][col]<h.a[i][col]);}
 const A=[[rand(7)-3,rand(7)-3],[rand(7)-3,rand(7)-3]],b=[rand(7)-3,rand(7)-3],modulus=1+rand(6),s=run('smith',{a:A,b,modulus:String(modulus)});
 assert.deepEqual(multiply(multiply(s.meta.u,A),s.meta.v),s.meta.a);assert.ok([1n,-1n].includes(determinant(s.meta.u)));assert.ok([1n,-1n].includes(determinant(s.meta.v)));
 let count=0;for(let x=0;x<modulus;x++)for(let y=0;y<modulus;y++)if(A.every((row,i)=>((row[0]*x+row[1]*y-b[i])%modulus+modulus)%modulus===0))count++;assert.equal(s.result.at(-1),String(count));
 const binaryA=BigInt(rand(1<<20)),binaryB=1n+BigInt(rand(1<<20)),g=run('gf2',{a:String(binaryA),b:String(binaryB)}).result.map(BigInt);assert.equal(g[0],bitProduct(binaryA,binaryB));assert.equal(bitProduct(g[1],binaryB)^g[2],binaryA);assert.ok(g[2]<binaryB);
 let x=[rand(3),rand(3)];if(x.every(v=>v===0))x[0]=1;const y=[rand(3),rand(3)],e=run('extension',{prime:3,modulus:[1,0,1],a:x,b:y}).result.map(BigInt);
 assert.equal(e[0],norm(BigInt(x[0]*y[0]-x[1]*y[1]),3n));assert.equal(e[1],norm(BigInt(x[0]*y[1]+x[1]*y[0]),3n));assert.equal(norm(BigInt(x[0])*e[2]-BigInt(x[1])*e[3],3n),1n);assert.equal(norm(BigInt(x[0])*e[3]+BigInt(x[1])*e[2],3n),0n);
 const weights=Array.from({length:6},()=>rand(13)-6),[v,w,z,r,q,p]=weights,skew=[[0,v,w,z],[-v,0,r,q],[-w,-r,0,p],[-z,-q,-p,0]],symmetric=skew.map((row,i)=>row.map((value,j)=>j<i?-value:value));
 assert.deepEqual(run('pfaffian',{a:skew,prime:101}).result,[String(norm(BigInt(v*p-w*q+z*r),101n))]);assert.deepEqual(run('hafnian',{a:symmetric}).result,[String(v*p+w*q+z*r)]);
 const f=run('frobenius',{a:A,prime:5}).meta;let polynomial=[1n];for(const factor of f.factors){const product=Array(polynomial.length+factor.length-1).fill(0n);polynomial.forEach((x,i)=>factor.forEach((y,j)=>product[i+j]=(product[i+j]+x*y)%5n));polynomial=product;}assert.deepEqual(polynomial,run('charpoly',{a:A,prime:5}).result.map(BigInt));
 const raw=[[null,rand(9)-4],[rand(9)-4,null]],power=rand(6);let dp=[[0n,null],[null,0n]];
 for(let k=0;k<power;k++){let next=[[null,null],[null,null]];for(let i=0;i<2;i++)for(let j=0;j<2;j++)for(let q=0;q<2;q++)if(dp[i][q]!==null&&raw[q][j]!==null){const w=dp[i][q]+BigInt(raw[q][j]);if(next[i][j]===null||next[i][j]>w)next[i][j]=w;}dp=next;}
 assert.deepEqual(run('semiring',{a:raw,operation:'min',exponent:power}).result,dp.flat().map(x=>x===null?'inf':String(x)));
}
for(const [id,input]of[['det_mod',{a:[],modulus:'0'}],['permanent',{a:[[1,2]]}],['pfaffian',{a:[[1]],prime:3}],['hafnian',{a:[[0,1],[2,0]]}],['smith',{a:[[1]],b:[],modulus:'2'}],['hermite',{a:[[1,2],[1]]}],['gf2',{a:'1',b:'0'}],['extension',{a:[1],b:[1],prime:3,modulus:[2,0,1]}],['frobenius',{a:[[1,2]],prime:3}],['charpoly',{a:[],prime:4}],['semiring',{a:[],operation:'xor',exponent:2}]])assert.throws(()=>run(id,input));
console.log('linear algebra extensions model: 12 C++ fixtures, 100 independent randomized batches and 11 invalid inputs passed');
