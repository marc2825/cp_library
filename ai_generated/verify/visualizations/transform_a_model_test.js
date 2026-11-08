'use strict';
const assert=require('node:assert/strict'),{build,specs}=require('../../visualizations/transform_a/model.js');
let seed=316320,cases=0,states=0;const rng=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;},M=(x,p)=>p?(x%p+p)%p:x;
function test(mode,input){const b=build(mode,input);cases++;states+=b.frames.length;assert.equal(b.frames.at(-1).title,'完了');const saved=JSON.stringify(b.frames[0]);if(b.frames.at(-1).grid[0]?.length)b.frames.at(-1).grid[0][0]=9999;if(b.frames.length>1)assert.equal(JSON.stringify(b.frames[0]),saved);return b;}
function convolution(a,b,p=0){const c=a.length&&b.length?Array(a.length+b.length-1).fill(0):[];a.forEach((x,i)=>b.forEach((y,j)=>c[i+j]=M(c[i+j]+x*y,p)));return c;}
function determinantPoly(a,p){const n=a.length,answer=Array(n+1).fill(0);function permute(prefix,remaining){if(!remaining.length){let inversions=0;for(let i=0;i<n;i++)for(let j=i+1;j<n;j++)if(prefix[i]>prefix[j])inversions++;let poly=[inversions%2?-1:1];for(let i=0;i<n;i++)poly=convolution(poly,i===prefix[i]?[-a[i][i],1]:[-a[i][prefix[i]],0],p);poly.forEach((x,i)=>answer[i]=M(answer[i]+x,p));return;}for(const x of remaining)permute([...prefix,x],remaining.filter(y=>y!==x));}permute([],Array.from({length:n},(_,i)=>i));return answer;}
function span(rows,d,p){let s=new Map([[JSON.stringify(Array(d).fill(0)),Array(d).fill(0)]]);for(const row of rows){const next=new Map();for(const v of s.values())for(let c=0;c<p;c++){const w=v.map((x,i)=>M(x+c*row[i],p));next.set(JSON.stringify(w),w);}s=next;}return new Set(s.keys());}
function product(a,b,n,k,m,p){return Array.from({length:n},(_,i)=>Array.from({length:m},(_,j)=>M(Array.from({length:k},(_,z)=>a[i][z]*b[z][j]).reduce((s,x)=>s+x,0),p)));}
function solveEnumerate(a,b,m){const solutions=[];for(let mask=0;mask<2**m;mask++)if(a.every((row,i)=>row.reduce((s,x,j)=>s+x*(mask>>j&1),0)%2===b[i]))solutions.push(mask);return new Set(solutions);}
for(let t=0;t<160;t++){
 const a=Array.from({length:rng(17)},()=>rng(9)-4),b=Array.from({length:rng(17)},()=>rng(9)-4);assert.deepEqual(test('karatsuba',{a,b}).result,convolution(a,b));
}
for(let t=0;t<120;t++){
 const n=2**rng(5),a=Array.from({length:n},()=>rng(11)-5),b=Array.from({length:n},()=>rng(11)-5),p=[0,3,17][rng(3)];
 const w=Array.from({length:n},(_,i)=>M(a.reduce((sum,x,j)=>sum+((i&j).toString(2).replace(/0/g,'').length%2?-x:x),0),p));
 assert.deepEqual(test('fwht',{f:a,modulus:p}).result,w);assert.deepEqual(test('fwht',{f:w,modulus:p,inverse:true}).result,a.map(x=>M(x,p)));
 for(const operation of ['and','or','xor']){const want=Array(n).fill(0);a.forEach((x,i)=>b.forEach((y,j)=>{const k=operation==='and'?i&j:operation==='or'?i|j:i^j;want[k]=M(want[k]+x*y,p);}));assert.deepEqual(test('bitwise',{a,b,modulus:p,operation}).result,want);}
 const ranked=test('ranked',{f:a,modulus:p});let processed=0;
 for(const f of ranked.frames.filter(f=>f.stage)){processed|=f.bit;for(let k=0;k<f.grid.length;k++)for(let s=0;s<n;s++){const expected=a.reduce((sum,x,j)=>sum+((j&s)===j&&((s^j)&~processed)===0&&j.toString(2).replace(/0/g,'').length===k?x:0),0);assert.equal(f.grid[k][s],M(expected,p));}}
}
for(const p of [2,3,5,17])for(let t=0;t<60;t++){
 const n=rng(5),a=Array.from({length:n},()=>Array.from({length:n},()=>rng(p))),inverse=test('inverse',{a,modulus:p});const c=determinantPoly(a,p);
 assert.equal(inverse.result!==null,c[0]!==0||n===0);
 for(const f of inverse.frames){if(f.title==='完了'||f.grid.some(row=>row.length!==2*n))continue;const left=f.grid.map(row=>row.slice(0,n)),right=f.grid.map(row=>row.slice(n));assert.deepEqual(product(right,a,n,n,n,p),left);}
 if(inverse.result)assert.deepEqual(product(a,inverse.result,n,n,n,p),Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>+(i===j))));
 const polynomial=test('charpoly',{a,modulus:p});assert.deepEqual(polynomial.result,c);
 for(const f of polynomial.frames){if(f.similarity)assert.deepEqual(determinantPoly(f.grid,p),c);if(f.polynomial)assert.deepEqual(f.polynomial,determinantPoly(f.grid.slice(0,f.size).map(row=>row.slice(0,f.size)),p));}
}
for(const p of [2,3])for(let t=0;t<150;t++){
 const dimension=rng(5),u=Array.from({length:rng(5)},()=>Array.from({length:dimension},()=>rng(p))),v=Array.from({length:rng(5)},()=>Array.from({length:dimension},()=>rng(p))),s=span(u,dimension,p),z=span(v,dimension,p),want=new Set([...s].filter(x=>z.has(x))),basis=test('intersection',{u,v,dimension,modulus:p}).result;assert.deepEqual(span(basis,dimension,p),want);assert.equal(p**basis.length,want.size);
}
for(let mask=0;mask<64;mask++)for(let rhs=0;rhs<4;rhs++){
 const a=Array.from({length:2},(_,i)=>Array.from({length:3},(_,j)=>(mask>>(i*3+j))&1)),b=[rhs&1,rhs>>1],want=solveEnumerate(a,b,3),r=test('gf2',{a,b,variables:3}).result;
 assert.equal(r.has_solution,want.size>0);if(!r.has_solution)continue;
 const particular=r.particular.reduce((sum,x,j)=>sum+(x<<j),0),basis=span(r.nullspace_basis,3,2),actual=new Set([...basis].map(s=>JSON.parse(s).reduce((sum,x,j)=>sum+(x<<j),0)^particular));assert.deepEqual(actual,want);assert.equal(2**r.nullspace_basis.length,basis.size);assert.equal(r.rank+r.nullspace_basis.length,3);
}
for(const mode of Object.keys(specs))test(mode,specs[mode][1]);
const invalid=[['fwht',{f:[]}],['fwht',{f:[1,2,3]}],['fwht',{f:[1,0],inverse:true}],['fwht',{f:[1,1],inverse:true,modulus:2}],['fwht',{f:[1,2],inverse:'yes'}],['bitwise',{a:[1],b:[1,2],operation:'and'}],['bitwise',{a:[1],b:[1],operation:'bad'}],['ranked',{f:[]}],['inverse',{a:[[1,2]],modulus:17}],['inverse',{a:[[1]],modulus:12}],['gf2',{a:[[2]],b:[0],variables:1}],['gf2',{a:[[1]],b:[2],variables:1}],['intersection',{u:[[1]],v:[[1,2]],dimension:1,modulus:3}],['charpoly',{a:[[1]],modulus:0}],['karatsuba',{a:[Infinity],b:[1]}]];
for(const [mode,input]of invalid)assert.throws(()=>build(mode,input));
assert.deepEqual(build('inverse',{a:[],modulus:2}).result,[]);assert.deepEqual(build('charpoly',{a:[],modulus:2}).result,[1]);assert.deepEqual(build('intersection',{u:[],v:[],dimension:4,modulus:3}).result,[]);
console.log(`${cases} inputs / ${states} snapshots / ${invalid.length} rejection cases: definitions, rank layers, row equivalence, enumerated spaces and permutation polynomials passed`);
