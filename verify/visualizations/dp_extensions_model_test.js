'use strict';
const assert=require('node:assert/strict');
const model=require('../../visualizations/dp_extensions/model.js');
const expected=require('../dp_extensions/visualization_expected.json');
let seed=57193,traces=0;const rand=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
function run(id,input){const trace=model.trace(id,input);traces++;assert.ok(trace.steps.length>=2);for(const step of trace.steps){assert.equal(typeof step.title,'string');assert.equal(typeof step.detail,'string');assert.ok(step.rows.every(r=>r.length===step.columns.length));}return trace.result;}
for(const mode of model.modes)assert.deepEqual(run(mode.id,mode.sample),expected[mode.id]);
function sequence(a,b,edit){const d=Array.from({length:a.length+1},()=>Array(b.length+1).fill(0));if(edit){for(let i=0;i<=a.length;i++)d[i][0]=i;for(let j=0;j<=b.length;j++)d[0][j]=j;}for(let i=1;i<=a.length;i++)for(let j=1;j<=b.length;j++)d[i][j]=edit?Math.min(d[i-1][j]+1,d[i][j-1]+1,d[i-1][j-1]+Number(a[i-1]!==b[j-1])):Math.max(d[i-1][j],d[i][j-1],d[i-1][j-1]+Number(a[i-1]===b[j-1]));return d[a.length][b.length];}
for(let t=0;t<140;t++){const a=Array.from({length:rand(150)},()=>String.fromCharCode(rand(5))).join(''),b=Array.from({length:rand(150)},()=>String.fromCharCode(rand(5))).join('');assert.equal(+run('bit_parallel_lcs',{a,b})[0],sequence(a,b,false));assert.equal(+run('myers_edit_distance',{a,b})[0],sequence(a,b,true));}
for(let t=0;t<100;t++){
  const n=2+rand(5),edges=[];for(let i=0;i<9;i++){const a=rand(n),b=rand(n);if(a!==b)edges.push([a,b]);}let paths=0,cycles=0;
  for(let mask=0;mask<(1<<edges.length);mask++){const degrees=Array(n).fill(0),g=Array.from({length:n},()=>[]);edges.forEach(([a,b],i)=>{if(mask>>i&1){degrees[a]++;degrees[b]++;g[a].push(b);g[b].push(a);}});const seen=new Set([0]),q=[0];for(let i=0;i<q.length;i++)for(const v of g[q[i]])if(!seen.has(v)){seen.add(v);q.push(v);}if(seen.size<n)continue;if(degrees.every(d=>d===2))cycles++;if(degrees.every((d,v)=>d===(v===0||v===n-1?1:2)))paths++;}
  assert.deepEqual(run('frontier_dp',{n,edges,source:0,target:n-1,kind:t%2?'path':'cycle'}),[paths,cycles].map(String));
}
for(let t=0;t<100;t++){
  const primary=1+rand(5),secondary=rand(3),rows=Array.from({length:rand(10)},()=>{const mask=rand(1<<(primary+secondary))|(1<<rand(primary));return Array.from({length:primary+secondary},(_,i)=>i).filter(i=>mask>>i&1);});let count=0;
  for(let mask=0;mask<(1<<rows.length);mask++){const times=Array(primary+secondary).fill(0);rows.forEach((r,i)=>{if(mask>>i&1)r.forEach(c=>times[c]++);});if(times.every((x,c)=>c<primary?x===1:x<=1))count++;}
  const result=run('exact_cover',{primary,secondary,rows});assert.equal(+result[0],count);if(count){const times=Array(primary+secondary).fill(0);for(const row of result.slice(1))rows[+row].forEach(c=>times[c]++);assert.ok(times.every((x,c)=>c<primary?x===1:x<=1));}
}
for(let t=0;t<100;t++){
  const n=2+rand(5),edges=Array.from({length:10},()=>[rand(n),rand(n),rand(5)]),source=rand(n),target=rand(n),d=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?0:Infinity));edges.forEach(([a,b,w])=>d[a][b]=Math.min(d[a][b],w));for(let k=0;k<n;k++)for(let i=0;i<n;i++)for(let j=0;j<n;j++)d[i][j]=Math.min(d[i][j],d[i][k]+d[k][j]);const h=d.map(row=>Number.isFinite(row[target])?rand(row[target]+1):0);
  for(const mode of ['a_star','ida_star']){const r=run(mode,{n,edges,source,target,h});assert.equal(r[0],Number.isFinite(d[source][target])?String(d[source][target]):'null');if(r[0]!=='null'){assert.equal(+r[1],source);assert.equal(+r.at(-1),target);let cost=0;for(let i=1;i+1<r.length;i++){const options=edges.filter(e=>e[0]===+r[i]&&e[1]===+r[i+1]);assert.ok(options.length);cost+=Math.min(...options.map(e=>e[2]));}assert.equal(cost,+r[0]);}}
  const unit=edges.map(([a,b])=>[a,b,1]),q=[source],distance=Array(n).fill(-1);distance[source]=0;for(let i=0;i<q.length;i++)for(const [a,b]of unit)if(a===q[i]&&distance[b]<0){distance[b]=distance[a]+1;q.push(b);}const r=run('bidirectional_search',{n,edges:unit,source,target});assert.equal(r[0],distance[target]<0?'null':String(distance[target]));
}
for(let t=0;t<100;t++){
  const n=1+rand(7),clauses=Array.from({length:rand(15)},()=>Array.from({length:rand(5)},()=>(1+rand(n))*(rand(2)?1:-1))),assumptions=[(1+rand(n))*(rand(2)?1:-1)];
  const satisfiable=cs=>{for(let mask=0;mask<(1<<n);mask++)if(cs.every(c=>c.some(x=>Boolean(mask>>(Math.abs(x)-1)&1)===(x>0))))return true;return false;};assert.deepEqual(run('sat_solver',{n,clauses,assumptions}),[satisfiable(clauses),satisfiable([...clauses,...assumptions.map(x=>[x])])].map(x=>String(+x)));
}
function subsets(items,capacity){let best=0,count=0,chosen=[];for(let mask=0;mask<(1<<items.length);mask++){let weight=0,value=0;items.forEach(([w,v],i)=>{if(mask>>i&1){weight+=w;value+=v;}});if(weight>capacity)continue;if(value>best){best=value;count=0;chosen=[];}if(value===best){count++;chosen.push(mask);}}return {best,count,chosen};}
function subsequences(a){const result=[];for(let m=0;m<(1<<a.length);m++)result.push(a.filter((_,j)=>m>>j&1));return result;}
for(let t=0;t<100;t++){
  const capacity=rand(16),items=Array.from({length:rand(8)},()=>[rand(7),rand(16)-6]),oracle=subsets(items,capacity),weights=items.map(x=>x[0]),target=rand(capacity+1);
  const r=run('knapsack_reconstruction',{capacity,items,weights,target});assert.equal(+r[0],oracle.best);let mask=0;r.slice(1,items.length+1).forEach((c,i)=>{assert.ok(c==='0'||c==='1');mask|=(+c)<<i;});assert.ok(oracle.chosen.includes(mask));const ids=r.slice(items.length+1);if(ids[0]!=='null'){assert.equal(new Set(ids).size,ids.length);assert.equal(ids.reduce((s,id)=>s+weights[+id],0),target);}
  assert.equal(+run('small_weight_knapsack',{capacity,items})[0],oracle.best);
  const seq=Array.from({length:rand(7)},()=>rand(4)),a=Array.from({length:rand(7)},()=>String(rand(3))).join(''),b=Array.from({length:rand(7)},()=>String(rand(3))).join('');let lis=0,lc=0,lcs=0,cc=0;for(const s of subsequences(seq))if(s.every((x,i)=>i===0||s[i-1]<x)){if(s.length>lis){lis=s.length;lc=0;}if(s.length===lis)lc++;}for(const s of subsequences([...a]))for(const u of subsequences([...b]))if(s.join('')===u.join('')){if(s.length>lcs){lcs=s.length;cc=0;}if(s.length===lcs)cc++;}
  assert.deepEqual(run('optimal_solution_count',{sequence:seq,a,b,capacity,items}),[lis,lc,lcs,cc,oracle.best,oracle.count].map(String));
}
for(let t=0;t<100;t++){const n=1+rand(4),k=1+rand(6),m=1+rand(5),a=Array.from({length:n},()=>Array.from({length:k},()=>rand(21)-10)),b=Array.from({length:k},(_,i)=>Array.from({length:m},(_,j)=>(i-j)**2));const expected=a.flatMap(row=>Array.from({length:m},(_,j)=>Math.min(...row.map((v,i)=>v+b[i][j]))));assert.deepEqual(run('monge_matrix_product',{a,b}),expected.map(String));}
for(let t=0;t<100;t++){
  const operations=[];let values=Array(401).fill(0);for(let j=0;j<10;j++){const kind=rand(7),a=rand(11)-5,w=rand(5);if(kind<3){operations.push([['abs','left','right'][kind],a,w]);values=values.map((v,i)=>v+w*(kind===0?Math.abs(i-200-a):kind===1?Math.max(0,a-i+200):Math.max(0,i-200-a)));}else if(kind===3){operations.push(['prefix']);for(let i=1;i<values.length;i++)values[i]=Math.min(values[i],values[i-1]);}else if(kind===4){operations.push(['suffix']);for(let i=399;i>=0;i--)values[i]=Math.min(values[i],values[i+1]);}else if(kind===5){operations.push(['reflect']);values.reverse();}else {operations.push(['shift',-2,3]);const before=values;values=values.map((_,i)=>Math.min(...before.slice(Math.max(0,i-3),Math.min(401,i+3))));}}
  const x=rand(31)-15,r=run('weighted_slope_trick',{operations,x});assert.equal(+r[0],Math.min(...values));assert.equal(+r[3],values[x+200]);if(r[1]!=='-inf')assert.equal(values[+r[1]+200],+r[0]);if(r[2]!=='inf')assert.equal(values[+r[2]+200],+r[0]);
}
function brutePath(matrices,k,s,t){let best=Infinity;const go=(p,v,cost)=>{if(p===matrices.length){if(v===t)best=Math.min(best,cost);return;}for(let j=0;j<k;j++)if(matrices[p][v][j]!==null)go(p+1,j,cost+matrices[p][v][j]);};go(0,s,0);return Number.isFinite(best)?String(best):'null';}
for(let t=0;t<100;t++){const k=1+rand(3),n=1+rand(5),matrix=()=>Array.from({length:k},()=>Array.from({length:k},()=>rand(4)?rand(11)-5:null)),matrices=Array.from({length:n},matrix),updates=Array.from({length:3},()=>[rand(n),matrix()]),source=rand(k),target=rand(k),first=brutePath(matrices,k,source,target),after=JSON.parse(JSON.stringify(matrices));for(const [p,m]of updates)after[p]=m;const last=brutePath(after,k,source,target),r=run('dynamic_finite_state_dp',{states:k,matrices,updates,source,target});assert.deepEqual(r.slice(0,2),[first,last]);if(last!=='null'){const path=r.slice(2).map(Number);assert.equal(path[0],source);assert.equal(path.at(-1),target);assert.equal(path.length,n+1);let cost=0;for(let p=0;p<n;p++){assert.notEqual(after[p][path[p]][path[p+1]],null);cost+=after[p][path[p]][path[p+1]];}assert.equal(String(cost),last);}}
assert.throws(()=>model.trace('a_star',{...model.modes.find(x=>x.id==='a_star').sample,h:[0,0,99,0]}));
assert.throws(()=>model.trace('monge_matrix_product',{a:[[0,0]],b:[[1,0],[0,1]]}));
assert.throws(()=>model.trace('sat_solver',{n:2,clauses:[[0]],assumptions:[]}));
assert.throws(()=>model.trace('weighted_slope_trick',{operations:[['abs',0,-1]],x:0}));
assert.throws(()=>model.trace('myers_edit_distance',{a:'あ',b:''}));
console.log(`dp extensions model: ${traces} independent traces, 14 C++ sample outputs, validation passed`);
