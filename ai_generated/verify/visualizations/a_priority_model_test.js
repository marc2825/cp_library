'use strict';
const assert=require('node:assert/strict');
const {build,specs}=require('../../visualizations/a_priority/model.js');
let batches=0,frames=0,seed=712367;
const rng=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
function run(m,input){const x=build(m,input);batches++;frames+=x.frames.length;assert.equal(x.frames.at(-1).title,'完了');for(const f of x.frames){assert.ok(f.rows.length);assert.ok(Array.isArray(f.columns));}return x.result;}
const sum=a=>a.reduce((a,b)=>a+b,0);
for(const [m,[,input]]of Object.entries(specs))run(m,input);
for(let n=0;n<=8;n++){
 const a=Array.from({length:n},(_,i)=>i);
 for(let s=-12;s<=12;s++){const x=run('cyclic_sequence',{a,shift:s,start:s,count:n*2});const norm=i=>(i%n+n)%n;assert.deepEqual(x.rotation,a.map((_,i)=>a[norm(i+s)]));assert.deepEqual(x.slice,Array.from({length:n*2},(_,i)=>a[norm(i+s)]));}
}
for(let n=0;n<=120;n++){
 const q=run('quotient_enumeration',{n});assert.deepEqual(q.flatMap(([l,r,v])=>Array.from({length:r-l+1},(_,i)=>[l+i,v])),Array.from({length:n},(_,i)=>[i+1,Math.floor(n/(i+1))]));
 const l=rng(20),r=l+rng(50),step=rng(30)+1;assert.deepEqual(run('multiple_enumeration',{l,r,step}),Array.from({length:r-l},(_,i)=>i+l).filter(x=>x%step===0));
}
for(let bits=0;bits<=6;bits++){const g=run('gray_code',{bits});assert.equal(new Set(g).size,2**bits);for(let i=1;i<g.length;i++){const x=g[i]^g[i-1];assert.equal(x&(x-1),0);}}
for(let n=0;n<=5;n++){const x=run('subset_pairs',{n}),a=[];for(let s=0;s<2**n;s++)for(let t=s;t>=0;t--)if((s&t)===t)a.push([s,t]);assert.deepEqual(x,a);}
function perms(a){if(!a.length)return[[]];return a.flatMap((v,i)=>perms(a.filter((_,j)=>i!==j)).map(p=>[v,...p]));}
for(let n=0;n<=5;n++){const all=perms(Array.from({length:n},(_,i)=>i));all.forEach((a,rank)=>{const r=rng(all.length);assert.deepEqual(run('permutation_rank',{a,rank:r}),{rank,permutation:all[r]});});}
for(let trial=0;trial<60;trial++){
 const n=1+rng(15),a=Array.from({length:n},()=>rng(21)-10),queries=Array.from({length:20},()=>{let l=rng(n+1),r=rng(n+1);return l<r?[l,r]:[r,l];});
 const expected=queries.map(([l,r])=>sum(a.slice(l,r)));assert.deepEqual(run('disjoint_sparse_table',{a,queries}),expected);assert.deepEqual(run('rollback_mo',{a,queries,block:1+rng(20)}),expected);
 assert.deepEqual(run('offline_rmq',{a,queries}),queries.map(([l,r])=>l===r?-1:l+a.slice(l,r).indexOf(Math.min(...a.slice(l,r)))));
 for(const mode of ['dynamic_segtree','dynamic_lazy_segtree','range_actions']){
  const values=Array(n).fill(0),ops=[],answers=[];
  for(let k=0;k<20;k++){let l=rng(n+1),r=rng(n+1);if(l>r)[l,r]=[r,l];if(k%3===0){ops.push(['sum',l,r]);answers.push(sum(values.slice(l,r)));}else if(mode==='dynamic_segtree'){const p=rng(n),v=rng(11)-5;ops.push(['set',p,v]);values[p]=v;}else{const a=rng(3)-1,b=rng(11)-5;ops.push(['affine',l,r,a,b]);for(let i=l;i<r;i++)values[i]=a*values[i]+b;}}
  assert.deepEqual(run(mode,{n,ops}),{sum:sum(values),answers});
 }
 const h=1+rng(6),w=1+rng(6),points=[],grid=Array.from({length:h},()=>Array(w).fill(0)),ops=[],answers=[];for(let x=0;x<h;x++)for(let y=0;y<w;y++)points.push([x,y]);
 for(let k=0;k<20;k++){if(k%3){const x=rng(h),y=rng(w),d=rng(21)-10;grid[x][y]+=d;ops.push(['add',x,y,d]);}else{let l=rng(h+1),r=rng(h+1),d=rng(w+1),u=rng(w+1);if(l>r)[l,r]=[r,l];if(d>u)[d,u]=[u,d];ops.push(['sum',l,d,r,u]);answers.push(sum(grid.slice(l,r).flatMap(row=>row.slice(d,u))));}}
 for(const m of ['fenwick_tree_2d','segtree_2d','compressed_fenwick_tree_2d'])assert.deepEqual(run(m,{h,w,points,ops}),{grid,answers});
 const intervalOps=[],covered=new Set(),rangeAnswers=[];for(let i=0;i<30;i++){let l=rng(20),r=rng(20);if(l>r)[l,r]=[r,l];const type=rng(4);if(type<2){intervalOps.push([type?'erase':'insert',l,r]);for(let x=l;x<r;x++)if(type)covered.delete(x);else covered.add(x);}else if(type===2){intervalOps.push(['contains',l]);rangeAnswers.push(covered.has(l));}else{intervalOps.push(['covers',l,r]);rangeAnswers.push(Array.from({length:r-l},(_,i)=>l+i).every(x=>covered.has(x)));}}
 const intervals=[];for(const x of [...covered].sort((a,b)=>a-b)){if(!intervals.length||intervals.at(-1)[1]!==x)intervals.push([x,x+1]);else intervals.at(-1)[1]++;}assert.deepEqual(run('range_set',{ops:intervalOps}),{intervals,answers:rangeAnswers});
 const merged=run('interval_operations',{intervals:Array.from({length:10},()=>{let a=rng(20),b=rng(20);return a<b?[a,b]:[b,a];}),cuts:[2,6,8,8,16]});assert.deepEqual(merged.split.flatMap(([l,r])=>Array.from({length:r-l},(_,i)=>l+i)),merged.merged.flatMap(([l,r])=>Array.from({length:r-l},(_,i)=>l+i)));
 for(const mode of ['treap','rbst','ordered_set']){let a=[],ops=[],answers=[];for(let k=0;k<30;k++){const type=rng(4),x=rng(20);if(type===0){ops.push(['insert',x]);if(mode!=='ordered_set'||!a.includes(x))a.push(x);a.sort((a,b)=>a-b);}else if(type===1){ops.push(['erase',x]);const i=a.indexOf(x);if(i>=0)a.splice(i,1);}else if(type===2){ops.push(['rank',x]);answers.push(a.filter(v=>v<x).length);}else{const i=rng(15);ops.push(['kth',i]);answers.push(a[i]??null);}}assert.deepEqual(run(mode,{seed:trial,ops}),{values:a,answers});}
 const tOps=[],tValues=[],tAnswers=[];let mask=0;for(let k=0;k<30;k++){const type=rng(5),x=rng(16);if(type===0){tOps.push(['insert',x]);tValues.push(x);}else if(type===1){tOps.push(['erase',x]);const i=tValues.indexOf(x);if(i>=0)tValues.splice(i,1);}else if(type===2){tOps.push(['xor',x]);mask^=x;for(let i=0;i<tValues.length;i++)tValues[i]^=x;}else if(type===3){tOps.push(['kth',x]);tAnswers.push([...tValues].sort((a,b)=>a-b)[x]??null);}else if(tValues.length){tOps.push(['xor_min',x]);tAnswers.push([...tValues].sort((a,b)=>(a^x)-(b^x))[0]);}}assert.deepEqual(run('binary_trie',{bits:4,ops:tOps}),{mask,size:tValues.length,answers:tAnswers});
 for(const size of [0,1,64,65,4097]){const set=new Set(),ops=[],answers=[];for(let k=0;k<30;k++){const type=size?rng(4):2,x=rng(size+1);if(type===0&&x<size){ops.push(['insert',x]);set.add(x);}else if(type===1&&x<size){ops.push(['erase',x]);set.delete(x);}else if(type===2){ops.push(['next',x]);answers.push(Math.min(size,...[...set].filter(v=>v>=x)));}else{ops.push(['prev',x]);answers.push(Math.max(-1,...[...set].filter(v=>v<=x)));}}assert.deepEqual(run('fast_set',{n:size,ops}),{size:set.size,answers});}
 for(const mode of ['persistent_stack','persistent_queue']){const versions=[[]],ops=[],answers=[];for(let k=0;k<25;k++){const b=rng(versions.length),a=versions[b].slice();if(!a.length||rng(2)){const x=rng(100);ops.push(['push',b,x]);a.push(x);versions.push(a);}else{ops.push(['pop',b]);if(mode==='persistent_stack')a.pop();else a.shift();versions.push(a);}const nonempty=versions.map((a,i)=>a.length?i:-1).filter(x=>x>=0);if(k%5===0&&nonempty.length){const b=nonempty[rng(nonempty.length)];ops.push(['front',b]);answers.push(mode==='persistent_stack'?versions[b].at(-1):versions[b][0]);}}assert.deepEqual(run(mode,{ops}),{versions,answers});}
 const edges=[];for(let v=1;v<n;v++)edges.push([v,rng(v)]);const paths=Array.from({length:15},()=>[rng(n),rng(n)]),g=Array.from({length:n},()=>[]);edges.forEach(([u,v])=>{g[u].push(v);g[v].push(u);});
 const pathsExpected=paths.map(([u,v])=>{const prev=Array(n).fill(-1),q=[u];prev[u]=u;for(let i=0;i<q.length;i++)for(const x of g[q[i]])if(prev[x]<0){prev[x]=q[i];q.push(x);}const a=[];for(let x=v;;x=prev[x]){a.push(x);if(x===u)break;}return a.sort((a,b)=>a-b);});assert.deepEqual(run('mo_on_tree',{n,edges,queries:paths}).answers,pathsExpected);
 const seq=a.slice(),sops=[],sans=[];for(let k=0;k<20;k++){let l=rng(seq.length+1),r=rng(seq.length+1);if(l>r)[l,r]=[r,l];if(k%3===0){sops.push(['sum',l,r]);sans.push(sum(seq.slice(l,r)));}else if(k%3===1){sops.push(['reverse',l,r]);seq.splice(l,r-l,...seq.slice(l,r).reverse());}else{const a=rng(3)-1,b=rng(5);sops.push(['affine',l,r,a,b]);for(let i=l;i<r;i++)seq[i]=a*seq[i]+b;}}assert.deepEqual(run('lazy_reversible_treap',{a,ops:sops}),{values:seq,answers:sans});
}
function labels(n){return Array.from({length:n},(_,i)=>i);}function join(a,u,v){const x=a[v],y=a[u];for(let i=0;i<a.length;i++)if(a[i]===x)a[i]=y;}
function groups(parent){return parent.map((_,i)=>{while(parent[i]>=0)i=parent[i];return i;});}
for(let trial=0;trial<80;trial++){
 const n=1+rng(12),merges=Array.from({length:20},()=>['merge',rng(n),rng(n)]),expected=labels(n);merges.forEach(([,u,v])=>join(expected,u,v));
 const aggOps=[],weights=Array(n).fill(1),current=labels(n),aggAnswers=[];for(const op of merges){aggOps.push(op);join(current,op[1],op[2]);const v=rng(n),x=rng(11)-5;aggOps.push(['add',v,x]);weights[v]+=x;aggOps.push(['prod',v]);aggAnswers.push(sum(weights.filter((_,i)=>current[i]===current[v])));} // 60操作を40以下に収める
 const limited=aggOps.slice(0,39),answers=aggAnswers.slice(0,13);assert.deepEqual(run('aggregate_disjoint_set_union',{n,values:Array(n).fill(1),ops:limited}).answers,answers);
 const mt=run('merge_tree',{n,ops:merges});const root=v=>{while(mt.parent[v]>=0)v=mt.parent[v];return v;};for(let u=0;u<n;u++)for(let v=0;v<n;v++)assert.equal(root(u)===root(v),expected[u]===expected[v]);
 const intOps=[],intLabels=labels(n);for(let k=0;k<25;k++){let l=rng(n+1),r=rng(n+1);if(l>r)[l,r]=[r,l];intOps.push(['merge',l,r]);for(let i=l+1;i<r;i++)join(intLabels,l,i);}const intResult=groups(run('interval_disjoint_set_union',{n,ops:intOps}).parent);for(let u=0;u<n;u++)for(let v=0;v<n;v++)assert.equal(intResult[u]===intResult[v],intLabels[u]===intLabels[v]);
 for(const mode of ['partially_persistent_disjoint_set_union','persistent_disjoint_set_union']){const versions=[labels(n)],ops=[],answers=[];for(let k=0;k<18;k++){const base=mode==='persistent_disjoint_set_union'?rng(versions.length):versions.length-1,a=versions[base].slice(),u=rng(n),v=rng(n);ops.push(mode==='persistent_disjoint_set_union'?['merge',base,u,v]:['merge',u,v]);join(a,u,v);versions.push(a);const t=rng(versions.length);ops.push(['same',t,u,v]);answers.push(versions[t][u]===versions[t][v]);}assert.deepEqual(run(mode,{n,ops}).answers,answers);}
 const edgeOps=[],g=Array.from({length:n},()=>[]),bip=[];for(let k=0;k<30;k++){const u=rng(n),v=rng(n);edgeOps.push(['edge',u,v]);g[u].push(v);g[v].push(u);const color=Array(n).fill(-1);let ok=true;for(let v=0;v<n;v++)if(color[v]<0){const q=[v];color[v]=0;for(let i=0;i<q.length;i++)for(const w of g[q[i]]){if(color[w]<0){color[w]=color[q[i]]^1;q.push(w);}else if(color[w]===color[q[i]])ok=false;}}bip.push(ok);}assert.deepEqual(run('bipartite_disjoint_set_union',{n,ops:edgeOps}).answers,bip);
}
for(const[m,input]of [['gray_code',{bits:64}],['permutation_rank',{a:[0,0],rank:0}],['mo_on_tree',{n:3,edges:[[0,1],[1,2],[2,0]],queries:[]}],['persistent_queue',{ops:[['pop',0]]}],['compressed_fenwick_tree_2d',{h:2,w:2,points:[],ops:[['add',1,1,1]]}]])assert.throws(()=>build(m,input));
console.log(`32 modes: ${batches} independent batches, ${frames} snapshots and invalid-input checks passed`);
