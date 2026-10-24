'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const model=require('../../visualizations/data_structure_extensions/model.js');
const expected=require('../data_structure_extensions/visualization_expected.json');
const manifest=require('../data_structure_extensions/manifest.json').algorithms;
assert.deepEqual(model.modes.map(x=>x.id),manifest.map(x=>x.mode));
let checks=0,seed=71831;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const run=(id,input)=>model.trace(id,input).result;
for(const m of model.modes){assert.deepEqual(run(m.id,m.sample),expected[m.id]);assert.ok(fs.existsSync(path.join(__dirname,'../../docs',m.doc)));checks++;}
for(let t=0;t<300;t++){
  const n=random(18),a=Array.from({length:n},()=>random(9)-2),queries=[];
  for(let j=0;j<8;j++){let l=random(n+1),r=random(n+1);if(l>r)[l,r]=[r,l];queries.push([l,r]);}
  assert.deepEqual(run('static_range_mex',{values:a,queries}),queries.map(([l,r])=>{const s=new Set(a.slice(l,r));let x=0;while(s.has(x))x++;return x;}));checks++;
  assert.deepEqual(run('range_majority',{values:a,queries}),queries.map(([l,r])=>{const c=new Map();for(const x of a.slice(l,r))c.set(x,(c.get(x)||0)+1);return [...c].find(([,v])=>v>(r-l)/2)?.[0]??null;}));checks++;
  const operations=[],values=a.slice(),answers=[];
  for(let j=0;j<20;j++){
    if(n&&random(2)){const p=random(n),x=random(30);operations.push(['set',p,x]);values[p]=x;}
    else {const [l,r]=queries[random(queries.length)],x=random(15);operations.push(['count',l,r,x]);answers.push(values.slice(l,r).filter(v=>v===x).length);}
  }
  assert.deepEqual(run('point_set_range_frequency',{values:a,operations}),{values,answers});checks++;
  const [l,r]=queries[0],c=run('segment_tree_cover',{size:n,left:l,right:r});let end=l;
  for(const [a,b]of c.intervals){assert.equal(a,end);assert.ok(a<b);end=b;}assert.equal(end,r);assert.equal(new Set(c.nodes).size,c.nodes.length);checks++;
  const heapOps=[],versions=[[]];
  for(let j=0;j<20;j++){
    const i=random(versions.length),k=random(versions.length);let type=random(3),next=versions[i].slice();
    if(!next.length||next.length+versions[k].length>100)type=0;
    if(type===0){const x=random(20);heapOps.push(['push',i,x]);next.push(x);}
    else if(type===1){heapOps.push(['pop',i]);next.shift();}
    else {heapOps.push(['meld',i,k]);next.push(...versions[k]);}
    next.sort((a,b)=>a-b);versions.push(next);
  }
  assert.deepEqual(run('persistent_leftist_heap',{operations:heapOps}),{tops:versions.map(a=>a[0]??null),sizes:versions.map(a=>a.length)});checks++;
  const queue=a.slice(),queueOps=[];
  for(let j=0;j<20;j++){if(!queue.length||random(2)){const x=random(20);queueOps.push(['push',x]);queue.push(x);}else{const min=random(2),x=min?Math.min(...queue):Math.max(...queue);queueOps.push(min?'pop_min':'pop_max');queue.splice(queue.indexOf(x),1);}}
  assert.deepEqual(run('double_ended_priority_queue',{values:a,operations:queueOps}),{min:queue.length?Math.min(...queue):null,max:queue.length?Math.max(...queue):null,size:queue.length});checks++;
  const size=1+random(12),trees=[Array(size).fill(0)],treeOps=[];
  for(let j=0;j<20;j++){const i=random(trees.length),type=random(3),v=trees[i].slice();if(type===0){const p=random(size),x=random(20);v[p]=x;treeOps.push(['set',i,p,x]);trees.push(v);}else if(type===1){const p=random(size+1);treeOps.push(['split',i,p]);trees.push(v.map((x,k)=>k<p?x:0),v.map((x,k)=>k>=p?x:0));}else{const k=random(trees.length);treeOps.push(['merge',i,k]);trees.push(v.map((x,p)=>x+trees[k][p]));}}
  assert.deepEqual(run('dynamic_segtree_merge_split',{size,operations:treeOps}),{totals:trees.map(a=>a.reduce((a,b)=>a+b,0))});checks++;
  const times=random(9),vertices=1+random(7),edges=[];
  for(let i=0;i<12;i++){let l=random(times+1),r=random(times+1);if(l>r)[l,r]=[r,l];edges.push([l,r,random(vertices),random(vertices)]);}
  const counts=Array.from({length:times},(_,time)=>{const adj=Array.from({length:vertices},()=>[]);for(const [l,r,a,b]of edges)if(l<=time&&time<r){adj[a].push(b);adj[b].push(a);}let count=0;const visited=new Set();for(let v=0;v<vertices;v++)if(!visited.has(v)){count++;const q=[v];visited.add(v);for(let i=0;i<q.length;i++)for(const u of adj[q[i]])if(!visited.has(u)){visited.add(u);q.push(u);}}return count;});
  assert.deepEqual(run('segment_tree_over_time',{times,vertices,edges}),counts);checks++;
  const weights=Array.from({length:vertices},()=>random(20)),dsOps=[['save']],groups=weights.map((_,i)=>i),history=[vertices];let count=vertices;
  for(let i=0;i<15;i++){const a=random(vertices),b=random(vertices),old=groups[b],next=groups[a];if(old!==next){count--;for(let j=0;j<vertices;j++)if(groups[j]===old)groups[j]=next;}dsOps.push(['merge',a,b]);history.push(count);}
  dsOps.push(['rollback',0]);history.push(vertices);
  assert.deepEqual(run('rollback_disjoint_set_union_with_data',{weights,operations:dsOps}),{components:vertices,sums:weights,history});checks++;
}
for(const [id,input]of [
 ['double_ended_priority_queue',{values:[],operations:['pop_min']}],
 ['static_range_mex',{values:[1],queries:[[1,0]]}],
 ['range_majority',{values:[1],queries:[[0,2]]}],
 ['point_set_range_frequency',{values:[],operations:[['set',0,1]]}],
 ['persistent_leftist_heap',{operations:[['pop',0]]}],
 ['dynamic_segtree_merge_split',{size:3,operations:[['merge',0,1]]}],
 ['segment_tree_cover',{size:3,left:-1,right:2}],
 ['segment_tree_over_time',{vertices:2,times:1,edges:[[0,1,0,2]]}],
 ['rollback_disjoint_set_union_with_data',{weights:[1],operations:[['rollback',0]]}]
]){assert.throws(()=>run(id,input));checks++;}
console.log(`data structure visualization: ${checks} fixture/oracle/boundary comparisons passed`);
