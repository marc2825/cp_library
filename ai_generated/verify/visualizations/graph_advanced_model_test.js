const assert=require('node:assert/strict');
const model=require('../../visualizations/graph_advanced/model.js');
const expected=require('../graph_advanced/visualization_expected.json');
let checks=0,seed=613;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const run=(id,input)=>{const r=model.trace(id,input);checks++;assert.ok(r.steps.length>=2);for(const s of r.steps){assert.ok(s.title&&s.detail);for(const e of s.state.edges)assert.ok(e[0]>=0&&e[0]<s.state.n&&e[1]>=0&&e[1]<s.state.n);}return r.result;};
for(const m of model.modes)assert.deepEqual(run(m.id,m.sample),expected[m.id]);
for(let trial=0;trial<120;trial++){
 const n=2+random(5),tree=[];for(let v=1;v<n;v++)tree.push([random(v),v,1+random(5)]);
 const distance=Array(n).fill(0),label=Array(n).fill(0),labeled=tree.map(([u,v,w])=>{const a=random(3);distance[v]=distance[u]+w;label[v]=(label[u]+a)%3;return [u,v,w,a];});
 assert.deepEqual(run('few_weight_shortest_path',{n,source:0,edges:tree}),distance.map(String));
 const t=n-1,depth=Array(n).fill(0);for(const [u,v]of tree)depth[v]=depth[u]+1;
 assert.deepEqual(run('shortest_nonzero_path',{n,source:0,target:t,modulus:3,edges:labeled}),label[t]?[String(distance[t]),String(depth[t])]:['null']);
 // A tree has one spanning tree; compute its diameter by independent DFS from each root.
 let diameter=0;for(let s=0;s<n;s++){const q=[[s,-1,0]];for(let i=0;i<q.length;i++){const [v,p,d]=q[i];diameter=Math.max(diameter,d);for(const [a,b,w]of tree){const u=a===v?b:b===v?a:-1;if(u>=0&&u!==p)q.push([u,v,d+w]);}}}
 assert.deepEqual(run('minimum_diameter_spanning_tree',{n,edges:tree}),[String(diameter),String(n-1)]);
 assert.deepEqual(run('tree_decomposition',{n,edges:tree}),['1',String(n)]);
 const rankEdges=Array.from({length:4},(_,v)=>[v,v,1+random(5)]),ranks=[...new Set(rankEdges.map(e=>e[2]))].sort((a,b)=>a-b);
 assert.deepEqual(run('rank_maximal_matching',{left:4,right:4,edges:rankEdges}),ranks.map(r=>String(rankEdges.filter(e=>e[2]===r).length)));
 const b=random(4),cost=random(11)-5;
 assert.deepEqual(run('cost_scaling_b_flow',{n:2,balance:[b,-b],edges:[[0,1,0,3,cost]]}),[String(b*cost),String(b)]);
 const path=Array.from({length:n-1},(_,v)=>[v,v+1]);
 assert.deepEqual(run('series_parallel_decomposition',{n,source:0,sink:n-1,edges:path}),[String(2*n-3)]);
 const perm=Array.from({length:n},(_,v)=>n-1-v),other=tree.map(([u,v])=>[perm[u],perm[v]]);
 assert.deepEqual(run('graph_isomorphism',{n,edges:tree,other}),['1',String(n)]);
}
for(let n=3;n<=6;n++){
 const cycle=Array.from({length:n},(_,v)=>[v,(v+1)%n,1]);
 assert.deepEqual(run('spqr_tree',{n,edges:cycle}),['1','0']);assert.deepEqual(run('tree_decomposition',{n,edges:cycle}),['2',String(n)]);
 assert.deepEqual(run('minimum_diameter_spanning_tree',{n,edges:cycle}),[String(n-1),String(n-1)]);
 const complete=[];for(let u=0;u<n;u++)for(let v=u+1;v<n;v++)complete.push([u,v,1]);
 assert.deepEqual(run('general_weighted_matching',{n,edges:complete}),[String(Math.floor(n/2)),String(Math.floor(n/2))]);
 assert.deepEqual(run('cograph_canonical_form',{n,edges:complete}),['J('+Array(n).fill('L0;').join('')+')']);
}
for(let loops=0;loops<=8;loops++){let factorial=1;for(let k=1;k<=loops;k++)factorial*=k;assert.deepEqual(run('euler_circuit_count',{n:1,source:0,first:loops?0:-1,edges:Array.from({length:loops},()=>[0,0])}),[String(factorial),String(loops?factorial/loops:1)]);}
assert.deepEqual(run('connected_graph_count',{n:6}),['0','1','1','4','38','728','26704']);
assert.deepEqual(run('strongly_connected_graph_count',{n:5}),['0','1','1','18','1606','565080']);
for(let count=1;count<=8;count++){const operations=Array.from({length:count},()=>['add',0,1]);operations.push(['query']);for(let i=0;i<count;i++){operations.push(['set',i,false],['query']);}assert.deepEqual(run('dynamic_unit_flow',{n:2,source:0,sink:1,operations}),Array.from({length:count+1},(_,i)=>String(count-i)));}
assert.deepEqual(run('shortest_nonzero_path',{n:5,source:0,target:2,modulus:2,edges:[[0,1,1,0],[1,2,1,0],[1,3,1,1],[3,4,1,0],[4,1,1,0]]}),['null']);
assert.deepEqual(run('cograph_canonical_form',{n:4,edges:[[0,1],[1,2],[2,3]]}),['null']);
assert.equal(model.trace('connected_graph_count',{n:12}).result.length,13);
assert.equal(model.trace('strongly_connected_graph_count',{n:12}).result.length,13);
for(const m of model.modes)assert.throws(()=>model.trace(m.id,null));
assert.throws(()=>model.trace('few_weight_shortest_path',{n:2,source:0,edges:[[0,1,-1]]}));
assert.throws(()=>model.trace('shortest_nonzero_path',{n:2,source:0,target:1,modulus:1,edges:[]}));
console.log(`graph advanced model: ${checks} traces, 15 C++ sample comparisons, independent tree/complete/cycle/count/update oracles passed`);
