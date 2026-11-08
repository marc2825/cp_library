"use strict";
const assert=require("node:assert/strict"),M=require("../../visualizations/graph_connectivity/model.js");
let seed=38003,batches=0,snapshots=0;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return (seed>>>8)%n;};
function reachability(g){
  const adj=Array.from({length:g.n},()=>[]);for(const[u,v]of g.edges)adj[u].push(v);
  return Array.from({length:g.n},(_,s)=>{const seen=new Set([s]),queue=[s];for(const u of queue)for(const v of adj[u])if(!seen.has(v)){seen.add(v);queue.push(v);}return seen;});
}
function components(g,remove=null){
  const adj=Array.from({length:g.n},()=>[]),missing=remove?.kind==="vertex"?remove.id:-1;
  g.edges.forEach(([u,v],id)=>{if(u!==missing&&v!==missing&&!(remove?.kind==="edge"&&id===remove.id)){adj[u].push(v);adj[v].push(u);}});
  const labels=Array(g.n).fill(-1);if(missing>=0)labels[missing]=-2;let count=0;
  for(let s=0;s<g.n;++s)if(labels[s]===-1){const queue=[s];labels[s]=count++;for(const u of queue)for(const v of adj[u])if(labels[v]===-1){labels[v]=labels[s];queue.push(v);}}
  return {labels,count};
}
function twoColorable(g){for(let mask=0;mask<2**g.n;++mask)if(g.edges.every(([u,v])=>((mask>>u)&1)!==((mask>>v)&1)))return true;return false;}
function run(mode,g){const copy=JSON.stringify(g),r=M.trace(mode,g);assert.equal(JSON.stringify(g),copy);assert.ok(r.frames.length);++batches;snapshots+=r.frames.length;return r;}
for(let trial=0;trial<160;++trial){
  const n=random(9),edges=n?Array.from({length:random(17)},()=>[random(n),random(n)]):[],g={n,edges};
  const reach=reachability(g),scc=run("scc",g);
  for(let u=0;u<n;++u)for(let v=0;v<n;++v)assert.equal(scc.state.component[u]===scc.state.component[v],reach[u].has(v)&&reach[v].has(u));
  for(const group of scc.state.groups)assert.deepEqual(group,group.slice().sort((a,b)=>a-b));
  for(const s of scc.frames){for(let c=0;c<s.groups.length;++c)for(const u of s.groups[c])for(const v of s.groups[c])assert.ok(reach[u].has(v)&&reach[v].has(u));}
  for(const deduplicate of [false,true]){
    const {state:s}=run("condensation",{...g,deduplicate}),expected=[];
    assert.deepEqual(s.component,scc.state.component);
    for(const[u,v]of edges){const a=s.component[u],b=s.component[v];if(a!==b)expected.push(`${a},${b}`);}
    const want=deduplicate?[...new Set(expected)].sort():expected.sort();
    assert.deepEqual(s.dagArcs.map(e=>`${e.from},${e.to}`).sort(),want);
    for(const e of s.dagArcs){const[u,v]=edges[e.original];assert.equal(s.component[u],e.from);assert.equal(s.component[v],e.to);assert.ok(e.from<e.to);}
    const dagReach=reachability({n:s.groups.length,edges:s.dagArcs.map(e=>[e.from,e.to])});
    for(let u=0;u<n;++u)for(let v=0;v<n;++v)assert.equal(dagReach[s.component[u]].has(s.component[v]),reach[u].has(v));
  }
  const base=components(g),bridges=edges.map((_,id)=>id).filter(id=>components(g,{kind:"edge",id}).count>base.count),cuts=Array.from({length:n},(_,id)=>id).filter(id=>components(g,{kind:"vertex",id}).count>base.count);
  for(const mode of ["lowlink","bridges","articulation"]){
    const {state:s,frames}=run(mode,g);assert.deepEqual(s.bridgeEdges,bridges);assert.deepEqual(s.articulation,cuts);
    assert.deepEqual(s.ord.slice().sort((a,b)=>a-b),Array.from({length:n},(_,i)=>i));
    for(const frame of frames){for(const id of frame.bridgeEdges)assert.ok(bridges.includes(id));for(const id of frame.articulation)assert.ok(cuts.includes(id));for(let v=0;v<n;++v)if(frame.ord[v]>=0)assert.ok(frame.low[v]<=frame.ord[v]);}
  }
  for(const kind of ["edge","vertex"]){
    const limit=kind==="edge"?edges.length:n;if(!limit)continue;const remove={kind,id:random(limit)},s=run(kind==="edge"?"bridges":"articulation",{...g,remove}).state;
    const want=components(g,remove);assert.equal(s.before,base.count);assert.equal(s.after,want.count);
    for(let u=0;u<n;++u)for(let v=0;v<n;++v)if(want.labels[u]>=0&&want.labels[v]>=0)assert.equal(s.component[u]===s.component[v],want.labels[u]===want.labels[v]);
    if(kind==="vertex")assert.equal(s.component[remove.id],-2);
  }
  const cc=run("components",g).state;assert.equal(cc.groups.length,base.count);
  for(let u=0;u<n;++u)for(let v=0;v<n;++v)assert.equal(cc.component[u]===cc.component[v],base.labels[u]===base.labels[v]);
  const b=run("bipartite",g).state;assert.equal(b.bipartite,twoColorable(g));
  if(b.bipartite){assert.ok(b.colors.every(c=>c===0||c===1));for(const[u,v]of edges)assert.notEqual(b.colors[u],b.colors[v]);}
  else{const[u,v]=edges[b.conflictEdge];assert.equal(b.colors[u],b.colors[v]);}
  const first=JSON.stringify(scc.frames[0]);scc.state.component.push(99);assert.equal(JSON.stringify(scc.frames[0]),first);
}
assert.deepEqual(run("articulation",{n:4,edges:[[0,1],[0,2],[0,3]]}).state.articulation,[0]);
assert.deepEqual(run("bridges",{n:2,edges:[[0,1],[0,1]]}).state.bridgeEdges,[]);
assert.equal(run("bipartite",{n:1,edges:[[0,0]]}).state.bipartite,false);
const isolated=run("articulation",{n:1,edges:[],remove:{kind:"vertex",id:0}}).state;
assert.equal(isolated.before,1);assert.equal(isolated.after,0);assert.deepEqual(isolated.articulation,[]);
console.log(`connectivity model: ${batches} batches / ${snapshots} snapshots, reachability, deletion, DAG multiplicity, root rules, coloring and snapshot isolation passed`);
