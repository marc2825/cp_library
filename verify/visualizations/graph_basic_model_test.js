"use strict";
const assert=require("node:assert/strict"),M=require("../../visualizations/graph_basic/model.js");
let seed=37003,batches=0,snapshots=0;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return (seed>>>8)%n;};
function adjacency(g){const a=Array.from({length:g.n},()=>[]);g.edges.forEach(([u,v,w],id)=>{a[u].push([v,w,id]);if(!g.directed)a[v].push([u,w,id]);});return a;}
function oracle(g,source,unit=false){
  const adj=adjacency(g);let d=Array(g.n).fill(Infinity);d[source]=0;let previous=d.slice();
  for(let k=0;k<g.n;++k){previous=d.slice();const next=d.slice();for(let u=0;u<g.n;++u)for(const[v,w]of adj[u])next[v]=Math.min(next[v],d[u]+(unit?1:w));d=next;}
  const negative=Array(g.n).fill(false),queue=[];
  d.forEach((v,i)=>{if(v<previous[i]){negative[i]=true;queue.push(i);}});
  for(const u of queue)for(const[v]of adj[u])if(!negative[v]){negative[v]=true;queue.push(v);}
  return {dist:previous.map(x=>x===Infinity?null:x),negative};
}
function checkPath(g,s,unit=false){
  if(!s.pathVertices.length){assert.ok(s.dist[s.target]===null||s.negative[s.target]||(s.mode==="dfs"&&s.component[s.source]!==s.component[s.target]));return;}
  assert.equal(s.pathVertices[0],s.source);assert.equal(s.pathVertices.at(-1),s.target);
  assert.equal(s.pathVertices.length,s.pathEdges.length+1);assert.equal(new Set(s.pathVertices).size,s.pathVertices.length);
  let cost=0;s.pathEdges.forEach((id,i)=>{const[u,v,w]=g.edges[id],a=s.pathVertices[i],b=s.pathVertices[i+1];assert.ok(u===a&&v===b||!g.directed&&u===b&&v===a);cost+=unit?1:w;});
  assert.equal(cost,s.dist[s.target]);
}
function dfsOracle(g){
  const adj=adjacency(g),seen=Array(g.n).fill(false),pre=[],post=[],parent=Array(g.n).fill(-1),tin=Array(g.n).fill(-1),tout=Array(g.n).fill(-1);
  function visit(v){seen[v]=true;tin[v]=pre.length;pre.push(v);for(const[u]of adj[v])if(!seen[u]){parent[u]=v;visit(u);}tout[v]=pre.length;post.push(v);}
  if(g.n)visit(g.source);for(let v=0;v<g.n;++v)if(!seen[v])visit(v);return {pre,post,parent,tin,tout};
}
function cycleExists(g,scope=false){
  const adj=adjacency(g),reachable=new Set(),queue=scope?[g.source]:Array.from({length:g.n},(_,i)=>i);
  for(const u of queue){if(reachable.has(u))continue;reachable.add(u);for(const[v]of adj[u])if(!reachable.has(v))queue.push(v);}
  for(let id=0;id<g.edges.length;++id){const[u,v]=g.edges[id];if(!reachable.has(u))continue;if(u===v)return true;
    const seen=new Set([v]),q=[v];for(const x of q)for(const[y,,e]of adj[x])if((g.directed||e!==id)&&!seen.has(y)){seen.add(y);q.push(y);}if(seen.has(u))return true;
  }return false;
}
function msfOracle(g){
  const labels=ids=>{const adj=Array.from({length:g.n},()=>[]);for(const i of ids){const[u,v]=g.edges[i];adj[u].push(v);adj[v].push(u);}const c=Array(g.n).fill(-1);let groups=0;
    for(let v=0;v<g.n;++v)if(c[v]<0){const q=[v];c[v]=groups++;for(const x of q)for(const y of adj[x])if(c[y]<0){c[y]=c[v];q.push(y);}}return {labels:c,groups};};
  const all=labels(g.edges.map((_,i)=>i));let best=Infinity;
  for(let mask=0;mask<2**g.edges.length;++mask){const ids=g.edges.map((_,i)=>i).filter(i=>mask>>i&1);if(ids.length!==g.n-all.groups)continue;const c=labels(ids);if(c.labels.every((x,i)=>x===all.labels[i]))best=Math.min(best,ids.reduce((s,i)=>s+g.edges[i][2],0));}
  return {best,...all,labelsFor:labels};
}
function run(mode,g){const original=JSON.stringify(g),result=M.trace(mode,g);assert.equal(JSON.stringify(g),original);++batches;snapshots+=result.frames.length;assert.ok(result.frames.length);return result;}
for(let trial=0;trial<120;++trial){
  const n=random(8),directed=!!random(2),type=trial%3,edges=[];
  if(n)for(let i=0,m=random(10);i<m;++i)edges.push([random(n),random(n),type===0?random(2):type===1?random(9):random(13)-6]);
  const g={n,directed,edges,source:n?random(n):0,target:n?random(n):0,scope:false};
  const dfs=run("dfs",g).state,expectedDFS=dfsOracle(g);
  assert.deepEqual(dfs.order,expectedDFS.pre);assert.deepEqual(dfs.postorder,expectedDFS.post);assert.deepEqual(dfs.parent,expectedDFS.parent);assert.deepEqual(dfs.tin,expectedDFS.tin);assert.deepEqual(dfs.tout,expectedDFS.tout);if(n)checkPath(g,dfs,true);
  const floyd=run("floyd",g).state;
  for(let source=0;source<n;++source){const want=oracle(g,source);assert.deepEqual(floyd.negativeMatrix[source],want.negative);want.dist.forEach((v,i)=>{if(!want.negative[i])assert.equal(floyd.matrix[source][i],v);});}
  if(n){
    for(const mode of ["bfs","bellman",...(type<2?["dijkstra","path"]:[]),...(type===0?["zero-one"]:[])]){
      const {state,frames}=run(mode,g),want=oracle(g,g.source,mode==="bfs");
      assert.deepEqual(state.negative,want.negative);want.dist.forEach((v,i)=>{if(!want.negative[i])assert.equal(state.dist[i],v);});checkPath(g,state,mode==="bfs");
      for(const s of frames){for(let v=0;v<n;++v)if(s.parentEdge[v]>=0){const[u,w]=g.edges[s.parentEdge[v]];assert.ok(u===s.parent[v]&&w===v||!g.directed&&u===v&&w===s.parent[v]);}}
    }
    checkPath(g,floyd);
  }
  for(const scope of [false,...(n?[true]:[])]){
    const s=run("cycle",{...g,scope}).state;assert.equal(s.cycle!==null,cycleExists(g,scope));
    if(s.cycle){const{vertices,edges:ids}=s.cycle;assert.ok(ids.length>0);assert.equal(ids.length,vertices.length);assert.equal(new Set(ids).size,ids.length);
      ids.forEach((id,i)=>{const[u,v]=g.edges[id],a=vertices[i],b=vertices[(i+1)%vertices.length];assert.ok(u===a&&v===b||!g.directed&&u===b&&v===a);});}
  }
  if(directed){const s=run("topological",g).state;assert.equal(s.acyclic,!cycleExists(g));if(s.acyclic){const pos=new Map(s.order.map((v,i)=>[v,i]));assert.equal(pos.size,n);for(const[u,v]of edges)assert.ok(pos.get(u)<pos.get(v));}}
  else{
    const want=msfOracle(g);
    for(const mode of ["kruskal","prim"]){const s=run(mode,g).state;assert.equal(s.cost,want.best);assert.equal(s.componentCount,want.groups);assert.equal(s.selected.length,n-want.groups);assert.deepEqual(want.labelsFor(s.selected).labels,want.labels);assert.equal(s.selected.reduce((a,i)=>a+edges[i][2],0),s.cost);}
  }
}
const bad={n:5,directed:true,edges:[[0,1,1],[1,1,-1],[1,2,0],[3,3,-1]],source:0,target:2};
assert.deepEqual(run("bellman",bad).state.negative,[false,true,true,false,false]);
assert.deepEqual(run("floyd",bad).state.negativeMatrix[3],[false,false,false,true,false]);
const multi={n:2,directed:false,edges:[[0,1,7],[0,1,1]],source:0,target:1,scope:false};
assert.equal(run("cycle",multi).state.cycle.edges.length,2);assert.deepEqual(run("path",multi).state.pathEdges,[1]);
console.log(`graph model: ${batches} batches / ${snapshots} snapshots, bounded-walk/BFS/DFS/subset oracles, cycles, negative influence, restored paths and parallel edges passed`);
