'use strict';
const assert = require('node:assert/strict');
const Model = require('../../visualizations/tree/model.js');
let seed = 311016, batches = 0, snapshots = 0;
function random(n) { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; }
function graph(n, edges) {
  const adjacency = Array.from({length:n}, () => []);
  edges.forEach(([u,v,w=1],id) => { adjacency[u].push({to:v,w,id}); adjacency[v].push({to:u,w,id}); });
  return adjacency;
}
function search(adjacency, s, banned=-1) {
  const parent=Array(adjacency.length).fill(-2), distance=Array(adjacency.length).fill(null), queue=[s];parent[s]=-1;distance[s]=0;
  for(const v of queue) for(const e of adjacency[v]) if(e.id!==banned && parent[e.to]===-2) {parent[e.to]=v;distance[e.to]=distance[v]+e.w;queue.push(e.to);}
  return {parent,distance,queue};
}
function oracle(n, edges, root) {
  const adjacency=graph(n,edges), component=Array(n).fill(-1), parent=Array(n).fill(-1), depth=Array(n).fill(0), sub=Array(n).fill(0), pre=[],post=[],walk=[],edgeWalk=[],tin=[],tout=[],roots=[];
  function visit(v,p,id) {
    component[v]=id;parent[v]=p;tin[v]=pre.length;pre.push(v);walk.push(v);sub[v]=1;
    for(const e of adjacency[v]) if(e.to!==p) {depth[e.to]=depth[v]+1;edgeWalk.push(e.id);visit(e.to,v,id);sub[v]+=sub[e.to];walk.push(v);edgeWalk.push(e.id);}
    post.push(v);tout[v]=pre.length;
  }
  if(n)for(const v of [root,...Array.from({length:n},(_,i)=>i)])if(component[v]===-1){roots.push(v);visit(v,-1,roots.length-1);}
  return {adjacency,component,parent,depth,sub,pre,post,walk,edgeWalk,tin,tout,roots};
}
function countIndependent(n,edges,allowed=(1<<n)-1) {
  let count=0;
  for(let set=allowed;;set=(set-1)&allowed){if(edges.every(([u,v])=>!(set>>u&1 && set>>v&1)))count++;if(!set)break;}
  return count;
}
function check(input) {
  const trace=Model.run(input),{n,edges,root,mode,s,t}=trace.config,frames=trace.frames,final=frames.at(-1);
  ++batches;snapshots+=frames.length;
  if(mode==='lifting') {
    const next=trace.config.next;
    function jump(v,k){const seen=new Map(),trail=[];while(v!==-1&&!seen.has(v)){seen.set(v,trail.length);trail.push(v);v=next[v];}
      if(k<BigInt(trail.length))return trail[Number(k)];if(v===-1)return -1;const begin=seen.get(v);return trail[begin+Number((k-BigInt(begin))%BigInt(trail.length-begin))];}
    for(const frame of frames)for(let b=0;b<frame.up.length;++b)for(let v=0;v<n;++v)assert.equal(frame.up[b][v],jump(v,1n<<BigInt(b)));
    assert.equal(final.result,n?jump(s,BigInt(trace.config.k)):'空');
    return trace;
  }
  const expected=oracle(n,edges,root),adjacency=expected.adjacency;
  assert.deepEqual(trace.tree.pre,expected.pre);assert.deepEqual(trace.tree.post,expected.post);assert.deepEqual(trace.tree.walk,expected.walk);assert.deepEqual(trace.tree.edgeWalk,expected.edgeWalk);
  if(!n){assert.equal(final.result,mode==='diameter'?0:'空');return trace;}
  const allDistances=Array.from({length:n},(_,v)=>search(adjacency,v).distance);
  for(const frame of frames) {
    assert.ok(frame.focus.every(v=>v>=0&&v<n));
    if(mode==='rooted'||mode==='euler') {
      for(const name of ['pre','post','walk','edgeWalk'])assert.deepEqual(frame[name],expected[name].slice(0,frame[name].length));
      for(let v=0;v<n;++v)for(const name of ['parent','depth','sub','tin','tout'])if(frame[name][v]!==null)assert.equal(frame[name][v],expected[name][v]);
    }
    if(frame.up)for(let b=0;b<frame.up.length;++b)for(let v=0;v<n;++v){let at=v;for(let k=0;k<2**b&&at!==-1;++k)at=expected.parent[at];assert.equal(frame.up[b][v],at);}
    if(mode==='diameter')for(let v=0;v<n;++v)if(frame.dist[v]!==null){const start=frames.slice(0,frames.indexOf(frame)+1).filter(f=>f.title.includes('走査を開始')).at(-1).focus[0];assert.equal(frame.dist[v],allDistances[start][v]);}
    if(mode==='dp')for(let v=0;v<n;++v)if(frame.values[v]) {
      const descendants=expected.pre.slice(expected.tin[v],expected.tout[v]);let white=0,black=0;
      for(let set=0;set<1<<descendants.length;++set){const selected=new Set(descendants.filter((_,i)=>set>>i&1));if(edges.every(([a,b])=>!selected.has(a)||!selected.has(b))){if(selected.has(v))black++;else white++;}}
      assert.deepEqual(frame.values[v],{white,black});
    }
    if(mode==='rerooting') {
      for(let v=0;v<n;++v){
        if(frame.values[v]){const descendants=expected.pre.slice(expected.tin[v],expected.tout[v]);assert.deepEqual(frame.values[v],{count:descendants.length,sum:descendants.reduce((a,u)=>a+allDistances[v][u],0)});}
        if(frame.answers[v]){const distances=allDistances[v].filter(d=>d!==null);assert.deepEqual(frame.answers[v],{count:distances.length,sum:distances.reduce((a,d)=>a+d,0)});}
      }
      if(frame.incoming) {
        const v=frame.center;
        frame.neighbors.forEach((u,i)=>{
          const e=adjacency[v].find(e=>e.to===u), side=search(adjacency,u,e.id).queue;
          assert.deepEqual(frame.incoming[i],{count:side.length,sum:side.reduce((a,x)=>a+allDistances[v][x],0)});
        });
        for(let i=0;i<frame.prefix.length;++i)for(const [name,selected]of [['prefix',frame.incoming.slice(0,i)],['suffix',frame.incoming.slice(i)]])assert.deepEqual(frame[name][i],selected.reduce((a,b)=>({count:a.count+b.count,sum:a.sum+b.sum}),{count:0,sum:0}));
      }
      for(const message of frame.messages){const e=adjacency[message.from].find(e=>e.to===message.to),side=search(adjacency,message.from,e.id).queue;assert.equal(message.count,side.length);assert.equal(message.sum,side.reduce((a,x)=>a+allDistances[message.to][x],0));}
    }
  }
  if(mode==='rooted')assert.equal(final.result,`${expected.roots.length} 成分`);
  if(mode==='euler'){assert.equal(final.walk.length,2*n-expected.roots.length);assert.equal(final.edgeWalk.length,2*edges.length);}
  if(mode==='lca'){
    let ancestor=-1;for(let a=s;a!==-1&&ancestor===-1;a=expected.parent[a])for(let b=t;b!==-1;b=expected.parent[b])if(a===b){ancestor=a;break;}
    assert.equal(final.result,ancestor);
    if(ancestor!==-1){const searchResult=search(adjacency,s),path=[];for(let v=t;v!==-1;v=searchResult.parent[v])path.push(v);path.reverse();assert.deepEqual(final.path,path);}
  }
  if(mode==='diameter'){assert.equal(final.result,Math.max(...allDistances.flat().filter(d=>d!==null)));const path=final.path;assert.equal(new Set(path).size,path.length);assert.equal(allDistances[path[0]][path.at(-1)],final.result);for(let i=1;i<path.length;++i)assert.ok(adjacency[path[i-1]].some(e=>e.to===path[i]));}
  if(mode==='dp')assert.equal(final.result,countIndependent(n,edges));
  if(mode==='rerooting')assert.equal(final.result,allDistances.map(ds=>ds.reduce((a,d)=>a+(d??0),0)).join(', '));
  return trace;
}
for(let trial=0;trial<240;++trial){
  const n=1+random(9),edges=[];
  for(let v=1;v<n;++v)if(random(4))edges.push([v,random(v),random(11)-5]);
  // Shuffle edge order and endpoint directions to exercise adjacency order.
  for(let i=edges.length-1;i>0;--i){const j=random(i+1);[edges[i],edges[j]]=[edges[j],edges[i]];}
  const base={n,edges,root:random(n),s:random(n),t:random(n),k:String(random(15))};
  for(const mode of ['rooted','lca','euler','rerooting','dp'])check({...base,mode});
  const connected=[];for(let v=1;v<n;++v)connected.push([v,random(v),random(8)]);
  check({...base,mode:'diameter',edges:connected});
  check({...base,mode:'lifting',next:Array.from({length:n},()=>random(n+1)-1),k:trial%3?'18446744073709551615':String(random(30))});
}
for(const mode of Model.MODES)check({mode,n:0,edges:[],next:[]});
for(const root of [0,1,2,3,4])for(const mode of ['rooted','lca','euler','rerooting','dp','diameter'])check({mode,n:5,root,s:4,t:0,k:3,edges:[[0,1,0],[1,2,0],[2,3,0],[3,4,0]]});
check({mode:'lifting',n:3,next:[1,2,0],s:0,k:0});check({mode:'lifting',n:1,next:[-1],s:0,k:'9223372036854775808'});
// Hidden query fields must not reject a smaller tree in a mode without queries.
assert.equal(Model.run({mode:'rooted',n:1,edges:[],s:3,t:5}).frames.at(-1).result,'1 成分');
for(const input of [
  {n:2,edges:[[0,0]]},{n:2,edges:[[0,1],[1,0]]},{n:3,edges:[[0,1],[1,2],[2,0]]},
  {n:2,edges:[[0,2]]},{n:2,edges:'0 1 1 2'},{n:13},{n:2,root:2},{n:2,k:'-1'},{n:2,k:'1.5'},
  {n:2,k:'18446744073709551616'},{mode:'diameter',n:2,edges:[]},{mode:'diameter',n:2,edges:[[0,1,-1]]},
  {mode:'lifting',n:2,next:[1]},{mode:'lifting',n:2,next:[0,2]}])assert.throws(()=>Model.run(input));
const trace=check({mode:'rerooting',n:4,edges:[[0,1,2],[1,2,-1],[1,3,4]]});
const before=JSON.stringify(trace.frames.slice(0,-1));trace.frames.at(-1).values[0].count=12345;assert.equal(JSON.stringify(trace.frames.slice(0,-1)),before);
console.log(`tree model: ${batches} batches / ${snapshots} snapshots; BFS distances, recursive tours, subset coloring, every directed message/prefix/suffix, 64-bit cycles and snapshot independence passed`);
