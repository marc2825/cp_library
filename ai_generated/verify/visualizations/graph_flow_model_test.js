const assert=require('node:assert/strict');
const {trace}=require('../../visualizations/graph_flow/model.js');
let batches=0,frames=0,seed=390039;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return Math.floor(seed/4294967296*n);};
function cut(n,edges,s,t){let best=Infinity;for(let mask=0;mask<2**n;++mask)if((mask>>s&1)&&!(mask>>t&1)){let value=0;for(const[u,v,c]of edges)if((mask>>u&1)&&!(mask>>v&1))value+=c;best=Math.min(best,value);}return best;}
function matching(left,right,edges){let states=new Set([0]);for(let u=0;u<left;++u){const next=new Set(states);for(const mask of states)for(const[a,v]of edges)if(a===u&&!(mask>>v&1))next.add(mask|1<<v);states=next;}return Math.max(...[...states].map(m=>m.toString(2).replaceAll('0','').length));}
function reach(n,edges,flows,source){const adj=Array.from({length:n},()=>[]);edges.forEach(([u,v,c],i)=>{if(flows[i]<c)adj[u].push(v);if(flows[i]>0)adj[v].push(u);});const seen=Array(n).fill(false),queue=[source];seen[source]=true;for(const u of queue)for(const v of adj[u])if(!seen[v]){seen[v]=true;queue.push(v);}return seen;}
function flowCheck(mode,input){const before=JSON.stringify(input),result=trace(mode,input),expected=cut(input.n,input.edges,input.source,input.sink),limit=mode==='cut'||input.limit==null?Infinity:input.limit;
  ++batches;frames+=result.frames.length;assert.equal(JSON.stringify(input),before);assert.equal(result.state.total,Math.min(expected,limit));
  for(const s of result.frames){const balance=Array(input.n).fill(0);s.edges.forEach(([u,v,c],i)=>{assert.ok(Number.isInteger(s.flows[i])&&s.flows[i]>=0&&s.flows[i]<=c);balance[u]-=s.flows[i];balance[v]+=s.flows[i];});
    balance.forEach((b,v)=>assert.equal(b,v===s.source?0-s.total:v===s.sink?s.total:0));
    if(s.phase==='path'){let at=s.source,minimum=Infinity;for(const arc of s.path){const[u,v,c]=s.edges[arc>>1],reverse=arc%2;assert.equal(at,reverse?v:u);const next=reverse?u:v;assert.equal(s.level[next],s.level[at]+1);const cap=reverse?s.flows[arc>>1]:c-s.flows[arc>>1];assert.ok(cap>0);minimum=Math.min(minimum,cap);at=next;}assert.equal(at,s.sink);assert.ok(s.delta>0&&s.delta<=minimum);}
  }
  const s=result.state,side=reach(input.n,input.edges,s.flows,input.source);assert.deepEqual(s.sourceSide,side);assert.equal(s.exhausted,!side[input.sink]);
  if(s.exhausted){assert.equal(s.cutCapacity,expected);assert.equal(s.total,expected);assert.deepEqual(s.cutEdges,input.edges.flatMap(([u,v],id)=>side[u]&&!side[v]?[id]:[]));}else{assert.equal(s.cutCapacity,null);assert.deepEqual(s.cutEdges,[]);}
  const snapshot=JSON.stringify(result.frames[0]);result.state.flows.fill(99);result.frames.at(-1).flows.fill(999);assert.equal(JSON.stringify(result.frames[0]),snapshot);
  return result.frames;
}
function matchingCheck(input){const before=JSON.stringify(input),result=trace('matching',input);++batches;frames+=result.frames.length;assert.equal(JSON.stringify(input),before);const expected=matching(input.leftSize,input.rightSize,input.edges);assert.equal(result.state.cardinality,expected);
  for(const s of result.frames){let count=0;for(let u=0;u<s.leftSize;++u)if(s.leftMatch[u]>=0){const v=s.leftMatch[u];assert.equal(s.rightMatch[v],u);assert.ok(s.edges.some(([a,b])=>a===u&&b===v));++count;}s.rightMatch.forEach((u,v)=>{if(u>=0)assert.equal(s.leftMatch[u],v);});assert.equal(count,s.cardinality);
    if(s.phase==='path'){assert.ok(s.path.length%2===1);assert.equal(s.leftMatch[s.path[0].left],-1);assert.equal(s.rightMatch[s.path.at(-1).right],-1);s.path.forEach((e,i)=>{assert.equal(e.matched,Boolean(i%2));assert.ok(s.edges.some(([u,v])=>u===e.left&&v===e.right));if(e.matched)assert.equal(s.leftMatch[e.left],e.right);if(i){const previous=s.path[i-1];assert.equal(i%2?previous.right:previous.left,i%2?e.right:e.left);}});}
  }
  assert.deepEqual(result.state.pairs,result.state.leftMatch.flatMap((v,u)=>v<0?[]:[[u,v]]));
  const snapshot=JSON.stringify(result.frames[0]);result.state.leftMatch.fill(99);result.frames.at(-1).leftMatch.fill(999);assert.equal(JSON.stringify(result.frames[0]),snapshot);return result.frames;
}
const reverse={n:6,source:0,sink:5,edges:[[0,1,1],[0,2,1],[1,3,1],[1,4,1],[2,3,1],[3,5,1],[4,5,1]],limit:null};
assert.ok(flowCheck('dinic',reverse).some(s=>s.phase==='path'&&s.path.some(id=>id%2)));
assert.ok(matchingCheck({leftSize:3,rightSize:3,edges:[[0,0],[0,1],[1,0],[2,1],[2,2]]}).some(s=>s.phase==='path'&&s.path.length===5));
for(let n=2;n<=3;++n){const pairs=[];for(let u=0;u<n;++u)for(let v=0;v<n;++v)if(u!==v)pairs.push([u,v]);for(let mask=0;mask<2**pairs.length;++mask){const edges=pairs.map(([u,v],i)=>[u,v,mask>>i&1]);flowCheck('dinic',{n,edges,source:0,sink:n-1,limit:null});}}
for(let leftSize=0;leftSize<=3;++leftSize)for(let rightSize=0;rightSize<=3;++rightSize)for(let mask=0;mask<2**(leftSize*rightSize);++mask){const edges=[];for(let u=0;u<leftSize;++u)for(let v=0;v<rightSize;++v)if(mask>>(u*rightSize+v)&1)edges.push([u,v]);matchingCheck({leftSize,rightSize,edges});}
for(let trial=0;trial<180;++trial){const n=2+random(7),source=random(n),sink=(source+1+random(n-1))%n,edges=Array.from({length:random(19)},()=>[random(n),random(n),random(7)]);for(const mode of ['dinic','cut'])flowCheck(mode,{n,source,sink,edges,limit:random(8)});flowCheck('dinic',{n,source,sink,edges,limit:null});
  const leftSize=random(7),rightSize=random(7),matchingEdges=leftSize&&rightSize?Array.from({length:random(19)},()=>[random(leftSize),random(rightSize)]):[];matchingCheck({leftSize,rightSize,edges:matchingEdges});}
for(const input of [{n:2,source:0,sink:0,edges:[]},{n:2,source:0,sink:1,edges:[[0,2,1]]},{n:2,source:0,sink:1,edges:[[0,1,-1]]},{n:2,source:0,sink:1,edges:[],limit:-1}])assert.throws(()=>trace('dinic',input));
assert.throws(()=>trace('matching',{leftSize:0,rightSize:1,edges:[[0,0]]}));
console.log(`flow model: ${batches} batches / ${frames} snapshots; cut enumeration, matching subset DP, feasible flows, residual reachability, reverse paths and snapshots passed`);
