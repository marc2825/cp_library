"use strict";
const assert=require("node:assert/strict"),M=require("../../visualizations/offline_query/model.js");
let seed=36003,cases=0,snapshots=0;
const rand=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return (seed>>>8)%n;};
const coord=()=>rand(13)-6;
const rectangle=()=>{
  const r=[coord(),coord(),coord(),coord()];
  if(rand(3)){if(r[0]>r[2])[r[0],r[2]]=[r[2],r[0]];if(r[1]>r[3])[r[1],r[3]]=[r[3],r[1]];}
  return r;
};
const brute=(points,[l,d,r,u])=>points.reduce((s,[x,y,w])=>s+(l<=x&&x<r&&d<=y&&y<u?w:0),0);
function reachable(n,edges,u,v){
  const g=Array.from({length:n},()=>[]);for(const[a,b]of edges){g[a].push(b);g[b].push(a);}
  const seen=new Set([u]),queue=[u];for(const x of queue)for(const y of g[x])if(!seen.has(y)){seen.add(y);queue.push(y);}return seen.has(v);
}
function validateBit(s){
  const frequency=s.ys.map(y=>s.activePoints.reduce((total,i)=>total+(s.points[i][1]===y?s.points[i][2]:0),0));
  assert.deepEqual(s.frequency,frequency);
  for(let i=1;i<s.bit.length;++i)assert.equal(s.bit[i],frequency.slice(i-(i&-i),i).reduce((a,b)=>a+b,0));
}
for(let trial=0;trial<160;++trial){
  const n=rand(17),array=Array.from({length:n},coord),queries=Array.from({length:10},()=>{const a=rand(n+1),b=rand(n+1);return [Math.min(a,b),Math.max(a,b)];});
  const input={array,queries,width:rand(4)},copy=JSON.stringify(input),mo=M.trace("mo",input);++cases;
  assert.equal(JSON.stringify(input),copy);
  assert.deepEqual(mo.answers,queries.map(([l,r])=>new Set(array.slice(l,r)).size));
  for(const s of mo.frames){
    ++snapshots;const expected={};for(const x of array.slice(s.l,s.r))expected[x]=(expected[x]||0)+1;
    for(const x of new Set(array))assert.equal(s.frequency[x]||0,expected[x]||0);
    assert.equal(s.distinct,new Set(array.slice(s.l,s.r)).size);
  }
  const vertices=1+rand(9),edges=Array.from({length:rand(17)},()=>[rand(vertices),rand(vertices)]);
  const pairs=Array.from({length:10},()=>[rand(vertices),rand(vertices)]),pbs=M.trace("pbs",{n:vertices,edges,queries:pairs});++cases;
  assert.deepEqual(pbs.answers,pairs.map(([u,v])=>{for(let t=0;t<=edges.length;++t)if(reachable(vertices,edges.slice(0,t),u,v))return t;return edges.length+1;}));
  for(const s of pbs.frames){
    ++snapshots;const leader=x=>{let count=0;while(s.parent[x]!==x){x=s.parent[x];assert.ok(++count<=vertices);}return x;};
    for(const[u,v]of pairs)assert.equal(leader(u)===leader(v),reachable(vertices,edges.slice(0,s.t),u,v));
    pairs.forEach(([u,v],i)=>{assert.ok(s.low[i]<s.high[i]);if(s.low[i]>=0)assert.equal(reachable(vertices,edges.slice(0,s.low[i]),u,v),false);if(s.high[i]<=edges.length)assert.equal(reachable(vertices,edges.slice(0,s.high[i]),u,v),true);});
  }
  const points=Array.from({length:rand(15)},()=>[coord(),coord(),coord()]);
  const rectangles=Array.from({length:8},rectangle);
  const sweep=M.trace("sweep",{points,queries:rectangles});++cases;
  assert.deepEqual(sweep.answers,rectangles.map(r=>brute(points,r)));
  for(const s of sweep.frames){++snapshots;validateBit(s);if(s.title.startsWith("Q")){assert.deepEqual(new Set(s.activePoints),new Set(points.map((p,i)=>p[0]<s.cursor?i:-1).filter(i=>i>=0)));}}
  const initial=points.slice(0,rand(7)),operations=[],expected=[],history=initial.slice();
  for(let i=0;i<18;++i){
    if(rand(2)){const p=[coord(),coord(),coord()];operations.push(["add",...p]);history.push(p);}
    else{const r=rectangle();operations.push(["sum",...r]);expected.push(brute(history,r));}
  }
  const cdq=M.trace("cdq",{points:initial,operations});++cases;
  assert.deepEqual(cdq.answers,expected);
  for(const s of cdq.frames){
    ++snapshots;validateBit(s);
    if(s.title.startsWith("右の Q")){
      const query=s.events.find(e=>e.id===s.activeId);
      for(const i of s.activePoints){const p=s.events.find(e=>e.point===i);assert.ok(p.time<query.time&&p.x<s.range[2]);}
    }
    if(s.title.includes("を x 順にマージ"))assert.ok(s.bit.every(x=>x===0)&&s.activePoints.length===0);
  }
  // Snapshots must not alias each other or the caller's data.
  const first=JSON.stringify(cdq.frames[0]);cdq.frames.at(-1).answers.push(123);assert.equal(JSON.stringify(cdq.frames[0]),first);
}
for(const [mode,input]of [["mo",{array:[],queries:[],width:0}],["pbs",{n:1,edges:[],queries:[]}],["sweep",{points:[],queries:[]}],["cdq",{points:[],operations:[]}]]){
  assert.deepEqual(M.trace(mode,input).answers,[]);++cases;
}
const boundaryPoints=[[1,2,4],[3,2,7],[1,2,-1]],rectangles=[[1,2,3,3],[1,2,1,3],[3,3,1,2]];
assert.deepEqual(M.trace("sweep",{points:boundaryPoints,queries:rectangles}).answers,[3,0,0]);
assert.deepEqual(M.trace("pbs",{n:2,edges:[],queries:[[0,0],[0,1]]}).answers,[0,1]);
assert.deepEqual(M.trace("cdq",{points:[],operations:[["sum",0,0,4,4],["add",1,2,4],["sum",0,0,4,4],["add",1,2,-1],["sum",1,2,3,3]]}).answers,[0,4,3]);
console.log(`offline model: ${cases+3} batches, ${snapshots} snapshots; naive ranges/BFS/time histories, BIT blocks, boundaries and rollback passed`);
