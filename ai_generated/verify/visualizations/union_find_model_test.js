"use strict";
const assert = require("node:assert/strict");
const model = require("../../visualizations/union_find/union_find_model.js");
let seed = 3206;
const random = n => { seed = (Math.imul(seed,1664525)+1013904223) >>> 0; return seed%n; };
function difference(edges,u,v) {
  const values = Array(edges.length).fill(null), todo = [u]; values[u] = 0;
  for (let i=0; i<todo.length; ++i) for (const [to,w] of edges[todo[i]]) {
    if (values[to] !== null) continue;
    values[to] = values[todo[i]]+w; todo.push(to);
  }
  return values[v];
}
function invariant(state) {
  const counts = Array(state.parent.length).fill(0);
  for (let v=0; v<state.parent.length; ++v) {
    let root=v, depth=0;
    while (state.parent[root] !== -1) { root=state.parent[root]; assert(++depth<=state.parent.length); }
    ++counts[root];
  }
  assert.equal(counts.filter(n=>n>0).length,state.count);
  counts.forEach((n,v)=>{ if (n) { assert.equal(state.size[v],n); assert.equal(state.weight[v],0); } });
}
for (const weighted of [false,true]) for (let trial=0; trial<150; ++trial) {
  const n=2+random(11), edges=Array.from({length:n},()=>[]);
  let state=model.initial(n,weighted);
  for (let step=0; step<100; ++step) {
    const u=random(n),v=random(n),known=difference(edges,u,v),kind=["merge","merge","leader","diff"][random(4)];
    let w=weighted ? random(41)-20 : 0;
    if (known !== null && random(3)===0) w=known;
    const before=JSON.stringify(state), result=model.trace(state,{kind,u,v,w});
    assert.equal(JSON.stringify(state),before,"history snapshots must not be mutated");
    if (kind==="merge") {
      const expected=known===null ? "Merged" : known===w ? "AlreadyConsistent" : "Contradiction";
      assert.equal(result.result,expected);
      if (known===null) { edges[u].push([v,w]); edges[v].push([u,-w]); }
    } else if (kind==="diff") assert.equal(result.result,known);
    result.frames.forEach(frame=>invariant(frame.state));
    state=result.state;
    for (let a=0; a<n; ++a) for (let b=0; b<n; ++b) {
      const x=model.inspect(state,a),y=model.inspect(state,b), expected=difference(edges,a,b);
      assert.equal(x.root===y.root ? y.potential-x.potential : null,expected);
    }
  }
}
console.log("visualization model: 300 random sequences, 30000 operations, every frame checked");
