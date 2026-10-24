'use strict';
const assert=require('node:assert/strict');
const model=require('../../visualizations/tree_extensions/model.js');
const expected=require('../tree_extensions/visualization_expected.json');
for(const m of model.modes)assert.deepEqual(model.trace(m.id,m.sample).result,expected[m.id]);
let seed=321739;const rnd=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
function parents(adj,r){const p=Array(adj.length).fill(-2),q=[r];p[r]=-1;for(const u of q)for(let v=0;v<adj.length;++v)if(adj[u][v]&&p[v]===-2){p[v]=u;q.push(v);}return p;}
function path(adj,u,v){const p=parents(adj,u),a=[];if(p[v]===-2)return a;for(let x=v;x!==-1;x=p[x])a.push(x);return a.reverse();}
function sub(adj,r,v){const p=parents(adj,r),a=[];for(let u=0;u<p.length;++u)if(p[u]!==-2){let x=u;while(x!==-1&&x!==v)x=p[x];if(x===v)a.push(u);}return a;}
let snapshots=0;
for(let trial=0;trial<100;++trial){
  const n=1+rnd(8),values=Array.from({length:n},()=>rnd(7)-2),a=values.slice(),edges=[],adj=Array.from({length:n},()=>Array(n).fill(false)),ops=[],want=[],states=[];
  for(let v=1;v<n;++v){const u=rnd(v);edges.push([u,v]);adj[u][v]=adj[v][u]=true;}
  for(let step=0;step<30;++step){let u=rnd(n),v=rnd(n),kind=rnd(13),op;
    if(kind===0){op=['link',u,v];if(!path(adj,u,v).length)adj[u][v]=adj[v][u]=true;}
    else if(kind===1){op=['cut',u,v];adj[u][v]=adj[v][u]=false;}
    else if(kind===2){a[u]=rnd(9)-4;op=['set',u,a[u]];}
    else if(kind<6){const mul=rnd(3)-1,b=rnd(7)-3;let target;
      if(kind===3){op=['path_apply',u,v,mul,b];target=path(adj,u,v);}
      else if(kind===4){op=['subtree_apply',u,v,mul,b];target=sub(adj,u,v);}
      else {op=['component_apply',u,mul,b];target=sub(adj,u,u);}
      for(const x of target)a[x]=mul*a[x]+b;
    }else if(kind===6){op=['independent',u];const included=new Set(sub(adj,u,u));let total=0n;
      for(let mask=0;mask<2**n;++mask){let ok=true,weight=1n;for(let i=0;i<n;++i)if(mask>>i&1){if(!included.has(i))ok=false;weight*=BigInt(a[i]);for(let j=0;j<i;++j)if((mask>>j&1)&&adj[i][j])ok=false;}if(ok)total+=weight;}want.push(String(total));
    }else if(kind===7){op=['distance',u];want.push(String(sub(adj,u,u).reduce((s,x)=>s+a[x]*(path(adj,u,x).length-1),0)));}
    else{let target;
      if(kind===8){op=['path_sum',u,v];target=path(adj,u,v);}
      else if(kind===9){op=['subtree_mass',u,v];target=sub(adj,u,v);}
      else{op=['component_sum',u];target=sub(adj,u,u);}
      want.push(target.length?String(target.reduce((s,x)=>s+a[x],0)):'null');
    }
    ops.push(op);states.push(a.slice());
  }
  for(const id of ['dynamic_top_tree','lazy_top_tree','dynamic_subtree_aggregate','fully_dynamic_tree_dp']){
    const trace=model.trace(id,{values,edges,ops});assert.deepEqual(trace.result,want);
    let k=0;for(const step of trace.steps){++snapshots;if(step.state?.operation)assert.deepEqual(step.state.values,states[k++]);
      if(step.state?.cluster){const c=step.state.cluster,all=c.spine.concat(c.off);assert.equal(new Set(all).size,all.length);assert.deepEqual(step.state.active,all);assert.equal(step.rows[0][2],c.spine.reduce((s,x)=>s+step.state.values[x],0));assert.equal(step.rows[1][2],c.off.reduce((s,x)=>s+step.state.values[x],0));}}
    assert.equal(k,ops.length);
  }
}
for(let n=1;n<=6;++n){let total=n**Math.max(0,n-2);for(let mask=0;mask<total;++mask){let x=mask;const code=Array.from({length:Math.max(0,n-2)},()=>{const v=x%n;x=Math.floor(x/n);return v;});assert.deepEqual(model.trace('prufer_code',{n,code}).result,[...code.map(String),'1','1']);}}
for(let trial=0;trial<100;++trial){
  const n=1+rnd(12),edges=[],values=Array.from({length:n},()=>rnd(17)),adj=Array.from({length:n},()=>Array(n).fill(false));
  for(let v=1;v<n;++v){const u=rnd(v);edges.push([u,v]);adj[u][v]=adj[v][u]=true;}
  const root=rnd(n),vertex=rnd(n),fixedRoot=rnd(n),want=sub(adj,root,vertex).reduce((s,x)=>s+values[x],0);
  assert.deepEqual(model.trace('rerooted_subtree',{values,edges,root,vertex,fixedRoot}).result,[String(want)]);
  const weighted=edges.map(([u,v])=>[u,v,rnd(8)]),d=Array.from({length:n},(_,u)=>Array.from({length:n},(_,v)=>u===v?0:1e9));
  for(const [u,v,w]of weighted)d[u][v]=d[v][u]=w;
  for(let k=0;k<n;++k)for(let u=0;u<n;++u)for(let v=0;v<n;++v)d[u][v]=Math.min(d[u][v],d[u][k]+d[k][v]);
  const ecc=d.map(row=>Math.max(...row)),radius=Math.min(...ecc),best=ecc.flatMap((x,v)=>x===radius?[v]:[]),trace=model.trace('tree_center',{n,edges:weighted});
  assert.deepEqual(trace.steps.at(-1).state.active,best);assert.equal(trace.steps.at(-1).rows[1][2],radius);
}
for(const [id,input]of [['prufer_code',{n:0,code:[]}],['prufer_code',{n:3,code:[3]}],['tree_center',{n:2,edges:[[0,1,-1]]}],['dynamic_top_tree',{values:[1,2],edges:[[0,1],[1,0]],ops:[]}],['lazy_top_tree',{values:[1],edges:[],ops:[['unknown',0]]}]])assert.throws(()=>model.trace(id,input));
console.log(`tree visualizations: 7 C++ reference examples, 400 dynamic traces, exhaustive small codes, 200 static traces, ${snapshots} snapshots passed`);
