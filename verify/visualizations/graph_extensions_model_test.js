const assert=require('node:assert/strict');
const model=require('../../visualizations/graph_extensions/model.js');
const expected=require('../graph_extensions/visualization_expected.json');
let seed=401,checks=0;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const run=(id,input)=>{const r=model.trace(id,input);checks++;assert.ok(r.steps.length>=2);for(const s of r.steps){assert.equal(s.state.values.length===0||s.state.values.length===s.state.n,true);assert.ok(s.title&&s.detail);}return r;};
for(const m of model.modes)assert.deepEqual(run(m.id,m.sample).result,expected[m.id],m.id);
function forestCost(n,edges){const p=Array.from({length:n},(_,i)=>i),find=x=>p[x]===x?x:(p[x]=find(p[x]));let sum=0;for(const [u,v,w]of edges.slice().sort((a,b)=>a[2]-b[2]))if(find(u)!==find(v)){p[find(u)]=find(v);sum+=w;}return sum;}
function stExists(n,edges,s,t){const middle=Array.from({length:n},(_,i)=>i).filter(v=>v!==s&&v!==t);let yes=false;function visit(prefix,left){if(!left.length){const order=[s,...prefix,t],rank=Array(n);order.forEach((v,i)=>rank[v]=i);if(order.every(v=>(v===s||edges.some(([a,b])=>a===v&&rank[b]<rank[v]||b===v&&rank[a]<rank[v]))&&(v===t||edges.some(([a,b])=>a===v&&rank[b]>rank[v]||b===v&&rank[a]>rank[v]))))yes=true;return;}for(const v of left)visit([...prefix,v],left.filter(x=>x!==v));}visit([],middle);return yes;}
for(let trial=0;trial<140;trial++){
 const n=2+random(4),edges=[];for(let u=0;u<n;u++)for(let v=u+1;v<n;v++)if(random(2))edges.push([u,v,random(9)]);
 const coloring=run('vizing_edge_coloring',{n,edges}),c=coloring.steps.at(-1).state.colors;assert.equal(c.length,edges.length);assert.ok(Number(coloring.result[1])<=Number(coloring.result[0])+1);for(let v=0;v<n;v++){const used=edges.flatMap(([u,w],i)=>u===v||w===v?[c[i]]:[]);assert.equal(new Set(used).size,used.length);}
 const degree=Array(n).fill(0);for(const [u,v]of edges){degree[u]++;degree[v]++;}const realized=run('degree_sequence',{degree}).steps.at(-1).state.edges,got=Array(n).fill(0);for(const [u,v]of realized){got[u]++;got[v]++;}assert.deepEqual(got,degree);
 const st=run('st_numbering',{n,s:0,t:n-1,edges});assert.equal(st.result[0]!=='null',stExists(n,edges,0,n-1));
 const tree=run('gomory_hu_tree',{n,edges}).steps.at(-1).state.edges;for(let s=0;s<n;s++)for(let t=s+1;t<n;t++){
  let cut=Infinity;for(let side=0;side<1<<n;side++)if((side>>s&1)&&!(side>>t&1))cut=Math.min(cut,edges.reduce((x,[u,v,w])=>x+(((side>>u&1)!==(side>>v&1))?w:0),0));
  const q=[s],distance=Array(n).fill(-1);distance[s]=Infinity;for(let i=0;i<q.length;i++)for(const [u,v,w]of tree){const next=u===q[i]?v:v===q[i]?u:-1;if(next>=0&&distance[next]<0){distance[next]=Math.min(distance[q[i]],w);q.push(next);}}assert.equal(distance[t],cut);
 }
 const online=run('incremental_msf',{n,edges});edges.forEach((_,i)=>assert.equal(Number(online.result[i]),forestCost(n,edges.slice(0,i+1))));
 const values=Array.from({length:n},()=>random(100)),all=[];for(let u=0;u<n;u++)for(let v=u+1;v<n;v++)all.push([u,v,values[u]^values[v]]);assert.equal(Number(run('boruvka',{values}).result[0]),forestCost(n,all));
 // Bipartite cover and DAG antichain independently enumerate vertex subsets.
 const left=1+random(3),right=1+random(3),be=[];for(let u=0;u<left;u++)for(let v=0;v<right;v++)if(random(2))be.push([u,v]);let cover=left+right;
 for(let mask=0;mask<1<<(left+right);mask++)if(be.every(([u,v])=>(mask>>u&1)||(mask>>(left+v)&1)))cover=Math.min(cover,mask.toString(2).replaceAll('0','').length);
 assert.equal(Number(run('bipartite_vertex_cover',{left,right,edges:be}).result[0]),cover);
 const reach=Array.from({length:n},()=>Array(n).fill(false));for(const [u,v]of edges)reach[u][v]=true;for(let k=0;k<n;k++)for(let u=0;u<n;u++)for(let v=0;v<n;v++)reach[u][v]||=reach[u][k]&&reach[k][v];let width=0;
 for(let mask=0;mask<1<<n;mask++){let ok=true;for(let u=0;u<n;u++)for(let v=0;v<n;v++)if((mask>>u&1)&&(mask>>v&1)&&reach[u][v])ok=false;if(ok)width=Math.max(width,mask.toString(2).replaceAll('0','').length);}assert.equal(Number(run('dag_maximum_antichain',{n,edges}).result[0]),width);
 // Every subset of original edges is a candidate edge cover.
 let edgeCover=Infinity;for(let mask=0;mask<1<<edges.length;mask++){let seen=0,count=0;edges.forEach(([u,v],i)=>{if(mask>>i&1){seen|=1<<u|1<<v;count++;}});if(seen===(1<<n)-1)edgeCover=Math.min(edgeCover,count);}assert.equal(run('minimum_edge_cover',{n,edges}).result[0],Number.isFinite(edgeCover)?String(edgeCover):'null');
 const directed=Array.from({length:random(13)},()=>[random(n),random(n),random(11)-5]),d=Array.from({length:n},(_,u)=>Array.from({length:n},(_,v)=>u===v?0:Infinity));for(const [u,v,w]of directed)d[u][v]=Math.min(d[u][v],w);for(let k=0;k<n;k++)for(let u=0;u<n;u++)for(let v=0;v<n;v++)d[u][v]=Math.min(d[u][v],d[u][k]+d[k][v]);const neg=d.some((row,v)=>row[v]<0);
 assert.equal(run('negative_cycle',{n,source:0,edges:directed}).result[0],String(Number(neg)));const diff=run('difference_constraints',{n,edges:directed});assert.equal(diff.result[0]==='null',neg);if(!neg)for(const [u,v,w]of directed)assert.ok(Number(diff.result[v])-Number(diff.result[u])<=w);
 const mean=run('minimum_mean_cycle',{n,edges:directed});let best=Infinity;for(let s=0;s<n;s++){function dfs(v,used,cost,length){for(const [u,w,c]of directed)if(u===v){if(w===s)best=Math.min(best,(cost+c)/(length+1));else if(!(used>>w&1))dfs(w,used|1<<w,cost+c,length+1);}}dfs(s,1<<s,0,0);}if(Number.isFinite(best))assert.ok(Math.abs(Number(mean.result[0])/Number(mean.result[1])-best)<1e-12);else assert.deepEqual(mean.result,['null']);
}
for(let trial=0;trial<100;trial++){
 const n=2+random(6),edges=[];for(let v=1;v<n;v++)edges.push([v,random(v)]);edges.push([random(n),random(n)]);const r=run('unicyclic_decomposition',{n,vertex:n-1,edges});assert.notEqual(r.result[0],'null');const state=r.steps.at(-1).state;assert.equal(Number(r.result[0]),state.selected.length);
 const ops=Array.from({length:5},()=>{let l=random(n+1),r=random(n+1),a=random(n+1),b=random(n+1);if(l>r)[l,r]=[r,l];if(a>b)[a,b]=[b,a];return [l,r,a,b,random(8)];}),d=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?0:Infinity));for(const [l,r,a,b,w]of ops)for(let u=l;u<r;u++)for(let v=a;v<b;v++)d[u][v]=Math.min(d[u][v],w);for(let k=0;k<n;k++)for(let u=0;u<n;u++)for(let v=0;v<n;v++)d[u][v]=Math.min(d[u][v],d[u][k]+d[k][v]);assert.deepEqual(run('segment_tree_graph',{n,source:0,ops}).result,d[0].map(x=>Number.isFinite(x)?String(x):'inf'));
}
assert.deepEqual(run('min_cost_disjoint_paths',{n:2,s:0,t:1,k:2,edges:[[0,1,2],[0,1,3]]}).result,['2','5','5']);
assert.deepEqual(run('min_cost_disjoint_paths',{n:2,s:0,t:1,k:1,edges:[]}).result,['null','null','null']);
assert.deepEqual(run('st_numbering',{n:2,s:0,t:1,edges:[]}).result,['null']);
for(const input of [{n:2,s:0,t:0,edges:[]},{n:2,s:0,t:1,edges:[[0,0]]}])assert.throws(()=>model.trace('st_numbering',input));
assert.throws(()=>model.trace('vizing_edge_coloring',{n:2,edges:[[0,1],[1,0]]}));
assert.throws(()=>model.trace('dag_maximum_antichain',{n:2,edges:[[0,1],[1,0]]}));
console.log(`graph extensions model: ${checks} traces / 15 C++ sample comparisons / independent exhaustive oracles passed`);
