(function(root){
'use strict';
const modes=[
 ['segment_tree_graph','区間辺グラフ','graph_basic',{n:4,source:0,ops:[[0,1,1,3,2],[1,3,3,4,5],[0,2,2,4,4]]}],
 ['bipartite_vertex_cover','二部最小頂点被覆','graph_flow',{left:3,right:3,edges:[[0,0],[1,0],[2,1]]}],
 ['minimum_edge_cover','最小辺被覆','graph_combinatorics',{n:4,edges:[[0,1],[1,2],[2,0],[2,3]]}],
 ['gomory_hu_tree','Gomory–Hu Tree','graph_flow',{n:3,edges:[[0,1,2],[1,2,3],[0,2,4]]}],
 ['st_numbering','st-Numbering','graph_connectivity',{n:4,s:0,t:2,edges:[[0,1],[1,2],[2,3],[3,0]]}],
 ['minimum_mean_cycle','最小平均閉路','graph_basic',{n:3,edges:[[0,1,2],[1,2,-4],[2,0,1]]}],
 ['degree_sequence','次数列の実現','graph_combinatorics',{degree:[2,2,2,2]}],
 ['boruvka','Borůvka / XOR-MST','graph_basic',{values:[0,1,4,5]}],
 ['incremental_msf','Incremental MSF','graph_basic',{n:4,edges:[[0,1,5],[1,2,4],[0,2,1],[2,3,2]]}],
 ['unicyclic_decomposition','なもり分解','graph_connectivity',{n:5,vertex:4,edges:[[0,1],[1,2],[2,0],[1,3],[3,4]]}],
 ['dag_maximum_antichain','DAGの最大反鎖','graph_combinatorics',{n:5,edges:[[0,2],[1,2],[2,3],[2,4]]}],
 ['vizing_edge_coloring','Δ+1辺彩色','graph_combinatorics',{n:3,edges:[[0,1],[1,2],[2,0]]}],
 ['difference_constraints','差分制約','graph_basic',{n:3,edges:[[0,1,-2],[1,2,1],[0,2,0]],extra:[2,0,0]}],
 ['negative_cycle','負閉路の抽出','graph_basic',{n:4,source:0,edges:[[0,1,2],[2,3,-3],[3,2,1]]}],
 ['min_cost_disjoint_paths','最小費用の素パス','graph_combinatorics',{n:4,s:0,t:3,k:2,directed:true,edges:[[0,1,1],[1,3,2],[0,2,2],[2,3,3],[1,2,0]]}]
].map(([id,title,cat,sample],i)=>({id,title:`G-${String(i+1).padStart(2,'0')} · ${title}`,doc:`${cat}/${id}.md`,sample}));
function integer(v,lo=0,hi=1000){if(!Number.isSafeInteger(v)||v<lo||v>hi)throw Error(`整数 ${lo}〜${hi} を指定してください`);return v;}
function array(v,max=40){if(!Array.isArray(v)||v.length>max)throw Error(`配列は${max}要素以下です`);return v;}
const copy=x=>JSON.parse(JSON.stringify(x));
function trace(id,input){
 if(!modes.some(m=>m.id===id))throw Error('モードがありません');
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください');
 const steps=[],result=[];let n=0,edges=[],directed=false,values=[],active=[],selected=[],colors=[];
 function add(title,detail,columns=[],rows=[],info={}){steps.push({title,detail,columns,rows:copy(rows),values:Object.entries(info).map(([label,value])=>({label,value:String(value)})),state:{n,edges:copy(edges),values:values.slice(),active:active.slice(),selected:selected.slice(),colors:colors.slice(),directed}});}
 function readEdges(a,weighted=false){return array(a,18).map(e=>{array(e,3);if(e.length<(weighted?3:2))throw Error('辺の引数が不足しています');return [integer(e[0],0,n-1),integer(e[1],0,n-1),weighted?integer(e[2],-100,100):1];});}
 function adj(){const a=Array.from({length:n},()=>[]);edges.forEach(([u,v,w],id)=>{a[u].push({v,w,id});if(!directed)a[v].push({v:u,w,id});});return a;}
 function matching(L,R,es,show=true){
  const left=Array(L).fill(-1),right=Array(R).fill(-1),a=Array.from({length:L},()=>[]);for(const [u,v]of es)a[u].push(v);
  function augment(u,seen){for(const v of a[u])if(!seen[v]){seen[v]=true;if(right[v]<0||augment(right[v],seen)){left[u]=v;right[v]=u;return true;}}return false;}
  for(let u=0;u<L;u++)if(augment(u,Array(R).fill(false))&&show)add('交互路でmatchingを1増やす','図解は小さい入力用のDFS増加路。C++はHopcroft–Karpです。',['左','相手'],left.map((v,i)=>[i,v]));
  const zl=Array(L).fill(false),zr=Array(R).fill(false),queue=[];for(let u=0;u<L;u++)if(left[u]<0){zl[u]=true;queue.push(u);}
  for(let i=0;i<queue.length;i++)for(const v of a[queue[i]])if(!zr[v]){zr[v]=true;const u=right[v];if(u>=0&&!zl[u]){zl[u]=true;queue.push(u);}}
  return {left,right,zl,zr,size:left.filter(v=>v>=0).length};
 }
 function bellman(source=-1,show=true){
  const d=Array(n).fill(source<0?0:Infinity),parent=Array(n).fill(-1),pe=Array(n).fill(-1);if(source>=0)d[source]=0;
  let x=-1;for(let round=0;round<n;round++){
   x=-1;edges.forEach(([u,v,w],id)=>{if(d[u]+w<d[v]){d[v]=d[u]+w;parent[v]=u;pe[v]=id;x=v;}});
   values=d.map(x=>Number.isFinite(x)?x:'∞');if(show)add(`緩和 ${round+1} 回目`,'全成分では全頂点0から開始。V回目にも更新があれば負閉路があります。',['頂点','距離','親','元辺ID'],d.map((x,v)=>[v,Number.isFinite(x)?x:'∞',parent[v],pe[v]]));
   if(x<0)return {d,cycle:null};
  }
  for(let i=0;i<n;i++)x=parent[x];const cycle=[],vertices=[];let v=x;
  do{vertices.push(v);cycle.push(pe[v]);v=parent[v];}while(v!==x);cycle.reverse();vertices.push(x);vertices.reverse();
  if(show){selected=cycle;active=vertices;add('親をV回辿って閉路を取り出す','この元辺の重みを合計すると負になります。',['辺ID','費用'],cycle.map(i=>[i,edges[i][2]]));}
  return {d,cycle,vertices,weight:cycle.reduce((s,i)=>s+edges[i][2],0)};
 }
 if(id==='degree_sequence'){
  const degree=array(input.degree,10).map(d=>integer(d,-1,10));n=degree.length;values=degree.slice();
  add('指定された次数','最大残次数の頂点を、次の大きい次数の頂点へ接続します。',['頂点','次数'],degree.map((d,v)=>[v,d]));
  let ok=degree.every(d=>d>=0&&d<n);
  while(ok&&degree.some(d=>d>0)){
   const order=degree.map((d,v)=>[d,v]).sort((a,b)=>b[0]-a[0]||b[1]-a[1]),[d,v]=order[0];degree[v]=0;active=[v];
   if(order.slice(1).filter(x=>x[0]>0).length<d){ok=false;break;}
   for(let i=1;i<=d;i++){const u=order[i][1];edges.push([v,u,1]);degree[u]--;active.push(u);}
   values=degree.slice();add(`頂点${v}の${d}辺を確定`,'選んだ相手の残次数を1減らします。',['頂点','残次数'],degree.map((d,v)=>[v,d]));
  }
  result.push(ok?String(edges.length):'null');add(ok?'実現完了':'実現できない','負次数や相手の不足があれば単純グラフとして実現できません。');
 }else if(id==='bipartite_vertex_cover'){
  const L=integer(input.left,0,6),R=integer(input.right,0,6);n=L+R;const es=array(input.edges,18).map(e=>[integer(e[0],0,L-1),integer(e[1],0,R-1)]);edges=es.map(([u,v])=>[u,L+v,1]);
  add('左右に分かれた頂点','未対応の左頂点から交互路を探索します。');const m=matching(L,R,es);active=[...m.zl.flatMap((x,i)=>x?[i]:[]),...m.zr.flatMap((x,i)=>x?[L+i]:[])];
  add('交互到達集合 Z','最小被覆=(左−Z)∪(右∩Z)。その補集合が最大独立集合です。',['頂点','側','Z','最小被覆'],Array.from({length:n},(_,v)=>[v,v<L?'左':'右',v<L?m.zl[v]:m.zr[v-L],v<L?!m.zl[v]:m.zr[v-L]]));result.push(String(m.size),String(m.size),String(n-m.size));
 }else if(id==='boruvka'){
  const a=array(input.values,10).map(x=>integer(x,0,65535));n=a.length;values=a.slice();for(let u=0;u<n;u++)for(let v=u+1;v<n;v++)edges.push([u,v,a[u]^a[v]]);
  const p=Array.from({length:n},(_,i)=>i);const find=x=>p[x]===x?x:(p[x]=find(p[x]));let cost=0;
  add('XOR完全グラフ','図解は全辺を列挙します。C++は成分外の候補をtrieで探します。');
  for(let round=0;round<n;round++){
   const best=new Map();edges.forEach(([u,v,w],i)=>{if(find(u)!==find(v))for(const c of [find(u),find(v)])if(!best.has(c)||edges[best.get(c)][2]>w)best.set(c,i);});
   if(!best.size)break;add(`フェーズ${round+1}: 成分の最小外向き辺`,'各成分から少なくとも1本、最小の候補を出します。',['成分','辺ID'],[...best]);
   for(const i of best.values()){const [u,v,w]=edges[i];if(find(u)!==find(v)){p[find(u)]=find(v);selected.push(i);cost+=w;}}
   values=p.map((_,v)=>`成分 ${find(v)}`);add('候補を併合','既に同じ成分なら候補を飛ばし、閉路を作らないようにします。',[],[],{総重み:cost});
  }result.push(String(cost));
 }else{
  n=integer(input.n,1,10);values=Array(n).fill('');directed=['segment_tree_graph','st_numbering','minimum_mean_cycle','dag_maximum_antichain','difference_constraints','negative_cycle'].includes(id)||(id==='min_cost_disjoint_paths'&&input.directed!==false);
  if(id!=='segment_tree_graph')edges=readEdges(input.edges,['gomory_hu_tree','minimum_mean_cycle','incremental_msf','difference_constraints','negative_cycle','min_cost_disjoint_paths'].includes(id));
  add('入力グラフ','辺IDは入力順の0始まりです。',['辺ID','from','to','重み'],edges.map((e,i)=>[i,...e]));
  if(id==='segment_tree_graph'){
   for(const op of array(input.ops,20)){
    if(array(op,5).length!==5)throw Error('[l,r,a,b,cost]を指定してください');const [l,r,a,b,w]=op;[l,r,a,b].forEach(x=>integer(x,0,n));integer(w,0,100);if(l>r||a>b)throw Error('半開区間の左右が逆です');
    const cover=(l,r)=>{let base=1;while(base<n)base*=2;const nodes=[];function walk(k,x,y){if(r<=x||y<=l)return;if(l<=x&&y<=r){nodes.push([k,x,y]);return;}const mid=(x+y)/2;walk(k*2,x,mid);walk(k*2+1,mid,y);}walk(1,0,base);return nodes;};
    add(`[${l},${r}) → [${a},${b})`,'入口の上向き木と出口の下向き木の間を接続。元頂点の全組に辺を持つのと同じ到達を表します。',['役割','ノード','l','r'],[...cover(l,r).map(x=>['入口',...x]),...cover(a,b).map(x=>['出口',...x])]);
    for(let u=l;u<r;u++)for(let v=a;v<b;v++)edges.push([u,v,w]);
   }
   const s=integer(input.source,0,n-1),d=Array(n).fill(Infinity);d[s]=0;
   for(let i=0;i<n;i++){for(const [u,v,w]of edges)d[v]=Math.min(d[v],d[u]+w);values=d.map(x=>Number.isFinite(x)?x:'∞');add('元頂点上の距離','図解の最短路は展開した辺で計算します。補助頂点には0費用の辺を使います。');}
   result.push(...d.map(x=>Number.isFinite(x)?String(x):'inf'));
  }else if(id==='minimum_edge_cover'){
   if(edges.some(([u,v])=>u===v))throw Error('自己ループなしの入力を指定してください');let best=[];
   function dfs(i,used,chosen){if(i===edges.length){if(chosen.length>best.length)best=chosen.slice();return;}dfs(i+1,used,chosen);const [u,v]=edges[i];if(!(used>>u&1)&&!(used>>v&1)){chosen.push(i);dfs(i+1,used|1<<u|1<<v,chosen);chosen.pop();}}dfs(0,0,[]);
   selected=best.slice();add('最大matchingを選ぶ','図解では部分集合を探索。C++は一般グラフ用のBlossom matchingを共有します。',['採用辺ID'],selected.map(i=>[i]));
   const seen=Array(n).fill(false);for(const i of selected){seen[edges[i][0]]=seen[edges[i][1]]=true;}let ok=true;
   for(let v=0;v<n;v++)if(!seen[v]){const i=edges.findIndex(([a,b])=>a===v||b===v);if(i<0){ok=false;break;}selected.push(i);seen[edges[i][0]]=seen[edges[i][1]]=true;active=[v];add('未対応頂点に接続辺を補う','最大matchingなので、未対応の頂点同士を直接つなぐ辺はありません。',['辺ID'],selected.map(i=>[i]));}
   result.push(ok?String(selected.length):'null');add(ok?'全頂点を被覆':'孤立頂点があるため不可能','辺の本数はV−matchingの大きさです。');
  }else if(id==='gomory_hu_tree'){
   if(edges.some(e=>e[2]<0))throw Error('容量は非負です');const p=Array(n).fill(0),weight=Array(n).fill(0),inputEdges=copy(edges);
   for(let s=1;s<n;s++){
    const t=p[s];let best=Infinity,side=0;for(let mask=0;mask<1<<n;mask++)if((mask>>s&1)&&!(mask>>t&1)){const c=inputEdges.reduce((sum,[u,v,w])=>sum+(((mask>>u&1)!==(mask>>v&1))?w:0),0);if(c<best){best=c;side=mask;}}
    active=Array.from({length:n},(_,v)=>v).filter(v=>side>>v&1);add(`${s}–${t} 最小カット`,'図解は全カットを列挙します。C++はDinicを各端点対でリセットして実行します。',['頂点','親'],p.map((v,i)=>[i,v]),{カット容量:best});
    for(let v=s+1;v<n;v++)if(p[v]===t&&(side>>v&1))p[v]=s;
    if(side>>p[t]&1){p[s]=p[t];p[t]=s;weight[s]=weight[t];weight[t]=best;}else weight[s]=best;
   }
   edges=Array.from({length:n-1},(_,i)=>[i+1,p[i+1],weight[i+1]]);selected=edges.map((_,i)=>i);active=[];add('Gomory–Hu木','木パス上の最小重みが元グラフの2点間の最小カット値です。',['子','親','容量'],edges);result.push(...weight.slice(1).sort((a,b)=>a-b).map(String));
  }else if(id==='st_numbering'){
   directed=false;if(edges.some(([u,v])=>u===v))throw Error('自己ループは扱いません');const s=integer(input.s,0,n-1),t=integer(input.t,0,n-1);if(s===t)throw Error('sとtは異なる頂点です');
   const original=adj().map(es=>es.map(e=>e.v)),a=original.map(row=>row.slice());a[s].unshift(t);a[t].unshift(s);const pre=Array(n).fill(-1),low=Array(n),par=Array(n).fill(-1),order=[];
   function dfs(v){pre[v]=low[v]=order.length;order.push(v);for(const w of a[v]){if(pre[w]<0){par[w]=v;dfs(w);}low[v]=Math.min(low[v],pre[w],par[w]===v?low[w]:Infinity);}}dfs(s);
   add('DFSとlowpoint','仮想st辺を最初に辿り、祖先へ戻れる最小の訪問番号を調べます。',['頂点','pre','low','parent'],pre.map((x,v)=>[v,x,low[v],par[v]]));
   const list=[s,t],before=Array(n).fill(false);before[s]=true;
   for(const v of order.slice(2)){const p=par[v],left=before[order[low[v]]],at=list.indexOf(p)+(left?0:1);list.splice(at,0,v);before[p]=!left;active=[v,p];add(`頂点${v}を親${p}の${left?'前':'後'}に挿入`,'lowpointの符号に応じて順序を更新し、親の符号を切り替えます。',['番号','頂点'],list.map((x,i)=>[i,x]));}
   const rank=Array(n).fill(-1);list.forEach((v,i)=>rank[v]=i);const ok=order.length===n&&list[0]===s&&list.at(-1)===t&&original.every((row,v)=>(v===s||row.some(w=>rank[w]<rank[v]))&&(v===t||row.some(w=>rank[w]>rank[v])));
   if(ok){edges=edges.map(([u,v,w])=>rank[u]<rank[v]?[u,v,w]:[v,u,w]);directed=true;values=rank;result.push(String(s),String(t),String(edges.length));}else result.push('null');add(ok?'双極向き付け':'st順序を作れません','元辺を小さい番号から大きい番号へ向けます。');
  }else if(id==='minimum_mean_cycle'){
   const dp=[Array(n).fill(0)];for(let k=1;k<=n;k++){const d=Array(n).fill(Infinity);for(const [u,v,w]of edges)d[v]=Math.min(d[v],dp[k-1][u]+w);dp.push(d);values=d.map(x=>Number.isFinite(x)?x:'∞');add(`長さ${k}のwalk DP`,'dp[k][v] = min(dp[k−1][u] + cost(u,v))。すべての頂点から出発可能にします。',['頂点','最小重み'],values.map((x,v)=>[v,x]));}
   let best=null;for(let v=0;v<n;v++)if(Number.isFinite(dp[n][v])){let mx=null;for(let k=0;k<n;k++)if(Number.isFinite(dp[k][v])){const c=[dp[n][v]-dp[k][v],n-k];if(!mx||c[0]*mx[1]>mx[0]*c[1])mx=c;}if(!best||mx[0]*best[1]<best[0]*mx[1])best=mx;}
   if(!best)result.push('null');else{let cycle=null;const a=adj();for(let s=0;s<n&&!cycle;s++){function dfs(v,used,sum,path){for(const e of a[v]){if(e.v===s&&(sum+e.w)*best[1]===best[0]*(path.length+1)){cycle=[...path,e.id];return;}if(!(used>>e.v&1)&&!cycle)dfs(e.v,used|1<<e.v,sum+e.w,[...path,e.id]);}}dfs(s,1<<s,0,[]);}selected=cycle;const gcd=(a,b)=>b?gcd(b,a%b):Math.abs(a),d=gcd(best[0],best[1]);result.push(String(best[0]/d),String(best[1]/d),String(cycle.length));}
   add('最小の平均を持つ閉路','図解は最適比に一致する単純閉路を列挙。C++は最適比で重みを調整し、tight辺から復元します。',['元辺ID','費用'],selected.map(i=>[i,edges[i][2]]));
  }else if(id==='incremental_msf'){
   const all=edges;edges=[];let cost=0;
   all.forEach(e=>{const i=edges.length;edges.push(e);const [s,t,w]=e;const a=Array.from({length:n},()=>[]);for(const id of selected){const [u,v]=edges[id];a[u].push([v,id]);a[v].push([u,id]);}
    const par=Array(n).fill(-1),via=Array(n).fill(-1),q=[s];par[s]=s;for(let j=0;j<q.length;j++)for(const [v,id]of a[q[j]])if(par[v]<0){par[v]=q[j];via[v]=id;q.push(v);}
    let removed=-1;if(s!==t){if(par[t]<0){selected.push(i);cost+=w;}else{const path=[];for(let v=t;v!==s;v=par[v])path.push(via[v]);const largest=path.sort((a,b)=>edges[b][2]-edges[a][2]||b-a)[0];if(w<edges[largest][2]){removed=largest;selected=selected.filter(x=>x!==largest);selected.push(i);cost+=w-edges[largest][2];}}}
    active=[s,t];add(`辺${i}を追加`,'閉路ができた場合、そのパス上の最大辺より軽いときだけ交換します。図解はBFS、C++はLCTを使います。',['採用ID','費用'],selected.map(i=>[i,edges[i][2]]),{除去ID:removed,総重み:cost});result.push(String(cost));});result.push(String(selected.length));
  }else if(id==='unicyclic_decomposition'){
   const a=adj(),degree=a.map(x=>x.length),alive=edges.map(()=>true),queue=degree.flatMap((d,v)=>d===1?[v]:[]);
   for(let i=0;i<queue.length;i++){const v=queue[i];for(const e of a[v])if(alive[e.id]){alive[e.id]=false;degree[v]--;degree[e.v]--;if(degree[e.v]===1)queue.push(e.v);}active=[v];values=degree.slice();add(`葉${v}を取り除く`,'葉を剥がしたあとに残る辺が唯一の閉路です。C++ではCactus分解の閉路を取り出します。');}
   selected=alive.flatMap((yes,i)=>yes?[i]:[]);const root=Array(n).fill(-1),depth=Array(n).fill(0),q=[];for(let v=0;v<n;v++)if(degree[v]){root[v]=v;q.push(v);}
   for(let i=0;i<q.length;i++)for(const e of a[q[i]])if(!alive[e.id]&&root[e.v]<0){root[e.v]=root[q[i]];depth[e.v]=depth[q[i]]+1;q.push(e.v);}
   // Connectivity is checked separately; m=n alone is insufficient.
   const seen=new Set([0]),walk=[0];for(let i=0;i<walk.length;i++)for(const e of a[walk[i]])if(!seen.has(e.v)){seen.add(e.v);walk.push(e.v);}
   const v=integer(input.vertex,0,n-1);if(edges.length!==n||seen.size!==n||!selected.length)result.push('null');else result.push(String(selected.length),String(root[v]),String(depth[v]));
   values=root.map((r,v)=>`根${r}/深さ${depth[v]}`);add('閉路から付随木を探索','各頂点を、その木が接続する閉路頂点へ割り当てます。',['頂点','根','深さ'],root.map((r,v)=>[v,r,depth[v]]));
  }else if(id==='dag_maximum_antichain'){
   const reach=Array.from({length:n},()=>Array(n).fill(false));for(const [u,v]of edges)reach[u][v]=true;
   for(let k=0;k<n;k++){for(let u=0;u<n;u++)for(let v=0;v<n;v++)reach[u][v]||=reach[u][k]&&reach[k][v];active=[k];add(`頂点${k}を経由する到達`,'元辺ではなく、到達可能なすべてのペアを二部グラフの辺にします。',['from','到達先'],reach.map((row,u)=>[u,row.flatMap((x,v)=>x?[v]:[]).join(',')]));}
   if(reach.some((row,v)=>row[v]))throw Error('DAGを指定してください');const es=[];for(let u=0;u<n;u++)for(let v=0;v<n;v++)if(reach[u][v])es.push([u,v]);const m=matching(n,n,es,false);active=m.zl.flatMap((yes,v)=>yes&&!m.zr[v]?[v]:[]);
   result.push(String(n-m.size),String(n-m.size));add('最大反鎖と最小鎖数','左が交互到達、右が非到達となる頂点を反鎖に選びます。',['頂点','左到達','右到達'],m.zl.map((x,v)=>[v,x,m.zr[v]]));
  }else if(id==='vizing_edge_coloring'){
   // Fan algorithm adapted from Simon Lindholm's KACTL EdgeColoring.h (CC0).
   const seen=new Set(),degree=Array(n).fill(0);for(const [u,v]of edges){const key=[Math.min(u,v),Math.max(u,v)].join(',');if(u===v||seen.has(key))throw Error('単純グラフを指定してください');seen.add(key);degree[u]++;degree[v]++;}
   const delta=Math.max(0,...degree),C=delta+1,a=Array.from({length:n},()=>Array(C).fill(-1)),free=Array(n).fill(0),fan=[],fc=[];
   edges.forEach(([u,v],edgeId)=>{
    fan[0]=v;const loc=Array(C).fill(0);let at=u,end=u,d,c=free[u],length=0,i=0;
    while(true){d=free[v];if(loc[d]||a[u][d]<0)break;v=a[u][d];loc[d]=++length;fc[length]=d;fan[length]=v;}
    active=[u,...fan.slice(0,length+1)];add(`辺${edgeId}: fanを作る`,'中心に接する辺の色を、直前のfan頂点で空いている色に沿って伸ばします。',['fan位置','頂点'],fan.slice(0,length+1).map((v,i)=>[i,v]),{中心:u,交互色:`${c},${d}`});
    fc[loc[d]]=c;for(let cd=d;at>=0;cd^=c^d,at=a[at][cd]){end=at;[a[at][cd],a[at][cd^c^d]]=[a[at][cd^c^d],a[at][cd]];}
    while(a[fan[i]][d]>=0){const left=fan[i],right=fan[i+1],color=fc[++i];a[u][color]=left;a[left][color]=u;a[right][color]=-1;free[right]=color;}
    a[u][d]=fan[i];a[fan[i]][d]=u;for(const y of [fan[0],u,end]){free[y]=0;while(a[y][free[y]]>=0)free[y]++;}
    colors=edges.map(([x,y],j)=>j<=edgeId?a[x].indexOf(y):-1);add('交互路を反転し、fanを回転','2色の連鎖を反転してからfanの色をずらし、新しい辺に空き色を割り当てます。',['辺ID','色'],colors.map((x,i)=>[i,x]));
   });result.push(String(delta),String(Math.max(-1,...colors)+1));
  }else if(id==='difference_constraints'){
   const first=bellman();if(first.cycle)result.push('null');else result.push(...first.d.map(String));if(input.extra!==undefined){const extra=readEdges([input.extra],true)[0];edges.push(extra);add('制約を1つ追加','solveは状態を変更しないため、制約追加後に再計算できます。');result.push(String(Number(Boolean(bellman().cycle))));}
  }else if(id==='negative_cycle'){
   const r=bellman();result.push(String(Number(Boolean(r.cycle))),String(r.weight??0));const s=integer(input.source,0,n-1),from=bellman(s);result.push(String(Number(Boolean(from.cycle))));
  }else if(id==='min_cost_disjoint_paths'){
   const s=integer(input.s,0,n-1),t=integer(input.t,0,n-1),k=integer(input.k,0,3);if(s===t||edges.some(e=>e[2]<0))throw Error('端点は異なり、費用は非負です');if(edges.length>12)throw Error('素パス図解は12辺以下です');
   const a=adj(),paths=[];function dfs(v,used,ids,cost){if(v===t){paths.push({ids,cost,inside:used&~(1<<s)&~(1<<t)});return;}for(const e of a[v])if(!(used>>e.v&1))dfs(e.v,used|1<<e.v,[...ids,e.id],cost+e.w);}dfs(s,1<<s,[],0);
   add('単純s-tパスを列挙','図解は全探索で最適値の意味を示します。C++は最小費用流で経路を組み替えます。',['候補','元辺ID','費用'],paths.map((p,i)=>[i,p.ids.join(','),p.cost]));
   const answers=[];for(const vertex of [false,true]){let best=Infinity,bestPaths=[];function choose(pos,left,used,inside,cost,chosen){if(!left){if(cost<best){best=cost;bestPaths=chosen.slice();}return;}for(let i=pos;i<paths.length;i++){const p=paths[i],mask=p.ids.reduce((s,id)=>s|1<<id,0);if(!(used&mask)&&(!vertex||!(inside&p.inside)))choose(i+1,left-1,used|mask,inside|p.inside,cost+p.cost,[...chosen,i]);}}choose(0,k,0,0,0,[]);answers.push(best);selected=bestPaths.flatMap(i=>paths[i].ids);add(vertex?'内部頂点も共有しない最小費用':'辺を共有しない最小費用','終端s,tは共有可能。辺IDを見て平行辺も区別します。',['選択パス','元辺ID'],bestPaths.map(i=>[i,paths[i].ids.join(',')]),{本数:k,費用:Number.isFinite(best)?best:'不可能'});}
   result.push(Number.isFinite(answers[0])?String(k):'null',...answers.map(x=>Number.isFinite(x)?String(x):'null'));
  }
 }
 if(steps.length===1)add('結果','入力の条件と結果を確認してください。');return {steps,result};
}
const api={modes,trace};if(typeof module!=='undefined')module.exports=api;else root.GraphExtensionsModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
