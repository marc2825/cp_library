(function(root){
'use strict';
const modes=[
 [16,'cost_scaling_b_flow','Cost Scaling b-Flow','graph_flow',{n:2,balance:[2,-2],edges:[[0,1,0,3,-4]]}],
 [20,'few_weight_shortest_path','少種類重みの最短路','graph_basic',{n:3,source:0,directed:true,edges:[[0,1,2],[1,2,2],[0,2,100]]}],
 [21,'shortest_nonzero_path','Shortest Nonzero Path','graph_basic',{n:3,source:0,target:2,modulus:2,edges:[[0,1,1,1],[1,2,1,1],[0,2,5,1]]}],
 [22,'minimum_diameter_spanning_tree','最小直径全域木','graph_basic',{n:3,edges:[[0,1,1],[1,2,1],[0,2,1]]}],
 [23,'spqr_tree','SPQR Tree','graph_connectivity',{n:3,edges:[[0,1],[1,2],[2,0],[0,1]]}],
 [24,'tree_decomposition','一般グラフの木幅','graph_connectivity',{n:4,edges:[[0,1],[1,2],[2,3],[3,0]]}],
 [25,'cograph_canonical_form','Cograph標準形','graph_connectivity',{n:3,edges:[[0,1],[1,2]]}],
 [26,'general_weighted_matching','一般最大重みmatching','graph_flow',{n:3,edges:[[0,1,5],[1,2,7],[2,0,4]]}],
 [27,'rank_maximal_matching','Rank-Maximal Matching','graph_flow',{left:2,right:2,edges:[[0,0,1],[0,1,2],[1,0,2]]}],
 [28,'dynamic_unit_flow','動的単位容量フロー','graph_flow',{n:3,source:0,sink:2,operations:[['add',0,1],['add',1,2],['query'],['set',0,false],['query'],['add',0,2],['query']]}],
 [29,'connected_graph_count','連結グラフ数','graph_combinatorics',{n:4}],
 [30,'strongly_connected_graph_count','強連結有向グラフ数','graph_combinatorics',{n:4}],
 [31,'euler_circuit_count','Euler閉路数 BEST','graph_combinatorics',{n:2,source:0,first:0,edges:[[0,1],[1,0],[0,1],[1,0]]}],
 [32,'graph_isomorphism','一般グラフ同型','graph_combinatorics',{n:3,edges:[[0,1],[1,2]],other:[[2,0],[0,1]]}],
 [33,'series_parallel_decomposition','二端子直並列分解','graph_combinatorics',{n:4,source:0,sink:3,edges:[[0,1],[1,3],[0,2],[2,3]]}]
].map(([number,id,title,category,sample])=>({id,title:`G-${number} · ${title}`,doc:`${category}/${id}.md`,sample}));
const clone=x=>JSON.parse(JSON.stringify(x));
function integer(x,lo=0,hi=8){if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`整数 ${lo}〜${hi} を指定してください`);return x;}
function array(a,max=16){if(!Array.isArray(a)||a.length>max)throw Error(`配列は${max}要素以下です`);return a;}
function trace(id,input){
 if(!modes.some(m=>m.id===id))throw Error('未知のモード');
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください');
 let n=integer(input.n??0,0,12),edges=[],directed=false,selected=[],active=[],values=[],result=[];const steps=[];
 const add=(title,detail,columns=[],rows=[],info={})=>{if(steps.length>=400)return;steps.push({title,detail,columns,rows:clone(rows),values:Object.entries(info).map(([label,value])=>({label,value:String(value)})),state:{n,edges:clone(edges),directed,selected:selected.slice(),active:active.slice(),values:values.slice()}});};
 const read=(raw,max=16)=>array(raw,max).map(e=>{array(e,5);if(e.length<2)throw Error('端点が不足しています');integer(e[0],0,n-1);integer(e[1],0,n-1);e.slice(2).forEach(x=>integer(x,-100,100));return e.slice();});
 const adjacency=(es=edges)=>{const a=Array.from({length:n},()=>[]);es.forEach(([u,v,w=1],i)=>{a[u].push([v,w,i]);if(!directed)a[v].push([u,w,i]);});return a;};
 const matrix=es=>{const a=Array.from({length:n},()=>Array(n).fill(0));for(const [u,v] of es){a[u][v]++;if(!directed&&u!==v)a[v][u]++;}return a;};
 const components=(vertices,es,complement=false)=>{const a=matrix(es),seen=new Set(),out=[];for(const s of vertices)if(!seen.has(s)){const q=[s];seen.add(s);for(let i=0;i<q.length;i++)for(const u of vertices)if(!seen.has(u)&&Boolean(a[q[i]][u])!==complement){seen.add(u);q.push(u);}out.push(q);}return out;};
 const permutations=(a,visit)=>{function go(i){if(i===a.length){visit(a);return;}for(let j=i;j<a.length;j++){[a[i],a[j]]=[a[j],a[i]];go(i+1);[a[i],a[j]]=[a[j],a[i]];}}go(0);};
 const vertices=()=>Array.from({length:n},(_,i)=>i);
 if(id==='connected_graph_count'||id==='strongly_connected_graph_count'){
  n=integer(input.n,0,12);const N=n;n=0;const c=Array.from({length:N+1},()=>[]),a=Array(N+1).fill(1n),ans=Array(N+1).fill(0n),eta=ans.slice();
  add('頂点0の係数','空グラフの連結/強連結の数は0と定義します。',['n','個数'],[[0,'0']]);
  for(let i=0;i<=N;i++){c[i]=Array(i+1).fill(1n);for(let j=1;j<i;j++)c[i][j]=c[i-1][j-1]+c[i-1][j];}
  for(let i=1;i<=N;i++){
   if(id==='connected_graph_count'){a[i]=2n**BigInt(i*(i-1)/2);ans[i]=a[i];for(let k=1;k<i;k++)ans[i]-=c[i-1][k-1]*ans[k]*a[i-k];}
   else{eta[i]=2n**BigInt(i*(i-1));for(let k=1;k<i;k++)eta[i]-=c[i][k]*2n**BigInt((i-1)*(i-k))*eta[k];ans[i]=eta[i];for(let k=1;k<i;k++)ans[i]+=c[i-1][k-1]*ans[k]*eta[i-k];}
   add(`${i}頂点の係数を確定`,id==='connected_graph_count'?'頂点0を含む真の成分の寄与を全グラフ数から引きます。':'Wrightの補助係数etaから強連結係数を復元します。',['n','個数','eta'],Array.from({length:i+1},(_,j)=>[j,String(ans[j]),String(eta[j])]),{個数:ans[i]});
  }result=ans.map(String);
 }else if(id==='rank_maximal_matching'){
  const L=integer(input.left,0,5),R=integer(input.right,0,5);n=L+R;const es=array(input.edges,14).map(e=>{array(e,3);if(e.length!==3)throw Error('左・右・順位が必要です');return [integer(e[0],0,L-1),integer(e[1],0,R-1),integer(e[2],1,100)];});edges=es.map(([u,v,r])=>[u,v+L,r]);
  const ranks=[...new Set(es.map(e=>e[2]))].sort((a,b)=>a-b);let best=Array(ranks.length).fill(0),choice=[];const profile=best.slice();
  add('順位を並べる','小さい順位の本数から辞書式に比較します。',['rank'],ranks.map(x=>[x]));
  function dfs(i,used,chosen){if(i===L){let greater=false;for(let j=0;j<profile.length;j++)if(profile[j]!==best[j]){greater=profile[j]>best[j];break;}if(greater){best=profile.slice();choice=chosen.slice();selected=choice;add('profileを更新','この図解は全matchingの比較です。C++は多倍長整数Hungarianを使います。',['rank','本数'],ranks.map((r,j)=>[r,best[j]]));}return;}dfs(i+1,used,chosen);es.forEach(([u,v,r],e)=>{if(u===i&&!(used>>v&1)){profile[ranks.indexOf(r)]++;dfs(i+1,used|1<<v,[...chosen,e]);profile[ranks.indexOf(r)]--;}});}dfs(0,0,[]);selected=choice;result=best.map(String);
 }else{
  integer(n,1,8);values=Array(n).fill('');directed=['cost_scaling_b_flow','dynamic_unit_flow','euler_circuit_count','series_parallel_decomposition'].includes(id)||input.directed===true;
  if(id!=='dynamic_unit_flow')edges=read(input.edges);
  add('入力グラフ','辺IDは入力順、頂点は0始まりです。',['辺ID','入力'],edges.map((e,i)=>[i,e.join(', ')]));
  if(id==='cost_scaling_b_flow'){
   const balance=array(input.balance,8);if(balance.length!==n)throw Error('balanceの長さが違います');balance.forEach(x=>integer(x,-20,20));let combinations=1;
   for(const e of edges){if(e.length!==5)throw Error('[from,to,lower,upper,cost]が必要です');integer(e[2],-3,4);integer(e[3],e[2],4);combinations*=e[3]-e[2]+1;}if(combinations>100000)throw Error('容量の組合せは100000通り以下です');
   let best=Infinity,chosen=[];const net=Array(n).fill(0),flow=[];
   function dfs(i,cost){if(i===edges.length){if(net.every((x,v)=>x===balance[v])&&cost<best){best=cost;chosen=flow.slice();values=net.slice();add('実行可能流の費用を比較','図解は容量内の全流量を列挙。C++のepsilon scalingの操作列ではありません。',['辺ID','flow'],flow.map((f,j)=>[j,f]),{費用:cost});}return;}const [u,v,lo,hi,c]=edges[i];for(let f=lo;f<=hi;f++){net[u]+=f;net[v]-=f;flow[i]=f;dfs(i+1,cost+f*c);net[u]-=f;net[v]+=f;}}dfs(0,0);result=Number.isFinite(best)?[String(best),...chosen.map(String)]:['null'];
  }else if(id==='few_weight_shortest_path'){
   if(edges.some(e=>e.length!==3||e[2]<0))throw Error('非負重み付き辺が必要です');const s=integer(input.source,0,n-1),a=adjacency(),d=Array(n).fill(Infinity),done=Array(n).fill(false),weights=[...new Set(edges.map(e=>e[2]))].sort((a,b)=>a-b),queues=weights.map(()=>[]);d[s]=0;let v=s;
   while(v>=0){done[v]=true;active=[v];for(const [u,w]of a[v])if(d[v]+w<d[u]){d[u]=d[v]+w;queues[weights.indexOf(w)].push([d[u],u]);}values=d.map(x=>Number.isFinite(x)?x:'∞');add(`${v}を確定`,'同じ重みの候補は投入順に距離が増えます。',['重み','FIFO候補 (距離,頂点)'],weights.map((w,i)=>[w,JSON.stringify(queues[i])]));v=-1;let best=Infinity;for(const q of queues){while(q.length&&(done[q[0][1]]||q[0][0]!==d[q[0][1]]))q.shift();if(q.length&&q[0][0]<best){best=q[0][0];v=q[0][1];}}}result=d.map(x=>Number.isFinite(x)?String(x):'inf');
  }else if(id==='shortest_nonzero_path'){
   const s=integer(input.source,0,n-1),t=integer(input.target,0,n-1),mod=integer(input.modulus,2,8);for(const e of edges){if(e.length!==4)throw Error('重みと加法群ラベルが必要です');integer(e[2],0,100);integer(e[3],0,mod-1);}let best=Infinity,path=[];const a=adjacency();
   function dfs(v,seen,d,label,p){if(v===t){if(label!==0&&d<best){best=d;path=p.slice();selected=path;add('非零の単純パスを更新','図解は有限加法群の単純パス全探索。C++は任意の群とblossom処理に対応します。',['辺ID'],path.map(i=>[i]),{距離:d,ラベル:label});}return;}for(const [u,w,e]of a[v])if(!(seen>>u&1)){const x=edges[e][0]===v?edges[e][3]:-edges[e][3];dfs(u,seen|1<<u,d+w,(label+x+mod)%mod,[...p,e]);}}dfs(s,1<<s,0,0,[]);selected=path;result=Number.isFinite(best)?[String(best),String(path.length)]:['null'];
  }else if(id==='minimum_diameter_spanning_tree'){
   for(const e of edges){if(e.length!==3)throw Error('辺重みが必要です');integer(e[2],0,100);}let best=Infinity,tree=[];
   for(let mask=0;mask<1<<edges.length;mask++){const chosen=edges.map((_,i)=>i).filter(i=>mask>>i&1);if(chosen.length!==n-1)continue;const d=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?0:Infinity));for(const i of chosen){const [u,v,w]=edges[i];d[u][v]=d[v][u]=Math.min(d[u][v],w);}for(let k=0;k<n;k++)for(let i=0;i<n;i++)for(let j=0;j<n;j++)d[i][j]=Math.min(d[i][j],d[i][k]+d[k][j]);const diameter=Math.max(...d.flat());if(diameter<best){best=diameter;tree=chosen;selected=chosen;add('より小さい直径の全域木','図解は辺集合を列挙。C++は辺の内部も含む連続中心から復元します。',['頂点',...vertices()],d.map((row,v)=>[v,...row]),{直径:diameter});}}
   selected=tree;result=Number.isFinite(best)?[String(best),String(tree.length)]:['null'];
  }else if(id==='tree_decomposition'){
   let best=Infinity,order=[];const original=matrix(edges);
   function eliminate(p,show=false){const a=original.map(row=>row.map(Boolean)),alive=Array(n).fill(true);let width=-1;for(const v of p){const neighbors=vertices().filter(u=>u!==v&&alive[u]&&a[v][u]);width=Math.max(width,neighbors.length);if(show){active=[v,...neighbors];values=alive.map(x=>x?'残存':'消去済');add(`${v}を消去`,'後の隣接頂点をcliqueに補い、{v}+隣接集合をbagにします。',['bag'],[[[v,...neighbors].join(', ')]],{幅:width});}for(const u of neighbors)for(const w of neighbors)if(u!==w)a[u][w]=true;alive[v]=false;}return width;}
   permutations(vertices(),p=>{const w=eliminate(p);if(w<best){best=w;order=p.slice();}});eliminate(order,true);result=[String(best),String(n)];
  }else if(id==='cograph_canonical_form'){
   const a=matrix(edges),colors=input.colors??Array(n).fill(0);array(colors,8);if(colors.length!==n)throw Error('色数が違います');colors.forEach(x=>integer(x,-100,100));if(edges.some(([u,v])=>u===v)||a.some(row=>row.some(x=>x>1)))throw Error('単純無向グラフが必要です');
   function solve(vs){if(vs.length===1)return `L${colors[vs[0]]};`;for(const complement of [false,true]){const parts=components(vs,edges,complement);if(parts.length>1){const codes=parts.map(solve);if(codes.includes(null))return null;codes.sort();active=vs;const code=`${complement?'J':'U'}(${codes.join('')})`;add(complement?'補グラフの成分をjoin':'連結成分のunion','子の構造文字列を整列して、頂点名によらない標準形を作ります。',['code'],[[code]]);return code;}}return null;}const code=solve(vertices());result=[code??'null'];
  }else if(id==='general_weighted_matching'){
   if(edges.some(e=>e.length!==3))throw Error('辺重みが必要です');let best=0,chosen=[];
   function dfs(i,mask,cost,ids){if(i===edges.length){if(cost>best){best=cost;chosen=ids.slice();selected=chosen;add('重み和を更新','図解は辺集合の全探索。C++は重み付きBlossomです。',['辺ID'],chosen.map(i=>[i]),{重み和:best});}return;}dfs(i+1,mask,cost,ids);const [u,v,w]=edges[i];if(u!==v&&!(mask>>u&1)&&!(mask>>v&1))dfs(i+1,mask|1<<u|1<<v,cost+w,[...ids,i]);}dfs(0,0,0,[]);selected=chosen;result=[String(best),String(chosen.length)];
  }else if(id==='dynamic_unit_flow'){
   const s=integer(input.source,0,n-1),t=integer(input.sink,0,n-1);if(s===t)throw Error('始点と終点は異なる必要があります');const enabled=[];
   const cut=()=>{let best=Infinity,side=0;for(let mask=0;mask<1<<n;mask++)if((mask>>s&1)&&!(mask>>t&1)){let total=0;edges.forEach(([u,v],i)=>{if(enabled[i]&&(mask>>u&1)&&!(mask>>v&1))total++;});if(total<best){best=total;side=mask;}}active=vertices().filter(v=>side>>v&1);selected=edges.map((_,i)=>i).filter(i=>enabled[i]);return best;};
   for(const op of array(input.operations,30)){array(op,4);if(op[0]==='add'){integer(op[1],0,n-1);integer(op[2],0,n-1);edges.push([op[1],op[2],1]);enabled.push(true);}else if(op[0]==='set'){integer(op[1],0,edges.length-1);if(typeof op[2]!=='boolean')throw Error('有効状態はboolです');enabled[op[1]]=op[2];}else if(op[0]!=='query')throw Error('操作はadd/set/queryです');const value=cut();if(op[0]==='query')result.push(String(value));add(op.join(' '),'図解は更新後の全カットを比較。C++は残余フローを修復します。',['辺ID','有効'],enabled.map((x,i)=>[i,x]),{最大流:value});}
  }else if(id==='euler_circuit_count'){
   if(edges.length>10)throw Error('Euler図解は10辺以下です');const s=integer(input.source,0,n-1),first=integer(input.first,-1,edges.length-1);if(first>=0&&edges[first][0]!==s)throw Error('先頭辺の始点が違います');let count=0,fixed=0;const a=adjacency();
   function dfs(v,mask,path){if(mask===(1<<edges.length)-1){if(v===s){count++;if(path[0]===first)fixed++;if(count<=10){selected=path;add('Euler辺列を1つ数える','辺IDを区別し、同じ始点でも順序が違えば別の列です。',['順番','辺ID'],path.map((e,i)=>[i,e]),{本数:count});}}return;}for(const [u,,e]of a[v])if(!(mask>>e&1))dfs(u,mask|1<<e,[...path,e]);}dfs(s,0,[]);result=[String(count),String(first<0?count:fixed)];
  }else if(id==='graph_isomorphism'){
   const other=read(input.other),a=matrix(edges),b=matrix(other),ca=input.colors??Array(n).fill(0),cb=input.other_colors??Array(n).fill(0);array(ca,8);array(cb,8);if(ca.length!==n||cb.length!==n)throw Error('色数が違います');ca.concat(cb).forEach(x=>integer(x,-100,100));let mapping=null;
   permutations(vertices(),p=>{if(mapping)return;if(vertices().every(u=>ca[u]===cb[p[u]]&&vertices().every(v=>a[u][v]===b[p[u]][p[v]])))mapping=p.slice();});if(mapping){values=mapping.map(x=>`→ ${x}`);add('対応を復元','全隣接多重度と頂点色を確認します。図解は頂点置換全探索です。',['元','相手'],mapping.map((u,v)=>[v,u]));}result=mapping?['1',String(n)]:['0'];
  }else if(id==='series_parallel_decomposition'){
   const s=integer(input.source,0,n-1),t=integer(input.sink,0,n-1);if(s===t)throw Error('端子を分けてください');let work=edges.map(([u,v],id)=>({u,v,id})),nodes=edges.length,ok=!edges.some(([u,v])=>u===v),used=new Set(edges.flatMap(e=>e.slice(0,2)));const removed=new Set();
   while(ok){let changed=false;outer:for(let i=0;i<work.length;i++)for(let j=i+1;j<work.length;j++)if(work[i].u===work[j].u&&work[i].v===work[j].v){const a=work[i],b=work[j];work.splice(j,1);work.splice(i,1);work.push({u:a.u,v:a.v,id:nodes++});active=[a.u,a.v];add('parallel合成','同じ両端を持つ枝を1つの式ノードにまとめます。',['子','親'],[[`${a.id}, ${b.id}`,nodes-1]]);changed=true;break outer;}if(changed)continue;
    for(let v=0;v<n;v++)if(v!==s&&v!==t&&!removed.has(v)){const ins=work.filter(e=>e.v===v),outs=work.filter(e=>e.u===v);if(ins.length===1&&outs.length===1){const a=ins[0],b=outs[0];if(a.u===b.v){ok=false;break;}work=work.filter(e=>e!==a&&e!==b);work.push({u:a.u,v:b.v,id:nodes++});removed.add(v);active=[a.u,v,b.v];add('series合成','内部頂点の入次数と出次数がともに1なら直列にまとめます。',['子','親'],[[`${a.id}, ${b.id}`,nodes-1]]);changed=true;break;}}
    if(!changed)break;
   }ok=ok&&used.size===n&&removed.size===n-2&&work.length===1&&work[0].u===s&&work[0].v===t;result=[ok?String(nodes):'null'];
  }else if(id==='spqr_tree'){
   if(edges.some(([u,v])=>u===v))throw Error('ループは扱いません');let ok=n>=2&&components(vertices(),edges).length===1;for(let v=0;v<n;v++)ok=ok&&components(vertices().filter(x=>x!==v),edges.filter(([a,b])=>a!==v&&b!==v)).length===1;
   if(!ok)result=['null'];else{
    const work=[edges.map(([u,v],id)=>({u,v,id,virtual:-1}))],nodes=[];let next=0;
    while(work.length){const es=work.pop(),vs=[...new Set(es.flatMap(e=>[e.u,e.v]))],degree=Array(n).fill(0);for(const e of es){degree[e.u]++;degree[e.v]++;}if(es.length===1){nodes.push({kind:'Q',es});continue;}if(vs.length===2){nodes.push({kind:'P',es});continue;}if(vs.every(v=>degree[v]===2)){nodes.push({kind:'S',es});continue;}let split=false;
     outer:for(const u of vs)for(const v of vs)if(u<v){const rest=vs.filter(x=>x!==u&&x!==v),groups=components(rest,es.filter(e=>![u,v].includes(e.u)&&![u,v].includes(e.v)).map(e=>[e.u,e.v])),parts=groups.map(()=>[]);for(const e of es){if([u,v].includes(e.u)&&[u,v].includes(e.v))parts.push([e]);else parts[groups.findIndex(g=>g.includes([u,v].includes(e.u)?e.v:e.u))].push(e);}if(parts.length<2||(parts.length===2&&parts.some(p=>p.length<2)))continue;split=true;active=[u,v];add('分離対でsplit','直接辺も1つのsplit componentです。3つ以上なら中央Pを作ります。',['端点','成分数'],[[`${u}, ${v}`,parts.length]]);if(parts.length===2){const e={u,v,id:-1,virtual:next++};for(const p of parts)work.push([...p,e]);}else{const parallel=[];for(const p of parts){if(p.length===1)parallel.push(p[0]);else{const e={u,v,id:-1,virtual:next++};parallel.push(e);work.push([...p,e]);}}nodes.push({kind:'P',es:parallel});}break outer;}
     if(!split)nodes.push({kind:'R',es});
    }
    const parent=nodes.map((_,i)=>i),find=v=>parent[v]===v?v:parent[v]=find(parent[v]),pairs=Array.from({length:next},()=>[]);nodes.forEach((node,i)=>node.es.forEach(e=>{if(e.virtual>=0)pairs[e.virtual].push(i);}));for(const [a,b]of pairs)if(nodes[a].kind===nodes[b].kind&&['S','P'].includes(nodes[a].kind))parent[find(a)]=find(b);const count=new Set(parent.map((_,i)=>find(i))).size;add('S-S / P-Pを併合','仮想辺で対応する隣接スケルトンを正規化します。',['node','kind','group'],nodes.map((x,i)=>[i,x.kind,find(i)]));result=[String(count),String(count-1)];
   }
  }
 }
 add('結果','入力を変更して候補や復元結果の変化を比較できます。',['出力'],result.map(x=>[x]));
 return {steps,result};
}
const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.GraphAdvancedModel=api;
})(typeof window==='undefined'?globalThis:window);
