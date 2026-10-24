(function(root){
  'use strict';
  const baseEdges=[[0,1],[1,2],[1,3]];
  const modes=[
    {id:'tree_center',title:'T-01 · 木の中心',sample:{n:3,edges:[[0,1,2],[1,2,5]]}},
    {id:'prufer_code',title:'T-02 · Prüfer Code',sample:{n:5,code:[3,3,1]}},
    {id:'rerooted_subtree',title:'T-03 · 根変更時の部分木',sample:{values:[1,2,3,4,5],edges:[[0,1],[0,2],[1,3],[1,4]],fixedRoot:0,root:3,vertex:1}},
    {id:'dynamic_top_tree',title:'T-20 · Dynamic Top Tree',sample:{values:[1,2,3,4],edges:baseEdges,ops:[['distance',0],['cut',1,3],['link',2,3],['distance',0],['subtree_mass',0,2]]}},
    {id:'lazy_top_tree',title:'T-21 · Lazy Top Tree',sample:{values:[1,2,3,4],edges:baseEdges,ops:[['path_apply',0,2,2,1],['subtree_apply',0,1,0,5],['path_sum',0,2],['cut',1,3],['link',0,3],['component_sum',0]]}},
    {id:'dynamic_subtree_aggregate',title:'T-22 · 動的部分木集約',sample:{values:[1,2,3,4],edges:baseEdges,ops:[['subtree_apply',0,1,1,10],['subtree_mass',0,1],['cut',1,3],['link',0,3],['subtree_mass',3,0],['path_sum',2,3]]}},
    {id:'fully_dynamic_tree_dp',title:'T-23 · Fully Dynamic Tree DP',sample:{values:[1,1,1,1],edges:baseEdges,ops:[['independent',0],['cut',1,3],['link',2,3],['independent',3],['set',1,0],['independent',0]]}}
  ].map(x=>({...x,doc:`tree/${x.id}.md`}));
  const integer=(x,lo=-1000000,hi=1000000)=>{if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`整数 ${lo}..${hi} を指定してください`);return x;};
  const list=(x,limit=100)=>{if(!Array.isArray(x)||x.length>limit)throw Error(`配列は${limit}要素以下です`);return x;};
  function graph(n,edges){
    const adj=Array.from({length:n},()=>[]);
    edges.forEach(([u,v,w=1],id)=>{adj[u].push({to:v,w,id});adj[v].push({to:u,w,id});});return adj;
  }
  function rooted(n,edges,r){
    const adj=graph(n,edges),parent=Array(n).fill(-2),depth=Array(n).fill(0),distance=Array(n).fill(0),order=[r];parent[r]=-1;
    for(const u of order)for(const {to:v,w}of adj[u])if(parent[v]===-2){parent[v]=u;depth[v]=depth[u]+1;distance[v]=distance[u]+w;order.push(v);}
    return {adj,parent,depth,distance,order};
  }
  function path(n,edges,u,v){const {parent}=rooted(n,edges,u);if(parent[v]===-2)return [];const a=[];for(let x=v;x!==-1;x=parent[x])a.push(x);return a.reverse();}
  function subtree(n,edges,r,v){const {parent,order}=rooted(n,edges,r);return order.filter(u=>{while(u!==-1&&u!==v)u=parent[u];return u===v;});}
  function validateEdges(n,input,weighted=false){const edges=[];for(const e of list(input,20)){list(e,weighted?3:2);if(e.length!==(weighted?3:2))throw Error('辺の要素数が違います');const u=integer(e[0],0,n-1),v=integer(e[1],0,n-1);if(path(n,edges,u,v).length)throw Error('閉路・自己ループ・多重辺は指定できません');edges.push(weighted?[u,v,integer(e[2],0,1000)]:[u,v]);}return edges;}
  function trace(id,input){
    if(!modes.some(x=>x.id===id))throw Error('不明なモード');
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSON objectを指定してください');
    const steps=[],result=[];
    const add=(title,detail,columns=[],rows=[],metrics={},state=null)=>steps.push({title,detail,columns,rows,values:Object.entries(metrics).map(([label,value])=>({label,value:String(value)})),state});
    add('入力を確認','頂点番号は0始まりです。入力を編集して実行できます。');
    if(id==='prufer_code'){
      const n=integer(input.n,1,16),code=list(input.code,14).map(v=>integer(v,0,n-1));
      if(code.length!==Math.max(0,n-2))throw Error('符号の長さはmax(0,n-2)です');
      const degree=Array(n).fill(1),edges=[];for(const v of code)++degree[v];
      for(const v of code){const u=degree.indexOf(1);edges.push([u,v]);--degree[u];--degree[v];add(`最小の葉 ${u} を ${v} に接続`,'符号の出現回数+1が初期次数。使用済みの葉の次数を0にします。',['頂点','残り次数'],degree.map((d,i)=>[i,d]),{出力辺:`${u}–${v}`},{n,edges:edges.map(e=>e.slice()),active:[u,v],values:Array(n).fill(1)});}
      if(n>=2){const leaves=degree.flatMap((d,i)=>d===1?[i]:[]);edges.push(leaves);}
      // 独立したencode段階を表示。
      const remaining=new Set(Array.from({length:n},(_,i)=>i)),adj=graph(n,edges),encoded=[];
      while(remaining.size>2){const u=[...remaining].find(x=>adj[x].filter(e=>remaining.has(e.to)).length===1),v=adj[u].find(e=>remaining.has(e.to)).to;encoded.push(v);remaining.delete(u);add(`符号化: 葉 ${u} を取り除く`,'残った木の最小の葉の隣接頂点を記録します。',['符号位置','値'],encoded.map((x,i)=>[i,x]),{残り頂点:[...remaining].join(',')},{n,edges:edges.map(e=>e.slice()),active:[u],values:Array(n).fill(1)});}
      result.push(...encoded.map(String),'1','1');
      add('往復変換の完了','末尾の1,1はn=1の頂点数とn=2の辺数です。復号時はnを明示して区別します。',['辺','両端'],edges.map((e,i)=>[i,e.join('–')]),{頂点数:n,辺数:edges.length});
    }else if(id==='tree_center'){
      const n=integer(input.n,1,20),edges=validateEdges(n,input.edges,true);
      if(edges.length!==n-1)throw Error('中心は連結な木を指定してください');
      const first=rooted(n,edges,0),s=first.distance.indexOf(Math.max(...first.distance)),a=rooted(n,edges,s),t=a.distance.indexOf(Math.max(...a.distance)),b=rooted(n,edges,t),diameter=a.distance[t];
      add('直径の両端を選ぶ','0からの最遠点s、そのsからの最遠点tを求めます。',['頂点','d(s,v)','d(t,v)','離心度'],a.distance.map((d,v)=>[v,d,b.distance[v],Math.max(d,b.distance[v])]),{s,t,直径:diameter},{n,edges,active:path(n,edges,s,t),values:a.distance});
      const ecc=a.distance.map((d,v)=>Math.max(d,b.distance[v])),radius=Math.min(...ecc),vertices=ecc.flatMap((d,v)=>d===radius?[v]:[]);
      const unit=edges.map(([u,v])=>[u,v]),ue=Array.from({length:n},(_,v)=>Math.max(...rooted(n,unit,v).distance)),ur=Math.min(...ue),uv=ue.flatMap((d,v)=>d===ur?[v]:[]);
      let vertex=-1,edgeId=-1,offset=0;const spine=path(n,edges,s,t);
      for(let i=0;i<spine.length;++i){const v=spine[i];if(2*a.distance[v]===diameter){vertex=v;break;}if(2*a.distance[v]>diameter){const from=spine[i-1];edgeId=edges.findIndex(([x,y])=>(x===from&&y===v)||(y===from&&x===v));offset=diameter-2*a.distance[from];break;}}
      add('頂点中心と連続中心','頂点に限定した中心と、辺の内部も許す直径中点を別に返します。',['種類','頂点/辺','半径'],[['重みなし',uv.join(','),ur],['重み付き頂点',vertices.join(','),radius],['連続中心',vertex>=0?`頂点${vertex}`:`辺${edgeId} / offset×2=${offset}`,`${diameter}/2`]],{}, {n,edges,active:vertices,values:ecc});
      result.push(...uv.map(String),...vertices.map(String),String(radius),String(edgeId),String(offset),String(diameter));
    }else{
      const values=list(input.values,16).map(v=>integer(v)),n=values.length;if(!n)throw Error('図解では1頂点以上を指定してください');
      let edges=validateEdges(n,input.edges);
      const vertex=v=>integer(v,0,n-1),sum=a=>a.reduce((s,v)=>s+values[v],0);
      const snapshot=(active=[],spine=[])=>({n,values:values.slice(),edges:edges.map(e=>e.slice()),active:active.slice(),spine:spine.slice()});
      if(id==='rerooted_subtree'){
        const fixed=vertex(input.fixedRoot),r=vertex(input.root),v=vertex(input.vertex);if(!path(n,edges,r,v).length)throw Error('rootとvertexは同じ成分にしてください');
        const parent=Array(n).fill(-2),size=Array(n).fill(1),heavy=Array(n).fill(-1),order=[],pos=Array(n),head=Array(n),adj=graph(n,edges),roots=[];
        function dfs(u,p){parent[u]=p;order.push(u);for(const e of adj[u])if(e.to!==p){dfs(e.to,u);size[u]+=size[e.to];if(heavy[u]<0||size[e.to]>size[heavy[u]])heavy[u]=e.to;}}
        for(const u of [fixed,...Array.from({length:n},(_,i)=>i)])if(parent[u]===-2){roots.push(u);dfs(u,-1);}
        const flat=[];function hld(u,h){head[u]=h;pos[u]=flat.length;flat.push(u);if(heavy[u]>=0)hld(heavy[u],h);for(const e of adj[u])if(parent[e.to]===u&&e.to!==heavy[u])hld(e.to,e.to);}
        for(const u of roots)hld(u,u);
        add('固定根のHLD順','heavyな子を先に並べると、元の部分木は1区間です。',['頂点','pos','size','head'],order.map(u=>[u,pos[u],size[u],head[u]]),{},snapshot());
        const target=subtree(n,edges,r,v),indices=target.map(u=>pos[u]).sort((a,b)=>a-b),intervals=[];
        for(const i of indices){if(intervals.length&&intervals.at(-1)[1]===i)++intervals.at(-1)[1];else intervals.push([i,i+1]);}
        add('仮の根の部分木を取り出す','v==rootなら成分全体。根がvの外なら従来の区間、内側なら根へ向かう子の部分木を除きます。',['l','r','含む頂点'],intervals.map(([l,r])=>[l,r,flat.slice(l,r).join(',')]),{部分木和:sum(target)},snapshot(target));result.push(String(sum(target)));
      }else{
        // canonical decompositionを再構築。splay回転列を模倣するものではない。
        function clusters(r,end=r){
          const data=rooted(n,edges,r),preferred=path(n,edges,r,end),next=new Map(preferred.slice(0,-1).map((u,i)=>[u,preferred[i+1]]));
          const sizes=Array(n).fill(1);for(const u of data.order.slice().reverse())if(data.parent[u]>=0)sizes[data.parent[u]]+=sizes[u];
          const nodes=[];
          function build(start,main){let all=[],spine=[],off=[],u=start;while(u>=0){const children=data.adj[u].map(e=>e.to).filter(v=>data.parent[v]===u);const h=main?(next.get(u)??-1):children.reduce((best,v)=>best<0||sizes[v]>sizes[best]?v:best,-1);let light=[];for(const v of children)if(v!==h){const child=build(v,false);light.push(...child.all);nodes.push({kind:'add_edge',spine:[],off:child.all.slice(),at:u});}if(light.length)nodes.push({kind:'rake',spine:[],off:light.slice(),at:u});spine.push(u);off.push(...light);all.push(u,...light);nodes.push({kind:'vertex',spine:[u],off:light.slice(),at:u});if(spine.length>1)nodes.push({kind:'compress',spine:spine.slice(),off:off.slice(),at:start});u=h;}return {all,spine,off};}
          const total=build(r,true);
          for(const node of nodes)add(node.kind+' cluster','spine上はcompress、脇の枝はadd_edgeでPointにしてrakeします。図解はこの分解を再構築します。',['領域','頂点','和'],[['spine',node.spine.join(','),sum(node.spine)],['off-spine',node.off.join(','),sum(node.off)]],{境界:node.at}, {...snapshot([...node.spine,...node.off],node.spine),cluster:node});
          return total;
        }
        add('初期の森','link/cutで辺集合が変わります。重みは頂点の値です。',['頂点','値'],values.map((x,v)=>[v,x]),{},snapshot());
        for(const operation of list(input.ops,40)){
          const op=list(operation,5),kind=op[0];let active=[],answer,spine=[];
          const lengths={link:3,cut:3,set:3,path_apply:5,subtree_apply:5,component_apply:4,path_sum:3,subtree_mass:3,component_sum:2,distance:2,independent:2};
          if(op.length!==lengths[kind])throw Error('操作名または引数の数が違います');
          const u=vertex(op[1]);
          if(kind==='link'){const v=vertex(op[2]);answer=path(n,edges,u,v).length?'false':'true';if(answer==='true')edges.push([u,v]);active=[u,v];}
          else if(kind==='cut'){const v=vertex(op[2]),i=edges.findIndex(([a,b])=>(a===u&&b===v)||(a===v&&b===u));answer=String(i>=0);if(i>=0)edges.splice(i,1);active=[u,v];}
          else if(kind==='set'){values[u]=integer(op[2]);active=[u];}
          else if(kind.endsWith('_apply')){
            const component=kind==='component_apply',v=component?u:vertex(op[2]);active=kind==='path_apply'?path(n,edges,u,v):subtree(n,edges,u,v);const a=integer(op[component?2:3],-10,10),b=integer(op[component?3:4]);
            clusters(u,kind==='path_apply'&&active.length?v:u);
            add('対象clusterに作用を保留',`x → ${a}x + ${b}。C++ではspineとoff-spineのtagを別に合成し、必要なときにpushします。`,['対象頂点','旧値','新値'],active.map(x=>[x,values[x],a*values[x]+b]),{作用順:'古い作用 → 新しい作用'},snapshot(active));
            for(const x of active)values[x]=integer(a*values[x]+b,-100000000,100000000);answer=String(active.length>0);
          }else{
            const v=op.length===3?vertex(op[2]):u;
            active=kind==='path_sum'?path(n,edges,u,v):subtree(n,edges,u,kind==='subtree_mass'?v:u);spine=kind==='path_sum'?active:[];
            if(!active.length)answer='null';
            else if(kind==='distance'){const d=rooted(n,edges,u).depth;answer=String(active.reduce((total,x)=>total+values[x]*d[x],0));}
            else if(kind==='independent'){
              const data=rooted(n,edges,u),white=Array(n).fill(1n),black=values.map(BigInt);
              for(const x of data.order.slice().reverse())if(data.parent[x]>=0){const p=data.parent[x];white[p]*=white[x]+black[x];black[p]*=white[x];}
              answer=String(white[u]+black[u]);add('白/黒の木DP','白なら子の白+黒、黒なら子の白だけを掛けます。C++では2×2行列のcompressでこの遷移を維持します。',['頂点','白','黒'],data.order.map(x=>[x,String(white[x]),String(black[x])]),{},snapshot(active));
            }else answer=String(sum(active));
            clusters(u,kind==='path_sum'&&active.length?v:u);result.push(answer);
          }
          add(JSON.stringify(op),'操作完了。ここから次の操作を開始します。',['頂点','現在値'],values.map((x,v)=>[v,x]),{結果:answer??'更新'}, {...snapshot(active,spine),operation:op.slice(),answer});
        }
      }
    }
    return {steps,result};
  }
  const api={modes,trace};if(typeof module!=='undefined')module.exports=api;else root.TreeExtensionsModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
