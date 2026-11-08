(function(root,factory){const api=factory();if(typeof module==="object"&&module.exports)module.exports=api;else root.GraphConnectivityModel=api;})(globalThis,function(){
  "use strict";
  const clone=x=>JSON.parse(JSON.stringify(x));
  function trace(mode,input){
    const directed=["scc","condensation"].includes(mode),n=input.n;
    const s={...clone(input),mode,directed,component:Array(n).fill(-1),groups:[],ord:Array(n).fill(-1),low:Array(n).fill(-1),parent:Array(n).fill(-1),parentEdge:Array(n).fill(-1),children:Array(n).fill(0),subtree:Array(n).fill(1),visit:Array(n).fill(0),colors:Array(n).fill(-1),finish:[],order:[],roots:[],work:[],bridgeEdges:[],articulation:[],dagArcs:[],phase:"initial",reverse:false,activeVertex:-1,activeEdge:-1,rule:null,removal:null,conflictEdge:-1,bipartite:null};
    const adj=Array.from({length:n},()=>[]),reverse=Array.from({length:n},()=>[]),frames=[];
    input.edges.forEach(([u,v],id)=>{adj[u].push({to:v,id});reverse[v].push({to:u,id});if(!directed)adj[v].push({to:u,id});});
    const frame=(title,detail,extra={})=>{Object.assign(s,extra,{title,detail});frames.push(clone(s));};
    let timer=0;
    frame("初期状態",directed?"有向辺で、行き来できる頂点をまとめます。":"無向辺の両方向は同じ論理辺 ID を持ちます。全成分を調べます。");
    if(directed){
      s.phase="forward";
      for(let root=0;root<n;++root){
        if(s.visit[root])continue;const stack=[[root,0]];s.visit[root]=1;s.ord[root]=timer++;s.work=[String(root)];
        frame(`正方向 DFS: 根 ${root}`,"最初の DFS では、戻る順番を finish に保存します。",{activeVertex:root,activeEdge:-1});
        while(stack.length){
          const top=stack[stack.length-1],v=top[0];
          if(top[1]===adj[v].length){s.visit[v]=2;s.finish.push(v);stack.pop();s.work=stack.map(([u])=>String(u));frame(`finish に ${v} を追加`,"隣接先の探索が終わり、帰りがけ順へ記録します。",{activeVertex:v,activeEdge:-1});continue;}
          const e=adj[v][top[1]++];
          if(!s.visit[e.to]){s.visit[e.to]=1;s.ord[e.to]=timer++;s.parent[e.to]=v;s.parentEdge[e.to]=e.id;stack.push([e.to,0]);s.work=stack.map(([u])=>String(u));frame(`辺 #${e.id} から ${e.to} へ`,"未訪問の頂点を探索スタックへ入れます。",{activeVertex:e.to,activeEdge:e.id});}
          else frame(`辺 #${e.id}: 訪問済み`,"この頂点の DFS を重ねて開始しません。",{activeVertex:v,activeEdge:e.id});
        }
      }
      s.reverse=true;s.phase="reverse";s.visit=Array(n).fill(0);s.parent=Array(n).fill(-1);s.parentEdge=Array(n).fill(-1);s.order=s.finish.slice().reverse();
      frame("finish の逆順で、逆グラフを探索",`根候補: ${s.order.join(" → ")||"空列"}。辺の向きを全て反転します。`,{activeVertex:-1,activeEdge:-1});
      for(const root of s.order){
        if(s.component[root]>=0)continue;const c=s.groups.length,stack=[root];s.groups.push([root]);s.component[root]=c;s.visit[root]=1;s.work=[String(root)];
        frame(`成分 C${c} を根 ${root} から作る`,"逆グラフで未割当の頂点へ進むと、一つの強連結成分を取り出せます。",{activeVertex:root,activeEdge:-1});
        while(stack.length){const v=stack.pop();s.visit[v]=2;s.work=stack.map(String);
          frame(`逆グラフの ${v} を処理`,"元グラフでこの頂点へ入る辺を、逆向きに辿ります。",{activeVertex:v,activeEdge:-1});
          for(const e of reverse[v])if(s.component[e.to]<0){s.component[e.to]=c;s.groups[c].push(e.to);s.visit[e.to]=1;s.parent[e.to]=v;s.parentEdge[e.to]=e.id;stack.push(e.to);s.work=stack.map(String);frame(`${e.to} を C${c} へ追加`,"別成分として確定した頂点へは進みません。",{activeVertex:e.to,activeEdge:e.id});}
        }
      }
      s.groups.forEach(group=>group.sort((a,b)=>a-b));s.reverse=false;s.phase="scc-done";s.work=[];
      frame("強連結成分が確定",`${s.groups.length} 成分。異なる成分を結ぶ辺は、小さい成分 ID から大きい ID へ向かいます。`,{activeVertex:-1,activeEdge:-1});
      if(mode==="condensation"){
        s.phase="condense";const seen=new Set();
        for(let from=0;from<s.groups.length;++from)for(const v of s.groups[from])for(const e of adj[v]){
          const to=s.component[e.to],key=`${from},${to}`;s.activeEdge=e.id;
          if(from===to){frame(`成分内の辺 #${e.id} を除外`,"同じ SCC の中の辺は、縮約 DAG の辺にしません。",{activeVertex:v});continue;}
          if(s.deduplicate!==false&&seen.has(key)){frame(`辺 #${e.id}: 同じ成分間の辺は集約済み`,"既に Cfrom→Cto があるため重複をまとめます。代表元辺は最小 ID とは限りません。",{activeVertex:v});continue;}
          seen.add(key);s.dagArcs.push({from,to,original:e.id});frame(`C${from} → C${to} を追加`,`縮約辺 #${s.dagArcs.length-1} に元辺 #${e.id} を対応させます。`,{activeVertex:v});
        }
        s.phase="done";frame("縮約 DAG が完成",`${s.groups.length} 頂点・${s.dagArcs.length} 辺。成分を頂点として、DAG 上の処理へ進めます。`,{activeVertex:-1,activeEdge:-1});
      }
    }else if(["lowlink","bridges","articulation"].includes(mode)){
      s.phase="dfs";const cut=Array(n).fill(false);
      for(let root=0;root<n;++root){
        if(s.ord[root]>=0)continue;const c=s.groups.length;s.groups.push([root]);s.roots.push(root);s.component[root]=c;s.ord[root]=s.low[root]=timer++;s.visit[root]=1;
        const stack=[[root,0]];s.work=[String(root)];
        frame(`根 ${root} から DFS`,"ord は発見順、low は部分木から木辺を下り、戻り辺を高々一本使って届く最小の ord です。",{activeVertex:root,activeEdge:-1,rule:null});
        while(stack.length){
          const top=stack[stack.length-1],v=top[0];
          if(top[1]<adj[v].length){
            const e=adj[v][top[1]++];
            if(e.id===s.parentEdge[v]){frame(`親辺 #${e.id} の逆向きを除外`,"親頂点でなく辺 ID で除外します。平行な別の辺は戻り辺として使えます。",{activeVertex:v,activeEdge:e.id,rule:null});continue;}
            if(s.ord[e.to]<0){++s.children[v];s.parent[e.to]=v;s.parentEdge[e.to]=e.id;s.component[e.to]=c;s.groups[c].push(e.to);s.ord[e.to]=s.low[e.to]=timer++;s.visit[e.to]=1;stack.push([e.to,0]);s.work=stack.map(([u])=>String(u));frame(`木辺 #${e.id}: ${e.to} を発見`,`ord[${e.to}]=low[${e.to}]=${s.ord[e.to]} で始めます。`,{activeVertex:e.to,activeEdge:e.id,rule:null});}
            else{const old=s.low[v];s.low[v]=Math.min(old,s.ord[e.to]);frame(`辺 #${e.id}: low[${v}] を比較`,`min(${old}, ord[${e.to}]=${s.ord[e.to]}) = ${s.low[v]}。木辺以外の訪問済み頂点への辺を調べます。`,{activeVertex:v,activeEdge:e.id,rule:null});}
          }else{
            stack.pop();s.work=stack.map(([u])=>String(u));s.visit[v]=2;const p=s.parent[v];
            if(p<0){cut[v]=s.children[v]>1;s.articulation=cut.map((yes,i)=>yes?i:-1).filter(i=>i>=0);s.rule={root:true,vertex:v,children:s.children[v],cut:cut[v]};frame(`根 ${v}: DFS の子は ${s.children[v]} 個`,cut[v]?"子が 2 個以上なので関節点です。":"根は子が 0〜1 個なら関節点ではありません。次数だけでは判定しません。",{activeVertex:v,activeEdge:-1});}
            else{
              s.subtree[p]+=s.subtree[v];s.low[p]=Math.min(s.low[p],s.low[v]);const bridge=s.ord[p]<s.low[v],condition=s.ord[p]<=s.low[v];
              if(bridge)s.bridgeEdges.push(s.parentEdge[v]);if(s.parent[p]>=0&&condition)cut[p]=true;s.articulation=cut.map((yes,i)=>yes?i:-1).filter(i=>i>=0);
              s.rule={root:false,parent:p,child:v,parentOrd:s.ord[p],childLow:s.low[v],bridge,cut:condition&&s.parent[p]>=0,parentIsRoot:s.parent[p]<0};
              frame(`子 ${v} から親 ${p} へ戻る`,`橋: ${s.ord[p]} < ${s.low[v]} は ${bridge}。根以外の関節点: ${s.ord[p]} ≤ ${s.low[v]} を使います。low[${p}] に子の low を引き継ぎます。`,{activeVertex:p,activeEdge:s.parentEdge[v]});
            }
          }
        }
      }
      s.bridgeEdges.sort((a,b)=>a-b);s.phase="done";s.work=[];
      frame("Lowlink・橋・関節点が確定",`橋 ID: ${s.bridgeEdges.join(", ")||"なし"} / 関節点: ${s.articulation.join(", ")||"なし"}。一覧は番号順です。`,{activeVertex:-1,activeEdge:-1,rule:null});
      if(s.remove){
        s.before=s.groups.length;s.removal=clone(s.remove);s.phase="deletion";s.component=Array(n).fill(-1);s.groups=[];s.visit=Array(n).fill(0);s.work=[];
        const missing=s.removal.kind==="vertex"?s.removal.id:-1;if(missing>=0)s.component[missing]=-2;
        frame(`${s.removal.kind==="edge"?"辺":"頂点"} ${s.removal.id} を仮に削除`,"元グラフの ord/low を保持し、削除後の連結成分を探索します。実装の Lowlink を再計算する操作ではありません。");
        for(let root=0;root<n;++root){
          if(root===missing||s.component[root]>=0)continue;const c=s.groups.length,queue=[root];s.groups.push([root]);s.component[root]=c;s.visit[root]=1;s.work=[String(root)];
          frame(`削除後の成分 ${c} を開始`,"残った辺を使って到達できる範囲を塗り分けます。",{activeVertex:root,activeEdge:-1});
          for(let p=0;p<queue.length;++p){const v=queue[p];s.visit[v]=2;
            for(const e of adj[v])if(e.to!==missing&&!(s.removal.kind==="edge"&&e.id===s.removal.id)&&s.component[e.to]<0){s.component[e.to]=c;s.groups[c].push(e.to);s.visit[e.to]=1;queue.push(e.to);s.work=queue.slice(p+1).map(String);frame(`${e.to} へ到達`,"削除した辺や頂点を使わずに進める経路です。",{activeVertex:e.to,activeEdge:e.id});}
          }
        }
        s.after=s.groups.length;s.work=[];frame(`成分数 ${s.before} → ${s.after}`,s.after>s.before?`成分数が増えるので、この${s.removal.kind==="edge"?"辺は橋":"頂点は関節点"}です。`:"成分数は増えません。孤立点を削除して成分数が減る場合も、関節点にはしません。",{activeVertex:-1,activeEdge:-1});
      }
    }else if(mode==="components"||mode==="bipartite"){
      s.phase="search";s.bipartite=mode==="bipartite"?true:null;
      for(let root=0;root<n;++root){
        if(s.component[root]>=0)continue;const c=s.groups.length,queue=[root];s.groups.push([root]);s.component[root]=c;s.colors[root]=0;s.visit[root]=1;s.work=[String(root)];
        frame(`未訪問の根 ${root} から開始`,mode==="bipartite"?"各連結成分の根を色 0 にし、隣接先を反対の色にします。":"この根とつながる頂点を同じ成分へ集めます。",{activeVertex:root,activeEdge:-1});
        for(let p=0;p<queue.length;++p){const v=queue[p];s.visit[v]=2;s.work=queue.slice(p+1).map(String);
          frame(`頂点 ${v} を処理`,"未処理の隣接先を探索します。",{activeVertex:v,activeEdge:-1});
          for(const e of adj[v]){
            if(mode==="bipartite"&&s.colors[e.to]===s.colors[v]){s.bipartite=false;s.conflictEdge=e.id;s.phase="done";frame(`辺 #${e.id} の両端が同じ色`,"二部グラフではありません。自己ループも同じ頂点を異なる色にはできないため失敗します。C++ は nullopt を返します。",{activeVertex:e.to,activeEdge:e.id});return {frames,state:clone(s)};}
            if(s.component[e.to]>=0)continue;s.component[e.to]=c;s.groups[c].push(e.to);s.visit[e.to]=1;s.colors[e.to]=s.colors[v]^1;queue.push(e.to);s.work=queue.slice(p+1).map(String);
            frame(`${e.to} を成分 ${c} へ追加`,mode==="bipartite"?`色[${e.to}] = ${s.colors[v]} XOR 1 = ${s.colors[e.to]}。`:"同じ探索で到達できた頂点は同じ成分です。",{activeVertex:e.to,activeEdge:e.id});
          }
        }
      }
      s.phase="done";s.work=[];frame(mode==="bipartite"?"全ての辺が異なる色を結ぶ":"連結成分が確定",mode==="bipartite"?"二部グラフです。孤立点を含め全頂点に 0/1 の色を返します。":"成分番号と、その成分に属する頂点一覧を返します。",{activeVertex:-1,activeEdge:-1});
    }else throw new Error("unknown mode");
    return {frames,state:clone(s)};
  }
  return {trace};
});
