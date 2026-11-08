(function(root,factory){const api=factory();if(typeof module==="object"&&module.exports)module.exports=api;else root.GraphBasicModel=api;})(globalThis,function(){
  "use strict";
  const clone=x=>JSON.parse(JSON.stringify(x));
  function trace(mode,input){
    const s={...clone(input),mode,dist:Array(input.n).fill(null),parent:Array(input.n).fill(-1),parentEdge:Array(input.n).fill(-1),negative:Array(input.n).fill(false),color:Array(input.n).fill(0),order:[],postorder:[],work:[],selected:[],pathVertices:[],pathEdges:[],activeVertex:-1,activeEdge:-1,phase:"search",cost:0,componentCount:0};
    const frames=[],adj=Array.from({length:s.n},()=>[]);
    s.edges.forEach(([u,v,w],id)=>{adj[u].push({from:u,to:v,w,id});if(!s.directed)adj[v].push({from:v,to:u,w,id});});
    const frame=(title,detail,extra={})=>{Object.assign(s,extra,{title,detail});if(mode!=="path"||s.phase==="restore")frames.push(clone(s));};
    const relax=(u,e,weight=e.w)=>{
      if(s.dist[u]===null)return false;
      const next=s.dist[u]+weight;
      if(s.dist[e.to]!==null&&s.dist[e.to]<=next)return false;
      s.dist[e.to]=next;s.parent[e.to]=u;s.parentEdge[e.to]=e.id;return true;
    };
    const restore=()=>{
      s.phase="restore";s.work=[];s.activeEdge=-1;s.activeVertex=s.target;
      frame("親配列から経路を復元する",`頂点 ${s.target} から始点 ${s.source} へ、parent と parent_edge を逆に辿ります。`);
      if(s.dist[s.target]===null||s.negative[s.target]||(mode==="dfs"&&s.component[s.target]!==s.component[s.source])){
        frame("復元できる経路はありません",s.negative[s.target]?"負閉路の影響で最短距離が定まらないため、空の経路を返します。":"指定した始点から到達できないため、空の経路を返します。");return;
      }
      let v=s.target;s.pathVertices=[v];
      while(v!==s.source){
        const id=s.parentEdge[v],u=s.parent[v];if(u<0||s.pathVertices.length>s.n)throw new Error("invalid predecessor chain");
        s.pathEdges.push(id);s.pathVertices.push(u);s.work=s.pathVertices.map(String);
        frame(`parent[${v}] = ${u} / 辺 #${id}`,"頂点列と辺番号列を後ろから集めます。多重辺でも parent_edge により使った辺が区別できます。",{activeVertex:u,activeEdge:id});v=u;
      }
      s.pathVertices.reverse();s.pathEdges.reverse();s.work=s.pathVertices.map(String);
      frame("始点 → 終点の順に反転",s.pathEdges.length?`頂点列 ${s.pathVertices.join(" → ")}、辺番号 ${s.pathEdges.join(", ")}。`:`始点自身への経路は {${s.source}}、辺列は空です。`,{activeEdge:-1,activeVertex:-1});
    };
    frame("初期状態","距離が未確定の頂点は ∞。辺番号は入力した順の 0 始まりです。");
    if(["bfs","zero-one","dijkstra","path"].includes(mode)){
      s.dist[s.source]=0;s.color[s.source]=1;let queue=[[0,s.source]];
      const work=()=>{s.work=queue.map(([d,v])=>mode==="bfs"?String(v):`(${d},${v})`);};work();
      frame(`始点 ${s.source} を追加`,mode==="bfs"?"FIFO キューで、辺数の少ない順に訪問します。":"(仮距離, 頂点) を保持します。同じ頂点の古い候補も残る場合があります。");
      while(queue.length){
        if(mode==="dijkstra"||mode==="path")queue.sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
        const [d,v]=queue.shift();work();
        if(d!==s.dist[v]){frame(`古い候補 (${d},${v}) を破棄`,"現在の dist と一致しない候補では、辺を再走査しません。",{activeVertex:v,activeEdge:-1});continue;}
        s.color[v]=2;s.order.push(v);
        frame(`頂点 ${v} を取り出す`,mode==="bfs"?`辺数 ${d} の頂点から隣接先を見ます。`:`仮距離 ${d} の候補を処理します。`,{activeVertex:v,activeEdge:-1});
        for(const e of adj[v]){
          if(mode==="bfs"&&s.dist[e.to]!==null){frame(`辺 #${e.id}: 訪問済み`,"BFS は初回発見時に最短の辺数が確定するため再登録しません。",{activeEdge:e.id});continue;}
          const old=s.dist[e.to],changed=relax(v,e,mode==="bfs"?1:e.w);
          if(changed){const entry=[s.dist[e.to],e.to];if(mode==="zero-one"&&e.w===0)queue.unshift(entry);else queue.push(entry);s.color[e.to]=Math.max(1,s.color[e.to]);}
          if(mode==="dijkstra"||mode==="path")queue.sort((a,b)=>a[0]-b[0]||a[1]-b[1]);work();
          frame(`辺 #${e.id}: ${changed?"緩和する":"更新しない"}`,changed?`dist[${e.to}]: ${old===null?"∞":old} → ${s.dist[e.to]}。${mode==="zero-one"?(e.w===0?"重み 0 なので deque の先頭へ。":"重み 1 なので末尾へ。"):"親の頂点と辺番号も保存します。"}`:"現在の距離以下にならないので親も変えません。",{activeEdge:e.id});
        }
      }
      frame("探索が完了","BFS の距離は辺の本数、0-1 BFS / Dijkstra の距離は辺の重みの和です。",{activeEdge:-1,activeVertex:-1});restore();
    }else if(mode==="dfs"||mode==="cycle"){
      s.component=Array(s.n).fill(-1);s.tin=Array(s.n).fill(-1);s.tout=Array(s.n).fill(-1);s.cycle=null;s.roots=[];
      const roots=mode==="cycle"&&s.scope? [s.source] : mode==="dfs"&&s.n ? [s.source,...Array.from({length:s.n},(_,i)=>i).filter(i=>i!==s.source)] : Array.from({length:s.n},(_,i)=>i);
      let found=false;
      for(const root of roots){
        if(s.color[root])continue;
        const component=s.roots.length;s.roots.push(root);s.dist[root]=0;s.color[root]=1;s.component[root]=component;s.tin[root]=s.order.length;s.order.push(root);
        const stack=[[root,0]];s.work=stack.map(([v])=>String(v));
        frame(`根 ${root} から開始`,"スタックには頂点と次に見る辺の位置を保持します。再帰呼出しを使わず、深いグラフも処理できます。",{activeVertex:root,activeEdge:-1});
        while(stack.length){
          const top=stack[stack.length-1],v=top[0];
          if(top[1]===adj[v].length){s.color[v]=2;s.postorder.push(v);s.tout[v]=s.order.length;stack.pop();s.work=stack.map(([u])=>String(u));frame(`頂点 ${v} から戻る`,"帰りがけ順へ追加し、灰色（探索中）から緑（終了）へ変えます。",{activeVertex:v,activeEdge:-1});continue;}
          const e=adj[v][top[1]++];s.activeEdge=e.id;
          if(mode==="cycle"&&!s.directed&&e.id===s.parentEdge[v]){frame(`親辺 #${e.id} の逆向きは除外`,"親の頂点ではなく辺番号で除外するため、同じ頂点間の別の辺は検出できます。",{activeVertex:v});continue;}
          if(s.color[e.to]===0){
            s.parent[e.to]=v;s.parentEdge[e.to]=e.id;s.dist[e.to]=s.dist[v]+1;s.component[e.to]=component;
            s.color[e.to]=1;s.tin[e.to]=s.order.length;s.order.push(e.to);stack.push([e.to,0]);s.work=stack.map(([u])=>String(u));
            frame(`未訪問の ${e.to} へ進む`,"現在の枝を先に深く辿ります。dist は DFS 木の深さで、最短距離ではありません。",{activeVertex:e.to});
          }else if(mode==="cycle"&&s.color[e.to]===1){
            const vertices=[],edges=[];for(let cur=v;cur!==e.to;cur=s.parent[cur]){vertices.push(cur);edges.push(s.parentEdge[cur]);}
            vertices.push(e.to);vertices.reverse();edges.reverse();edges.push(e.id);s.cycle={vertices,edges};s.selected=edges.slice();found=true;
            frame("探索中の祖先へ戻る辺を発見",`閉路 ${vertices.join(" → ")} → ${vertices[0]}。自己ループは 1 辺、多重辺は 2 辺の閉路にもなります。`,{activeVertex:e.to});break;
          }else frame(`頂点 ${e.to} は訪問済み`,mode==="cycle"?"終了済みの頂点へ向かう辺だけでは閉路と判定しません。":"再訪せず、次の辺を調べます。",{activeVertex:v});
        }
        if(found)break;
      }
      if(mode==="dfs"){frame("全成分の DFS が完了","preorder と postorder、親・深さ、各 DFS 木の部分木区間を取得できます。",{activeEdge:-1,activeVertex:-1});if(s.n)restore();}
      else if(!found)frame("閉路はありません",s.scope?"指定した始点から到達できる範囲に限定した結果です。":"全頂点を根候補として調べました。",{activeEdge:-1,activeVertex:-1});
    }else if(mode==="bellman"){
      s.dist[s.source]=0;s.round=0;
      for(let round=1;round<s.n;++round){
        s.round=round;let changed=false;frame(`緩和 ${round} 周目`,"到達できる頂点から辺を走査します。最大 V−1 周で、負閉路のない最短路を求めます。",{activeEdge:-1});
        for(let id=0;id<s.edges.length;++id){
          const [u,v,w]=s.edges[id],arcs=s.directed?[[u,v]]:[[u,v],[v,u]];
          for(const[from,to]of arcs){if(s.dist[from]===null)continue;const old=s.dist[to],updated=relax(from,{to,w,id});changed||=updated;
            frame(`辺 #${id}: ${updated?"緩和する":"更新しない"}`,updated?`${old===null?"∞":old} → ${s.dist[to]}。同じ周に更新した距離も後続の走査で使います。`:"距離は変わりません。",{activeVertex:from,activeEdge:id});}
        }
        if(!changed){frame("更新がないため早期終了","到達可能な負閉路があれば更新が止まらないため、この時点では影響頂点もありません。");break;}
      }
      const queue=[];
      for(let v=0;v<s.n;++v)if(s.dist[v]!==null)for(const e of adj[v])if(s.dist[v]+e.w<s.dist[e.to]&&!s.negative[e.to]){s.negative[e.to]=true;queue.push(e.to);s.work=queue.map(String);frame(`さらに改善する辺 #${e.id}`,"負閉路の影響がある頂点を見つけました。距離を最短距離として使わず、フラグを付けます。",{activeVertex:e.to,activeEdge:e.id});}
      for(let p=0;p<queue.length;++p){const v=queue[p];s.work=queue.slice(p+1).map(String);for(const e of adj[v])if(!s.negative[e.to]){s.negative[e.to]=true;queue.push(e.to);s.work=queue.slice(p+1).map(String);frame(`影響を ${e.to} へ伝播`,"負閉路から先へ進める頂点も、距離をいくらでも小さくできます。",{activeVertex:e.to,activeEdge:e.id});}}
      s.work=[];frame("負閉路の影響範囲を確定","始点から届かない負閉路は、この始点の結果には影響しません。",{activeEdge:-1,activeVertex:-1});restore();
    }else if(mode==="floyd"){
      s.matrix=Array.from({length:s.n},(_,i)=>Array.from({length:s.n},(_,j)=>i===j?0:null));
      const parent=Array.from({length:s.n},()=>Array(s.n).fill(-1)),parentEdge=clone(parent);
      for(let u=0;u<s.n;++u)for(const e of adj[u])if(s.matrix[u][e.to]===null||e.w<s.matrix[u][e.to]){s.matrix[u][e.to]=e.w;parent[u][e.to]=u;parentEdge[u][e.to]=e.id;}
      const update=()=>{if(s.n){s.dist=s.matrix[s.source];s.parent=parent[s.source];s.parentEdge=parentEdge[s.source];}};update();
      frame("対角を 0、直接の辺を最小重みで初期化","多重辺は最小重みだけを初期値に使い、負の自己ループも残します。");
      for(let k=0;k<s.n;++k){s.k=k;s.work=Array.from({length:k+1},(_,i)=>String(i));frame(`中継頂点 k=${k} を許可`,"k は最も外側のループです。dist[i][k]+dist[k][j] と現在値を比べます。",{activeVertex:k,activeEdge:-1});
        for(let i=0;i<s.n;++i)if(s.matrix[i][k]!==null)for(let j=0;j<s.n;++j)if(s.matrix[k][j]!==null){const next=s.matrix[i][k]+s.matrix[k][j];if(s.matrix[i][j]===null||next<s.matrix[i][j]){s.matrix[i][j]=next;parent[i][j]=parent[k][j];parentEdge[i][j]=parentEdge[k][j];s.cell=[i,j];update();frame(`dist[${i}][${j}] を ${next} へ更新`,`頂点 ${k} 経由が短いため、最後の辺の親情報も引き継ぎます。`);}}}
      s.negativeMatrix=Array.from({length:s.n},()=>Array(s.n).fill(false));
      for(let k=0;k<s.n;++k)if(s.matrix[k][k]<0)for(let i=0;i<s.n;++i)if(s.matrix[i][k]!==null)for(let j=0;j<s.n;++j)if(s.matrix[k][j]!==null)s.negativeMatrix[i][j]=true;
      if(s.n)s.negative=s.negativeMatrix[s.source];s.cell=null;
      frame("全点対の距離と負閉路の影響を確定","i→負閉路→j と進める組だけが −∞ になります。表示する始点の行からも経路を復元できます。",{activeVertex:-1});if(s.n)restore();
    }else if(mode==="topological"){
      s.indegree=Array(s.n).fill(0);for(const[,v]of s.edges)++s.indegree[v];
      const queue=Array.from({length:s.n},(_,i)=>i).filter(v=>s.indegree[v]===0);s.work=queue.map(String);
      frame("入次数 0 の頂点を集める","前提となる頂点が残っていないものから、FIFO キューへ入れます。");
      while(queue.length){const v=queue.shift();s.order.push(v);s.color[v]=2;s.work=queue.map(String);frame(`頂点 ${v} を順序へ追加`,"出辺を取り除く代わりに、隣接先の入次数を減らします。",{activeVertex:v,activeEdge:-1});
        for(const e of adj[v]){--s.indegree[e.to];if(!s.indegree[e.to])queue.push(e.to);s.work=queue.map(String);frame(`入次数[${e.to}] → ${s.indegree[e.to]}`,s.indegree[e.to]===0?"0 になったので処理可能です。":"まだ未処理の前提が残っています。",{activeEdge:e.id});}}
      s.acyclic=s.order.length===s.n;frame(s.acyclic?"トポロジカル順序が完成":"閉路があるため nullopt",s.acyclic?`順序: ${s.order.join(" → ")||"空列"}`:"残った頂点には、閉路の先で待たされている頂点も含まれます。残りすべてが閉路上とは限りません。",{activeVertex:-1,activeEdge:-1});
    }else if(mode==="kruskal"||mode==="prim"){
      if(mode==="kruskal"){
        s.dsu=Array.from({length:s.n},(_,i)=>i);s.componentCount=s.n;
        const root=x=>{while(s.dsu[x]!==x)x=s.dsu[x];return x;};
        const order=s.edges.map((_,i)=>i).sort((a,b)=>s.edges[a][2]-s.edges[b][2]||a-b);s.work=order.map(i=>`#${i}:${s.edges[i][2]}`);
        frame("辺を重み順にソート","同じ重みなら辺番号順。Union-Find で、端点が別成分かを判定します。");
        for(const id of order){const[u,v,w]=s.edges[id],a=root(u),b=root(v);s.work.shift();if(a!==b){s.dsu[a]=b;s.selected.push(id);s.cost+=w;--s.componentCount;}
          frame(`辺 #${id} を${a!==b?"採用":"除外"}`,a!==b?`別成分を接続。総コスト ${s.cost}、残る成分数 ${s.componentCount}。`:"端点が同じ成分なので、加えると閉路になります。自己ループも除外します。",{activeEdge:id,activeVertex:-1});}
      }else{
        const used=Array(s.n).fill(false),queue=[];
        const work=()=>{queue.sort((a,b)=>a[0]-b[0]||a[1]-b[1]);s.work=queue.map(([w,id])=>`#${id}:${w}`);};
        const visit=v=>{used[v]=true;s.color[v]=2;for(const e of adj[v])if(!used[e.to])queue.push([e.w,e.id]);work();};
        for(let root=0;root<s.n;++root){if(used[root])continue;++s.componentCount;visit(root);frame(`成分の根 ${root} を追加`,"非連結なら次の未訪問頂点から再開し、最小全域森を作ります。",{activeVertex:root,activeEdge:-1});
          while(queue.length){const[w,id]=queue.shift(),[u,v]=s.edges[id];work();if(used[u]&&used[v]){frame(`古い辺候補 #${id} を除外`,"両端とも既に木に入っているため、加えると閉路になります。",{activeEdge:id});continue;}
            s.selected.push(id);s.cost+=w;const to=used[u]?v:u;visit(to);frame(`辺 #${id} で頂点 ${to} を追加`,`木と外側を結ぶ候補の中で最小。総コスト ${s.cost}。`,{activeVertex:to,activeEdge:id});}}
      }
      frame("最小全域森が完成",`${s.componentCount} 成分、${s.selected.length} 辺、コスト ${s.cost}。負の重みも扱えます。`,{activeVertex:-1,activeEdge:-1});
    }else throw new Error("unknown mode");
    return {frames,state:clone(s)};
  }
  return {trace};
});
