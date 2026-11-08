(function(){
  "use strict";
  const $=id=>document.getElementById(id),M=GraphBasicModel;
  const config={
    bfs:{file:"bfs",title:"辺数の少ない順に訪問する",principle:"FIFO キューを使い、先に見つかった頂点から隣接先を調べます。初めて発見したときに親と距離を保存します。",formula:"dist[to] = dist[v] + 1\n時間 O(V+E)",pitfall:"入力された重みは無視します。重み付きの最短距離が必要なら 0-1 BFS や Dijkstra を選びます。"},
    dfs:{file:"dfs",title:"一つの枝を深く辿り、戻る",principle:"頂点と次の辺の位置をスタックへ保存し、再帰 DFS と同じ訪問順を作ります。最初の根を調べた後、未訪問の頂点から再開します。",formula:"部分木: preorder[tin[v]..tout[v])\n時間 O(V+E) / 再帰なし",pitfall:"表示する dist は DFS 木の深さです。有向グラフの DFS 木の成分は、強連結成分や弱連結成分とは異なります。"},
    "zero-one":{file:"zero_one_bfs",title:"重み 0 は前へ、1 は後ろへ",principle:"改善した候補を、重み 0 なら deque の先頭、1 なら末尾に入れます。現在の距離と合わない古い候補は破棄します。",formula:"w=0: push_front / w=1: push_back\n時間 O(V+E)",pitfall:"各辺の重みが 0 または 1 であることが条件です。訪問済みだけで緩和を止めず、距離が短くなったかを見ます。"},
    dijkstra:{file:"dijkstra",title:"仮距離が最小の候補を取り出す",principle:"優先度付きキューに (距離,頂点) を追加します。最小距離の候補から辺を緩和し、古い候補を破棄します。",formula:"時間 O(V + E log(E+1))\n親の更新は距離が厳密に短くなったとき",pitfall:"全辺が非負であることが条件です。図は優先度順の候補列を示し、ヒープの内部配列は描いていません。"},
    bellman:{file:"bellman_ford",title:"辺の緩和を繰り返す",principle:"V−1 周の緩和後にも改善できる頂点を見つけ、その先へ影響を伝播します。影響する頂点だけが −∞ です。",formula:"時間 O(V+VE)\n到達不能: ∞ / 負閉路の影響: −∞",pitfall:"始点から到達できない負閉路は、この始点の結果には影響しません。影響下の数値 dist を最短距離として使わず、negative_cycle を確認します。"},
    floyd:{file:"floyd_warshall",title:"使ってよい中継頂点を増やす",principle:"中継 k を最も外側のループにして全点対を更新します。負の対角成分を経由できる組は、距離が定まりません。",formula:"d[i][j] = min(d[i][j], d[i][k]+d[k][j])\n時間 O(V³) / メモリ O(V²)",pitfall:"∞ に重みを足しません。負閉路がある場合も、影響を受けない始点・終点間の経路は復元できます。"},
    topological:{file:"topological_sort",title:"未処理の前提がない頂点から並べる",principle:"入次数 0 の頂点を取り出して出辺を処理します。すべて取り出せれば DAG のトポロジカル順序です。",formula:"Kahn 法 / 時間 O(V+E)\n閉路あり → nullopt",pitfall:"閉路があると順序全体を返しません。キューが止まった後の残りには、閉路の先で待つ頂点も含まれます。順序は辞書順最小を保証しません。"},
    cycle:{file:"cycle",title:"探索中の祖先へ戻る辺を探す",principle:"未訪問・探索中・終了の 3 状態で DFS します。探索中の祖先への辺が見つかれば、親辺を逆に辿って閉路を取り出せます。",formula:"時間 O(V+E) / 再帰なし\n無向グラフ: 親辺 ID の逆向きだけを除外",pitfall:"無向グラフでは親の頂点で除外すると多重辺の閉路を見逃します。自己ループは 1 辺、平行な 2 辺も閉路です。"},
    kruskal:{file:"kruskal",title:"軽い辺から別成分をつなぐ",principle:"辺を重み順に並べ、両端が別成分なら採用します。成分管理には Union-Find を使います。",formula:"時間 O(V + E log(E+1))\n採用辺数 = V − 成分数",pitfall:"無向グラフ用です。負辺・多重辺・非連結にも対応し、非連結なら成分ごとの最小全域木を合わせた森を返します。"},
    prim:{file:"prim",title:"木から外へ伸びる最小の辺を選ぶ",principle:"今の木と未訪問頂点を結ぶ辺を優先度付きキューへ入れ、最小の候補から木を広げます。孤立した成分からも再開します。",formula:"時間 O(V + E log(E+1))\n非連結 → 最小全域森",pitfall:"Dijkstra と似たキューを使いますが、優先する値は始点からの距離ではなく辺単体の重みです。負辺も扱えます。"},
    path:{file:"shortest_path",title:"親を辿り、列を反転する",principle:"各頂点に直前の頂点と辺番号を保持します。終点から逆向きに集めた列を反転すると、始点からの経路になります。",formula:"vertex_path(t) / edge_path(t)\n時間 O(経路長) / 空列は復元不可",pitfall:"到達不能・負閉路の影響下では空列です。始点自身への頂点列は {s}、辺列は空。図は Dijkstra の計算後から始めます。"}
  };
  const defaults={
    bfs:"0 1\n1 2\n0 2\n2 3\n4 5",dfs:"0 1\n1 2\n0 2\n2 3\n4 5",
    "zero-one":"0 1 1\n0 2 0\n2 1 0\n1 3 1\n2 3 1\n4 5 0",
    dijkstra:"0 1 7\n0 2 2\n2 1 1\n1 3 2\n2 3 9\n3 4 1",path:"0 1 7\n0 2 2\n2 1 1\n1 3 2\n2 3 9\n3 4 1",
    bellman:"0 1 4\n0 2 5\n1 2 -2\n2 3 3\n4 5 -1",floyd:"0 1 4\n0 2 5\n1 2 -2\n2 3 3\n4 5 -1",
    topological:"0 2\n1 2\n1 3\n2 4\n3 4\n4 5",cycle:"0 1\n1 2\n2 0\n2 3\n4 5",
    kruskal:"0 1 4\n0 2 1\n1 2 2\n1 3 5\n2 3 8\n4 5 -2",prim:"0 1 4\n0 2 1\n1 2 2\n1 3 5\n2 3 8\n4 5 -2"
  };
  const esc=x=>String(x).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
  const integer=(x,label)=>{if(!String(x).trim())throw new Error(`${label}には整数を入力してください。`);const v=Number(x);if(!Number.isInteger(v)||Math.abs(v)>1000000)throw new Error(`${label}は ±100 万以内の整数です。`);return v;};
  const shortest=mode=>["bfs","dfs","zero-one","dijkstra","bellman","floyd","path"].includes(mode),mst=mode=>["kruskal","prim"].includes(mode);
  let mode="bfs",frames=[],position=0,timer=null;
  function input(){
    const n=integer($("vertices").value,"頂点数");if(n<0||n>8)throw new Error("頂点数は 0〜8 です。");
    if(!n&&["bfs","zero-one","dijkstra","bellman","path"].includes(mode))throw new Error("この探索には 1 個以上の頂点と有効な始点が必要です。");
    const rows=$("edges").value.trim()?$("edges").value.trim().split(/\n+/):[];if(rows.length>16)throw new Error("辺は最大 16 本です。");
    const edges=rows.map(row=>{const a=row.trim().split(/[\s,]+/).map(x=>integer(x,"辺"));if(a.length<2||a.length>3)throw new Error("辺は u v または u v w です。");if(a.length===2)a.push(1);if(a[0]<0||a[0]>=n||a[1]<0||a[1]>=n)throw new Error("辺の頂点は 0 以上、頂点数未満です。");return a;});
    if(mode==="zero-one"&&edges.some(e=>e[2]!==0&&e[2]!==1))throw new Error("0-1 BFS の重みは 0 または 1 です。");
    if(["dijkstra","path"].includes(mode)&&edges.some(e=>e[2]<0))throw new Error("Dijkstra の重みは非負です。");
    const source=shortest(mode)||mode==="cycle"&&$("scope").checked?integer($("source").value,"始点"):0;
    const target=shortest(mode)?integer($("target").value,"終点"):0;
    if(n&&(source<0||source>=n||target<0||target>=n))throw new Error("始点・終点は 0 以上、頂点数未満です。");
    return {n,edges,source,target,directed:$("kind").value==="directed",scope:n>0&&$("scope").checked};
  }
  function stop(){if(timer)clearInterval(timer);timer=null;$("play").textContent="再生 ▶";}
  function run(){try{const result=M.trace(mode,input());stop();frames=result.frames;position=0;$("input-error").textContent="";render();}catch(e){$("input-error").textContent=e.message;}}
  function scenario(type="regular"){
    $("vertices").value="6";$("source").value="0";$("target").value=["dijkstra","path"].includes(mode)?"4":"3";$("scope").checked=false;$("kind").value=mst(mode)?"undirected":"directed";$("edges").value=defaults[mode];
    if(type==="disconnected"){$("edges").value="0 1 1\n1 2 1\n3 4 1";$("target").value="5";}
    if(type==="negative"){$("edges").value="0 1 2\n1 2 -3\n2 1 1\n2 3 2\n4 5 -1\n5 4 0";}
    if(type==="parallel"){$("vertices").value="3";$("target").value="2";$("kind").value="undirected";$("edges").value="0 1 1\n0 1 0\n1 2 1\n2 2 0";}
    if(type==="cycle"){$("vertices").value="4";$("kind").value="directed";$("edges").value="0 1\n1 2\n2 1\n2 3";}
    run();
  }
  function select(next){
    mode=config[next]?next:"bfs";stop();
    for(const b of document.querySelectorAll("[data-mode]"))b.setAttribute("aria-pressed",String(b.dataset.mode===mode));
    const c=config[mode];for(const key of ["principle","formula","pitfall"])$(key).textContent=c[key];$("principle-title").textContent=c.title;
    $("docs-link").href=`../../docs/graph_basic/${c.file}.md`;$("code-link").href=`../../graph_basic/${c.file}.hpp`;
    $("source-target").hidden=!shortest(mode)&&mode!=="cycle";$("target-field").hidden=!shortest(mode);$("scope-field").hidden=mode!=="cycle";
    $("kind").disabled=mst(mode)||mode==="topological";
    $("input-note").textContent="0 始まり、最大 8 頂点・16 辺。"+(mode==="bfs"||mode==="dfs"?" 重みは無視します。":mode==="zero-one"?" 重みは 0/1 のみ。":mode==="dijkstra"||mode==="path"?" 重みは非負。":" 重み・中間値は図解用の整数で扱います。");
    const choices=[["regular","基本の例"],["disconnected","非連結・到達不能"],...(["bellman","floyd"].includes(mode)?[["negative","届く／届かない負閉路"]]:[]),...(mode!=="topological"?[["parallel","多重辺・自己ループ"]]:[]),...(["cycle","topological"].includes(mode)?[["cycle","閉路とその先の頂点"]]:[])];
    $("scenario-type").innerHTML=choices.map(([v,t])=>`<option value="${v}">${t}</option>`).join("");scenario();
  }
  const text=(x,y,value,cls="caption",anchor="start")=>`<text x="${x}" y="${y}" class="${cls}" text-anchor="${anchor}">${esc(value)}</text>`;
  const distance=(s,v)=>s.negative[v]?"−∞":s.dist[v]===null?"∞":s.dist[v];
  function diagram(s){
    let svg='<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0L10 5L0 10Z" fill="context-stroke"/></marker></defs>';
    svg+=text(30,28,s.directed?"有向グラフ / 辺 #ID: 重み":"無向グラフ / 両方向で同じ辺 ID","heading");
    const pos=Array.from({length:s.n},(_,i)=>[410+172*Math.cos(-Math.PI/2+i*2*Math.PI/s.n),220+150*Math.sin(-Math.PI/2+i*2*Math.PI/s.n)]),groups=new Map();
    s.edges.forEach(([u,v],i)=>{const key=[Math.min(u,v),Math.max(u,v)].join(',');if(!groups.has(key))groups.set(key,[]);groups.get(key).push(i);});
    for(let id=0;id<s.edges.length;++id){
      const[u,v,w]=s.edges[id],[x1,y1]=pos[u],[x2,y2]=pos[v],list=groups.get([Math.min(u,v),Math.max(u,v)].join(',')),slot=list.indexOf(id);let path,lx,ly;
      if(u===v){const rise=62+slot*19;path=`M${x1-15} ${y1-17}C${x1-56} ${y1-rise},${x1+56} ${y1-rise},${x1+15} ${y1-17}`;lx=x1;ly=y1-rise+3;}
      else{
        const dx=x2-x1,dy=y2-y1,length=Math.hypot(dx,dy),ux=dx/length,uy=dy/length,sign=u<v?1:-1,offset=(slot-(list.length-1)/2)*38*sign;
        const cx=(x1+x2)/2-uy*offset,cy=(y1+y2)/2+ux*offset;
        path=`M${x1+ux*24} ${y1+uy*24}Q${cx} ${cy},${x2-ux*27} ${y2-uy*27}`;lx=(x1+2*cx+x2)/4;ly=(y1+2*cy+y2)/4-5;
      }
      const cls=`graph-edge${s.parentEdge.includes(id)?" parent":""}${s.selected.includes(id)?" selected":""}${s.pathEdges.includes(id)?" path":""}${s.activeEdge===id?" active":""}`;
      svg+=`<path class="${cls}" d="${path}" marker-end="${s.directed?'url(#arrow)':"none"}"/>${text(lx,ly,`#${id}: ${w}`,"label","middle")}`;
    }
    const root=v=>{while(s.dsu&&s.dsu[v]!==v)v=s.dsu[v];return v;},palette=["#d6ece0","#ffe4c6","#dce5fa","#f5dcec","#e5edca","#d7ebf1","#eadff2","#eae5db"];
    pos.forEach(([x,y],v)=>{
      const active=s.activeVertex===v||s.pathVertices.includes(v),fill=s.negative[v]?"#ffe0e2":active?"#fff1df":s.dsu?palette[root(v)%palette.length]:s.color[v]===2?"#d6ece0":s.color[v]===1?"#e1e6e1":"white";
      svg+=`<circle cx="${x}" cy="${y}" r="23" fill="${fill}" stroke="${s.negative[v]?"#ae3b42":active?"#bd6935":"#729987"}" stroke-width="${active?3:1.5}"/>${text(x,y+6,v,"node-value","middle")}`;
      const label=mst(mode)?s.dsu?`成分 ${root(v)}`:"":mode==="topological"?`入次数 ${s.indegree?.[v]??"—"}`:`d=${distance(s,v)}`;
      svg+=text(x,y+43,label,"caption","middle");
    });
    if(!s.n)svg+=text(410,200,"空グラフ","heading","middle");
    const caption=s.phase==="restore"?"逆に集めた頂点 / 最後に反転":mode==="dfs"||mode==="cycle"?"探索スタック（左が根、右が現在位置）":mode==="kruskal"?"未処理の辺（重み順）":mode==="prim"?"辺の候補（重み, ID 順）":mode==="dijkstra"?"優先度付きキュー（距離, 頂点順）":mode==="zero-one"?"deque（左が先頭）":mode==="bellman"?"負閉路の影響を伝える候補":mode==="floyd"?"許可した中継頂点":"キュー（左が先頭）";
    svg+=text(30,445,caption,"heading");
    s.work.forEach((value,i)=>{const x=35+(i%6)*128,y=465+Math.floor(i/6)*36;svg+=`<rect class="work-cell" x="${x}" y="${y}" width="120" height="28" rx="6"/>${text(x+60,y+19,value,"work-value","middle")}`;});
    if(!s.work.length)svg+=text(40,484,"空列");
    return [svg,Math.max(530,480+Math.ceil(s.work.length/6)*36)];
  }
  function render(){
    if(!frames.length)return;const s=frames[position],[svg,height]=diagram(s);$("diagram").innerHTML=svg;$("diagram").setAttribute("viewBox",`0 0 840 ${height}`);
    $("step-title").textContent=s.title;$("step-description").textContent=s.detail;
    $("stat-label").textContent=mode==="prim"?"開始した成分数":mode==="kruskal"?"現在の成分数":mode==="bellman"?"緩和の周":mode==="floyd"?"中継 k":mode==="topological"?"頂点数":"始点";
    $("state-value").textContent=mst(mode)?s.componentCount:mode==="bellman"?s.round||0:mode==="floyd"?s.k??"—":mode==="topological"?s.n:s.source;
    $("count-label").textContent=mst(mode)?"採用辺数":"処理した頂点";$("count-value").textContent=mst(mode)?s.selected.length:s.order.length;
    $("result-label").textContent=mst(mode)?"総コスト":mode==="cycle"?"検出した閉路":mode==="topological"?"順序の結果":s.phase==="restore"?"復元する経路":"終点の仮距離";
    $("result").textContent=mst(mode)?s.cost:mode==="cycle"?s.cycle?`${s.cycle.edges.length} 辺`:position===frames.length-1?"なし":"—":mode==="topological"?s.acyclic===false?"閉路あり":s.acyclic?"DAG":"—":s.phase==="restore"?s.pathVertices.length?s.pathVertices.join("→"):"空列":s.n?distance(s,s.target):"—";
    const headers=mst(mode)?["辺 ID","端点","重み","採用"]:mode==="topological"?["頂点","入次数","順序位置"]:mode==="dfs"?["頂点","深さ","親 / 辺","tin,tout"]:["頂点","距離","親","親辺"];
    $("table-title").textContent=mst(mode)?"採用した辺":mode==="topological"?"入次数と順序":mode==="dfs"?"DFS 木の情報":"距離と親";
    $("table-note").textContent=mode==="dfs"?`preorder: ${s.order.join(", ")} / postorder: ${s.postorder.join(", ")}`:mst(mode)?"採用辺の端点は、入力時の向きを保持します。無向なので端点の順は意味を変えません。":"∞ は到達不能、−∞ は負閉路の影響。途中の値は計算中の状態です。";
    $("table-head").innerHTML=`<tr>${headers.map(h=>`<th>${h}</th>`).join("")}</tr>`;
    $("values").innerHTML=mst(mode)?s.edges.map(([u,v,w],i)=>`<tr class="${s.activeEdge===i?"active":""}" data-selected="${s.selected.includes(i)}"><td>${i}</td><td>${u}–${v}</td><td>${w}</td><td>${s.selected.includes(i)?"✓":"—"}</td></tr>`).join(""):Array.from({length:s.n},(_,v)=>`<tr class="${s.activeVertex===v?"active":""}" data-distance="${s.negative[v]?"-inf":s.dist[v]===null?"inf":s.dist[v]}"><td>${v}</td>${mode==="topological"?`<td>${s.indegree?.[v]??"—"}</td><td>${s.order.includes(v)?s.order.indexOf(v):"—"}</td>`:mode==="dfs"?`<td>${s.dist[v]??"—"}</td><td>${s.parent[v]} / ${s.parentEdge[v]}</td><td>${s.tin?.[v]??-1},${s.tout?.[v]??-1}</td>`:`<td class="${s.negative[v]?"negative":""}">${distance(s,v)}</td><td>${s.parent[v]}</td><td>${s.parentEdge[v]}</td>`}</tr>`).join("");
    $("matrix-wrap").hidden=mode!=="floyd"||!s.matrix;
    if(s.matrix){$("matrix-head").innerHTML=`<tr><th>s / t</th>${s.matrix.map((_,i)=>`<th>${i}</th>`).join("")}</tr>`;$("matrix").innerHTML=s.matrix.map((row,i)=>`<tr><th>${i}</th>${row.map((v,j)=>`<td class="${s.negativeMatrix?.[i][j]?"negative":""}">${s.negativeMatrix?.[i][j]?"−∞":v===null?"∞":v}</td>`).join("")}</tr>`).join("");}
    $("progress").textContent=`${position+1} / ${frames.length}`;$("previous").disabled=position===0;$("next").disabled=position===frames.length-1;$("finish").disabled=position===frames.length-1;$("restore").hidden=!shortest(mode);$("restore").disabled=!frames.some(f=>f.phase==="restore");
  }
  $("run").onclick=run;$("scenario").onclick=()=>scenario($("scenario-type").value);
  $("previous").onclick=()=>{stop();position=Math.max(0,position-1);render();};$("next").onclick=()=>{stop();position=Math.min(frames.length-1,position+1);render();};$("finish").onclick=()=>{stop();position=frames.length-1;render();};
  $("restore").onclick=()=>{stop();const index=frames.findIndex(s=>s.phase==="restore");if(index>=0)position=index;render();};
  $("play").onclick=()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 ■";timer=setInterval(()=>{if(position<frames.length-1){++position;render();}else stop();},500);};
  for(const b of document.querySelectorAll("[data-mode]"))b.onclick=()=>{if(location.hash.slice(1)===b.dataset.mode)select(b.dataset.mode);else location.hash=b.dataset.mode;};
  window.addEventListener("hashchange",()=>select(location.hash.slice(1)));select(location.hash.slice(1));
})();
