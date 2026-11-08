(function(){
  "use strict";
  const $=id=>document.getElementById(id),M=GraphConnectivityModel;
  const config={
    scc:{file:"scc",title:"帰りがけの逆順で、逆グラフを探索する",principle:"Kosaraju 法は正方向 DFS の終了順を保存し、その逆順に逆グラフを探索します。互いに行き来できる頂点が同じ SCC になります。",formula:"同じ SCC ⇔ u→v と v→u の両方がある\n異なる成分の辺: component[u] < component[v]\n時間 O(V+E) / 再帰なし",pitfall:"一方向に届くだけでは同じ SCC ではありません。辺の向きを反転する第二パスと、終了順の逆順を組み合わせます。"},
    condensation:{file:"condensation",title:"成分を一頂点に縮約する",principle:"SCC 内部の辺を除き、異なる成分間の辺を DAG にします。既定では同じ成分間の平行辺をまとめます。",formula:"元辺 (u,v) → (component[u],component[v])\n内部辺を除外 / 必要なら重複を集約\n時間 O(V+E)",pitfall:"original_edge は代表元辺を一つ保持します。最小辺 ID や、重みの最小値・合計を保証するものではありません。"},
    lowlink:{file:"low_link",title:"部分木から戻れる最小の発見番号を求める",principle:"ord は発見順。low は部分木を下って戻り辺を高々一本使い、到達できる最小の ord です。子から戻るときに親へ伝播します。",formula:"low[v] = min(ord[v], 戻り辺先の ord, 子の low)\n親辺の逆向きは辺 ID で除外\n時間 O(V+E) / 再帰なし",pitfall:"訪問済み頂点への辺では ord[to] を使います。親頂点への全辺を除外すると、多重辺で誤判定します。"},
    bridges:{file:"bridges",title:"親より上へ戻れない木辺が橋",principle:"子の部分木から親やその祖先へ戻れなければ、その木辺を削除すると成分数が増えます。等号を含めない比較が必要です。",formula:"木辺 (p,v) が橋 ⇔ ord[p] < low[v]\n削除後の成分数 > 元の成分数\n時間 O(V+E)",pitfall:"平行な別の辺で親へ戻れると low[v]=ord[p] になり、橋ではありません。自己ループも橋にはなりません。"},
    articulation:{file:"articulation_points",title:"親より上へ戻れない部分木が残る頂点",principle:"根以外は、ある子の部分木から自分より上に戻れないと関節点です。DFS の根は子が 2 個以上あるときに限ります。",formula:"根以外: ある子 v で ord[p] ≤ low[v]\nDFS の根: DFS 木の子が 2 個以上\n時間 O(V+E)",pitfall:"子の数は元グラフの次数ではありません。孤立点を削除して成分数が減っても、関節点にはなりません。"},
    bipartite:{file:"bipartite",title:"全ての辺を異なる色で結ぶ",principle:"各連結成分の根を色 0 とし、隣接先を反対の色にします。同色の端点を結ぶ辺があれば、二部グラフではありません。",formula:"color[to] = color[v] XOR 1\n成功: vector<int>（0/1） / 失敗: nullopt\n時間 O(V+E)",pitfall:"非連結でも全成分を調べます。自己ループは失敗、多重辺だけでは失敗しません。図の衝突辺は、奇閉路全体の復元結果ではありません。"},
    components:{file:"connected_components",title:"探索ごとに連結成分の番号を付ける",principle:"未訪問の頂点を根にして到達範囲へ同じ番号を付けます。次の根は別成分です。孤立点も一つの成分になります。",formula:"same(u,v) ⇔ component[u] = component[v]\nsize() は成分数\n時間 O(V+E)",pitfall:"この関数は無向グラフ用です。有向グラフで相互に届く範囲は SCC を使います。図は探索の原理を BFS で示します。"}
  };
  const directed=()=>["scc","condensation"].includes(mode),lowmode=()=>["lowlink","bridges","articulation"].includes(mode);
  const defaultDirected="0 1\n1 2\n2 0\n2 3\n0 3\n3 4\n4 3\n4 5\n6 6";
  const defaultLow="0 1\n1 2\n2 0\n1 3\n3 4\n3 4\n3 5";
  const palette=["#d6ece0","#ffe4c6","#dce5fa","#f5dcec","#e5edca","#d7ebf1","#eadff2","#eae5db"];
  const esc=x=>String(x).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
  let mode="scc",frames=[],position=0,timer=null;
  const integer=(x,label)=>{if(!String(x).trim())throw new Error(`${label}には整数を入力してください。`);const v=Number(x);if(!Number.isSafeInteger(v))throw new Error(`${label}には整数を入力してください。`);return v;};
  function input(){
    const n=integer($("vertices").value,"頂点数");if(n<0||n>8)throw new Error("頂点数は 0〜8 です。");
    const lines=$("edges").value.trim()?$("edges").value.trim().split(/\n+/):[];if(lines.length>16)throw new Error("辺は最大 16 本です。");
    const edges=lines.map(line=>{const a=line.trim().split(/[\s,]+/).map(x=>integer(x,"辺の端点"));if(a.length!==2)throw new Error("1 行に端点 u v の 2 個を入力してください。");if(a.some(v=>v<0||v>=n))throw new Error("辺の頂点は 0 以上、頂点数未満です。");return a;});
    return {n,edges,deduplicate:$("deduplicate").checked};
  }
  function stop(){if(timer)clearInterval(timer);timer=null;$("play").textContent="再生 ▶";}
  function run(inspect=false){
    try{
      const data=input();if(inspect){const kind=$("deletion-kind").value,id=integer($("deletion-id").value,"削除番号"),limit=kind==="edge"?data.edges.length:data.n;if(id<0||id>=limit)throw new Error("削除番号がグラフの範囲外です。");data.remove={kind,id};}
      const result=M.trace(mode,data);stop();frames=result.frames;position=inspect?frames.findIndex(f=>f.phase==="deletion"):0;$("input-error").textContent="";render();
    }catch(e){$("input-error").textContent=e.message;}
  }
  function scenario(type="regular"){
    $("vertices").value="7";$("deduplicate").checked=true;$("deletion-kind").value=mode==="articulation"?"vertex":"edge";$("deletion-id").value="3";
    $("edges").value=directed()?defaultDirected:mode==="bipartite"?"0 1\n1 2\n2 3\n3 0\n4 5":defaultLow;
    if(type==="chain"){$("vertices").value="5";$("edges").value="0 1\n1 2\n2 3\n3 4";$("deletion-id").value="1";}
    if(type==="root"){$("vertices").value="4";$("edges").value="0 1\n0 2\n0 3";$("deletion-id").value="0";}
    if(type==="parallel"){$("vertices").value="2";$("edges").value="0 1\n0 1";$("deletion-id").value="0";}
    if(type==="loop"){$("vertices").value="3";$("edges").value="0 0\n1 2";$("deletion-id").value="0";}
    if(type==="odd"){$("vertices").value="3";$("edges").value="0 1\n1 2\n2 0";}
    if(type==="oneway"){$("vertices").value="3";$("edges").value="0 1\n1 2";}
    if(type==="empty"){$("vertices").value="0";$("edges").value="";}
    run();
  }
  function select(next){
    mode=config[next]?next:"scc";stop();for(const b of document.querySelectorAll("[data-mode]"))b.setAttribute("aria-pressed",String(b.dataset.mode===mode));
    const c=config[mode];$("kind").value=directed()?"有向":"無向";$("dedup-field").hidden=mode!=="condensation";$("deletion-controls").hidden=!lowmode();$("second-pass").hidden=!directed();
    $("principle-title").textContent=c.title;for(const key of ["principle","formula","pitfall"])$(key).textContent=c[key];
    $("docs-link").href=`../../docs/graph_connectivity/${c.file}.md`;$("code-link").href=`../../graph_connectivity/${c.file}.hpp`;
    const choices=[["regular","基本の例"],...(directed()?[["oneway","一方向の鎖"]]:[["chain","鎖と切れ目"],["root","根に 3 個の子"]]),["parallel","平行な 2 辺"],["loop","自己ループ・別成分"],...(!directed()?[["odd","三角形の奇閉路"]]:[]),["empty","空グラフ"]];
    $("scenario-type").innerHTML=choices.map(([v,t])=>`<option value="${v}">${t}</option>`).join("");scenario();
  }
  const text=(x,y,value,cls="caption",anchor="start")=>`<text x="${x}" y="${y}" class="${cls}" text-anchor="${anchor}">${esc(value)}</text>`;
  function drawing(s){
    let svg='<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0L10 5L0 10Z" fill="context-stroke"/></marker></defs>';
    const pos=Array.from({length:s.n},(_,i)=>[410+170*Math.cos(-Math.PI/2+i*2*Math.PI/s.n),220+150*Math.sin(-Math.PI/2+i*2*Math.PI/s.n)]),pairs=new Map();
    svg+=text(30,28,s.reverse?"逆グラフ（元の辺 ID の向きを反転）":s.directed?"元の有向グラフ":"無向グラフ（辺 ID は両方向で共通）","heading");
    s.edges.forEach(([u,v],id)=>{const key=[Math.min(u,v),Math.max(u,v)].join(',');if(!pairs.has(key))pairs.set(key,[]);pairs.get(key).push(id);});
    s.edges.forEach(([originalU,originalV],id)=>{
      const[u,v]=s.reverse?[originalV,originalU]:[originalU,originalV],[x1,y1]=pos[u],[x2,y2]=pos[v],list=pairs.get([Math.min(u,v),Math.max(u,v)].join(',')),slot=list.indexOf(id);let path,lx,ly;
      if(u===v){const rise=62+slot*18;path=`M${x1-15} ${y1-17}C${x1-54} ${y1-rise},${x1+54} ${y1-rise},${x1+15} ${y1-17}`;lx=x1;ly=y1-rise+4;}
      else{const dx=x2-x1,dy=y2-y1,len=Math.hypot(dx,dy),ux=dx/len,uy=dy/len,offset=(slot-(list.length-1)/2)*40*(u<v?1:-1),cx=(x1+x2)/2-uy*offset,cy=(y1+y2)/2+ux*offset;path=`M${x1+ux*24} ${y1+uy*24}Q${cx} ${cy},${x2-ux*27} ${y2-uy*27}`;lx=(x1+2*cx+x2)/4;ly=(y1+2*cy+y2)/4-5;}
      const removed=s.removal&&(s.removal.kind==="edge"?s.removal.id===id:u===s.removal.id||v===s.removal.id),tree=s.parentEdge.includes(id)&&(!s.directed||["forward","reverse"].includes(s.phase));
      const cls=`graph-edge${tree?" tree":""}${s.bridgeEdges.includes(id)||s.conflictEdge===id?" critical":""}${s.activeEdge===id&&s.conflictEdge!==id?" active":""}${removed?" removed":""}`;
      svg+=`<path class="${cls}" d="${path}" marker-end="${s.directed?'url(#arrow)':"none"}"/>${text(lx,ly,`#${id}`,"edge-label","middle")}`;
    });
    pos.forEach(([x,y],v)=>{
      const removed=s.removal?.kind==="vertex"&&s.removal.id===v,active=s.activeVertex===v,componentColored=s.directed||mode==="components"||s.phase==="deletion",conflict=s.conflictEdge>=0&&s.edges[s.conflictEdge].includes(v),critical=s.articulation.includes(v)||conflict;
      let fill="white";if(mode==="bipartite"&&s.colors[v]>=0)fill=palette[s.colors[v]];else if(componentColored&&s.component[v]>=0)fill=palette[s.component[v]%palette.length];else if(s.visit[v])fill=s.visit[v]===2?"#d6ece0":"#e1e6e1";
      svg+=`<circle cx="${x}" cy="${y}" r="23" fill="${removed?"#f2eeeb":fill}" stroke="${critical?"#b1444b":active?"#bd6935":"#729987"}" stroke-width="${critical||active?3:1.5}"/>${text(x,y+6,v,"node-value","middle")}`;
      if(removed)svg+=`<path class="cross" d="M${x-15} ${y-15}L${x+15} ${y+15}M${x+15} ${y-15}L${x-15} ${y+15}"/>`;
      const label=lowmode()?`ord ${s.ord[v]<0?"—":s.ord[v]} / low ${s.low[v]<0?"—":s.low[v]}`:mode==="bipartite"?`色 ${s.colors[v]<0?"—":s.colors[v]}`:`C${s.component[v]<0?"—":s.component[v]}`;
      svg+=text(x,y+44,label,"node-info","middle");
    });
    if(!s.n)svg+=text(410,210,"空グラフ","heading","middle");
    const caption=s.phase==="reverse"?"探索スタック（右が次の処理）":s.phase==="forward"||s.phase==="dfs"?"探索スタック（左が根、右が現在の頂点）":"探索候補（左が次の処理）";
    svg+=text(30,445,caption,"heading");s.work.forEach((v,i)=>{const x=35+(i%8)*96,y=465+Math.floor(i/8)*34;svg+=`<rect class="work-cell" x="${x}" y="${y}" width="88" height="27" rx="6"/>${text(x+44,y+18,v,"work-value","middle")}`;});if(!s.work.length)svg+=text(40,484,"空列");
    let height=Math.max(530,490+Math.ceil(s.work.length/8)*34);
    if(mode==="condensation"&&["scc-done","condense","done"].includes(s.phase)){
      svg+=text(30,height+5,"縮約 DAG：成分 ID はトポロジカル順","heading");const cy=height+110;
      s.dagArcs.forEach((e,i)=>{const x1=100+e.from*90,x2=100+e.to*90,lift=48+(e.to-e.from)*8+i*3;svg+=`<path class="dag-edge" d="M${x1+21} ${cy-4}Q${(x1+x2)/2} ${cy-lift},${x2-24} ${cy-4}" marker-end="url(#arrow)"/>${text((x1+x2)/2,cy-lift/2-5,`#${i}`,"edge-label","middle")}`;});
      s.groups.forEach((group,c)=>{const x=100+c*90;svg+=`<circle cx="${x}" cy="${cy}" r="22" fill="${palette[c%palette.length]}" stroke="#729987"/>${text(x,cy+5,`C${c}`,"node-value","middle")}${text(x,cy+42,`${group.length} 頂点`,"caption","middle")}`;});height+=185;
    }
    return [svg,height];
  }
  function render(){
    if(!frames.length)return;const s=frames[position],final=position===frames.length-1,[svg,height]=drawing(s);$("diagram").innerHTML=svg;$("diagram").setAttribute("viewBox",`0 0 840 ${height}`);
    $("step-title").textContent=s.title;$("step-description").textContent=s.detail;
    $("phase-value").textContent={initial:"初期",forward:"正方向 DFS",reverse:"逆方向 DFS","scc-done":"SCC 確定",condense:"縮約",dfs:"DFS",done:"完了",deletion:"仮に削除",search:"探索"}[s.phase];
    $("count-label").textContent=s.removal?"削除後の成分数":"探索した成分数";$("count-value").textContent=s.groups.length;
    $("result-label").textContent=s.removal?"成分数の変化":mode==="condensation"?"縮約辺数":"結果";
    $("result").textContent=s.removal?`${s.before}→${s.after??"?"}`:mode==="condensation"?s.dagArcs.length:mode==="bipartite"?final?s.bipartite?"二部":"非二部":"判定中":mode==="bridges"?`${s.bridgeEdges.length} 本`:mode==="articulation"?`${s.articulation.length} 個`:mode==="lowlink"?`${s.bridgeEdges.length} 橋 / ${s.articulation.length} 関節`:final?`${s.groups.length} 成分`:"処理中";
    $("rule").hidden=!s.rule;
    if(s.rule){const r=s.rule;$("rule").textContent=r.root?`根 ${r.vertex}: DFS の子 ${r.children} > 1 → ${r.cut?"関節点":"関節点でない"}`:`親 p=${r.parent}, 子 v=${r.child}\n橋: ord[p]=${r.parentOrd} < low[v]=${r.childLow} → ${r.bridge}\n${r.parentIsRoot?"親は DFS の根なので、関節点は子の個数で別に判定":`関節点: ord[p]=${r.parentOrd} ≤ low[v]=${r.childLow} → ${r.cut}`}`;}
    const headings=lowmode()?["頂点","ord","low","親 / 辺","子の数","部分木サイズ"]:mode==="bipartite"?["頂点","色","成分"]:directed()?["頂点","成分","DFS 終了位置"]:["頂点","成分"];
    $("table-title").textContent=lowmode()?"元のグラフの ord / low":mode==="bipartite"?"0/1 の彩色":"成分と終了順";
    $("table-note").textContent=directed()?`finish: ${s.finish.join(", ")||"—"} / 逆順: ${s.order.join(", ")||"—"}`:s.removal?"ord/low は削除前の値です。成分の色分けと一覧は削除後を示します。":mode==="bipartite"?"衝突時は途中の色も表示します。C++ の失敗時の戻り値は nullopt です。":"未発見は —。根の子は DFS 木の子の数です。";
    $("table-head").innerHTML=`<tr>${headings.map(v=>`<th>${v}</th>`).join("")}</tr>`;
    $("values").innerHTML=Array.from({length:s.n},(_,v)=>`<tr class="${v===s.activeVertex?"active":""}" data-component="${s.component[v]}" data-ord="${s.ord[v]}" data-low="${s.low[v]}" data-color="${s.colors[v]}"><td>${v}${s.component[v]===-2?" (削除)":""}</td>${lowmode()?`<td>${s.ord[v]<0?"—":s.ord[v]}</td><td>${s.low[v]<0?"—":s.low[v]}</td><td>${s.parent[v]} / ${s.parentEdge[v]}</td><td>${s.children[v]}</td><td>${s.subtree[v]}</td>`:mode==="bipartite"?`<td>${s.colors[v]<0?"—":s.colors[v]}</td><td>${s.component[v]<0?"—":s.component[v]}</td>`:`<td>${s.component[v]<0?"—":s.component[v]}</td>${directed()?`<td>${s.finish.includes(v)?s.finish.indexOf(v):"—"}</td>`:""}`}</tr>`).join("");
    $("groups-title").textContent=s.removal?"削除後の連結成分":"成分の頂点";$("groups").innerHTML=s.groups.map((group,c)=>`<span style="background:${palette[c%palette.length]}">C${c}: ${group.join(", ")}</span>`).join("");
    $("outputs").textContent=lowmode()?`橋 ID: ${s.bridgeEdges.join(", ")||"なし"}\n関節点: ${s.articulation.join(", ")||"なし"}`:mode==="bipartite"?`色 0: ${s.colors.map((c,i)=>c===0?i:-1).filter(i=>i>=0).join(", ")||"なし"}\n色 1: ${s.colors.map((c,i)=>c===1?i:-1).filter(i=>i>=0).join(", ")||"なし"}`:`${s.groups.length} 成分${directed()?" / 異なる成分の辺は ID の小→大":""}`;
    $("dag-table-wrap").hidden=mode!=="condensation";$("dag-values").innerHTML=s.dagArcs.map((e,i)=>`<tr><td>${i}</td><td>${e.from}</td><td>${e.to}</td><td>${e.original}</td></tr>`).join("");
    $("progress").textContent=`${position+1} / ${frames.length}`;$("previous").disabled=position===0;$("next").disabled=final;$("finish").disabled=final;$("second-pass").disabled=!frames.some(f=>f.phase==="reverse");
  }
  $("run").onclick=()=>run();$("inspect").onclick=()=>run(true);$("scenario").onclick=()=>scenario($("scenario-type").value);
  $("previous").onclick=()=>{stop();position=Math.max(0,position-1);render();};$("next").onclick=()=>{stop();position=Math.min(frames.length-1,position+1);render();};$("finish").onclick=()=>{stop();position=frames.length-1;render();};
  $("second-pass").onclick=()=>{stop();const index=frames.findIndex(f=>f.phase==="reverse");if(index>=0)position=index;render();};
  $("play").onclick=()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 ■";timer=setInterval(()=>{if(position<frames.length-1){++position;render();}else stop();},500);};
  for(const b of document.querySelectorAll("[data-mode]"))b.onclick=()=>{if(location.hash.slice(1)===b.dataset.mode)select(b.dataset.mode);else location.hash=b.dataset.mode;};
  window.addEventListener("hashchange",()=>select(location.hash.slice(1)));select(location.hash.slice(1));
})();
