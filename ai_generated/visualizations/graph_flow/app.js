(function(){
  "use strict";
  const $=id=>document.getElementById(id),M=GraphFlowModel;
  const config={
    dinic:{file:"dinic",title:"最短の層に沿って、まとめて流す",principle:"残余容量が正の辺で BFS を行い、level が一つ増える辺で増加路を探します。その層で流せなくなったら BFS をやり直します。逆辺を通る操作は、元辺の流量を取り消します。",formula:"正方向の残余容量 = capacity − flow\n逆方向の残余容量 = flow\n一般の整数容量: O(V²E)、記憶 O(V+E)",pitfall:"max_flow の戻り値は今回追加した量です。上限で止めた段階では最大流とは限りません。同じネットワークで始点・終点を変える前に reset_flow() を呼びます。"},
    cut:{file:"min_cut",title:"残余辺で届く範囲が、最小カットの片側",principle:"最大流まで流し切り、残余ネットワークで始点から届く頂点を S、残りを T にします。元の有向辺の S→T を集めると、その容量の合計が最大流に一致します。",formula:"最大流量 = 最小 s-t カット容量\nS→T の元辺のみを復元（容量 0 の辺も含む）\n流し切った後の復元: O(V+E)",pitfall:"minimum_cut は流れを最大まで追加してから復元します。元辺の T→S や内部の残余逆辺はカット辺に含めません。図のこのモードでも流量上限を使わず最後まで計算します。"},
    matching:{file:"bipartite_matching",title:"交互路を反転して、組を一つ増やす",principle:"未マッチの左頂点から、未採用辺を左→右、採用辺を右→左に辿ります。未マッチの右へ届けば、経路の辺を交互に採用・解除して組数を増やせます。Hopcroft–Karp は同じ最短の層で複数の交互路を探します。",formula:"左右それぞれ 0 始まり / 未マッチ = −1\ncardinality / left_match / right_match / pairs\nO((V+E)√(V+1))、記憶 O(V+E)",pitfall:"二部グラフであることと左右の分割は入力側で与えます。3.8 の二部判定とは別の操作です。最大の組数を保証し、相手の辞書順最小は保証しません。"}
  };
  let mode="dinic",frames=[],position=0,timer=null;
  const esc=x=>String(x).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
  const integer=(x,label,min,max)=>{if(!String(x).trim())throw new Error(`${label}を入力してください。`);const v=Number(x);if(!Number.isSafeInteger(v)||v<min||v>max)throw new Error(`${label}は ${min}〜${max} の整数です。`);return v;};
  function input(){
    const matching=mode==="matching",data=matching?{leftSize:integer($("left-size").value,"左の頂点数",0,6),rightSize:integer($("right-size").value,"右の頂点数",0,6)}:{n:integer($("vertices").value,"頂点数",2,8)};
    if(!matching){data.source=integer($("source").value,"始点",0,data.n-1);data.sink=integer($("sink").value,"終点",0,data.n-1);if(data.source===data.sink)throw new Error("始点と終点は異なる頂点にしてください。");data.limit=mode==="cut"||!$("limit").value.trim()?null:integer($("limit").value,"流量上限",0,1000000);}
    const lines=$("edges").value.trim()?$("edges").value.trim().split(/\n+/):[];if(lines.length>18)throw new Error("辺は最大 18 本です。");
    data.edges=lines.map(line=>{const a=line.trim().split(/[\s,]+/);if(a.length!==(matching?2:3))throw new Error(matching?"1 行に left right の 2 個です。":"1 行に u v capacity の 3 個です。");return matching?[integer(a[0],"左頂点",0,data.leftSize-1),integer(a[1],"右頂点",0,data.rightSize-1)]:[integer(a[0],"辺の頂点",0,data.n-1),integer(a[1],"辺の頂点",0,data.n-1),integer(a[2],"容量",0,99)];});return data;
  }
  function stop(){if(timer)clearInterval(timer);timer=null;$("play").textContent="再生 ▶";}
  function run(){try{const result=M.trace(mode,input());stop();frames=result.frames;position=0;$("input-error").textContent="";render();}catch(e){$("input-error").textContent=e.message;}}
  function scenario(type="regular"){
    $("limit").value="";$("vertices").value=6;$("source").value=0;$("sink").value=5;$("left-size").value=3;$("right-size").value=3;
    $("edges").value=mode==="matching"?"0 0\n0 1\n1 0\n2 1\n2 2":"0 1 1\n0 2 1\n1 3 1\n1 4 1\n2 3 1\n3 5 1\n4 5 1";
    if(mode==="matching"){
      if(type==="disconnected"){$("left-size").value=4;$("right-size").value=3;$("edges").value="0 0\n1 1";}
      if(type==="parallel")$("edges").value="0 0\n0 0\n1 0\n2 2";
      if(type==="empty"){$("left-size").value=0;$("right-size").value=0;$("edges").value="";}
      if(type==="unequal"){$("left-size").value=4;$("right-size").value=2;$("edges").value="0 0\n1 0\n2 1\n3 1";}
    }else{
      if(type==="capacities"){$("vertices").value=4;$("sink").value=3;$("edges").value="0 1 3\n0 2 2\n1 2 1\n1 3 2\n2 3 3";}
      if(type==="disconnected"){$("vertices").value=4;$("sink").value=3;$("edges").value="0 1 3\n2 3 4";}
      if(type==="parallel"){$("vertices").value=3;$("sink").value=2;$("edges").value="0 1 2\n0 1 3\n1 0 4\n1 1 8\n1 2 4";}
      if(type==="zero"){$("vertices").value=3;$("sink").value=2;$("edges").value="0 1 0\n0 2 0\n1 2 3";}
      if(type==="empty"){$("vertices").value=2;$("sink").value=1;$("edges").value="";}
      if(type==="limited")$("limit").value=1;
    }
    run();
  }
  function select(next){mode=config[next]?next:"dinic";stop();const matching=mode==="matching",c=config[mode];for(const b of document.querySelectorAll('[data-mode]'))b.setAttribute('aria-pressed',String(b.dataset.mode===mode));
    $("flow-fields").hidden=matching;$("matching-fields").hidden=!matching;$("reverse-field").hidden=matching;$("limit-field").hidden=mode!=="dinic";$("edges-label").textContent=matching?"辺 left right（1行に1辺）":"有向辺 u v capacity（1行に1辺）";
    $("input-note").textContent=matching?"辺は最大 18 本。左右は別の番号です。実行すると空のマッチングから履歴を再生成します。":"容量は 0〜99、辺は最大 18 本。実行すると流れ 0 から履歴を再生成します。";
    $("augment").textContent=matching?"次の交互路へ":"次の増加路へ";$("value-label").textContent=matching?"現在の組数":"現在の流量";$("certificate-label").textContent=matching?"左→右の最短区間数":"最小カット容量";$("legend-extra").textContent=matching?"青破線：組み替えで外す辺":"青破線：逆辺 / 赤：カット辺";
    $("principle-title").textContent=c.title;for(const id of ["principle","formula","pitfall"])$(id).textContent=c[id];$("docs-link").href=`../../docs/graph_flow/${c.file}.md`;$("code-link").href=`../../graph_flow/${c.file}.hpp`;
    const choices=[["regular",matching?"組み替えが必要な例":"逆辺が必要な例"],...(matching?[["unequal","左右の頂点数が違う"]]:[["capacities","容量とボトルネック"]]),["disconnected","非連結・孤立点"],["parallel",matching?"重複する辺":"平行辺・逆向き辺・ループ"],...(!matching?[["zero","容量 0 のカット辺"]]:[]),...(mode==="dinic"?[["limited","流量上限で停止"]]:[]),["empty","辺のないグラフ"]];$("scenario-type").innerHTML=choices.map(([v,t])=>`<option value="${v}">${t}</option>`).join("");scenario();
  }
  const text=(x,y,value,cls="caption",anchor="start")=>`<text x="${x}" y="${y}" class="${cls}" text-anchor="${anchor}">${esc(value)}</text>`;
  const defs='<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M0 0L10 5L0 10Z" fill="context-stroke"/></marker></defs>';
  function curve(a,b,offset,loop=0){const[x1,y1]=a,[x2,y2]=b;if(x1===x2&&y1===y2){const h=60+loop*18;return {d:`M${x1-15} ${y1-17}C${x1-52} ${y1-h},${x1+52} ${y1-h},${x1+15} ${y1-17}`,x:x1,y:y1-h+4};}const dx=x2-x1,dy=y2-y1,length=Math.hypot(dx,dy),ux=dx/length,uy=dy/length,cx=(x1+x2)/2-uy*offset,cy=(y1+y2)/2+ux*offset;return {d:`M${x1+ux*25} ${y1+uy*25}Q${cx} ${cy},${x2-ux*28} ${y2-uy*28}`,x:.62*.62*x1+2*.62*.38*cx+.38*.38*x2,y:.62*.62*y1+2*.62*.38*cy+.38*.38*y2-6};}
  function drawing(s){let svg=defs;const matching=mode==="matching";
    if(!matching){
      const pos=Array(s.n),other=Array.from({length:s.n},(_,v)=>v).filter(v=>v!==s.source&&v!==s.sink),rows=Math.ceil(other.length/2);pos[s.source]=[70,230];pos[s.sink]=[770,230];other.forEach((v,i)=>pos[v]=[i<rows?270:550,rows===1?230:110+(i%rows)*230/(rows-1)]);
      svg+=text(25,28,"元辺: #ID flow/capacity　/　青破線: 逆辺の残余容量","heading");const groups=new Map();s.edges.forEach(([u,v],id)=>{const key=[Math.min(u,v),Math.max(u,v)].join(',');if(!groups.has(key))groups.set(key,[]);groups.get(key).push(id);});
      s.edges.forEach(([u,v,c],id)=>{const group=groups.get([Math.min(u,v),Math.max(u,v)].join(',')),slot=group.indexOf(id),offset=(slot-(group.length-1)/2)*46*(u<v?1:-1),g=curve(pos[u],pos[v],offset,slot),active=s.path.includes(2*id),cut=s.cutEdges.includes(id);svg+=`<path data-edge="${id}" class="graph-edge${s.flows[id]?" used":""}${cut?" cut":""}${active?" active":""}" d="${g.d}" marker-end="url(#arrow)"/>${text(g.x,g.y,`#${id} ${s.flows[id]}/${c}`,"edge-label","middle")}`;
        if($("show-reverse").checked&&s.flows[id]>0){const reverse=curve(pos[v],pos[u],-offset+38*(u<v?1:-1),slot);svg+=`<path data-reverse="${id}" class="graph-edge reverse${s.path.includes(2*id+1)?" active":""}" d="${reverse.d}" marker-end="url(#arrow)"/>${text(reverse.x,reverse.y,`↶#${id} ${s.flows[id]}`,"edge-label reverse-label","middle")}`;}
      });
      pos.forEach(([x,y],v)=>{const partition=["reach","done"].includes(s.phase),fill=partition?s.sourceSide[v]?"#d6ece0":"#f4dfe4":s.work.includes(v)?"#fff0dc":"white";svg+=`<circle cx="${x}" cy="${y}" r="24" fill="${fill}" stroke="#729987" stroke-width="2"/>${text(x,y+6,v,"node-value","middle")}${text(x,y+44,`${v===s.source?'s / ':v===s.sink?'t / ':''}${partition?s.sourceSide[v]?'S':'T':`level ${s.level[v]<0?'—':s.level[v]}`}`,"node-info","middle")}`;});
    }else{
      svg+=text(35,28,"未採用辺を左→右、採用辺を右→左。経路の色は反転前の追加／削除。","heading");const point=(side,v)=>[side?630:210,(side?s.rightSize:s.leftSize)===1?230:75+v*300/Math.max(1,(side?s.rightSize:s.leftSize)-1)];
      s.edges.forEach(([u,v],id)=>{const onPath=s.path.find(e=>e.left===u&&e.right===v),used=s.leftMatch[u]===v,a=point(0,u),b=point(1,v),g=curve(onPath?.matched?b:a,onPath?.matched?a:b,0);svg+=`<path data-edge="${id}" class="graph-edge${used?' used':''}${onPath?onPath.matched?' reverse':' active':''}" d="${g.d}" marker-end="${onPath?'url(#arrow)':'none'}"/>`;});
      for(const side of [0,1])for(let v=0;v<(side?s.rightSize:s.leftSize);++v){const[x,y]=point(side,v),mate=(side?s.rightMatch:s.leftMatch)[v];svg+=`<circle cx="${x}" cy="${y}" r="24" fill="${mate<0?'white':'#d6ece0'}" stroke="#729987" stroke-width="2"/>${text(x,y+6,`${side?'R':'L'}${v}`,"node-value","middle")}${text(x+(side?42:-42),y+4,side?`相手 ${mate<0?'—':`L${mate}`}`:`d=${s.distance[v]<0?'—':s.distance[v]} / 相手 ${mate<0?'—':`R${mate}`}`,"node-info",side?'start':'end')}`;}
      if(!s.leftSize&&!s.rightSize)svg+=text(420,230,"空の二部グラフ","heading","middle");
    }
    const caption=s.phase==="bfs"?"BFS の発見列（薄色は処理済み）":s.phase==="reach"?"始点側へ到達した順":"探索スタック（右が現在の頂点）";svg+=text(25,425,caption,"heading");s.work.forEach((v,i)=>{const x=28+i%8*98,y=445+Math.floor(i/8)*34;svg+=`<g class="${s.phase==='bfs'&&i<s.head?'processed':''}"><rect class="work-cell" x="${x}" y="${y}" width="90" height="27" rx="6"/>${text(x+45,y+18,`${matching?'L':''}${v}`,"work-value","middle")}</g>`;});if(!s.work.length)svg+=text(35,465,"空列");return [svg,Math.max(500,475+Math.ceil(s.work.length/8)*34)];
  }
  function render(){if(!frames.length)return;const s=frames[position],matching=mode==="matching",[svg,height]=drawing(s);$("diagram").innerHTML=svg;$("diagram").setAttribute('viewBox',`0 0 840 ${height}`);$("result").textContent=matching?`${s.cardinality} 組`:String(s.total);$("certificate").textContent=matching?s.shortest??"—":s.cutCapacity??(s.phase==="done"?"未確定":"—");$("phase-value").textContent={initial:"初期",bfs:"BFS",dfs:"層に沿う探索",path:"経路発見",augment:matching?"組を更新":"流れを更新",reach:"到達範囲",done:"完了"}[s.phase];$("step-title").textContent=s.title;$("step-description").textContent=s.detail;
    $("path-summary").hidden=!s.path.length;$("path-summary").textContent=matching?s.path.map(e=>`${e.matched?'−':'＋'} L${e.left}—R${e.right}`).join(' / '):s.path.map(id=>`#${id>>1}${id%2?' 逆向き':' 正方向'}`).join(' → ')+(s.delta?`\n今回流す量: ${s.delta}`:'');
    $("table-title").textContent=matching?"左の相手と探索の層":"元辺と残余容量";$("table-note").textContent=matching?"相手 −1 は未マッチ。distance −1 は未発見またはこの段階で使い切った頂点。左右の同じ番号は別の頂点です。":"容量・流量は元辺ごとに表示します。逆向きの元辺と残余逆辺は別々です。level −1 は未発見またはこの段階で使い切った頂点。";
    const headings=matching?["左","右の相手","distance","次の候補位置"]:["元辺 ID","from→to","容量 c","流量 f","正向き c−f","逆向き f"];
    $("table-head").innerHTML=`<tr>${headings.map(t=>`<th>${t}</th>`).join('')}</tr>`;
    $("values").innerHTML=matching?s.leftMatch.map((v,u)=>`<tr data-match="${v}"><td>L${u}</td><td>${v<0?'−1':`R${v}`}</td><td>${s.distance[u]}</td><td>${s.next[u]}</td></tr>`).join(''):s.edges.map(([u,v,c],id)=>`<tr data-flow="${s.flows[id]}" data-cut="${s.cutEdges.includes(id)}"><td>#${id}</td><td>${u}→${v}</td><td>${c}</td><td>${s.flows[id]}</td><td>${c-s.flows[id]}</td><td>${s.flows[id]}</td></tr>`).join('');
    $("secondary-title").textContent=matching?"右の相手":"探索の層と current arc";$("secondary-head").innerHTML=matching?"<tr><th>右</th><th>左の相手</th></tr>":"<tr><th>頂点</th><th>level</th><th>次の候補位置</th><th>始点側 S</th></tr>";$("secondary-values").innerHTML=matching?s.rightMatch.map((u,v)=>`<tr><td>R${v}</td><td>${u<0?'−1':`L${u}`}</td></tr>`).join(''):s.level.map((level,v)=>`<tr><td>${v}</td><td>${level}</td><td>${s.next[v]}</td><td>${['reach','done'].includes(s.phase)?s.sourceSide[v]?'S':'T':'—'}</td></tr>`).join('');
    $("outputs").textContent=matching?`現在の組: ${s.leftMatch.flatMap((v,u)=>v<0?[]:[`L${u}—R${v}`]).join(', ')||'なし'}`:s.cutCapacity!=null?`S: ${s.sourceSide.flatMap((side,v)=>side?[v]:[]).join(', ')}\nカット辺 ID: ${s.cutEdges.join(', ')||'なし'}\n最大流 ${s.total} = カット容量 ${s.cutCapacity}`:s.phase==='done'?"上限で停止。終点へ残余辺で届くため、最小カットは未確定です。":"流し切った後に、始点側とカット辺を復元します。";
    $("progress").textContent=`${position+1} / ${frames.length}`;$("previous").disabled=position===0;$("next").disabled=position===frames.length-1;$("augment").disabled=position===frames.length-1;
  }
  function move(next){stop();position=Math.max(0,Math.min(frames.length-1,next));render();}
  $("run").onclick=run;$("scenario").onclick=()=>scenario($("scenario-type").value);$("previous").onclick=()=>move(position-1);$("next").onclick=()=>move(position+1);$("finish").onclick=()=>move(frames.length-1);$("show-reverse").onchange=render;
  $("augment").onclick=()=>{const i=frames.findIndex((s,i)=>i>position&&s.phase==='path');move(i<0?frames.length-1:i);};
  $("play").onclick=()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 ⏸";timer=setInterval(()=>{if(position<frames.length-1){++position;render();}else stop();},550);};
  for(const b of document.querySelectorAll('[data-mode]'))b.onclick=()=>{if(location.hash.slice(1)===b.dataset.mode)select(b.dataset.mode);else location.hash=b.dataset.mode;};window.addEventListener('hashchange',()=>select(location.hash.slice(1)));select(location.hash.slice(1));
})();
