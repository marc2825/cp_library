(function(){
  "use strict";
  const $=id=>document.getElementById(id),M=OfflineQueryModel;
  const settings={
    mo:{file:"mo",title:"区間を少しずつ動かす",principle:"静的な区間の答えを、両端の追加・削除だけで更新します。左端をブロックに分けて順序を変えると、区間の移動を共有できます。",formula:"B ≈ N / √Q\n移動回数 O(N²/B + QB + N)",pitfall:"図は異なる値の数を集計します。転倒数などでは左右の追加・削除を別々に定義します。空区間の答えは 0 です。",label:"区間 l r（1 行に 1 問い合わせ）",note:"0 始まり、[l,r)。空区間可。最大 10 件。値は ±100 万以内。"},
    pbs:{file:"parallel_binary_search",title:"同じ mid で状態を共有する",principle:"各問い合わせの二分探索を進め、mid が同じものを一度の更新列走査で判定します。毎ラウンド reset し、時刻 t は最初の t 回の更新を適用した状態です。",formula:"low = -1, high = M+1\nfalse … false | true … true\n答え: 0..M / M+1（該当なし）",pitfall:"図の判定は頂点間の連結です。辺を追加するだけなので単調です。辺の削除を混ぜると、この条件は保たれません。",label:"連結を調べる頂点 u v",note:"頂点は 0 始まり。最大 16 辺・10 問い合わせ。同じ頂点なら答えは 0。"},
    sweep:{file:"sweep_line",title:"片方の座標を走査する",principle:"x の小さい点から追加し、y の重みを BIT に保持します。各長方形で x<right の寄与を加え、x<left の寄与を引きます。",formula:"G(x,d,u) = Σ(px<x, d≤py<u) w\n答え = G(right,down,up) − G(left,down,up)",pitfall:"境界と同じ x の点は、その境界の問い合わせより後に追加します。座標から 1 を引く必要がありません。",label:"長方形 left down right up",note:"[left,right) × [down,up)。最大 14 点・8 問い合わせ。座標・重みは ±100 万以内。"},
    cdq:{file:"cdq_rectangle_sum",title:"過去から未来への寄与を分割する",principle:"操作の時刻順を二分し、左側の点追加が右側の問い合わせへ与える寄与を Sweep Line で集計します。追加した重みを巻き戻してから次の分割へ進みます。",formula:"F(x,y) = Σ(px<x, py<y) w\n答え = F(r,u) − F(l,u) − F(r,d) + F(l,d)\n時間 O(E log²E), E = P + 4Q（空区間を除く）",pitfall:"時刻列は展開後のイベント順を示します。図で薄い点には未来の更新も含まれ、BIT に入るのは現在の左側の対象点だけです。表示中の答えは途中の寄与です。",label:"操作（add x y w / sum l d r u）",note:"初期点は全操作より前に追加。最大 6 初期点・18 操作。座標・重みは ±100 万以内。"}
  };
  const defaults={mo:"0 3\n1 7\n2 2\n4 8\n0 8",pbs:"0 0\n0 3\n0 4\n0 5",sweep:"1 2 3 3\n0 0 4 5\n3 2 3 3",cdq:"sum 0 0 4 5\nadd 1 2 -2\nsum 1 2 3 3\nadd 0 0 5\nsum 0 0 4 5"};
  let mode="mo",frames=[],position=0,timer=null;
  const esc=x=>String(x).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
  const integer=(text,label)=>{if(!String(text).trim())throw new Error(`${label}には整数を入力してください。`);const v=Number(text);if(!Number.isInteger(v)||Math.abs(v)>1000000)throw new Error(`${label}は ±100 万以内の整数です。`);return v;};
  const numbers=text=>text.trim()?text.trim().split(/[\s,]+/).map(x=>integer(x,"値")):[];
  const rows=(id,columns,limit)=>{const lines=$(id).value.trim()?$(id).value.trim().split(/\n+/):[];if(lines.length>limit)throw new Error(`最大 ${limit} 行にしてください。`);return lines.map(line=>{const a=numbers(line);if(a.length!==columns)throw new Error(`1 行に ${columns} 個の数値を入力してください。`);return a;});};
  function input(){
    if(mode==="mo"){
      const array=numbers($("array").value),queries=rows("queries",2,10),width=integer($("width").value,"ブロック幅");
      if(array.length>16)throw new Error("配列は最大 16 要素です。");
      if(width<0)throw new Error("ブロック幅は 0 以上です。");
      if(queries.some(([l,r])=>l<0||l>r||r>array.length))throw new Error("区間は 0 ≤ l ≤ r ≤ 配列長です。");
      return {array,queries,width};
    }
    if(mode==="pbs"){
      const n=integer($("vertices").value,"頂点数"),edges=rows("edges",2,16),queries=rows("queries",2,10);
      if(n<1||n>10)throw new Error("頂点数は 1〜10 です。");
      if([...edges,...queries].some(a=>a.some(x=>x<0||x>=n)))throw new Error("頂点は 0 以上、頂点数未満です。");
      return {n,edges,queries};
    }
    const points=rows("points",3,mode==="sweep"?14:6);
    if(mode==="sweep")return {points,queries:rows("queries",4,8)};
    const lines=$("queries").value.trim()?$("queries").value.trim().split(/\n+/):[];
    if(lines.length>18)throw new Error("操作は最大 18 行です。");
    const operations=lines.map(line=>{
      const [type,...args]=line.trim().split(/[\s,]+/);
      if(!["add","sum"].includes(type)||args.length!==(type==="add"?3:4))throw new Error("操作は add x y w または sum l d r u です。");
      return [type,...args.map(x=>integer(x,"座標・重み"))];
    });
    return {points,operations};
  }
  function stop(){if(timer)clearInterval(timer);timer=null;$("play").textContent="再生 ▶";}
  function run(){try{const result=M.trace(mode,input());stop();frames=result.frames;position=0;$("input-error").textContent="";render();}catch(e){$("input-error").textContent=e.message;}}
  function select(next){
    mode=settings[next]?next:"mo";stop();
    for(const button of document.querySelectorAll("[data-mode]"))button.setAttribute("aria-pressed",String(button.dataset.mode===mode));
    for(const element of document.querySelectorAll("[data-for]"))element.hidden=!element.dataset.for.split(" ").includes(mode);
    const o=settings[mode];$("query-label").textContent=o.label;$("input-note").textContent=o.note;
    $("principle-title").textContent=o.title;$("principle").textContent=o.principle;$("formula").textContent=o.formula;$("pitfall").textContent=o.pitfall;
    $("docs-link").href=`../../docs/offline_query/${o.file}.md`;$("code-link").href=`../../offline_query/${o.file}.hpp`;
    $("queries").value=defaults[mode];run();
  }
  const text=(x,y,value,cls="caption",anchor="start")=>`<text x="${x}" y="${y}" class="${cls}" text-anchor="${anchor}">${esc(value)}</text>`;
  const cell=(x,y,w,h,value,small,active=false,muted=false)=>`<g class="cell${active?" active":""}${muted?" muted":""}"><rect x="${x}" y="${y}" width="${w}" height="${h}" rx="6"/>${text(x+w/2,y+21,value,"value","middle")}${text(x+w/2,y+h-8,small,"caption","middle")}</g>`;
  function drawMo(s){
    let svg=text(30,30,`現在 [${s.l}, ${s.r}) / Q${s.activeId<0?"—":s.activeId}`,"heading");
    const w=760/Math.max(1,s.array.length);
    s.array.forEach((v,i)=>{svg+=cell(35+i*w,65,w-5,58,v,`i=${i}`,s.activePoint===i,!(s.l<=i&&i<s.r));});
    if(!s.array.length)svg+=text(40,95,"空配列");
    svg+=`<path d="M${35+s.l*w} 55v-12h${(s.r-s.l)*w}v12" class="sweep"/>`;
    svg+=text(30,163,`頻度を更新 → 0 ↔ 1 の変化で種類数 ${s.distinct}`,"heading");
    [...new Set(s.array)].sort((a,b)=>a-b).forEach((v,i)=>{const h=(s.frequency[v]||0)*16;svg+=`<rect x="${40+i*45}" y="${240-h}" width="30" height="${h}" fill="#1b7361"/>${text(55+i*45,260,v,"caption","middle")}${text(55+i*45,225-h,s.frequency[v]||0,"caption","middle")}`;});
    svg+=text(30,302,"並べ替えた問い合わせの移動経路（横 l / 縦 r）","heading");
    const scale=v=>v*170/Math.max(1,s.array.length),x0=70,y0=490;
    svg+=`<path d="M${x0} 320v170h210" class="axis"/>`;
    svg+=`<polyline class="query-path" points="${s.order.map(id=>`${x0+scale(s.queries[id][0])},${y0-scale(s.queries[id][1])}`).join(" ")}"/>`;
    s.order.forEach((id,step)=>{const [l,r]=s.queries[id],x=x0+scale(l),y=y0-scale(r);svg+=`<circle cx="${x}" cy="${y}" r="6" fill="${id===s.activeId?"#bd6935":"#1b7361"}"/>${text(x+8,y-8,`Q${id}`)}`;svg+=text(330,333+step*17,`${step+1}. Q${id}: [${l}, ${r})${s.done.includes(id)?" ✓":""}`);});
    return [svg,520];
  }
  function drawPBS(s){
    let svg=text(30,28,`ラウンド ${s.round} / 最初の ${s.t} 本の辺`,"heading");
    const pos=Array.from({length:s.n},(_,i)=>[240+125*Math.cos(i*2*Math.PI/s.n-Math.PI/2),155+105*Math.sin(i*2*Math.PI/s.n-Math.PI/2)]);
    const leader=x=>{while(s.parent[x]!==x)x=s.parent[x];return x;};
    const palette=["#d6ece0","#ffe4c6","#dde5fb","#f5dcec","#e4eac6","#d9eaf2"];
    s.edges.slice(0,s.t).forEach(([u,v])=>{svg+=`<line x1="${pos[u][0]}" y1="${pos[u][1]}" x2="${pos[v][0]}" y2="${pos[v][1]}" class="edge"/>`;});
    pos.forEach(([x,y],i)=>{svg+=`<circle cx="${x}" cy="${y}" r="20" fill="${palette[leader(i)%palette.length]}" stroke="#1b7361"/>${text(x,y+5,i,"value","middle")}`;});
    svg+=text(430,80,"追加する辺の順（更新 i → 状態 i+1）","heading");
    s.edges.forEach(([u,v],i)=>{svg+=text(430+(i>=8?175:0),110+(i%8)*20,`${i}: ${u}–${v}${i<s.t?" ✓":""}`);});
    svg+=text(30,290,"探索範囲 (low, high] と、このラウンドの mid","heading");
    const step=690/(s.edges.length+2),base=100;
    for(let t=-1;t<=s.edges.length+1;++t)svg+=text(base+(t+1)*step,314,t,"caption","middle");
    s.queries.forEach((q,i)=>{const y=335+i*28,l=base+(s.low[i]+1)*step,r=base+(s.high[i]+1)*step;svg+=text(28,y+4,`Q${i}`);svg+=`<path d="M${l} ${y}H${r}" stroke="${i===s.activeId?"#bd6935":"#1b7361"}" stroke-width="5" opacity=".5"/>`;svg+=`<circle cx="${l}" cy="${y}" r="4" fill="white" stroke="#1b7361"/><circle cx="${r}" cy="${y}" r="4" fill="#1b7361"/>`;if(s.mid[i]!==null)svg+=`<path d="M${base+(s.mid[i]+1)*step} ${y-7}v14" class="sweep"/>`;});
    return [svg,Math.max(440,365+s.queries.length*28)];
  }
  function drawSpace(s){
    const coordsX=s.points.map(p=>p[0]),coordsY=s.points.map(p=>p[1]);
    (s.queries||[]).forEach(([l,d,r,u])=>{coordsX.push(l,r);coordsY.push(d,u);});
    const xmin=Math.min(-1,...coordsX),xmax=Math.max(1,...coordsX),ymin=Math.min(-1,...coordsY),ymax=Math.max(1,...coordsY);
    const X=x=>65+(x-xmin)*420/(xmax-xmin),Y=y=>285-(y-ymin)*230/(ymax-ymin);
    let svg=text(30,25,s.mode==="sweep"?"点と走査線：x が同じ点はまだ入れない":"左側の更新だけを BIT に入れる","heading");
    svg+=`<path d="M65 45v240h450" class="axis"/>${text(510,307,"x")}${text(43,48,"y")}`;
    if(s.activeId>=0){
      let [l,d,r,u]=s.queries[s.activeId];
      if(s.mode==="cdq"&&s.range){[,,r,u]=s.range;l=xmin;d=ymin;}
      if(l<r&&d<u)svg+=`<rect class="region" x="${X(l)}" y="${Y(u)}" width="${X(r)-X(l)}" height="${Y(d)-Y(u)}"/>`;
    }
    if(s.cursor!==null)svg+=`<path d="M${X(s.cursor)} 40V290" class="sweep"/>${text(X(s.cursor),325,`x=${s.cursor}`,"caption","middle")}`;
    const groups=new Map();s.points.forEach((p,i)=>{const key=p.slice(0,2).join(',');if(!groups.has(key))groups.set(key,[]);groups.get(key).push(i);});
    let row=0;
    for(const ids of groups.values()){
      const p=s.points[ids[0]],active=ids.includes(s.activePoint),held=ids.some(i=>s.activePoints.includes(i));
      svg+=`<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="${active?9:6}" fill="${active?"#bd6935":held?"#1b7361":"#c8d2ca"}"/>`;
      svg+=text(X(p[0])+9,Y(p[1])-8,`P${ids.join(",")}`);
      svg+=text(540,55+row++*18,`P${ids.join(",")}: (${p[0]},${p[1]}) w=${ids.reduce((a,i)=>a+s.points[i][2],0)}`);
    }
    svg+=text(30,358,"圧縮した y の重み / BIT の担当区間","heading");
    const columns=Math.min(12,Math.max(1,s.ys.length)),w=750/columns;
    s.ys.forEach((y,i)=>{const bitIndex=i+1,x=35+(i%columns)*w,v=385+Math.floor(i/columns)*75;svg+=cell(x,v,w-5,63,s.bit[bitIndex],`[${bitIndex-(bitIndex&-bitIndex)},${bitIndex})`,s.activePoint>=0&&s.points[s.activePoint][1]===y);svg+=text(x+w/2,v-5,`y=${y} · w=${s.frequency[i]}`,"caption","middle");});
    if(!s.ys.length)svg+=text(40,400,"点がないので BIT は空です。");
    let height=390+Math.max(1,Math.ceil(s.ys.length/columns))*75;
    if(s.mode==="cdq"){
      const [l,m,r]=s.interval;svg+=text(30,height+5,`展開イベントの時刻順：左 [${l},${m}) → 右 [${m},${r}) / 深さ ${s.depth||0}`,"heading");
      s.events.forEach((e,i)=>{const x=35+(i%26)*29,y=height+22+Math.floor(i/26)*45;svg+=`<rect x="${x}" y="${y}" width="26" height="25" rx="3" fill="${i>=l&&i<m?"#d6ece0":i>=m&&i<r?"#ffe4c6":"#f2f3ef"}"/>${text(x+13,y+16,e.id<0?`U${e.point}`:`${e.id}${e.sign>0?"+":"−"}`,"caption","middle")}${text(x+13,y+39,i,"caption","middle")}`;});
      height+=55+Math.ceil(s.events.length/26)*45;
    }
    return [svg,Math.max(480,height)];
  }
  function render(){
    if(!frames.length)return;const s=frames[position];
    const [svg,height]=mode==="mo"?drawMo(s):mode==="pbs"?drawPBS(s):drawSpace(s);
    $("diagram").setAttribute("viewBox",`0 0 840 ${height}`);$("diagram").innerHTML=svg;
    $("step-title").textContent=s.title;$("step-description").textContent=s.detail;
    $("stat-label").textContent=mode==="mo"?"現在の区間":mode==="pbs"?"ラウンド / 状態":"走査線 x";
    $("state-value").textContent=mode==="mo"?`[${s.l}, ${s.r})`:mode==="pbs"?`${s.round} / ${s.t}`:s.cursor===null?"—":s.cursor;
    $("count-label").textContent=mode==="mo"?"異なる値":mode==="pbs"?"追加した辺":"BIT に入れた点";
    $("count-value").textContent=mode==="mo"?s.distinct:mode==="pbs"?s.t:s.activePoints.length;
    const done=mode==="mo"?s.done.length:mode==="pbs"?s.answers.filter(x=>x!==null).length:s.finished?s.answers.length:0;
    $("result").textContent=`${done} / ${s.answers.length}`;
    $("answer-note").textContent=mode==="pbs"?`0 は初期状態、${s.edges.length+1} は最後まで未連結。low/high は探索中の番兵を含みます。`:mode==="mo"?"— は未処理。処理順を変えても、表示は元の登録順です。":"処理途中は寄与の部分和です。すべて完了した位置で答えが確定します。";
    $("table-head").innerHTML=mode==="pbs"?"<tr><th>ID / 頂点</th><th>low</th><th>mid</th><th>high</th><th>答え</th></tr>":"<tr><th>ID</th><th>区間・長方形</th><th>答え / 部分和</th></tr>";
    $("answers").innerHTML=s.queries.map((q,i)=>`<tr class="${i===s.activeId?"active":""}" data-answer="${s.answers[i]===null?"":s.answers[i]}">${mode==="pbs"?`<td>Q${i}: ${q.join("–")}</td><td>${s.low[i]}</td><td>${s.mid[i]??"—"}</td><td>${s.high[i]}</td><td>${s.answers[i]??"—"}</td>`:`<td>Q${i}</td><td>${q.length===2?`[${q[0]}, ${q[1]})`:`[${q[0]},${q[2]}) × [${q[1]},${q[3]})`}</td><td>${s.answers[i]??"—"}</td>`}</tr>`).join("");
    $("progress").textContent=`${position+1} / ${frames.length}`;$("previous").disabled=position===0;$("next").disabled=position===frames.length-1;$("finish").disabled=position===frames.length-1;
  }
  $("run").onclick=run;
  $("previous").onclick=()=>{stop();position=Math.max(0,position-1);render();};
  $("next").onclick=()=>{stop();position=Math.min(frames.length-1,position+1);render();};
  $("finish").onclick=()=>{stop();position=frames.length-1;render();};
  $("play").onclick=()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 ■";timer=setInterval(()=>{if(position<frames.length-1){++position;render();}else stop();},500);};
  $("scenario").onclick=()=>{
    $("array").value="4 1 4 2 1 3 2 4";$("width").value="0";$("vertices").value="6";$("edges").value="0 1\n2 3\n1 2\n3 4";
    $("points").value=mode==="cdq"?"":"1 2 4\n3 2 7\n1 2 -1\n2 4 2";
    $("queries").value=mode==="cdq"?"sum 0 0 4 5\nadd 1 2 4\nsum 0 0 4 5\nadd 1 2 -1\nadd 3 2 7\nsum 1 2 3 3":defaults[mode];run();
  };
  for(const button of document.querySelectorAll("[data-mode]"))button.onclick=()=>{if(location.hash.slice(1)===button.dataset.mode)select(button.dataset.mode);else location.hash=button.dataset.mode;};
  window.addEventListener("hashchange",()=>select(location.hash.slice(1)));
  select(location.hash.slice(1));
})();
