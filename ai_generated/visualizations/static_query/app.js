/* global StaticQueryModel */
(function(){
  "use strict";
  const M=StaticQueryModel,$=id=>document.getElementById(id),svgNS="http://www.w3.org/2000/svg";
  const info={
    sparse:{file:"sparse_table",title:"重複を許す 2 区間で答える",principle:"各段 k には長さ 2^k の区間を格納します。問い合わせは左右の 2 区間を使います。重なった部分を二度使っても結果が同じ、という冪等性が O(1) 取得の条件です。",formula:"k = floor(log₂(r−l))\nop(table[k][l], table[k][r−2^k])",pitfall:"和では重複部分を二重計上するため、この方式は使えません。配列を変更したら全体を再構築します。"},
    stack:{file:"monotonic_stack",title:"最近傍に不要な候補を取り除く",principle:"新しい値の条件を満たさない候補を頂上から取り除きます。残った頂上が最近傍です。左側を探すときは左から、右側を探すときは右から走査します。",formula:"厳密: old < new（または >）\n同値を含む: old ≤ new（または ≥）",pitfall:"同じ値を入力し、厳密／同値を含むを切り替えてください。候補数は、これまで読んだ入力の数とは異なります。"},
    queue:{file:"monotonic_queue",title:"論理キューと、最良値の候補を分ける",principle:"新要素より悪い末尾候補を捨てます。その候補は新要素より早く期限切れになるため不要です。同値は残し、最も左の添字を返します。",formula:"push: 後ろの悪い候補を削除\npop: 論理左端が通過した候補だけ削除",pitfall:"pop() は最良候補を消す操作ではありません。既に候補から消えた入力を pop した場合、候補列はそのままです。"},
    window:{file:"sliding_window_minimum",title:"期限切れを除いて、窓を 1 つ進める",principle:"長さ k になったら最良値を出力します。次の追加の前に最も古い論理要素を取り除きます。各候補は高々一度ずつ出入りするので、全窓を O(n) で求められます。",formula:"窓 [l,r) / r−l = k\n最良値 = candidates.front()",pitfall:"argmin は元配列の添字です。等値は最も左を選び、期限切れになったら次の等値へ移ります。k>n では出力する窓がありません。"},
    swag:{file:"swag",title:"2 スタックで、列の集約順序を保つ",principle:"back には末尾から追加し、front の頂上から削除します。front が空なら back を逆順に移し替えます。累積の演算方向も逆にして、論理列の順序を保ちます。",formula:"back: op(以前の積, x)\nfront: op(x, 以前の積)\n全体: op(front の積, back の積)",pitfall:"連結にすると、順序の違いが見えます。和で同じ結果になる場合でも、関数合成や行列積では向きを守る必要があります。pop は最悪 O(n)、ならし O(1) です。"},
    swag_deque:{file:"swag_deque",title:"左右を半分ずつにして、両端を操作する",principle:"front の逆順と back を合わせたものが論理列です。削除する側が空なら、列の順序を保ちながら左右に半分ずつ再配置します。各スタックの累積を使って全体の積を取得します。",formula:"push_front / push_back\npop_front / pop_back\nprod = op(front の積, back の積)",pitfall:"前後の削除を交互に試してください。全要素を片側へ往復させず、半分ずつ残すことで更新はならし O(1) です。1回の再構築は O(n) かかります。"}
  };
  let mode,frames=[],position=0,timer=null;
  function add(parent,tag,attributes,text){const e=document.createElementNS(svgNS,tag);for(const[k,v]of Object.entries(attributes))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=String(text);parent.append(e);return e;}
  const compact=x=>String(x).length>11?String(x).slice(0,9)+"…":String(x);
  function cell(svg,x,y,width,value,small,active=false,expired=false){const g=add(svg,"g",{class:`cell ${active?"active":""} ${expired?"expired":""}`});add(g,"rect",{x,y,width,height:49,rx:7});add(g,"text",{x:x+width/2,y:y+21,"text-anchor":"middle",class:"cell-value"},compact(value));add(g,"text",{x:x+width/2,y:y+39,"text-anchor":"middle",class:"cell-small"},small);return g;}
  function draw(frame){
    const svg=$("diagram"),s=frame.state,active=new Set(frame.active);svg.replaceChildren();
    const n=Math.max(1,s.a.length,s.items.length,s.next),width=Math.max(840,120+n*62);
    svg.setAttribute("viewBox",`0 0 ${width} ${s.mode==="sparse"?Math.max(370,150+s.table.length*68):450}`);
    const title=(y,text)=>add(svg,"text",{x:20,y,class:"section-title"},text),note=(y,text)=>add(svg,"text",{x:20,y,class:"annotation"},text);
    const row=(data,y,makeValue,makeSmall,makeActive=()=>false,makeExpired=()=>false)=>{data.forEach((x,i)=>cell(svg,110+i*62,y,55,makeValue(x,i),makeSmall(x,i),makeActive(x,i),makeExpired(x,i)));if(!data.length)add(svg,"text",{x:110,y:y+28,class:"annotation"},"空");};
    if(mode==="sparse"){
      title(25,"テーブル: セルの開始添字は左から 0,1,2,…");
      s.table.forEach((values,k)=>{const y=45+k*68;add(svg,"text",{x:20,y:y+28,class:"annotation"},`幅 ${2**k}`);row(values,y,x=>x,(_,i)=>`[${i},${i+2**k})`,(_,i)=>active.has(`${k}:${i}`));});
      const y=60+s.table.length*68;
      if(s.blocks.length){title(y,"問い合わせを覆う 2 つの区間");s.blocks.forEach(([l,r],i)=>{add(svg,"rect",{x:110+l*62,y:y+15+i*35,width:(r-l)*62-7,height:27,rx:5,class:i?"range-b":"range-a"});add(svg,"text",{x:110+l*62+7,y:y+33+i*35,class:"annotation"},`[${l},${r})`);});}
      else note(y,"「前計算を追う」で段ごとの構築、「問い合わせ」で 2 区間の重なりを確認できます。");
    }else if(mode==="stack"){
      title(25,`入力配列 · ${s.direction==="previous"?"左 → 右":"右 → 左"}に走査`);
      row(s.a,45,x=>x,(_,i)=>`i=${i}`,(_,i)=>active.has(i),(_,i)=>s.answers[i]!==null&&!active.has(i));
      title(150,"残った候補（底 → 頂上）");row(s.candidates,170,x=>x.value,x=>`i=${x.index}`,x=>active.has(x.index));
      note(270,"橙色の候補は、このステップで処理した要素です。頂上が次の比較の起点になります。");
      title(330,"最近傍の添字（−1 は該当なし）");row(s.answers,350,x=>x===null?"·":x,(_,i)=>`ans[${i}]`);
    }else if(mode==="swag"||mode==="swag_deque"){
      const logical=s.rebuildSource??(s.transferring?s.back.concat(s.front.slice().reverse()):s.front.slice().reverse().concat(s.back));
      title(25,mode==="swag_deque"?"論理Deque（左が先頭）":"論理キュー（左が先頭）");row(logical,45,x=>x.value,x=>`i=${x.index}`,x=>active.has(x.index));
      title(150,"front · 底 → 頂上（右端から pop）");row(s.front,170,x=>x.value,x=>`積: ${compact(x.aggregate)}`,x=>active.has(x.index));
      title(285,"back · 底 → 頂上（右端へ push）");row(s.back,305,x=>x.value,x=>`積: ${compact(x.aggregate)}`,x=>active.has(x.index));
      note(415,s.rebuildSource?"再構築中: 上段は元の論理列、下段は構築途中の新しいスタックです。":s.transferring?"移し替え途中: back の残り → front の逆順が元の列です。":"上段の論理順序は保ち、下段のスタックと累積の向きを変えて保持します。");
    }else{
      const stream=mode==="window"?s.a:s.items;
      title(25,mode==="window"?`入力配列 · 窓幅 ${s.k}`:"追加した論理要素（灰色は削除済み）");
      row(stream,45,x=>x,(_,i)=>`i=${i}`,(_,i)=>active.has(i),(_,i)=>i<s.left);
      if(mode==="window"&&s.items.length){add(svg,"rect",{x:110+s.left*62,y:102,width:(s.items.length-s.left)*62-7,height:18,rx:4,class:"range-a"});add(svg,"text",{x:110+s.left*62+5,y:115,class:"cell-small"},`[${s.left},${s.items.length})`);}
      title(175,"最良値の候補（左端が best）");row(s.candidates,195,x=>x.value,x=>`i=${x.index}`,x=>active.has(x.index));
      if(mode==="window"){title(310,"各窓の出力");row(s.outputs,330,x=>x.value,x=>`[${x.l},${x.r})`);}
      else{note(320,`論理長 ${s.items.length-s.left} / 候補数 ${s.candidates.length}`);note(350,"論理先頭の要素が候補から消えていても、pop は正しく期限を進めます。");}
    }
  }
  function stop(){if(timer)clearInterval(timer);timer=null;$("play").textContent="再生 ▶";}
  function table(headers,rows){$("table-head").replaceChildren();$("values").replaceChildren();const tr=document.createElement("tr");for(const text of headers){const th=document.createElement("th");th.textContent=text;tr.append(th);}$("table-head").append(tr);for(const cells of rows){const row=document.createElement("tr");for(const text of cells){const td=document.createElement("td");td.textContent=text;row.append(td);}$("values").append(row);}}
  function render(){
    const frame=frames[position],s=frame.state;draw(frame);
    $("processed").textContent=mode==="sparse"?`${s.a.length} 要素`:`${s.cursor} / ${s.a.length}`;
    $("count-caption").textContent=mode==="sparse"?"前計算した段":["swag","swag_deque","queue"].includes(mode)?"論理列の長さ":"候補数";
    $("count").textContent=mode==="sparse"?s.table.length:["swag","swag_deque","queue"].includes(mode)?M.logical(s).length:s.candidates.length;
    $("result").textContent=frame.result===null?"—":frame.result===""?"ε":frame.result;
    $("step-title").textContent=frame.title;$("step-description").textContent=frame.detail;
    $("progress").textContent=`${position+1} / ${frames.length} ステップ`;
    $("previous").disabled=position===0;$("next").disabled=$("finish").disabled=position===frames.length-1;
    $("step").disabled=s.cursor>=s.a.length;
    if(mode==="window"){
      $("table-title").textContent="窓ごとの結果";$("table-note").textContent="元配列の最良値の位置を表示します。同値は最も左を選びます。";
      table(["区間","最良値","元の添字"],s.outputs.map(x=>[`[${x.l},${x.r})`,x.value,x.index]));
    }else if(mode==="stack"){
      $("table-title").textContent="最近傍の答え";$("table-note").textContent="· は未処理、−1 は条件を満たす要素がないことを表します。";
      table(["添字","入力値","最近傍"],s.a.map((x,i)=>[i,x,s.answers[i]===null?"·":s.answers[i]]));
    }else if(mode==="swag"||mode==="swag_deque"){
      $("table-title").textContent="論理順序と現在の積";$("table-note").textContent=s.rebuildSource?"再構築が完了するまでは、元の論理列とその積を表示します。":s.transferring?"移し替え中は back に残る前半と front へ移した後半を並べ、元の列を表示します。":"front の逆順と back の順序を連結した列です。積はその順序で計算します。";
      const aggregate=M.fold(s);$("table-note").textContent+=` 現在の積: ${aggregate===""?"空文字 ε":aggregate}。`;
      table(["位置","値"],M.logical(s).map((x,i)=>[i,x]));
    }else if(mode==="queue"){
      $("table-title").textContent="論理キューの中身";$("table-note").textContent="この表では学習用に入力を表示しています。C++ の単調キューは候補だけを保持します。";
      table(["通算添字","値","候補"],s.items.slice(s.left).map((x,i)=>[s.left+i,x,s.candidates.some(c=>c.index===s.left+i)?"残る":"破棄"]));
    }else{
      $("table-title").textContent="入力配列";$("table-note").textContent="実装では配列内に収まる区間だけを各段に格納します。";
      table(["添字","値"],s.a.map((x,i)=>[i,x]));
    }
  }
  function protect(fn){return()=>{try{$("input-error").textContent="";fn();}catch(e){$("input-error").textContent=e.message;}};}
  function number(id){if(!$(id).value.trim())throw new Error("空欄にせず整数を入力してください。");const x=Number($(id).value);if(!Number.isSafeInteger(x)||Math.abs(x)>1000000)throw new Error("−1000000〜1000000 の整数を入力してください。");return x;}
  function reset(){
    const raw=$("initial").value.trim();if(!raw)throw new Error("初期配列を 1〜16 要素入力してください。");
    const a=raw.split(/[\s,]+/).map(Number);if(a.length>16||a.some(x=>!Number.isSafeInteger(x)||Math.abs(x)>1000))throw new Error("初期配列は −1000〜1000 の整数を 1〜16 個です。");
    const k=mode==="window"?number("width"):3;if(k>32)throw new Error("図解の窓幅は 1〜32 です。");
    const s=M.create(mode,a,{order:$("order").value,strict:$("strict").value==="strict",direction:$("direction").value,aggregation:$("aggregation").value,k});
    stop();frames=[{state:s,title:"初期状態",detail:mode==="sparse"?"前計算済みのテーブルです。構築の手順も「前計算を追う」で確認できます。":"まだ入力を処理していません。次の入力を処理するか、操作列を読み込みましょう。",active:[],result:null,stable:true}];position=0;
    $("left").value=Math.min(1,a.length);$("right").value=Math.min(7,a.length);render();
  }
  function execute(request){
    let origin=position;while(!frames[origin].stable&&origin<frames.length-1)++origin;
    const s=frames[origin].state;
    if(["push","push_front","push_back","step"].includes(request.type)&&["queue","swag","swag_deque"].includes(mode)&&(mode==="queue"?s.items.length:s.next)>=16)throw new Error("図解では通算 16 回まで追加できます。初期化して続けてください。");
    const r=M.run(s,request);stop();frames=frames.slice(0,origin+1).concat(r.frames);position=origin+1;render();
  }
  function select(value){
    mode=M.modes.includes(value)?value:"sparse";const current=info[mode];
    document.querySelectorAll("[data-mode]").forEach(b=>b.setAttribute("aria-pressed",String(b.dataset.mode===mode)));
    for(const [id,text]of [["principle-title",current.title],["principle",current.principle],["formula",current.formula],["pitfall",current.pitfall]])$(id).textContent=text;
    $("docs-link").href=`../../docs/static_query/${current.file}.md`;$("code-link").href=`../../static_query/${current.file}.hpp`;
    $("range-fields").hidden=mode!=="sparse";$("order-field").hidden=mode==="swag"||mode==="swag_deque";$("stack-fields").hidden=mode!=="stack";$("window-field").hidden=mode!=="window";$("swag-field").hidden=mode!=="swag"&&mode!=="swag_deque";
    $("build").hidden=mode!=="sparse";$("step").hidden=mode==="sparse";$("query").hidden=mode==="stack"||mode==="window";$("pop").hidden=$("manual-fields").hidden=!["queue","swag","swag_deque"].includes(mode);
    $("push-front").hidden=$("pop-back").hidden=$("clear").hidden=mode!=="swag_deque";
    $("push").textContent=mode==="swag_deque"?"末尾へ追加 push_back()":"push(value)";
    $("pop").textContent=mode==="swag_deque"?"先頭を削除 pop_front()":"先頭を削除 pop()";
    $("query").textContent=mode==="sparse"?"区間積 prod(l,r)":mode==="queue"?"最良値 best()":"全体の積 prod()";
    try{reset();}catch(e){$("initial").value="4 2 2 5 1 3 1 6";$("width").value=3;reset();}
  }
  document.querySelectorAll("[data-mode]").forEach(b=>b.addEventListener("click",()=>{if(location.hash.slice(1)===b.dataset.mode)select(b.dataset.mode);else location.hash=b.dataset.mode;}));
  window.addEventListener("hashchange",()=>select(location.hash.slice(1)));
  $("reset").addEventListener("click",protect(reset));
  for(const id of ["order","strict","direction","aggregation"])$(id).addEventListener("change",protect(reset));
  $("step").addEventListener("click",protect(()=>execute({type:"step"})));
  $("push").addEventListener("click",protect(()=>execute({type:mode==="swag_deque"?"push_back":"push",value:number("value")})));
  $("pop").addEventListener("click",protect(()=>execute({type:mode==="swag_deque"?"pop_front":"pop"})));
  $("push-front").addEventListener("click",protect(()=>execute({type:"push_front",value:number("value")})));
  $("pop-back").addEventListener("click",protect(()=>execute({type:"pop_back"})));
  $("clear").addEventListener("click",protect(()=>execute({type:"clear"})));
  $("build").addEventListener("click",protect(()=>execute({type:"build"})));
  $("query").addEventListener("click",protect(()=>execute(mode==="sparse"?{type:"query",l:number("left"),r:number("right")}:{type:"query"})));
  $("previous").addEventListener("click",()=>{stop();if(position>0)--position;render();});
  $("next").addEventListener("click",()=>{stop();if(position<frames.length-1)++position;render();});
  $("finish").addEventListener("click",()=>{stop();position=frames.length-1;render();});
  $("play").addEventListener("click",()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 Ⅱ";render();timer=setInterval(()=>{if(position<frames.length-1){++position;render();}if(position===frames.length-1)stop();},700);});
  $("scenario").addEventListener("click",protect(()=>{
    $("initial").value="4 2 2 5 1 3 1 6";$("width").value=3;reset();
    const step={type:"step"},requests=mode==="sparse"?[{type:"build"},{type:"query",l:1,r:7}]:mode==="stack"||mode==="window"?Array(8).fill(step):mode==="queue"?[step,step,step,step,{type:"pop"},{type:"pop"},{type:"query"},step,{type:"query"}]:mode==="swag_deque"?[step,step,step,step,{type:"pop_front"},{type:"push_front",value:9},{type:"pop_back"},{type:"pop_back"},{type:"pop_back"},{type:"push_back",value:7},{type:"query"}]:[step,step,step,{type:"pop"},step,step,{type:"query"}];
    let s=frames[0].state;for(const request of requests){const r=M.run(s,request);frames.push(...r.frames);s=r.state;}position=1;render();
  }));
  select(location.hash.slice(1));
})();
