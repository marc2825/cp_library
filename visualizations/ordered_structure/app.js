/* global OrderedStructureModel */
(function(){
  "use strict";
  const M=OrderedStructureModel,$=id=>document.getElementById(id),svgNS="http://www.w3.org/2000/svg";
  const info={
    set:{file:"ordered_associative",title:"同じキーを一つだけ保持する",principle:"set は重複のない順序付き集合です。lower_bound は同値を含む右側の境界、upper_bound は同値を除く右側の境界です。そこから前へ一つ戻せば左側の候補になります。",formula:"predecessor(x): lower_bound(x) の直前\nsuccessor(x): upper_bound(x)",pitfall:"begin() の前へ戻ったり、end() を参照・削除したりしないようにします。補助関数は境界を確認し、該当なしを end() で返します。"},
    multiset:{file:"ordered_associative",title:"1 個だけの削除と、全削除を区別する",principle:"multiset は同じ値を複数保持します。erase_one は検索した 1 要素だけを削除します。標準の erase(key) は同値をすべて削除します。",formula:"erase_one(ms,x): find → erase(iterator)\nms.erase(x): 同値を全削除",pitfall:"count(x) は重複の個数に比例する時間も必要です。標準コンテナで順位や k 番目を移動して求めると線形時間がかかります。"},
    map:{file:"ordered_associative",title:"挿入・上書き・参照を選ぶ",principle:"try_emplace は既存の値を保持し、insert_or_assign は既存の値を上書きします。find は不在でも挿入しません。operator[] は不在キーを初期値で挿入します。",formula:"try_emplace: 既存を保持\ninsert_or_assign: 既存を上書き\nfind: 参照だけ / m[x]: 不在なら挿入",pitfall:"キーで並べます。対応する値の大小で並べるコンテナではありません。図解では初期値を key×10、operator[] の初期値を 0 にしています。"},
    compression:{file:"coordinate_compression",title:"値の順序を、連続した添字へ変える",principle:"入力をソートし、同値を一つにまとめます。元の値は保存し、添字から戻せます。登録していない閾値でも、lower_bound/upper_bound による境界位置は求められます。",formula:"index(x): 登録された同値の位置\nlower_bound(x): 最初の x 以上の位置\n見つからない境界は M",pitfall:"添字の差は元の値の距離ではありません。index の未登録と、lower_bound が返す挿入位置を区別してください。"},
    statistics:{file:"compressed_order_statistics",title:"順位は頻度の累積和、k 番目はその逆",principle:"登録座標ごとの非負の頻度を BIT に入れます。閾値を座標の境界へ変換して prefix を足すと順位になります。k 番目は、累積和が k+1 に届く最初の座標を探します。",formula:"order_of_key(x) = prefix(lower_bound(x))\nkth(k) = value(BIT.lower_bound(k+1))",pitfall:"挿入候補は先に登録します。問い合わせの閾値は未登録でも使えます。削除しすぎて頻度が負になる更新は拒否します。"}
  };
  const queries={set:[["predecessor","前の要素"],["successor","次の要素"],["count","存在する個数"]],multiset:[["count","同値の個数"],["predecessor","前の要素"],["successor","次の要素"]],map:[["lookup","find（挿入しない）"],["bracket","m[x]（不在なら挿入）"],["predecessor","前のキー"],["successor","次のキー"]],compression:[["index","index（登録添字）"],["lower","lower_bound（以上の境界）"],["upper","upper_bound（超える境界）"]],statistics:[["kth","kth（0 始まり）"],["rank","order_of_key（未満の個数）"],["rank_le","以下の個数"],["count","同値の個数"],["range","値域 [x,r) の個数"],["predecessor","前の値"],["successor","次の値"]]};
  let mode,frames=[],position=0,timer=null;
  function add(parent,tag,attributes,text){const e=document.createElementNS(svgNS,tag);for(const[k,v]of Object.entries(attributes))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=String(text);parent.append(e);return e;}
  function cell(svg,x,y,value,note,active){const g=add(svg,"g",{class:`cell ${active?"active":""}`});add(g,"rect",{x,y,width:62,height:53,rx:7});add(g,"text",{x:x+31,y:y+23,"text-anchor":"middle",class:"cell-value"},value);add(g,"text",{x:x+31,y:y+43,"text-anchor":"middle",class:"cell-small"},note);}
  function draw(frame){
    const s=frame.state,svg=$("diagram"),active=new Set(frame.active),bitActive=new Set(frame.bitActive),keys=mode==="map"?s.entries.map(e=>e.key):mode==="compression"||mode==="statistics"?s.coordinates:s.values;
    const n=Math.max(keys.length,s.initial.length),width=Math.max(840,150+n*72),height=mode==="statistics"?Math.max(410,230+keys.length*29):mode==="compression"?410:305;
    svg.replaceChildren();svg.setAttribute("viewBox",`0 0 ${width} ${height}`);svg.style.minWidth=`${width}px`;
    const title=(y,text)=>add(svg,"text",{x:20,y,class:"section-title"},text),note=(y,text)=>add(svg,"text",{x:20,y,class:"annotation"},text);
    const row=(a,y,value,small,mark=(_,i)=>active.has(i))=>{a.forEach((x,i)=>cell(svg,115+i*72,y,value(x,i),small(x,i),mark(x,i)));if(!a.length)note(y+30,"空");};
    if(mode==="compression"){
      title(25,"元の入力（変更しない）");row(s.initial,45,x=>x,(_,i)=>`入力 ${i}`,()=>false);
      title(155,"比較器順にソート");const sorted=s.initial.slice().sort((x,y)=>s.reverse?y-x:x-y);row(sorted,175,x=>x,()=>"重複を含む",()=>false);
      title(285,"同値をまとめて、座標を登録");row(keys,305,x=>x,(_,i)=>`添字 ${i}`);
      note(390,s.building?"異なる値だけを順に登録しています。":"元の値と添字は、この対応表で相互に変換できます。");
    }else if(mode==="statistics"){
      title(25,"値 → 圧縮添字 → 頻度");row(keys,45,x=>x,(_,i)=>`i=${i} · ${s.frequency[i]} 個`);
      title(155,"BIT[i] のブロックと、その頻度の和");
      for(let i=1;i<s.bit.length;++i){
        const l=i-(i&-i),y=175+(i-1)*29,x=115+l*72,w=(i-l)*72-10,g=add(svg,"g",{class:`cell ${bitActive.has(i)?"active":""}`});
        add(svg,"text",{x:20,y:y+17,class:"annotation"},`BIT[${i}]`);add(g,"rect",{x,y,width:w,height:23,rx:5});add(g,"text",{x:x+w/2,y:y+17,"text-anchor":"middle",class:"cell-value"},s.bit[i]);add(svg,"text",{x:115+keys.length*72+8,y:y+17,class:"annotation"},`[${l},${i})`);
      }
      note(height-18,"頻度は非負。更新途中は各 BIT ブロックを一つずつ変更しています。");
    }else{
      title(25,mode==="map"?"キーの順に並ぶ対応表":"比較器順の論理的な並び");
      if(frame.region){const[l,r]=frame.region;add(svg,"rect",{x:109+l*72,y:65,width:Math.max(0,(r-l)*72),height:69,rx:9,class:"search-region"});}
      row(keys,73,x=>x,(_,i)=>mode==="map"?`→ ${s.entries[i].value}`:`位置 ${i}`);
      note(185,s.reverse?"反復順序は降順です。「前／後」も、この順序で読みます。":"反復順序は昇順です。前後検索は、同値を含むかどうかを選べます。");
      note(225,"図は順序と比較の概念を示します。標準コンテナの内部木の形は実装依存です。");
      if(mode==="multiset")note(265,"同値の要素もそれぞれ保持します。erase_one と erase(key) の違いを試してください。");
    }
  }
  function stop(){if(timer)clearInterval(timer);timer=null;$("play").textContent="再生 ▶";}
  function table(headers,rows){$("table-head").replaceChildren();$("values").replaceChildren();const tr=document.createElement("tr");for(const text of headers){const th=document.createElement("th");th.textContent=text;tr.append(th);}$("table-head").append(tr);for(const values of rows){const row=document.createElement("tr");for(const text of values){const td=document.createElement("td");td.textContent=text;row.append(td);}$("values").append(row);}}
  function format(value){if(value===null)return "該当なし";if(typeof value==="boolean")return String(value);if(typeof value==="object")return `${value.key} → ${value.value}`;return String(value);}
  function render(){
    const frame=frames[position],s=frame.state;draw(frame);
    $("size-caption").textContent=mode==="compression"?"入力の要素数":"現在の要素数";$("distinct-caption").textContent=mode==="statistics"?"登録座標数":"種類数";
    $("size").textContent=mode==="statistics"?s.total:mode==="map"?s.entries.length:mode==="compression"?s.initial.length:s.values.length;
    $("distinct").textContent=mode==="statistics"||mode==="compression"?s.coordinates.length:mode==="map"?s.entries.length:new Set(s.values).size;
    $("result").textContent=frame.result?format(frame.result.value):"—";$("step-title").textContent=frame.title;$("step-description").textContent=frame.detail;$("explanation").dataset.tone=frame.tone;
    $("progress").textContent=`${position+1} / ${frames.length} ステップ`;$("previous").disabled=position===0;$("next").disabled=$("finish").disabled=position===frames.length-1;
    $("table-title").textContent=mode==="statistics"?"座標ごとの頻度と累積和":mode==="compression"?"元の入力と圧縮添字":"現在の内容";
    if(mode==="statistics"){let sum=0;table(["値","添字","頻度","累積和"],s.coordinates.map((x,i)=>{sum+=s.frequency[i];return[x,i,s.frequency[i],sum];}));$("table-note").textContent="累積和は頻度から表示します。BIT の更新途中には、まだ変更されていないブロックもあります。";}
    else if(mode==="compression"){table(["入力位置","元の値","圧縮添字"],s.initial.map((x,i)=>[i,x,s.coordinates.includes(x)?s.coordinates.indexOf(x):"未登録"]));$("table-note").textContent="元の配列を書き換えず、同じ値に同じ添字を割り当てます。";}
    else if(mode==="map"){table(["キー","対応値"],s.entries.map(e=>[e.key,e.value]));$("table-note").textContent="初期状態ではキー×10 を対応値にしています。操作で別の値を登録できます。";}
    else{table(["位置","値"],s.values.map((x,i)=>[i,x]));$("table-note").textContent="ここでの位置は説明用です。標準コンテナで k 番目へ iterator を進めるには線形の移動が必要です。";}
  }
  function protect(fn){return()=>{try{$("input-error").textContent="";fn();}catch(e){$("input-error").textContent=e.message;}};}
  function number(id){if(!$(id).value.trim())throw new Error("空欄にせず整数を入力してください。");const x=Number($(id).value);if(!Number.isSafeInteger(x)||Math.abs(x)>1000000)throw new Error("−1000000〜1000000 の整数を入力してください。");return x;}
  function list(id){const raw=$(id).value.trim();if(!raw)return[];const a=raw.split(/[\s,]+/).map(Number);if(a.length>16||a.some(x=>!Number.isSafeInteger(x)||Math.abs(x)>1000))throw new Error("初期要素・候補は −1000〜1000 の整数を 16 個まで入力できます。");return a;}
  function reset(){const initial=list("initial"),universe=mode==="statistics"?list("universe"):initial,s=M.create(mode,initial,{universe,reverse:$("order").value==="descending"});stop();frames=[{state:s,title:"初期状態",detail:mode==="statistics"?"初期要素の頻度を BIT に構築しました。候補にはない値でも、閾値として検索できます。":"入力を設定しました。操作を選び、次へ／再生で変化を追ってください。",active:[],bitActive:[],region:null,tone:"normal",result:null,stable:true}];position=0;render();}
  function execute(request){
    let origin=position;while(!frames[origin].stable&&origin<frames.length-1)++origin;
    const s=frames[origin].state;
    if(["insert","assign","bracket"].includes(request.type)&&["set","multiset","map"].includes(mode)){
      const keys=mode==="map"?s.entries.map(e=>e.key):s.values;
      if(keys.length>=16&&(mode==="multiset"||!keys.includes(request.x)))throw new Error("図解の保持要素は 16 個までです。削除するか初期化してください。");
    }
    const r=M.run(s,request);stop();frames=frames.slice(0,origin+1).concat(r.frames);position=origin+1;render();
  }
  function updateQuery(){const type=$("query-type").value;$("rank-field").hidden=type!=="kth";$("high-field").hidden=type!=="range";$("inclusive-field").hidden=type!=="predecessor"&&type!=="successor";}
  function select(value){
    mode=M.modes.includes(value)?value:"set";const current=info[mode];document.querySelectorAll("[data-mode]").forEach(b=>b.setAttribute("aria-pressed",String(b.dataset.mode===mode)));
    for(const[id,text]of [["principle-title",current.title],["principle",current.principle],["formula",current.formula],["pitfall",current.pitfall]])$(id).textContent=text;
    $("docs-link").href=`../../docs/ordered_structure/${current.file}.md`;$("code-link").href=`../../ordered_structure/${current.file}.hpp`;
    $("universe-field").hidden=$("count-field").hidden=mode!=="statistics";$("mapped-field").hidden=$("assign").hidden=mode!=="map";$("build").hidden=mode!=="compression";$("clear").hidden=mode!=="statistics";
    $("insert").hidden=$("erase").hidden=mode==="compression";$("erase-all").hidden=mode==="compression"||mode==="map";$("pop-fields").hidden=mode!=="set"&&mode!=="multiset";
    $("insert").textContent=mode==="map"?"try_emplace（既存を保持）":"insert(x)";
    $("initial-note").textContent="空欄なら空。要素・候補は各 16 個まで。順序変更でも初期化します。";
    $("query-type").replaceChildren();for(const[type,label]of queries[mode]){const option=document.createElement("option");option.value=type;option.textContent=label;$("query-type").append(option);}updateQuery();
    try{reset();}catch(e){$("initial").value="100 -5 20 20";$("universe").value="-5 20 100";reset();}
  }
  document.querySelectorAll("[data-mode]").forEach(b=>b.addEventListener("click",()=>{if(location.hash.slice(1)===b.dataset.mode)select(b.dataset.mode);else location.hash=b.dataset.mode;}));window.addEventListener("hashchange",()=>select(location.hash.slice(1)));
  $("reset").addEventListener("click",protect(reset));$("order").addEventListener("change",protect(reset));$("query-type").addEventListener("change",updateQuery);
  $("insert").addEventListener("click",protect(()=>execute({type:"insert",x:number("key"),...(mode==="statistics"?{count:number("amount")}:{}),...(mode==="map"?{value:number("mapped")}: {})})));
  $("assign").addEventListener("click",protect(()=>execute({type:"assign",x:number("key"),value:number("mapped")})));
  $("erase").addEventListener("click",protect(()=>execute({type:"erase",x:number("key"),...(mode==="statistics"?{count:number("amount")}: {})})));
  $("erase-all").addEventListener("click",protect(()=>execute({type:"erase_all",x:number("key")})));
  for(const[id,type]of [["build","build"],["clear","clear"],["pop-first","pop_first"],["pop-last","pop_last"]])$(id).addEventListener("click",protect(()=>execute({type})));
  $("query").addEventListener("click",protect(()=>{const type=$("query-type").value;execute(type==="kth"?{type,k:number("rank")}:{type,x:number("key"),inclusive:$("inclusive").checked,...(type==="range"?{high:number("high")}: {})});}));
  $("previous").addEventListener("click",()=>{stop();if(position>0)--position;render();});$("next").addEventListener("click",()=>{stop();if(position<frames.length-1)++position;render();});$("finish").addEventListener("click",()=>{stop();position=frames.length-1;render();});
  $("play").addEventListener("click",()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 Ⅱ";render();timer=setInterval(()=>{if(position<frames.length-1){++position;render();}if(position===frames.length-1)stop();},700);});
  $("scenario").addEventListener("click",protect(()=>{
    $("initial").value="100 -5 20 20";$("universe").value="-5 20 100";reset();
    const requests=mode==="set"?[{type:"insert",x:20},{type:"insert",x:7},{type:"predecessor",x:20},{type:"successor",x:20,inclusive:true}]:mode==="multiset"?[{type:"insert",x:20},{type:"erase",x:20},{type:"count",x:20},{type:"erase_all",x:20},{type:"count",x:20}]:mode==="map"?[{type:"insert",x:20,value:7},{type:"assign",x:20,value:7},{type:"lookup",x:7},{type:"bracket",x:7}]:mode==="compression"?[{type:"build"},{type:"index",x:7},{type:"lower",x:7}]:[{type:"insert",x:20,count:2},{type:"erase",x:20,count:7},{type:"insert",x:7,count:1},{type:"rank",x:7},{type:"kth",k:3}];
    let s=frames[0].state;for(const request of requests){const r=M.run(s,request);frames.push(...r.frames);s=r.state;}position=1;render();
  }));
  select(location.hash.slice(1));
})();
