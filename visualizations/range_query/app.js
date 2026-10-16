/* global RangeQueryModel */
(function () {
  "use strict";
  const M = RangeQueryModel, $ = id => document.getElementById(id);
  const info = {
    fenwick: {name:"Fenwick", file:"fenwick_tree", title:"lowbit が区間の幅を決める", principle:"BIT[i] は [i−lowbit(i), i) の和です。更新は右の大きな区間へ、prefix は左の未集計区間へ進みます。lower_bound では大きな幅から目標未満のブロックを飛ばします。", formula:"lowbit(i) = i & −i\nadd: i += lowbit(i) / sum: i -= lowbit(i)", pitfall:"lower_bound は要素の添字を返します。目標が総和を超えたら n、w≤0 なら 0。探索中は各要素が非負である必要があります。"},
    segtree: {name:"Segtree", file:"segtree", title:"区間を O(log n) 個へ分解する", principle:"一点を変更したら祖先を再計算します。問い合わせ区間に完全に含まれるノードを使い、含まれない部分は読みません。ここでは和を表示しますが、C++ は min/max・関数合成にも対応します。", formula:"parent = op(left, right)\nprod(l,r): 左から右の順序を保つ", pitfall:"結合律と単位元が必要です。関数合成などでは左右の順序を入れ替えないでください。"},
    lazy: {name:"Lazy", file:"lazy_segtree", title:"集約値を更新し、子への適用を保留する", principle:"完全に覆う区間は、そのノードの和と保留更新だけを変更します。部分区間を読む／変更するときに、先に保留を子へ伝えます。オレンジの式はまだ子へ伝えていない更新です。", formula:"x → a*x+b / sum → a*sum+b*len\ncomposition(f,g) = f ∘ g（g の後に f）", pitfall:"a=1,b=5 は加算、a=0,b=2 は代入です。加算→代入と代入→加算を比べてみましょう。子の格納値は保留が届くまで古い場合があります。"},
    dual: {name:"Dual", file:"dual_segtree", title:"区間の更新だけを内部ノードに置く", principle:"内部ノードは和を保持せず、未伝播の更新だけを保持します。一点取得は根から葉へ保留を伝えます。重なる更新も、古いものから適用されるように順序を保ちます。", formula:"get(p): 根から葉まで push\ncomposition(f,g) = f ∘ g", pitfall:"a=0 の代入と a=1 の加算を混ぜると、合成の向きの重要性が分かります。区間和が必要なら Lazy を使います。"},
    "range-bit": {name:"2-BIT", file:"range_add_range_sum_fenwick_tree", title:"差分と、その重みを 2 本で持つ", principle:"B1 に差分 D[i]、B2 に i*D[i] を保持します。[l,r) の加算は l に +x、r に −x。各差分が prefix に何回現れるかを r−i で数えると、2 本から区間和を復元できます。", formula:"prefix(r) = r*B1.sum(r) − B2.sum(r)\nsum(l,r) = prefix(r) − prefix(l)", pitfall:"r==n の差分は保存しません。境界更新の途中では、開始側の加算が先に入り、終了側の打ち消しが後に入る様子を表示します。"}
  };
  let mode, frames = [], position = 0, timer = null;
  const svgNS = "http://www.w3.org/2000/svg";
  function element(tag, attrs, text) {
    const e = document.createElementNS(svgNS, tag);
    for (const [k,v] of Object.entries(attrs)) e.setAttribute(k,String(v));
    if (text !== undefined) e.textContent = String(text);
    return e;
  }
  const add = (parent, tag, attrs, text) => { const e = element(tag,attrs,text); parent.append(e); return e; };
  function draw(frame) {
    const s = frame.state, svg = $("diagram"), active = new Set(frame.active);
    svg.replaceChildren();
    if (s.nodes) {
      const levels = Math.log2(s.base), height = Math.max(270, (levels + 1) * 84 + 20);
      svg.setAttribute("viewBox", `0 0 820 ${height}`);
      const xy = k => { const depth = Math.floor(Math.log2(k)), order = k - 2 ** depth; return [820*(order+0.5)/(2**depth), 42+depth*84]; };
      for (let k=2;k<s.nodes.length;++k) {const [x,y]=xy(k),[px,py]=xy(k>>1);add(svg,"line",{x1:px,y1:py+23,x2:x,y2:y-25,class:"edge"});}
      for (let k=1;k<s.nodes.length;++k) {
        const node=s.nodes[k],[x,y]=xy(k),w=Math.min(122,820/(2**Math.floor(Math.log2(k)))-7);
        const group=add(svg,"g",{class:`range-node ${active.has(k)?"active":""} ${node.l>=s.n?"padded":""}`,"data-node":k});
        add(group,"rect",{x:x-w/2,y:y-24,width:w,height:53,rx:7});
        add(group,"text",{x,y:y-9,"text-anchor":"middle",class:"range-label"},`[${node.l},${node.r})`);
        add(group,"text",{x,y:y+9,"text-anchor":"middle",class:"node-value"},node.sum===null?"更新のみ":node.sum);
        if (k<s.base && (mode==="lazy"||mode==="dual")) add(svg,"text",{x,y:y+42,"text-anchor":"middle",class:"tag-label"},node.tag.a===1&&node.tag.b===0?"id":`${node.tag.a}x+(${node.tag.b})`);
      }
    } else {
      const rowHeight=27, per=s.n*rowHeight+52, dual=!!s.weighted;
      svg.setAttribute("viewBox",`0 0 820 ${per*(dual?2:1)+15}`);
      function drawBit(bit, top, title) {
        add(svg,"text",{x:18,y:top+20,class:"bit-title"},title);
        for(let k=1;k<=s.n;++k){
          const l=k-(k&-k),y=top+38+(k-1)*rowHeight,x=110+l*620/s.n,w=(k-l)*620/s.n;
          const group=add(svg,"g",{class:`range-node ${active.has(k)?"active":""}`});
          add(group,"text",{x:18,y:y+15,class:"bit-label"},`[${k}]`);
          add(group,"rect",{x,y,width:w,height:22,rx:5});
          add(group,"text",{x:x+w/2,y:y+16,"text-anchor":"middle",class:"node-value"},bit[k]);
          add(group,"text",{x:744,y:y+15,class:"range-label"},`[${l},${k})`);
        }
      }
      drawBit(s.bit,0,dual?"B1 · 差分の和":"BIT · ブロックの和");
      if(dual)drawBit(s.weighted,per,"B2 · 添字 × 差分の和");
    }
  }
  function stop() { if(timer) clearInterval(timer); timer=null; $("play").textContent="再生 ▶"; }
  function render() {
    const frame=frames[position],s=frame.state,a=M.values(s);
    draw(frame);
    $("size-label").textContent=s.n;
    $("result").textContent=frame.result===null?"—":frame.result;
    $("step-title").textContent=frame.title; $("step-description").textContent=frame.detail;
    $("progress").textContent=`${position+1} / ${frames.length} ステップ`;
    $("previous").disabled=position===0;
    $("next").disabled=$("finish").disabled=position===frames.length-1;
    $("values").replaceChildren(); let sum=0;
    a.forEach((x,i)=>{sum+=x;const row=document.createElement("tr");for(const value of [i,x,sum]){const td=document.createElement("td");td.textContent=value;row.append(td);}$("values").append(row);});
    $("value-note").textContent=s.nodes?"保留中の更新まで反映した論理的な値です。図の子に格納した値とは異なる場合があります。":"現在表示している BIT から復元した値です。更新途中では一部のブロックだけが変更されています。";
  }
  function install(s) {stop();frames=[{state:s,title:"配列を初期化",detail:"操作を選び、次へ／再生で内部の変化を確認しましょう。",active:[],result:null,stable:true}];position=0;$("input-error").textContent="";render();}
  function reset() {
    const raw=$("initial").value.trim();
    if(!raw)throw new Error("初期値を 1〜16 個入力してください。");
    const a=raw.split(/[\s,]+/).map(Number);
    if(a.length>16||a.some(x=>!Number.isSafeInteger(x)||Math.abs(x)>1000))throw new Error("初期値は −1000〜1000 の整数を 1〜16 個入力してください。");
    install(M.create(mode,a));
    $("left").value=Math.min(1,a.length);$("right").value=Math.min(6,a.length);$("point").value=Math.min(3,a.length-1);
  }
  function protect(callback) {return ()=>{try{$("input-error").textContent="";callback();}catch(e){$("input-error").textContent=e.message;}};}
  function number(id) {if(!$(id).value.trim())throw new Error("空欄にせず整数を入力してください。");const x=Number($(id).value);if(!Number.isSafeInteger(x)||Math.abs(x)>1000000)throw new Error("−1000000〜1000000 の整数を入力してください。");return x;}
  function execute(op) {
    let origin=position;
    while(!frames[origin].stable && origin<frames.length-1)++origin;
    const result=M.run(frames[origin].state,op);stop();frames=frames.slice(0,origin+1).concat(result.frames);position=origin+1;render();
  }
  function select(next) {
    mode=M.modes.includes(next)?next:"fenwick";
    const current=info[mode];
    document.querySelectorAll("[data-mode]").forEach(b=>b.setAttribute("aria-pressed",String(b.dataset.mode===mode)));
    $("mode-label").textContent=current.name;$("principle-title").textContent=current.title;$("principle").textContent=current.principle;$("formula").textContent=current.formula;$("pitfall").textContent=current.pitfall;
    $("docs-link").href=`../../docs/range_query/${current.file}.md`;$("code-link").href=`../../range_query/${current.file}.hpp`;
    $("affine-fields").hidden=!(mode==="lazy"||mode==="dual");$("search-fields").hidden=mode!=="fenwick";$("set").hidden=mode==="segtree"||mode==="range-bit";
    $("update").textContent=mode==="fenwick"?"一点加算 add(p,x)":mode==="segtree"?"一点代入 set(p,x)":mode==="range-bit"?"区間加算 add(l,r,x)":"区間更新 apply(l,r,{a,b})";
    $("query").textContent=mode==="dual"?"一点取得 get(p)":mode==="segtree"||mode==="lazy"?"区間和 prod(l,r)":"区間和 sum(l,r)";
    try{reset();}catch(e){$("initial").value="1 2 3 4 5 6 7 8";reset();}
  }
  document.querySelectorAll("[data-mode]").forEach(b=>b.addEventListener("click",()=>{if(location.hash.slice(1)===b.dataset.mode)select(b.dataset.mode);else location.hash=b.dataset.mode;}));
  window.addEventListener("hashchange",()=>select(location.hash.slice(1)));
  $("reset").addEventListener("click",protect(reset));
  $("update").addEventListener("click",protect(()=>{
    if(mode==="fenwick")execute({type:"add",p:number("point"),x:number("value")});
    else if(mode==="segtree")execute({type:"set",p:number("point"),x:number("value")});
    else if(mode==="range-bit")execute({type:"add",l:number("left"),r:number("right"),x:number("value")});
    else execute({type:"apply",l:number("left"),r:number("right"),a:number("multiplier"),b:number("offset")});
  }));
  $("query").addEventListener("click",protect(()=>execute(mode==="dual"?{type:"get",p:number("point")}:{type:"sum",l:number("left"),r:number("right")})));
  $("set").addEventListener("click",protect(()=>execute({type:"set",p:number("point"),x:number("value")})));
  $("search").addEventListener("click",protect(()=>execute({type:"lower",w:number("target")})));
  $("previous").addEventListener("click",()=>{stop();if(position>0)--position;render();});
  $("next").addEventListener("click",()=>{stop();if(position<frames.length-1)++position;render();});
  $("finish").addEventListener("click",()=>{stop();position=frames.length-1;render();});
  $("play").addEventListener("click",()=>{if(timer){stop();return;}if(position===frames.length-1)position=0;$("play").textContent="停止 Ⅱ";render();timer=setInterval(()=>{if(position<frames.length-1){++position;render();}if(position===frames.length-1)stop();},700);});
  $("scenario").addEventListener("click",protect(()=>{
    $("initial").value="1 2 3 4 5 6 7 8";reset();
    const steps=mode==="fenwick"?[{type:"add",p:3,x:5},{type:"sum",l:1,r:6},{type:"lower",w:15}]:mode==="segtree"?[{type:"set",p:3,x:9},{type:"sum",l:1,r:7}]:mode==="range-bit"?[{type:"add",l:1,r:6,x:5},{type:"sum",l:2,r:7}]:[{type:"apply",l:0,r:8,a:1,b:5},{type:"apply",l:2,r:6,a:0,b:2},mode==="dual"?{type:"get",p:3}:{type:"sum",l:1,r:7}];
    let s=frames[0].state;
    for(const op of steps){const result=M.run(s,op);frames.push(...result.frames);s=result.state;}
    position=1;render();
  }));
  select(location.hash.slice(1));
})();
