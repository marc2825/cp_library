(function(){
'use strict';
const $=id=>document.getElementById(id),M=window.LinearRecurrenceModel,NS='http://www.w3.org/2000/svg';
const names={matrix:'行列累乗',kitamasa:'Kitamasa',berlekamp:'Berlekamp–Massey',bostan:'Bostan–Mori'},docs={matrix:'matrix_recurrence',kitamasa:'kitamasa',berlekamp:'berlekamp_massey',bostan:'bostan_mori'};
const conditions={matrix:'状態は列ベクトルです。s(k)=T^k s(0)。通常の漸化式では状態を新しい項から古い項へ並べます。加算・乗算だけなので合成数法・法1でも使えます。',kitamasa:'特性多項式はF(X)=X^d-c[0]X^(d-1)-…-c[d-1]。重みは初期項a[0],a[1],…の順です。係数cの順と逆になる箇所を確認できます。合成数法・法1も可。',berlekamp:'素数法が必要です。得られる次数Lは観測列を満たす最小次数です。真の漸化式の次数がd以下と分かっていれば、先頭2d項で以降も決まります。短い任意の観測だけでは未知の将来は一意に決まりません。',bostan:'Q[0]≠0。[x^k]P/Qを求めます。漸化式からはQ=1-Σc[j]x^(j+1)、P=(initial*Q) mod x^dを使います。積のNTT容量が必要で、図の対応法は17・97・998244353です。'};
let mode=M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'matrix',data=null,index=0,timer=null;
const list=a=>'['+a.map(String).join(', ')+']';
function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
function fill(c){for(const id of ['a','b','n','modulus','source','matrix'])$(id).value=c[id];}
function raw(){const r={mode};for(const id of ['a','b','n','modulus','source','matrix'])r[id]=$(id).value;return r;}
function element(tag,attrs={},text){const e=document.createElementNS(NS,tag);for(const [key,value]of Object.entries(attrs))e.setAttribute(key,String(value));if(text!==undefined)e.textContent=String(text);$('diagram').appendChild(e);return e;}
function draw(frame){
 $('diagram').replaceChildren();const count=Math.max(1,...frame.rows.map(row=>row.values.length)),width=Math.max(760,140+count*82);let y=35;
 element('text',{x:22,y,class:'heading'},frame.matrix?'左から掛ける状態遷移':mode==='berlekamp'?'観測列と接続多項式':mode==='bostan'?'昇冪順の分子・分母':'初期項の昇順に並べた重み');y+=28;
 if(frame.matrix&&frame.matrix.length){
  const matrix=frame.matrix;element('text',{x:22,y:y+25,class:'row-label'},`T^(2^${frame.bit})`);
  matrix.forEach((row,i)=>row.forEach((v,j)=>{const x=130+j*82,top=y+i*45;element('rect',{x,y:top,width:76,height:39,rx:5,class:'cell'});element('text',{x:x+38,y:top+25,'text-anchor':'middle',class:'value'},v);}));y+=matrix.length*45+30;
 }
 for(const row of frame.rows){element('text',{x:22,y:y+29,class:'row-label'},row.name);
  if(!row.values.length)element('text',{x:132,y:y+29,class:'caption'},'[]');
  row.values.forEach((v,i)=>{const x=130+i*82;element('rect',{x,y,width:76,height:46,rx:6,class:'cell'+(i===frame.active?' active':'')});element('text',{x:x+38,y:y+15,'text-anchor':'middle',class:'caption'},String(i));element('text',{x:x+38,y:y+34,'text-anchor':'middle',class:'value'},v);});y+=70;
 }
 $('diagram').setAttribute('viewBox',`0 0 ${width} ${Math.max(220,y+15)}`);
}
function render(){
 if(!data)return;const f=data.frames[index];draw(f);$('phase').textContent=names[mode];$('metric-label').textContent=mode==='berlekamp'?'現在の推定次数L':mode==='bostan'?'残りの項番号k':'処理したビット';$('metric').textContent=mode==='berlekamp'?String(f.L??'—'):mode==='bostan'?String(f.k??'—'):String(f.bit??'—');$('modulus-value').textContent=String(data.config.p);$('step-title').textContent=f.title;$('description').textContent=f.description;$('formula').textContent=f.formula||'有限体／剰余環で計算した結果です。';$('progress').textContent=`${index+1} / ${data.frames.length}`;$('previous').disabled=index===0;for(const id of ['next','play','milestone','finish'])$(id).disabled=index===data.frames.length-1;
 $('result').textContent=f.result===null?'結果: 計算途中':f.inferred?`係数c: ${list(f.result)} / a[${data.config.n}]=${f.target}`:`結果: ${list(f.result)}`;
 const head=document.createElement('tr');for(const value of ['添字',...f.rows.map(row=>row.name)]){const th=document.createElement('th');th.textContent=value;head.appendChild(th);}$('value-head').replaceChildren(head);$('values').replaceChildren();
 for(let i=0;i<Math.max(0,...f.rows.map(row=>row.values.length));++i){const tr=document.createElement('tr');if(i===f.active)tr.className='active';for(const value of [i,...f.rows.map(row=>row.values[i]??'—')]){const td=document.createElement('td');td.textContent=String(value);tr.appendChild(td);}$('values').appendChild(tr);}
 $('bm-details').hidden=mode!=='berlekamp';$('bm-state').textContent=f.L===undefined?'観測を1つずつ読み、接続式を更新します。':`L=${f.L}, shift=${f.m??'—'}, last=${f.last??'—'}, delta=${f.delta??'—'}`;$('terms').replaceChildren();for(const term of f.terms||[]){const tr=document.createElement('tr');for(const value of [term.j,term.c,term.index,term.sample,term.value]){const td=document.createElement('td');td.textContent=String(value);tr.appendChild(td);}$('terms').appendChild(tr);}
}
function visibility(){
 const custom=mode==='matrix'&&$('source').value==='custom',rational=mode==='bostan'&&$('source').value==='rational';$('source-field').hidden=!['matrix','bostan'].includes(mode);for(const option of $('source').options)option.hidden=(option.value==='custom'&&mode!=='matrix')||(option.value==='rational'&&mode!=='bostan');$('matrix-field').hidden=!custom;$('b-field').hidden=mode==='berlekamp'||custom;
 $('a-label').textContent=mode==='berlekamp'?'観測した先頭の項 s[0],s[1],…':rational?'分子 P の係数':custom?'状態 s(0)（列ベクトル）':'初期値 a[0],a[1],…';$('b-label').textContent=rational?'分母 Q の係数':'係数 c[0],c[1],…';$('n-label').textContent=custom?'遷移を適用する回数 k':'求める項番号 n（0始まり）';$('input-note').textContent=mode==='berlekamp'?'観測20項まで。空は零列として延長します。負数は法で正規化します。':'漸化式6次、直接P/Qは各12項まで。係数は直前の項から指定します。';$('doc-link').href=`../../docs/linear_recurrence/${docs[mode]}.md`;$('conditions').textContent=conditions[mode];for(const button of document.querySelectorAll('[data-mode]'))button.setAttribute('aria-pressed',String(button.dataset.mode===mode));
}
function run(){stop();visibility();$('input-error').textContent='';try{data=M.run(raw());index=0;render();}catch(error){data=null;$('input-error').textContent=error.message;$('result').textContent='入力を確認してください。';$('diagram').replaceChildren();$('values').replaceChildren();for(const id of ['previous','next','play','milestone','finish'])$(id).disabled=true;$('progress').textContent='';}}
function change(next){mode=next;fill(M.defaults(mode));run();}
for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>{if(location.hash.slice(1)===button.dataset.mode)change(button.dataset.mode);else location.hash=button.dataset.mode;});window.addEventListener('hashchange',()=>change(M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'matrix'));
$('source').addEventListener('change',visibility);$('run').addEventListener('click',run);$('scenario').addEventListener('click',()=>{fill(M.scenario(mode,$('scenario-type').value));run();});$('previous').addEventListener('click',()=>{stop();if(data&&index){index--;render();}});$('next').addEventListener('click',()=>{stop();if(data&&index<data.frames.length-1){index++;render();}});$('finish').addEventListener('click',()=>{stop();if(data){index=data.frames.length-1;render();}});$('milestone').addEventListener('click',()=>{stop();if(data){do{index++;}while(index<data.frames.length-1&&!['weights','bm-corrected','bostan-reduced','matrix-apply'].includes(data.frames[index].stage));render();}});$('play').addEventListener('click',()=>{if(timer){stop();return;}if(!data)return;$('play').textContent='停止';timer=setInterval(()=>{if(index<data.frames.length-1){index++;render();}if(index===data.frames.length-1)stop();},500);});
change(mode);
})();
