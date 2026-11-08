(function(){
'use strict';
const $=id=>document.getElementById(id),M=window.FPSModel,NS='http://www.w3.org/2000/svg';
const names={polynomial:'加減乗',calculus:'微積分・切り詰め',inv:'inv',log:'log',exp:'exp',pow:'pow',sqrt:'sqrt',division:'除算・剰余',multipoint:'多点評価',interpolation:'補間',taylor:'Taylor Shift',bostan:'Bostan–Mori'};
const docs={polynomial:'polynomial',calculus:'polynomial',inv:'inv',log:'log',exp:'exp',pow:'pow',sqrt:'sqrt',division:'division',multipoint:'multipoint_evaluation',interpolation:'interpolation',taylor:'taylor_shift',bostan:'bostan_mori'};
const conditions={polynomial:'加減算は同じ次数同士。乗算では添字の和が結果の次数になります。',calculus:'積分は各次数の逆元が必要です。切り詰めは指定した個数を返し、不足分を0補完します。',inv:'定数項は非0。現在の正しさがx^mまであれば、誤差の二乗はx^(2m)で消えます。',log:'定数項は1。積分定数を0に固定します。',exp:'入力の定数項は0。結果の定数項は1。',pow:'指数は非負のuint64。0^0=1。先頭の次数の移動と、係数の法による計算を区別します。',sqrt:'奇素数法。先頭の非0次数は偶数、係数は平方剰余が必要です。最初の根は小さい方を選びます。',division:'除数は零多項式以外。末尾0を除いた商・剰余を返します。',multipoint:'評価点の重複を許します。葉のX-xᵢで割った剰余がf(xᵢ)です。',interpolation:'点は法で相異なる必要があります。M=∏(X-xᵢ)の導関数で重みを作ります。',taylor:'有限多項式のf(x+c)を計算します。高次の係数も低次へ寄与します。',bostan:'分母の定数項は非0。項番号は0始まりのuint64。分子の次数が分母以上でも計算できます。'};
let mode=M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'polynomial',data=null,index=0,timer=null;
function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
function raw(){const r={mode};for(const id of ['a','b','modulus','precision','exponent','shift','operation','calculus'])r[id]=$(id).value;return r;}
function fill(c){for(const id of ['a','b','modulus','precision','exponent','shift','operation','calculus'])$(id).value=c[id];}
function element(tag,attrs={},text){const e=document.createElementNS(NS,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=String(text);$('diagram').appendChild(e);return e;}
function draw(frame){
 const svg=$('diagram');svg.replaceChildren();const t=frame.tree,rowLength=Math.max(1,...frame.rows.map(r=>r.values.length)),width=Math.max(760,140+rowLength*82,t?t.base*130+40:0);let y=35;
 element('text',{x:22,y,class:'heading'},t?'一次式の積と、剰余の伝播':'昇冪順の係数列');y+=25;
 if(t){
  const depth=Math.log2(t.base),top=y,pos=i=>{const level=Math.floor(Math.log2(i)),offset=i-2**level;return {x:20+(offset+0.5)*(width-40)/2**level,y:top+level*78};};
  for(let i=2;i<t.nodes.length;++i){const a=pos(i>>1),b=pos(i);element('line',{x1:a.x,y1:a.y+38,x2:b.x,y2:b.y,class:'wire'});}
  for(let i=1;i<t.nodes.length;++i){const point=pos(i),values=t.nodes[i],isActive=i===frame.node;element('rect',{x:point.x-54,y:point.y,width:108,height:44,rx:7,class:'tree-node'+(isActive?' active':'')});element('text',{x:point.x,y:point.y+17,'text-anchor':'middle',class:'tree-label'},'M'+i);element('text',{x:point.x,y:point.y+33,'text-anchor':'middle',class:'caption'},values.length?'degree '+(values.length-1):'未構築');}
  y=top+depth*78+80;
 }
 for(const row of frame.rows){
  element('text',{x:22,y:y+28,class:'row-label'},row.name);
  if(!row.values.length)element('text',{x:135,y:y+28,class:'caption'},'[] （零多項式）');
  row.values.forEach((v,i)=>{const x=130+i*82;element('rect',{x,y,width:76,height:46,rx:6,class:'cell'+(i===frame.active?' active':'')});element('text',{x:x+38,y:y+16,'text-anchor':'middle',class:'caption'},i===0?'x⁰':`x^${i}`);element('text',{x:x+38,y:y+34,'text-anchor':'middle',class:'value'},v);});y+=70;
 }
 svg.setAttribute('viewBox',`0 0 ${width} ${Math.max(220,y+15)}`);
}
const list=a=>'['+a.map(String).join(', ')+']';
function render(){
 if(!data)return;const frame=data.frames[index];draw(frame);$('step-title').textContent=frame.title;$('description').textContent=frame.description;$('formula').textContent=frame.formula||'各係数は有限体で計算します。';$('precision-value').textContent=frame.precision===undefined?'—':`${frame.precision}項`;$('modulus-value').textContent=String(data.config.p);$('mode-value').textContent=names[mode];$('progress').textContent=`${index+1} / ${data.frames.length}`;
 $('previous').disabled=index===0;for(const id of ['next','finish','milestone','play'])$(id).disabled=index===data.frames.length-1;
 $('result').textContent=frame.division?`商: ${list(frame.division.q)} / 剰余: ${list(frame.division.r)}`:frame.result!==null?`結果: ${list(frame.result)}`:index===data.frames.length-1&&mode==='sqrt'?'結果: 平方根なし':'結果: 計算途中';
 $('value-head').replaceChildren();const head=document.createElement('tr');for(const title of ['次数',...frame.rows.map(r=>r.name)]){const e=document.createElement('th');e.textContent=title;head.appendChild(e);}$('value-head').appendChild(head);$('values').replaceChildren();
 for(let i=0;i<Math.max(0,...frame.rows.map(r=>r.values.length));++i){const tr=document.createElement('tr');if(frame.active===i)tr.className='active';for(const value of [i,...frame.rows.map(r=>i<r.values.length?r.values[i]:'—')]){const td=document.createElement('td');td.textContent=String(value);tr.appendChild(td);}$('values').appendChild(tr);}
 $('tree-details').hidden=!frame.tree;$('tree-values').replaceChildren();if(frame.tree)for(let i=1;i<frame.tree.nodes.length;++i){const tr=document.createElement('tr');if(frame.node===i)tr.className='active';const partial=(frame.remainders||frame.work||[])[i];for(const value of ['M'+i,list(frame.tree.nodes[i]),partial?list(partial):'—']){const td=document.createElement('td');td.textContent=value;tr.appendChild(td);}$('tree-values').appendChild(tr);}
}
function run(){stop();visibility();$('input-error').textContent='';try{data=M.run(raw());index=0;render();}catch(error){data=null;$('input-error').textContent=error.message;$('result').textContent='入力を確認してください。';$('diagram').replaceChildren();$('values').replaceChildren();$('tree-details').hidden=true;for(const id of ['previous','next','finish','milestone','play'])$(id).disabled=true;$('progress').textContent='';}}
function visibility(){
 $('b-field').hidden=!['polynomial','division','multipoint','interpolation','bostan'].includes(mode);$('operation-field').hidden=mode!=='polynomial';$('calculus-field').hidden=mode!=='calculus';$('precision-field').hidden=!(['inv','log','exp','pow','sqrt'].includes(mode)||(mode==='calculus'&&$('calculus').value==='truncate'));$('exponent-field').hidden=!['pow','bostan'].includes(mode);$('shift-field').hidden=mode!=='taylor';
 $('a-label').textContent=mode==='interpolation'?'評価点 xs':mode==='bostan'?'分子 P の係数':'係数列 A';$('b-label').textContent=mode==='interpolation'?'各点での値 ys':mode==='multipoint'?'評価点 xs':mode==='bostan'?'分母 Q の係数':mode==='division'?'除数 B の係数':'係数列 B';$('exponent-label').textContent=mode==='bostan'?'取り出す項番号 k（0始まり）':'指数 k';$('conditions').textContent=conditions[mode];$('input-note').textContent='各列16個まで。空欄は空の係数列。負数も法で正規化します。';$('doc-link').href=`../../docs/fps/${docs[mode]}.md`;
 for(const button of document.querySelectorAll('[data-mode]'))button.setAttribute('aria-pressed',String(button.dataset.mode===mode));
}
function change(next){stop();mode=next;fill(M.defaults(mode));visibility();run();}
for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>{if(location.hash.slice(1)===button.dataset.mode)change(button.dataset.mode);else location.hash=button.dataset.mode;});
window.addEventListener('hashchange',()=>change(M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'polynomial'));
$('calculus').addEventListener('change',visibility);
$('run').addEventListener('click',run);$('scenario').addEventListener('click',()=>{fill(M.scenario(mode,$('scenario-type').value));visibility();run();});$('previous').addEventListener('click',()=>{stop();if(data&&index){index--;render();}});$('next').addEventListener('click',()=>{stop();if(data&&index<data.frames.length-1){index++;render();}});$('finish').addEventListener('click',()=>{stop();if(data){index=data.frames.length-1;render();}});$('milestone').addEventListener('click',()=>{stop();if(data){do{index++;}while(index<data.frames.length-1&&!data.frames[index].check);render();}});$('play').addEventListener('click',()=>{if(timer){stop();return;}if(!data)return;$('play').textContent='停止';timer=setInterval(()=>{if(index<data.frames.length-1){index++;render();}if(index===data.frames.length-1)stop();},500);});
change(mode);
})();
