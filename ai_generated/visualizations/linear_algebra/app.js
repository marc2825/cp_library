(function(){
'use strict';
const $=id=>document.getElementById(id),M=window.LinearAlgebraModel,NS='http://www.w3.org/2000/svg';
const names={matrix:'Matrix',product:'行列積',power:'行列累乗',gauss:'Gauss–Jordan',xor:'XOR基底'},docs={matrix:'matrix',product:'matrix_product',power:'matrix_power',gauss:'gauss_jordan',xor:'xor_basis'};
let mode=M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'matrix',data=null,index=0,timer=null;
const fmt=x=>typeof x==='number'?(Number.isInteger(x)&&Math.abs(x)<1e15?String(x):Number(x.toPrecision(7)).toString()):String(x),list=a=>'['+a.map(fmt).join(', ')+']',matrixText=a=>`${a.rows}×${a.cols} `+a.data.map(list).join(' ');
function stop(){clearInterval(timer);timer=null;$('play').textContent='再生';}
function fill(c){for(const id of ['rows','inner','cols','a','b','rhs','exponent','modulus','epsilon','vectors','seed','operation'])$(id).value=c[id];}
function raw(){const r={mode};for(const id of ['rows','inner','cols','a','b','rhs','exponent','modulus','epsilon','vectors','seed','operation'])r[id]=$(id).value;return r;}
function el(tag,attrs={},text){const e=document.createElementNS(NS,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=String(text);$('diagram').appendChild(e);return e;}
function draw(f){
 const svg=$('diagram'),c=data.config;svg.replaceChildren();let width=760,height=300;
 function grid(a,x,y,title,highlight=()=>'',separator=-1){
  const cell=Math.max(68,...a.data.flat().map(v=>fmt(v).length*7+16)),w=Math.max(120,a.cols*cell);el('text',{x,y,class:'heading'},`${title} (${a.rows}×${a.cols})`);
  if(!a.rows||!a.cols)el('text',{x:x+8,y:y+43,class:'caption'},'要素なし（次元は保持）');
  for(let j=0;j<a.cols;++j)el('text',{x:x+j*cell+cell/2,y:y+22,'text-anchor':'middle',class:'caption'},separator===j?'b':j);
  for(let i=0;i<a.rows;++i){el('text',{x:x-10,y:y+48+i*38,'text-anchor':'end',class:'caption'},i);for(let j=0;j<a.cols;++j){const px=x+j*cell,py=y+30+i*38;el('rect',{x:px,y:py,width:cell-7,height:29,rx:4,class:'cell'+highlight(i,j)});el('text',{x:px+(cell-7)/2,y:py+19,'text-anchor':'middle',class:'value'},fmt(a.data[i][j]));}}
  if(separator>0)el('line',{x1:x+separator*cell-4,y1:y+27,x2:x+separator*cell-4,y2:y+30+a.rows*38,class:'wire'});
  width=Math.max(width,x+w+25);height=Math.max(height,y+70+a.rows*38);return {w,h:70+a.rows*38};
 }
 if(mode==='xor'){
  const max=[...c.vectors,c.seed].reduce((a,b)=>a>b?a:b,0n),bits=Math.max(4,max.toString(2).length);width=Math.max(760,160+bits*18);
  el('text',{x:24,y:30,class:'heading'},f.phase==='query'?'数値順の候補を大きいpivotから比較':'最上位ビットをそろえてXOR消去');
  const rows=[['現在値',f.value??0n]];for(let bit=63;bit>=0;--bit)if(f.pivots[bit])rows.push([`pivot ${bit}`,f.pivots[bit]]);height=100+rows.length*65;
  for(let bit=0;bit<bits;++bit)el('text',{x:128+(bits-1-bit)*18,y:54,'text-anchor':'middle',class:'caption'},bit);
  rows.forEach(([name,value],r)=>{const y=68+r*65;el('text',{x:22,y:y+17,class:'caption'},name);for(let bit=0;bit<bits;++bit){const x=120+(bits-1-bit)*18,on=Number(value>>BigInt(bit)&1n);el('rect',{x,y,width:16,height:24,rx:3,class:'cell'+(on?(r?' source':' active'):'')});el('text',{x:x+8,y:y+17,'text-anchor':'middle',class:'caption'},on);}el('text',{x:120,y:y+43,class:'caption'},`整数: ${value}`);});
 }else if(mode==='matrix'){
  const a=grid(f.a,40,35,'A',(i,j)=>f.cell&&i===f.cell[0]&&j===f.cell[1]?' source':'');grid(f.target,80+a.w,35,'転置',(i,j)=>f.cell&&i===f.cell[1]&&j===f.cell[0]?' active':'');
 }else if(mode==='product'){
  const [row,col]=f.cell||[-1,-1],a=grid(f.a,40,35,'A',(i,j)=>i===row&&(f.k===undefined||j===f.k)?' source':'');const b=grid(f.b,90+a.w,35,'B',(i,j)=>j===col&&(f.k===undefined||i===f.k)?' source':'');grid(f.out,40,65+Math.max(a.h,b.h),'C',(i,j)=>i===row&&j===col?' active':'');
 }else if(mode==='power'){
  el('text',{x:25,y:30,class:'heading'},`確定した指数 ${f.used} / 次の重み ${f.weight}`);const a=grid(f.acc,40,76,'答え A^used',()=>f.stage==='select'?' active':'');grid(f.base,90+a.w,76,'基底 A^weight',()=>f.stage==='square'?' active':' source');
 }else{
  grid(f.r,40,45,'[A | b]',(i,j)=>i===f.row?' source':i===f.target?' active':j===f.col&&i===f.pivot?' source':'',c.cols);
 }
 svg.setAttribute('viewBox',`0 0 ${width} ${height}`);svg.style.width=width>950?`${width}px`:'100%';
}
function table(headings,rows){const head=document.createElement('tr');for(const value of headings){const th=document.createElement('th');th.textContent=value;head.appendChild(th);}$('detail-head').replaceChildren(head);$('details').replaceChildren();for(const row of rows){const tr=document.createElement('tr');for(const value of row){const td=document.createElement('td');td.textContent=String(value);tr.appendChild(td);}$('details').appendChild(tr);}}
function render(){
 if(!data)return;const f=data.frames[index],c=data.config;draw(f);$('mode-value').textContent=names[mode];$('metric-label').textContent=mode==='gauss'||mode==='xor'?'rank':mode==='power'?'確定した指数':'次元';$('metric').textContent=mode==='gauss'||mode==='xor'?f.rank:mode==='power'?String(f.used):`${c.a.rows}×${c.a.cols}`;$('state-label').textContent=mode==='power'?'残りの指数':mode==='xor'?'処理した入力':'段階';$('state').textContent=mode==='power'?String(f.remaining):mode==='xor'?String(f.prefix.length):({input:'入力',copy:'転置',term:'部分和',start:'開始',complete:'確定',done:'完了',pivot:'pivot選択',swap:'行交換',normalize:'正規化',eliminate:'消去',skip:'自由列'}[f.stage]||f.stage);$('step-title').textContent=f.title;$('description').textContent=f.description;$('progress').textContent=`${index+1} / ${data.frames.length}`;$('previous').disabled=!index;for(const id of ['next','play','milestone','finish'])$(id).disabled=index===data.frames.length-1;
 let result='計算途中';if(f.result!==null)result=mode==='gauss'?({none:'解なし',unique:'一意解',multiple:'自由変数あり'}[f.result.status]+` / rank=${f.result.rank} / 特解=${list(f.result.particular)}`):mode==='xor'?String(f.result):matrixText(f.result);$('result').textContent='結果: '+result;
 $('solution').hidden=mode!=='gauss'||f.result===null;$('solution-vectors').replaceChildren();if(mode==='gauss'&&f.result){$('solution-note').textContent=f.result.status==='none'?'矛盾する式があるため、特解と零空間のベクトルは返しません。':`x = 特解 + Σ t[j] * 基底[j]。自由変数の数は${f.result.basis.length}です。`;for(const [name,v]of [['特解',f.result.particular],...f.result.basis.map((v,i)=>[`基底${i}`,v])]){if(f.result.status==='none')break;const tr=document.createElement('tr');for(const value of [name,list(v)]){const td=document.createElement('td');td.textContent=value;tr.appendChild(td);}$('solution-vectors').appendChild(tr);}}
 if(mode==='product'){
  $('formula').textContent=f.cell?`C(${f.cell[0]},${f.cell[1]}) = Σ A(${f.cell[0]},k) × B(k,${f.cell[1]})`:'C(i,j) = Σ A(i,k) × B(k,j)';const [i,j]=f.cell||[0,0];table(['k','A(i,k)','B(k,j)','取り込んだ項'],Array.from({length:c.inner},(_,k)=>[k,c.a.data[i]?.[k]??'—',c.b.data[k]?.[j]??'—',f.stage==='term'?k<=f.k?'✓':'':f.stage==='complete'?'✓':'']));
 }else if(mode==='power'){$('formula').textContent=`A^${c.exponent} = A^${f.used} × (A^${f.weight})^${f.remaining}`;table(['値','保持している意味'],[['答え',`A^${f.used}`],['基底',`A^${f.weight}`],['残り',f.remaining],['次のbit',f.bit]]);
 }else if(mode==='gauss'){$('formula').textContent=f.stage==='eliminate'?`R${f.target} ← R${f.target} − (${fmt(f.factor)}) R${f.row}`:f.stage==='normalize'?`R${f.row} ← R${f.row} / (${fmt(f.divisor)})`:'[A|b] → [R|c],   x = x₀ + Σ t[j] v[j]';table(['pivot行','変数の列'],f.pivots.map((col,row)=>[row,col]));
 }else if(mode==='xor'){$('formula').textContent=f.stage==='query'?`${f.before} XOR ${f.pivots[f.bit]} = ${f.candidate}\n${f.take?'候補を採用':'現在値を保持'}`:'最上位の1が重なるベクトルをXORして消す';table(['pivotのビット','ベクトル'],f.pivots.map((value,bit)=>[bit,value]).filter(([,v])=>v!==0n).reverse());
 }else{$('formula').textContent='転置: B(j,i) = A(i,j)\n単位行列: I(i,j) = (i == j ? 1 : 0)';table(['行優先の位置','行','列','値'],c.a.data.flatMap((row,i)=>row.map((v,j)=>[i*c.a.cols+j,i,j,v])));}
 $('detail-title').textContent={matrix:'行優先の保存順',product:'現在の内積の項',power:'二分累乗の不変条件',gauss:'確定したpivot列',xor:'現在の基底'}[mode];
}
function visibility(){const x=mode==='xor';$('matrix-fields').hidden=x;$('xor-fields').hidden=!x;$('inner-field').hidden=mode!=='product';$('cols-field').hidden=mode==='power';$('b-field').hidden=mode!=='product';$('rhs-field').hidden=mode!=='gauss';$('exponent-field').hidden=mode!=='power';$('epsilon-field').hidden=mode!=='gauss'||$('modulus').value!=='real';for(const option of $('modulus').options)option.disabled=mode==='gauss'?!['real','2','17','998244353'].includes(option.value):option.value==='real';$('doc-link').href=`../../docs/linear_algebra/${docs[mode]}.md`;$('input-note').textContent=x?'値は0〜2^64−1。空のXORも許し、0は必ず表せます。':'各次元0〜5。行列と右辺の項数を次元に合わせます。';$('conditions').textContent=x?'基底はpivotの最上位ビットで整理します。値自体は挿入順で変わり得ます。min/maxは符号なしの数値順です。':mode==='gauss'?'実数は絶対epsilonと部分ピボット、modは素数法の逆元を使います。合成数法の一般の連立方程式は扱いません。':'整数は各積と部分和がlong longに収まる必要があります。mod型の積・累乗は合成数法や法1でも使えます。';for(const button of document.querySelectorAll('[data-mode]'))button.setAttribute('aria-pressed',String(button.dataset.mode===mode));}
function run(){stop();visibility();$('input-error').textContent='';try{data=M.run(raw());index=0;render();}catch(error){data=null;$('input-error').textContent=error.message;$('result').textContent='入力を確認してください。';$('diagram').replaceChildren();$('details').replaceChildren();$('solution').hidden=true;for(const id of ['previous','next','play','milestone','finish'])$(id).disabled=true;$('progress').textContent='';}}
function change(next){mode=next;fill(M.defaults(mode));run();}
for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>{if(location.hash.slice(1)===button.dataset.mode)change(button.dataset.mode);else location.hash=button.dataset.mode;});window.addEventListener('hashchange',()=>change(M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'matrix'));
$('modulus').addEventListener('change',visibility);$('run').addEventListener('click',run);$('scenario').addEventListener('click',()=>{fill(M.scenario(mode,$('scenario-type').value));run();});$('previous').addEventListener('click',()=>{stop();if(data&&index){index--;render();}});$('next').addEventListener('click',()=>{stop();if(data&&index<data.frames.length-1){index++;render();}});$('finish').addEventListener('click',()=>{stop();if(data){index=data.frames.length-1;render();}});$('milestone').addEventListener('click',()=>{stop();if(data){do{index++;}while(index<data.frames.length-1&&!['complete','select','insert','done'].includes(data.frames[index].stage));render();}});$('play').addEventListener('click',()=>{if(timer){stop();return;}if(!data)return;$('play').textContent='停止';timer=setInterval(()=>{if(index<data.frames.length-1){index++;render();}if(index===data.frames.length-1)stop();},500);});change(mode);
})();
