(function(){
'use strict';
const M=ArithmeticConvolutionModel,$=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
const names={'divisor-zeta':'約数ゼータ','divisor-mobius':'約数メビウス','multiple-zeta':'倍数ゼータ','multiple-mobius':'倍数メビウス',dirichlet:'Dirichlet',gcd:'GCD畳み込み',lcm:'LCM畳み込み'};
const files={'divisor-zeta':'divisor_zeta_transform','divisor-mobius':'divisor_mobius_transform','multiple-zeta':'multiple_zeta_transform','multiple-mobius':'multiple_mobius_transform',dirichlet:'dirichlet_convolution',gcd:'gcd_convolution',lcm:'lcm_convolution'};
const notes={
 'divisor-zeta':['F[n] = Σ(d | n) f[d]\nf[k*p] += f[k]  /  kを小さい順に','素数の冪の鎖を、小さい指数から大きい指数へ累積します。','添字0は保持します。係数は乗法的関数でなくてもかまいません。'],
 'divisor-mobius':['f[n] = Σ(d | n) μ(n/d) F[d]\nf[k*p] -= f[k]  /  逆順で戻す','素数を大きい順、kも大きい順に走査し、ゼータ変換の更新を取り消します。','μは平方因子があれば0、それ以外は素因数の個数で符号が決まります。図のμは定義の説明用です。'],
 'multiple-zeta':['F[n] = Σ(n | m, m ≤ N) f[m]\nf[k] += f[k*p]  /  kを大きい順に','素数の冪の鎖を、大きい指数から小さい指数へ累積します。','上限より大きい倍数は集計しません。'],
 'multiple-mobius':['f[n] = Σ(kn ≤ N) μ(k) F[kn]\nf[k] -= f[k*p]  /  逆順で戻す','素数を大きい順、kを小さい順に走査し、倍数ゼータ変換の更新を取り消します。','GCDがnの倍数の集計から、GCDがnちょうどの集計へ戻すときにも使います。'],
 dirichlet:['c[n] = Σ(i*j = n) a[i]*b[j]','添字の積へ加えます。1*1なら約数個数、id*1なら約数和になります。','出力上限は入力の最大長−1。上限を拡張したい場合は入力を0で埋めます。'],
 gcd:['倍数ゼータ → 要素積 → 倍数メビウス\nc[k] = Σ(gcd(i,j) = k) a[i]*b[j]','i,jがともにkの倍数なら、GCDもkの倍数です。この集計を逆変換して、ちょうどのGCDへ分けます。','小さいGCDにも大きい添字が寄与します。入力を先に切らないでください。'],
 lcm:['約数ゼータ → 要素積 → 約数メビウス\nc[k] = Σ(lcm(i,j) = k) a[i]*b[j]','i,jがともにkの約数なら、LCMもkの約数です。この集計を逆変換して、ちょうどのLCMへ分けます。','Nを超えるLCMは出力しません。定義の表と、出力上限を変えた例で確認できます。']};
let mode=M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'divisor-zeta',batch,index=0,timer=null;
const conv=()=>['dirichlet','gcd','lcm'].includes(mode);
function stop(){if(timer!==null){clearInterval(timer);timer=null;}$('play').textContent='再生';}
function fill(c){for(const id of ['a','b','zero','modulus','focus'])if(c[id]!==undefined)$(id).value=Array.isArray(c[id])?c[id].join(' '):c[id];}
function execute(){stop();try{const c={mode};for(const id of ['a','b','zero','modulus','focus'])c[id]=$(id).value;batch=M.run(c);index=0;$('focus').value=batch.config.focus||1;$('input-error').textContent='';render();}catch(error){$('input-error').textContent=error.message;}}
function choose(next){stop();mode=next;for(const button of document.querySelectorAll('[data-mode]'))button.setAttribute('aria-pressed',String(button.dataset.mode===mode));$('b-field').hidden=!conv();$('zero-field').hidden=conv();$('a-label').textContent=conv()?'a[1], a[2], …':'f[1], f[2], …';$('doc-link').href=`../../docs/number_theory_convolution/${files[mode]}.md`;$('scenario-type').value='basic';fill(M.defaults(mode));execute();if(location.hash!==`#${mode}`)history.replaceState(null,'',`#${mode}`);}
function svg(tag,attrs={},value){const node=document.createElementNS(NS,tag);for(const[k,v]of Object.entries(attrs))node.setAttribute(k,v);if(value!==undefined)node.textContent=String(value);$('diagram').appendChild(node);return node;}
function text(x,y,value,cls='caption',attrs={}){return svg('text',{x,y,class:cls,...attrs},value);}
function fit(node,width){if(node.getComputedTextLength()>width){node.setAttribute('textLength',width);node.setAttribute('lengthAdjust','spacingAndGlyphs');}}
function cell(x,y,w,h,label,value,kind=''){svg('rect',{x,y,width:w,height:h,rx:7,class:`cell ${kind}`});fit(text(x+10,y+17,label,'small'),w-20);fit(text(x+10,y+h-13,value,'value',{style:`font-size:${String(value).length>18?11:String(value).length>11?13:17}px`}),w-20);}
function table(headId,bodyId,headers,rows){$(headId).replaceChildren();$(bodyId).replaceChildren();const header=document.createElement('tr');for(const value of headers){const th=document.createElement('th');th.textContent=value;header.appendChild(th);}$(headId).appendChild(header);appendRows(bodyId,rows);}
function appendRows(id,rows){for(const values of rows){const row=document.createElement('tr');for(const value of values){const td=document.createElement('td');td.textContent=value??'—';row.appendChild(td);}$(id).appendChild(row);}}
function mu(n){let result=1;for(let p=2;p<=n;++p)if(n%p===0){n/=p;result=-result;if(n%p===0)return 0;}return result;}
function gcd(a,b){while(b){[a,b]=[b,a%b];}return a;}
function definition(c){const rows=[];let total=0n;const n=c.focus,m=BigInt(c.modulus),reduce=x=>m?(x%m+m)%m:x;
 if(!conv()){
  const divisor=mode.startsWith('divisor'),inverse=mode.endsWith('mobius');for(let j=1;j<=c.a.length;++j)if(divisor?n%j===0:j%n===0){const coefficient=inverse?mu(divisor?n/j:j/n):1,value=BigInt(c.a[j-1])*BigInt(coefficient);rows.push([j,coefficient,c.a[j-1],String(reduce(value))]);total+=value;}
  table('term-columns','terms',['元の添字','係数 μ / 1','元の値','寄与'],rows);
 }else{
  for(let i=1;i<=c.a.length;++i)for(let j=1;j<=c.b.length;++j){const target=mode==='dirichlet'?i*j:mode==='gcd'?gcd(i,j):i/gcd(i,j)*j;if(target===n){const value=BigInt(c.a[i-1])*BigInt(c.b[j-1]);rows.push([i,j,c.a[i-1]+' × '+c.b[j-1],String(reduce(value))]);total+=value;}}
  table('term-columns','terms',['i','j','a[i] × b[j]','寄与'],rows);
 }
 $('terms-title').textContent=n?`添字 ${n} の定義（元の入力から直接集計）`:'正の範囲は空';$('term-total').textContent=n?`定義から求めた最終値：${reduce(total)}${m?' (mod '+m+')':''}`:'正の添字はありません。';
}
function render(){const c=batch.config,f=batch.frames[index],op=f.operation;$('result-label').textContent=c.focus?`${conv()?'c':'f'}[${c.focus}] の結果`:'正の範囲';$('result').textContent=f.result;$('phase-value').textContent=f.phase;$('bound').textContent=c.n;$('progress').textContent=`${index+1} / ${batch.frames.length}`;$('step-title').textContent=f.title;$('description').textContent=f.description;$('previous').disabled=index===0;for(const id of ['next','finish','milestone'])$(id).disabled=index===batch.frames.length-1;
 const[formula,principle,conditions]=notes[mode];$('formula').textContent=formula;$('principle').textContent=principle;$('conditions').textContent=conditions+' 標準整数の途中値があふれる場合、C++は例外になります。';$('extra').textContent=`${f.done?'出力':'作業中'}: ${f.output.slice(1).join(', ')||'空'}\n${conv()?'出力の添字0 = 0':`保持した f[0] = ${f.a[0]}`} / ${c.modulus==='0'?'整数':'mod '+c.modulus}`;
 $('diagram').replaceChildren();text(24,28,names[mode]+' / '+f.phase,'heading');
 let y=48;if(conv()&&mode!=='dirichlet'){
  const stages=[['a-zeta','Aのゼータ'],['b-zeta','Bのゼータ'],['product','要素積'],['h-mobius','逆変換']];stages.forEach(([id,label],i)=>{svg('rect',{x:24+i*188,y,width:166,height:36,rx:7,class:`pipeline ${f.stage===id?'active':''}`});text(36+i*188,y+24,label,'small');if(i<3)text(198+i*188,y+24,'→');});y+=55;
 }
 if(op?.kind==='add'||op?.kind==='subtract'){
  cell(24,y,220,64,`更新前 f[${op.target}]`,op.before);text(272,y+37,op.kind==='add'?'+':'−','value');cell(310,y,214,64,`読む f[${op.source}]`,op.value,'source');text(542,y+37,'→','value');cell(578,y,176,64,`更新後 f[${op.target}]`,op.after,'target');y+=87;
 }else if(op?.kind==='pair'||op?.kind==='multiply'){
  text(24,y+23,op.kind==='pair'?`a[${op.i}] × b[${op.j}] → c[${op.target}]`:`A'[${op.target}] × B'[${op.target}]`,'heading');cell(390,y,170,62,'加える / 掛けた値',op.value,'source');cell(580,y,174,62,'現在の結果',op.after,'target');y+=85;
 }else{const node=text(24,y+22,f.done?'計算が終わりました。下の定義の表と比較できます。':f.p?`素数 p=${f.p}`:'入力から更新の履歴を作っています。');fit(node,720);y+=44;}
 const drawRow=(row,label,rowId)=>{text(24,y+15,label,'caption');y+=25;if(!c.n){text(24,y+23,'正の添字なし');y+=54;return;}for(let i=1;i<=c.n;++i){const source=(f.active===rowId&&op?.source===i)||(op?.kind==='pair'&&((rowId==='a'&&op.i===i)||(rowId==='b'&&op.j===i)))||(op?.kind==='multiply'&&(rowId==='a'||rowId==='b')&&op.target===i),target=f.active===rowId&&op?.target===i;cell(24+((i-1)%6)*124,y+Math.floor((i-1)/6)*66,114,57,`${i}`,row[i],source?'source':target?'target':c.focus===i?'focus':'');}y+=Math.ceil(c.n/6)*66+15;};
 drawRow(f.a,conv()?(mode==='dirichlet'?'入力 A':"A（ゼータ中・後は A'）"):'作業中の f','a');if(conv()){drawRow(f.b,mode==='dirichlet'?'入力 B':"B（ゼータ中・後は B'）",'b');drawRow(f.h,mode==='dirichlet'?'積の添字へ集計する C':'C（要素積 → 逆変換 → 結果）','h');}
 $('diagram').setAttribute('viewBox',`0 0 780 ${Math.max(280,y+12)}`);
 const a0=[conv()?'0':c.zero,...c.a],b0=['0',...c.b];
 table('columns','values',conv()?['添字','元A','元B','作業A','作業B','作業C']:['添字','元の値','作業中'],Array.from({length:c.n+1},(_,i)=>conv()?[i,a0[i]??'0',b0[i]??'0',f.a[i],f.b[i],f.h[i]]:[i,a0[i],f.a[i]]));
 const stageNames={transform:names[mode],'a-zeta':'Aのゼータ','b-zeta':'Bのゼータ',product:'要素積','h-mobius':'逆変換',direct:'積へ集計'};
 $('history').replaceChildren();appendRows('history',f.history.map(h=>[stageNames[h.stage],h.target,h.kind==='add'?`+ ${h.value}`:h.kind==='subtract'?`− ${h.value}`:h.kind==='pair'?`(${h.i},${h.j})`: '×',h.after]));definition(c);
}
function move(next){index=Math.max(0,Math.min(batch.frames.length-1,next));render();if(index===batch.frames.length-1)stop();}
for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>choose(button.dataset.mode));$('run').addEventListener('click',execute);$('scenario').addEventListener('click',()=>{fill(M.scenario(mode,$('scenario-type').value));execute();});$('previous').addEventListener('click',()=>{stop();move(index-1);});$('next').addEventListener('click',()=>{stop();move(index+1);});$('finish').addEventListener('click',()=>{stop();move(batch.frames.length-1);});$('milestone').addEventListener('click',()=>{stop();let next=index+1;while(next<batch.frames.length-1&&!batch.frames[next].checkpoint)++next;move(next);});$('play').addEventListener('click',()=>{if(timer!==null){stop();return;}if(index===batch.frames.length-1)move(0);$('play').textContent='停止';timer=setInterval(()=>move(index+1),550);});window.addEventListener('hashchange',()=>{if(M.MODES.includes(location.hash.slice(1)))choose(location.hash.slice(1));});choose(mode);
})();
