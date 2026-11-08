(function(){
'use strict';
const M=NumberTheoryModel,$=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
const names={gcd:'GCD・LCM',extended:'拡張Euclid',pow:'mod累乗',inverse:'mod逆元',sieve:'素数篩',spf:'SPF',factor:'素因数分解',divisors:'約数列挙'};
const docs={gcd:'gcd_lcm',extended:'extended_gcd',pow:'mod_pow',inverse:'mod_inverse',sieve:'prime_sieve',spf:'smallest_prime_factor',factor:'prime_factorization',divisors:'divisors'};
const labels={gcd:'GCD',extended:'GCD',pow:'剰余',inverse:'逆元',sieve:'素数の個数',spf:'分解の結果',factor:'分解の結果',divisors:'約数の個数'};
const descriptions={
 gcd:['gcd(a,b) = gcd(b,a mod b)\nlcm(a,b) = |a|/gcd(a,b) × |b|','余りが小さくなるため、対数回で止まります。どちらかが0ならLCMは0です。','GCDは符号なしの非負結果です。LCMのULLONG_MAX超過はnullopt。'],
 extended:['a × x + b × y = r\n次の係数 = 前の係数 − 商 × 今の係数','最後の非零余りとその係数がBézoutの恒等式を満たします。','C++では余り0の不要な係数更新を省きます。この図ではBigIntでその係数も表示します。解の最小化は保証しません。'],
 pow:['答え × 底^(残りの指数) ≡ a^e (mod m)\n1のビットを採用し、底を二乗する','積には、処理済みのビットが表す冪だけを掛けます。表で各ビットの採用前後を比較できます。','e≥0、m≥1。C++は大きい法の乗算を加算・倍増へ切り替え、積のあふれを避けます。図解はBigIntで同じ剰余を求めます。'],
 inverse:['a × x + m × y = g\ng=1 のとき逆元 = x mod m','素数法を仮定しません。GCDが1以外なら、合同式a*x≡1を満たす整数はありません。','法1の代表元は0です。逆元なしはC++でnullopt。'],
 sieve:['素数 p の倍数 p², p²+p, … を消す','緑は未除去の候補、橙は今回除去した数です。√N以下の素数の処理が終わると、残った数がすべて素数です。','図解の上限は80。C++ではメモリを確保できるintの非負上限を指定できます。'],
 spf:['p ≤ SPF[i] のとき SPF[i×p] = p\nn → n / SPF[n] とたどる','i*pではpが最小素因数です。各合成数を一度だけ生成し、前処理後は表から素因数を取り出します。','表の0,1は特別扱い。図解のquery=0は分解を省く指定です。C++のfactorizeは1〜Nだけ。'],
 factor:['n = (見つけた素数の冪の積) × 残り\n候補 p ≤ 残り/p の間だけ試す','2を取り除いた後は奇数だけを試します。候補の平方が残りを超えたら、残りは1か素数です。','図解はn≤10,000。C++の入力型はlong longですが、最悪O(√n)の時間がかかります。'],
 divisors:['n % d = 0 ⇒ d と n/d は約数の対\n小さい列 + 大きい列の逆順','小さい列は昇順、大きい列は降順に増えるので、最後に逆順でつなげます。','正の約数だけを返します。平方根は重複させません。C++の走査はO(√n)。']};
let mode=M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'gcd',batch,index=0,timer=null;
function stop(){if(timer!==null){clearInterval(timer);timer=null;}$('play').textContent='再生';}
function fill(c){for(const id of ['a','b','exponent','modulus','limit','query','n'])if(c[id]!==undefined)$(id).value=c[id];}
function execute(){stop();try{const c={mode};for(const id of ['a','b','exponent','modulus','limit','query','n'])c[id]=$(id).value;batch=M.run(c);index=0;$('input-error').textContent='';render();}catch(error){$('input-error').textContent=error.message;}}
function choose(next){
 stop();mode=next;for(const button of document.querySelectorAll('[data-mode]'))button.setAttribute('aria-pressed',String(button.dataset.mode===mode));
 const fields={a:['gcd','extended','pow','inverse'],b:['gcd','extended'],exponent:['pow'],modulus:['pow','inverse'],limit:['sieve','spf'],query:['spf'],n:['factor','divisors']};
 for(const[id,modes]of Object.entries(fields))$(id+'-field').hidden=!modes.includes(mode);
 $('doc-link').href=`../../docs/number_theory_basic/${docs[mode]}.md`;$('result-label').textContent=labels[mode];
 $('input-note').textContent=['gcd','extended','pow','inverse'].includes(mode)?'a,bは符号付き64ビット。指数は0以上、法は1以上。大きい整数も十進でそのまま入力できます。':'図の大きさを抑えるため、篩は80、試し割りは10,000までです。';
 $('scenario-type').value='basic';fill(M.defaults(mode));execute();if(location.hash!==`#${mode}`)history.replaceState(null,'',`#${mode}`);
}
function svg(tag,attrs={},text){const node=document.createElementNS(NS,tag);for(const[k,v]of Object.entries(attrs))node.setAttribute(k,v);if(text!==undefined)node.textContent=text;$('diagram').appendChild(node);return node;}
function text(x,y,value,cls='caption',attrs={}){return svg('text',{x,y,class:cls,...attrs},String(value));}
function cell(x,y,w,h,label,value,kind=''){
 svg('rect',{x,y,width:w,height:h,rx:8,class:`cell ${kind}`});
 const labelNode=text(x+12,y+19,label,'small');
 const valueNode=text(x+12,y+h-16,value,'value',{style:`font-size:${String(value).length>24?10:String(value).length>15?12:16}px`});
 for(const node of [labelNode,valueNode])if(node.getComputedTextLength()>w-24){node.setAttribute('textLength',w-24);node.setAttribute('lengthAdjust','spacingAndGlyphs');}
}
function table(headers,rows){$('columns').replaceChildren();$('values').replaceChildren();const tr=document.createElement('tr');for(const h of headers){const th=document.createElement('th');th.textContent=h;tr.appendChild(th);}$('columns').appendChild(tr);for(const values of rows){const row=document.createElement('tr');for(const v of values){const td=document.createElement('td');td.textContent=v??'—';row.appendChild(td);}$('values').appendChild(row);}}
function render(){
 const f=batch.frames[index],c=batch.config;$('result').textContent=f.result;$('phase-value').textContent=f.phase;$('step-count').textContent=String(index);$('progress').textContent=`${index+1} / ${batch.frames.length}`;$('step-title').textContent=f.title;$('description').textContent=f.description;$('extra').textContent=f.extra;
 $('previous').disabled=index===0;for(const id of ['next','finish','milestone'])$(id).disabled=index===batch.frames.length-1;
 const[formula,principle,conditions]=descriptions[mode];$('formula').textContent=formula;$('principle').textContent=principle;$('conditions').textContent=conditions;$('details').textContent='';$('diagram').replaceChildren();text(24,28,names[mode]+' / '+f.phase,'heading');
 if(['gcd','extended','inverse'].includes(mode)){
  cell(24,52,350,68,'現在の余り r₀',f.r0,'selected');cell(400,52,350,68,'次の余り r₁',f.r1,'active');
  if(mode==='gcd'){
   const r0=BigInt(f.r0),r1=BigInt(f.r1),ratio=r0?Number(r1*680n/r0):0;
   svg('rect',{x:34,y:158,width:680,height:38,rx:6,class:'bar'});if(r1)svg('rect',{x:34,y:218,width:ratio||1,height:38,rx:6,class:'bar active'});
   text(34,145,'r₀ に対する r₁ の大きさ（長さは比率）');text(34,283,`商 = ${f.q??'—'}、割った余りを次の状態へ`);
   table(['割られる数','割る数','商','余り'],f.history.map(h=>[h.dividend,h.divisor,h.q,h.remainder]));
  }else{
   cell(24,143,350,68,'r₀ の係数 x, y',`${f.x0}, ${f.y0}`,'selected');cell(400,143,350,68,'r₁ の係数 x, y',`${f.x1}, ${f.y1}`,'active');
   text(24,247,'元の入力に対する恒等式');const b=mode==='inverse'?c.modulus:c.b;
   text(24,274,`${c.a} × (${f.x0}) + ${b} × (${f.y0}) = ${f.r0}`,'small');
   table(['余り','aの係数x',mode==='inverse'?'mの係数y':'bの係数y'],[[f.r0,f.x0,f.y0],[f.r1,f.x1,f.y1]]);
   $('details').textContent=`割り算 ${f.history.length} 回。`+(mode==='inverse'&&f.phase==='完了'?` ${f.result==='なし'?'GCD≠1のため逆元なし':`正規化した逆元: ${f.result}`}`:'');
  }
 }else if(mode==='pow'){
  cell(24,52,228,74,'現在の積',f.acc,'selected');cell(276,52,228,74,'次に掛ける底',f.value);cell(528,52,228,74,'残りの指数',f.remaining,'active');
  const shown=f.bits.slice(-8);text(24,158,`下位から処理したビット（直近${shown.length}個 / 全${f.bits.length}個）`);
  shown.forEach((b,j)=>cell(24+(j%4)*188,180+Math.floor(j/4)*70,174,60,`指数の重み ${b.weight}`,`${b.bit} → 積 ${b.acc}`,b.bit==='1'?'selected':''));
  table(['重み','bit','この桁の底','前の積','更新後の積'],f.bits.map(b=>[b.weight,b.bit,b.base,b.previous,b.acc]));
 }else if(mode==='sieve'||mode==='spf'){
  const count=c.limit+1,cols=10,width=68,height=40,start=50;
  for(let n=0;n<count;++n){const x=24+(n%cols)*73,y=start+Math.floor(n/cols)*48;
   const current=mode==='sieve'?f.marked.includes(n):n===f.i||f.products.some(p=>p.value===n);
   const selected=mode==='sieve'?f.table[n]:n>=2&&f.table[n]!==0;
   svg('rect',{x,y,width,height,rx:5,class:`cell ${current?'active':selected?'selected':''}`});
   if(mode==='sieve')text(x+width/2,y+25,n,f.table[n]?'value':'caption',{'text-anchor':'middle'});
   else{text(x+7,y+15,n,'small');text(x+width/2,y+33,f.table[n]||'—','value',{'text-anchor':'middle'});}
  }
  $('diagram').setAttribute('viewBox',`0 0 780 ${Math.max(280,start+Math.ceil(count/cols)*48+20)}`);
  if(mode==='sieve'){table(['数','現在の状態'],f.table.map((yes,n)=>[n,n<2?'非素数':yes?(f.phase==='完了'?'素数':'候補'):'合成数']));$('details').textContent=f.p?`素数p=${f.p}。今回初めて消した数: ${f.marked.join(', ')||'なし'}`:'橙は今回の除去。0と1は初めから候補に含めません。';}
  else{table(['n','SPF[n]','素数か'],f.table.map((p,n)=>[n,p,n<2?'—':p===n?'素数':p?'合成数':'未確定']));$('details').textContent=f.phase==='分解'||f.phase==='完了'?f.route.map(r=>`${r.before} ÷ ${r.p}^${r.exponent} → ${r.after}`).join(' / ')||'分解の経路は空です。':f.products.map(p=>`${f.i} × ${p.p} = ${p.value}`).join(' / ')||'この段階で生成する合成数はありません。';}
 }else if(mode==='factor'){
  cell(24,55,350,78,'残りの整数',f.remaining,'active');cell(400,55,350,78,'今回の候補',f.p??'—');
  text(24,170,'取り出した素数の冪');f.factors.forEach(([p,e],i)=>cell(24+(i%4)*178,195+Math.floor(i/4)*82,164,70,'素数 ^ 指数',`${p}^${e}`,'selected'));
  if(!f.factors.length)text(24,230,'まだ素因数はありません');
  table(['候補','割る前','指数','割った後'],f.trials.map(t=>[t.p,t.before,t.exponent,t.after]));
 }else{
  text(24,62,'小さい約数 →');f.small.forEach((d,i)=>cell(24+(i%8)*92,78+Math.floor(i/8)*58,82,52,'約数',d,'selected'));
  const y=95+Math.max(1,Math.ceil(f.small.length/8))*58;text(24,y,'大きい約数（最後に逆順へ） ←');f.large.forEach((d,i)=>cell(24+(i%8)*92,y+16+Math.floor(i/8)*58,82,52,'約数',d));
  $('diagram').setAttribute('viewBox',`0 0 780 ${Math.max(300,y+30+Math.ceil(f.large.length/8)*58+25)}`);
  table(['d','割り切れる','対になる約数'],f.trials.map(t=>[t.d,t.divides?'はい':'いいえ',t.partner]));$('details').textContent=f.divisors?`昇順: ${f.divisors.join(', ')}`:`今回調べるd = ${f.d??'—'}`;
 }
 if(!['sieve','spf','divisors'].includes(mode))$('diagram').setAttribute('viewBox',`0 0 780 ${mode==='factor'&&f.factors.length>4?380:340}`);
}
function move(next){index=Math.max(0,Math.min(batch.frames.length-1,next));render();if(index===batch.frames.length-1)stop();}
for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>choose(button.dataset.mode));
$('run').addEventListener('click',execute);$('scenario').addEventListener('click',()=>{fill(M.scenario(mode,$('scenario-type').value));execute();});
$('previous').addEventListener('click',()=>{stop();move(index-1);});$('next').addEventListener('click',()=>{stop();move(index+1);});$('finish').addEventListener('click',()=>{stop();move(batch.frames.length-1);});
$('milestone').addEventListener('click',()=>{stop();let next=index+1;while(next<batch.frames.length-1&&batch.frames[next].phase===batch.frames[index].phase)++next;move(next);});
$('play').addEventListener('click',()=>{if(timer!==null){stop();return;}if(index===batch.frames.length-1)move(0);$('play').textContent='停止';timer=setInterval(()=>move(index+1),550);});
window.addEventListener('hashchange',()=>{if(M.MODES.includes(location.hash.slice(1)))choose(location.hash.slice(1));});choose(mode);
})();
