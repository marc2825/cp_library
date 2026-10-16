(function(root){
'use strict';
const MODES=['polynomial','calculus','inv','log','exp','pow','sqrt','division','multipoint','interpolation','taylor','bostan'];
const norm=(a,p)=>(a%p+p)%p;
function power(a,k,p){let r=1n;for(;k;k>>=1n,a=a*a%p)if(k&1n)r=r*a%p;return r;}
function inverse(a,p){if(!a)throw Error('逆元がありません。定数項は非0が必要です。');return power(a,p-2n,p);}
const pre=(a,n)=>Array.from({length:n},(_,i)=>a[i]||0n);
function trim(a){a=[...a];while(a.length&&!a.at(-1))a.pop();return a;}
function multiply(a,b,p,n){if(n===undefined)n=a.length&&b.length?a.length+b.length-1:0;const c=Array(n).fill(0n);for(let i=0;i<a.length&&i<n;++i)for(let j=0;j<b.length&&i+j<n;++j)c[i+j]=(c[i+j]+a[i]*b[j])%p;return c;}
const add=(a,b,p,sign=1n)=>Array.from({length:Math.max(a.length,b.length)},(_,i)=>norm((a[i]||0n)+sign*(b[i]||0n),p));
const derivative=(a,p)=>a.slice(1).map((v,i)=>v*BigInt(i+1)%p);
function integral(a,p){if(BigInt(a.length)>=p)throw Error('積分の添字が法以上です。');return [0n,...a.map((v,i)=>v*inverse(BigInt(i+1),p)%p)];}
function scalarRoot(a,p){
 if(!a)return 0n;if(power(a,(p-1n)/2n,p)!==1n)return null;
 let r;if(p%4n===3n)r=power(a,(p+1n)/4n,p);else{
  let s=0,q=p-1n;while(!(q&1n)){s++;q>>=1n;}let z=2n;while(power(z,(p-1n)/2n,p)===1n)z++;
  let c=power(z,q,p),t=power(a,q,p);r=power(a,(q+1n)/2n,p);
  while(t!==1n){let i=1,w=t*t%p;while(w!==1n){w=w*w%p;i++;}const b=power(c,1n<<BigInt(s-i-1),p);r=r*b%p;c=b*b%p;t=t*c%p;s=i;}
 }return r<p-r?r:p-r;
}
function defaults(mode){return {mode,a:mode==='exp'?'0 1':mode==='pow'?'0 2 1':mode==='sqrt'?'0 0 4 4 1':mode==='bostan'?'0 1':mode==='interpolation'?'0 1 2':mode==='inv'?'1 -1':'1 2 3',b:mode==='division'?'1 1':mode==='bostan'?'1 -1 -1':mode==='multipoint'?'0 1 2':mode==='interpolation'?'1 6 0':'4 1',modulus:'17',precision:'6',exponent:mode==='bostan'?'10':'3',shift:'2',operation:'multiply',calculus:'derivative'};}
function scenario(mode,type){const c=defaults(mode);if(type==='zero'){c.a='';if(mode==='interpolation')c.b='';}if(type==='precision-zero')c.precision='0';if(type==='leading')c.a='0 0 4 4 1';if(type==='nonresidue')c.a='3';if(type==='invalid-constant')c.a=mode==='exp'?'1 2':'0 1';if(type==='duplicate'){c.a=mode==='interpolation'?'1 18':'1 2 3';c.b=mode==='interpolation'?'2 2':'1 18 2';}if(type==='zero-divisor')c.b='0 0';if(type==='large-index'){c.exponent='18446744073709551615';if(mode==='pow')c.a='0 1';}if(type==='large-modulus'){c.modulus='998244353';c.precision='12';}return c;}
function parse(raw){
 const mode=MODES.includes(raw.mode)?raw.mode:'polynomial',p=BigInt(raw.modulus||17),limit=p===17n?8: p===97n?16:4194304;
 if(![17n,97n,998244353n].includes(p))throw Error('法は17、97、998244353から選んでください。');
 function integer(value,name,low,high){if(!/^-?\d+$/.test(String(value)))throw Error(`${name}は整数で指定します。`);const x=BigInt(value);if(x<low||x>high)throw Error(`${name}が範囲外です。`);return x;}
 function coefficients(value){const s=String(value||'').trim();if(!s)return [];const a=s.split(/[\s,]+/).map(v=>integer(v,'係数',-(1n<<31n),(1n<<31n)-1n));if(a.length>16)throw Error('係数列は16要素までです。');return a.map(v=>norm(v,p));}
 const a=coefficients(raw.a),b=coefficients(raw.b),n=Number(integer(raw.precision??6,'出力精度',0n,16n)),k=integer(raw.exponent??3,'指数・項番号',0n,(1n<<64n)-1n),c=norm(integer(raw.shift??2,'平行移動量',-(1n<<63n),(1n<<63n)-1n),p);
 const operation=['add','subtract','multiply'].includes(raw.operation)?raw.operation:'multiply',calculus=['derivative','integral','truncate'].includes(raw.calculus)?raw.calculus:'derivative';
 if(['inv','log','exp','pow','sqrt'].includes(mode)&&n>limit)throw Error(`法${p}の出力精度は${limit}以下です。`);
 if(mode==='polynomial'&&operation==='multiply'&&a.length&&b.length&&a.length+b.length-1>2*limit)throw Error('積の長さがNTT容量を超えます。');
 if(['division','bostan'].includes(mode)&&Math.max(trim(a).length,trim(b).length)>limit)throw Error(`多項式の有効長は${limit}以下です。`);
 if(mode==='multipoint'&&b.length&&(b.length>=limit||trim(a).length>limit))throw Error(`点数は${limit-1}以下、多項式の有効長は${limit}以下です。`);
 if(mode==='interpolation'&&(a.length!==b.length))throw Error('点と値の長さが一致していません。');
 if(mode==='interpolation'&&a.length>=limit)throw Error(`補間の点数は${limit-1}以下です。`);
 if(mode==='taylor'&&a.length>limit)throw Error(`多項式の長さは${limit}以下です。`);
 return {mode,a,b,p,n,k,c,operation,calculus};
}
function run(raw){
 const c=parse(raw),{a,b,p,n}=c,frames=[];let tree=null,node=0;
 const emit=(title,description,rows=[],formula='',extra={})=>frames.push(structuredClone({title,description,rows:rows.map(([name,values])=>({name,values})),formula,tree,node,result:null,...extra}));
 const done=result=>{emit('計算の完了',result===null?'平方根は存在しません。':'入力した条件での結果です。',[['結果',result||[]]],'',{result});return {config:c,frames,result};};
 function inv(f,size,trace=true){
  if(!size)return [];if(!f[0])throw Error('invには非0の定数項が必要です。');let g=[inverse(f[0],p)];
  if(trace)emit('逆数の初期値','定数項の逆元から始めます。',[['f',pre(f,size)],['g',g]],'g₀ = 1/f₀',{precision:1});
  while(g.length<size){const m=Math.min(size,2*g.length),fg=multiply(pre(f,m),g,p,m),e=fg.map(x=>norm(-x,p));e[0]=norm(e[0]+2n,p);
   if(trace)emit(`逆数: 精度${m}への補正`,'f*gの誤差を打ち消す係数列2-f*gを作ります。',[['f*g',fg],['2-f*g',e],['旧g',g]],'g ← g(2-fg) mod x^m',{precision:m});
   g=multiply(g,e,p,m);if(trace)emit(`逆数: ${m}項が確定`,'f*gの先頭m項は1,0,…,0です。',[['g',g],['f*g',multiply(f,g,p,m)]],'f*g ≡ 1 (mod x^m)',{precision:m,check:'inverse',source:f,current:g});
  }return g;
 }
 function log(f,size,trace=true){
  if(!size)return [];if(f[0]!==1n)throw Error('logには定数項1が必要です。');const d=derivative(pre(f,size),p),g=inv(f,size,false),q=multiply(d,g,p,size-1),r=integral(q,p);
  if(trace){emit('log: 微分と逆数','定数項1のfを微分し、逆数と掛けます。',[['f\u2032',d],['1/f',g]],'log(f) = ∫ f\u2032/f dx');emit('log: 積分前の係数','必要なsize-1項に切り詰めてから積分します。',[['f\u2032/f',q],['log(f)',r]],'定数項を0として積分');}return r;
 }
 function exp(f,size,trace=true){
  if(!size)return [];if(f[0])throw Error('expには定数項0が必要です。');let g=[1n];if(trace)emit('expの初期値','定数項1から精度を倍増します。',[['g',g]],'g₀=1',{precision:1});
  while(g.length<size){const m=Math.min(size,2*g.length),l=log(g,m,false),e=add(pre(f,m),l,p,-1n);e[0]=(e[0]+1n)%p;
   if(trace)emit(`exp: 精度${m}への補正`,'現在のlog(g)と目標fとの差を求めます。',[['f',pre(f,m)],['log(g)',l],['1+f-log(g)',e]],'g ← g(1+f-log(g)) mod x^m',{precision:m});
   g=multiply(g,e,p,m);if(trace)emit(`exp: ${m}項が確定`,'補正後のgは、この精度でlog(g)=fを満たします。',[['g',g]],'log(g) ≡ f (mod x^m)',{precision:m,check:'exp',source:f,current:g});
  }return g;
 }
 function divmod(f,h,trace=true){
  let r=trim(f),d=trim(h);if(!d.length)throw Error('零多項式では除算できません。');const q=Array(Math.max(0,r.length-d.length+1)).fill(0n);
  if(trace)emit('除算: 次数を確認','最高次の項から商を決め、divisor*商の項を引きます。',[['f',r],['divisor',d],['商',q]],'f = divisor*q + r');
  while(r.length>=d.length){const i=r.length-d.length;q[i]=r.at(-1)*inverse(d.at(-1),p)%p;const term=Array(i).fill(0n).concat(d.map(x=>x*q[i]%p));r=trim(add(r,term,p,-1n));
   if(trace)emit(`除算: 商の${i}次項`,'最高次の項を消した後の剰余を表示します。',[['商',q],['引く項',term],['剰余',r]],'f = divisor*q + r',{active:i,check:'division',source:f,divisor:d,quotient:q,remainder:r});
  }return [trim(q),r];
 }
 function build(xs){
  let base=1;while(base<xs.length)base*=2;const nodes=Array.from({length:2*base},()=>[]);tree={base,count:xs.length,nodes};
  for(let i=0;i<base;++i)nodes[base+i]=i<xs.length?[norm(-xs[i],p),1n]:[1n];
  emit('積木: 葉の一次式','各評価点xᵢに対するX-xᵢを置きます。余った葉は1です。',[['点',xs]],'Mᵢ(X)=X-xᵢ');
  for(let i=base-1;i;--i){node=i;nodes[i]=multiply(nodes[2*i],nodes[2*i+1],p);emit(`積木: M${i}を構築`,'子の多項式を掛けて、この区間の一次式の積を作ります。',[['左',nodes[2*i]],['右',nodes[2*i+1]],['積',nodes[i]]],'Mᵢ=M₂ᵢ*M₂ᵢ₊₁');}node=0;return tree;
 }
 function evaluate(f,t,trace=true){
  const values=Array(t.count).fill(0n),remainders=Array.from({length:t.nodes.length},()=>[]);
  function descend(i,r){node=i;remainders[i]=r;if(trace)emit(`評価: 節点M${i}の剰余`,'同じ根を持つ多項式で割った剰余を子へ渡します。葉では定数が評価値です。',[['剰余',r]],'f(xᵢ) = (f mod M)(xᵢ)',{remainders});
   if(i>=t.base){if(i-t.base<t.count)values[i-t.base]=r[0]||0n;return;}
   descend(2*i,divmod(r,t.nodes[2*i],false)[1]);if(t.nodes[2*i+1].length>1)descend(2*i+1,divmod(r,t.nodes[2*i+1],false)[1]);
  }if(t.count)descend(1,divmod(f,t.nodes[1],false)[1]);node=0;return values;
 }
 emit('入力と係数の順序','左から定数項、x¹、x²、…の係数です。法で正規化した値を表示します。',[['A',a],['B',b]],`mod ${p}`);
 if(c.mode==='polynomial'){
  if(c.operation!=='multiply'){const result=add(a,b,p,c.operation==='subtract'?-1n:1n);for(let i=0;i<result.length;++i)emit(`係数${i}を${c.operation==='add'?'加算':'減算'}`,'同じ次数の係数を計算します。足りない項は0です。',[['A',a],['B',b],['結果',result.slice(0,i+1)]],'cᵢ = aᵢ ± bᵢ',{active:i});return done(result);}
  const result=Array(a.length&&b.length?a.length+b.length-1:0).fill(0n);
  for(let k=0;k<result.length;++k){const terms=[];for(let i=0;i<a.length;++i){const j=k-i;if(j>=0&&j<b.length){result[k]=(result[k]+a[i]*b[j])%p;terms.push(`${i}+${j}`);}}emit(`積の${k}次係数`,'添字の和がkになる項の積を集めます。',[['A',a],['B',b],['積',result]],terms.length?`i+j=${k}: ${terms.join(', ')}`:'寄与する項なし',{active:k});}return done(result);
 }
 if(c.mode==='calculus'){
  const result=c.calculus==='derivative'?derivative(a,p):c.calculus==='integral'?integral(a,p):pre(a,n);
  for(let i=0;i<result.length;++i)emit(`係数${i}を計算`,c.calculus==='truncate'?'高次の項を捨て、不足する項は0で埋めます。':c.calculus==='integral'?'次数を1増やし、その次数の逆元を掛けます。定数項は0です。':'次数を掛け、次数を1減らします。',[['入力',a],['出力',result.slice(0,i+1)]],c.calculus==='truncate'?`mod x^${n}`:c.calculus==='integral'?'bᵢ₊₁=aᵢ/(i+1)':'bᵢ=(i+1)aᵢ₊₁',{active:i});return done(result);
 }
 if(c.mode==='inv')return done(inv(a,n));
 if(c.mode==='log')return done(log(a,n));
 if(c.mode==='exp')return done(exp(a,n));
 if(c.mode==='pow'){
  if(!n)return done([]);if(!c.k)return done(pre([1n],n));const v=a.findIndex((x,i)=>i<n&&x);if(v<0||BigInt(v)*c.k>=BigInt(n))return done(Array(n).fill(0n));
  const s=Number(BigInt(v)*c.k),size=n-s,lead=a[v],unit=pre(a.slice(v),size).map(x=>x*inverse(lead,p)%p);
  emit('累乗: 先頭の非0項を分離','a=x^v*c*u、u₀=1と分解します。次数の移動量v*kは整数として計算します。',[['u',unit]],`v=${v}, v*k=${s}, c=${lead}`);
  const l=log(unit,size),scaled=l.map(x=>x*(c.k%p)%p);emit('累乗: 対数をk倍','係数に掛けるkは法で計算します。次数の移動量とは別です。',[['log(u)',l],['k log(u)',scaled]],'u^k=exp(k log(u))');
  const r=exp(scaled,size),result=Array(n).fill(0n),scale=power(lead,c.k,p);for(let i=0;i<r.length;++i)result[s+i]=r[i]*scale%p;emit('累乗: 定数倍と次数の移動','c^kを掛けてx^(v*k)だけ移動します。',[['結果',result]],'a^k=x^(v*k)*c^k*exp(k log(u))');return done(result);
 }
 if(c.mode==='sqrt'){
  if(!n)return done([]);const v=a.findIndex((x,i)=>i<n&&x);if(v<0)return done(Array(n).fill(0n));if(v%2){emit('平方根なし: 奇数の先頭次数','平方の最小次数は必ず偶数です。',[['f',pre(a,n)]],`v=${v}`);return done(null);}const r=scalarRoot(a[v],p);if(r===null){emit('平方根なし: 非平方剰余','最初の非0係数が有限体で平方になりません。',[['f',pre(a,n)]],`係数${a[v]}に平方根なし`);return done(null);}
  const size=n-v,f=pre(a.slice(v),size);let g=[r];emit('平方根: 初期値と次数の移動','先頭の係数の平方根は、小さい方を選びます。',[['f/x^v',f],['g',g]],`v=${v}, shift=${v/2}, g₀=${r}`,{precision:1});
  while(g.length<size){const m=Math.min(size,2*g.length),q=multiply(pre(f,m),inv(g,m,false),p,m);emit(`平方根: 精度${m}への補正`,'gとf/gの平均で精度を倍増します。',[['旧g',g],['f/g',q]],'g ← (g+f/g)/2 mod x^m',{precision:m});g=add(pre(g,m),q,p).map(x=>x*inverse(2n,p)%p);emit(`平方根: ${m}項が確定`,'正規化した系列の平方を確認できます。',[['g',g],['g²',multiply(g,g,p,m)]],'g² ≡ f/x^v (mod x^m)',{precision:m,check:'sqrt',source:f,current:g});}
  return done(pre(Array(v/2).fill(0n).concat(g),n));
 }
 if(c.mode==='division'){const [q,r]=divmod(a,b);emit('商と剰余','剰余の次数は除数より小さくなります。',[['商q',q],['剰余r',r]],'f=divisor*q+r',{division:{q,r}});const output=done(q);output.remainder=r;output.frames.at(-1).division={q,r};return output;}
 if(c.mode==='multipoint'){
  if(!b.length)return done([]);const t=build(b);return done(evaluate(a,t));
 }
 if(c.mode==='interpolation'){
  if(!a.length)return done([]);const t=build(a),d=derivative(t.nodes[1],p),den=evaluate(d,t,false);node=0;
  if(den.some(x=>!x))throw Error('補間点は法で相異なる必要があります。');const weights=b.map((y,i)=>y*inverse(den[i],p)%p),work=Array.from({length:2*t.base},()=>[0n]);
  emit('補間: 導関数と重み','M′(xᵢ)で値を割り、Lagrangeの重みを作ります。',[['M\u2032',d],['M\u2032(xᵢ)',den],['重み',weights]],'wᵢ=yᵢ/M\u2032(xᵢ)');
  for(let i=0;i<t.count;++i)work[t.base+i]=[weights[i]];
  for(let i=t.base-1;i;--i){node=i;work[i]=add(multiply(work[2*i],t.nodes[2*i+1],p),multiply(work[2*i+1],t.nodes[2*i],p),p);emit(`補間: 節点${i}を合成`,'左の答えには右の積、右の答えには左の積を掛けて加えます。',[['左の答え',work[2*i]],['右の答え',work[2*i+1]],['合成後',work[i]]],'Hᵢ=H₂ᵢ*M₂ᵢ₊₁+H₂ᵢ₊₁*M₂ᵢ',{work});}node=0;return done(pre(work[1],a.length));
 }
 if(c.mode==='taylor'){
  if(!a.length)return done([]);const fact=Array(a.length).fill(1n),ifac=Array(a.length).fill(1n);for(let i=1;i<a.length;++i)fact[i]=fact[i-1]*BigInt(i)%p;for(let i=0;i<a.length;++i)ifac[i]=inverse(fact[i],p);
  const left=a.map((x,i)=>x*fact[i]%p).reverse(),right=a.map((_,i)=>power(c.c,BigInt(i),p)*ifac[i]%p);emit('Taylor Shift: 階乗で重み付け','aᵢ*i!を逆順に並べ、c^i/i!との積に帰着します。',[['i!',fact],['逆順 aᵢ*i!',left],['c^i/i!',right]],'g(x)=f(x+c)');const z=multiply(left,right,p);emit('Taylor Shift: 畳み込み','積の先頭n項を使います。',[['積',z]],'gᵢ = 積[n-1-i]/i!');return done(a.map((_,i)=>z[a.length-1-i]*ifac[i]%p));
 }
 if(c.mode==='bostan'){
  let P=trim(a),Q=trim(b),k=c.k;if(!Q[0])throw Error('Bostan–Moriには非0の分母定数項が必要です。');
  while(k&&P.length){const neg=Q.map((x,i)=>i%2?norm(-x,p):x),s=multiply(P,neg,p),t=multiply(Q,neg,p);emit(`Bostan–Mori: k=${k}`,'分母にQ(-x)を掛けると奇数次の係数が0になります。',[['P',P],['Q',Q],['Q(-x)',neg],['P*Q(-x)',s],['Q*Q(-x)',t]],'P/Q = P*Q(-x)/(Q(x)Q(-x))',{k,check:'bostan-product',numerator:P,denominator:Q,denominatorProduct:t});const parity=Number(k&1n);P=trim(s.filter((_,i)=>i%2===parity));Q=trim(t.filter((_,i)=>i%2===0));k>>=1n;emit('Bostan–Mori: 偶奇の係数を抽出',`分子は${parity?'奇数':'偶数'}次、分母は偶数次を選び、kを半分にします。`,[['新P',P],['新Q',Q]],`次のk=${k}`,{k,check:'bostan-reduced',numerator:P,denominator:Q});}
  return done([P.length?P[0]*inverse(Q[0],p)%p:0n]);
 }
 throw Error('unknown mode');
}
const api={MODES,defaults,scenario,parse,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.FPSModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
