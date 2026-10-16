(function(root){
'use strict';
const MODES=['matrix','kitamasa','berlekamp','bostan'];
const norm=(x,p)=>(x%p+p)%p;
function scalarPower(a,k,p){let r=1n%p;for(;k;k>>=1n,a=a*a%p)if(k&1n)r=r*a%p;return r;}
const inverse=(x,p)=>scalarPower(x,p-2n,p);
const pre=(a,n)=>Array.from({length:n},(_,i)=>a[i]||0n);
function trim(a){a=[...a];while(a.length&&!a.at(-1))a.pop();return a;}
function product(a,b,p,n){if(n===undefined)n=a.length&&b.length?a.length+b.length-1:0;const r=Array(n).fill(0n);for(let i=0;i<a.length&&i<n;++i)for(let j=0;j<b.length&&i+j<n;++j)r[i+j]=(r[i+j]+a[i]*b[j])%p;return r;}
function matrixProduct(a,b,p){return a.map(row=>b[0].map((_,j)=>row.reduce((s,x,k)=>(s+x*b[k][j])%p,0n)));}
const apply=(a,s,p)=>a.map(row=>row.reduce((r,x,i)=>(r+x*s[i])%p,0n));
function defaults(mode){return {mode,a:mode==='berlekamp'?'0 1 1 2 3 5 8 13':'0 1',b:'1 1',n:'10',modulus:'17',source:'recurrence',matrix:'1 1\n1 0'};}
function scenario(mode,type){const c=defaults(mode);if(type==='zero')c.a=mode==='berlekamp'?'0 0 0 0':'0 0';if(type==='empty')c.a='';if(type==='n-zero')c.n='0';if(type==='huge')c.n='18446744073709551615';if(type==='composite')c.modulus='12';if(type==='prime-large')c.modulus='1000000007';if(type==='asymmetric'){c.a='2 5 9';c.b='2 -3 4';}if(type==='late')c.a='0 0 1';if(type==='trailing-zero')c.a='1 0 0 0';if(type==='affine'){c.source='custom';c.a='2 1';c.matrix='1 3\n0 1';c.modulus='12';c.n='5';}if(type==='improper'){c.source='rational';c.a='1 2 3 4';c.b='2';c.n='3';}if(type==='q-zero'){c.source='rational';c.a='1';c.b='0 1';}return c;}
function parse(raw){
 const mode=MODES.includes(raw.mode)?raw.mode:'matrix';
 if(!/^\d+$/.test(String(raw.modulus)))throw Error('法を選んでください。');const p=BigInt(raw.modulus);if(![1n,2n,12n,17n,97n,998244353n,1000000007n].includes(p))throw Error('利用できる法ではありません。');
 function integer(text,name,low,high){if(!/^-?\d+$/.test(String(text)))throw Error(`${name}は整数で指定します。`);const x=BigInt(text);if(x<low||x>high)throw Error(`${name}が範囲外です。`);return x;}
 function coefficients(text,max){const a=String(text||'').trim();if(!a)return [];const values=a.split(/[\s,]+/).map(x=>norm(integer(x,'係数',-(1n<<31n),(1n<<31n)-1n),p));if(values.length>max)throw Error(`入力列は${max}項までです。`);return values;}
 const a=coefficients(raw.a,mode==='berlekamp'?20:mode==='bostan'&&raw.source==='rational'?12:6),b=coefficients(raw.b,mode==='bostan'&&raw.source==='rational'?12:6),n=integer(raw.n,'項番号・回数',0n,(1n<<64n)-1n),source=String(raw.source||'recurrence');let matrix=[];
 if(mode==='berlekamp'&&[1n,12n].includes(p))throw Error('Berlekamp–Masseyには素数法が必要です。');
 if(mode==='matrix'&&source==='custom'){
  const rows=String(raw.matrix||'').trim();matrix=rows?rows.split(/\n+/).map(row=>coefficients(row,6)):[];
  if(matrix.length>6||matrix.some(row=>row.length!==matrix.length)||a.length!==matrix.length)throw Error('行列は6×6以下の正方行列で、状態の長さと一致させてください。');
 }else if(['matrix','kitamasa','bostan'].includes(mode)&&!(mode==='bostan'&&source==='rational')){
  if(!a.length||a.length!==b.length)throw Error('初期値と係数は同じ非空長にしてください。');
 }
 if(mode==='bostan'){
  if(![17n,97n,998244353n].includes(p))throw Error('このBostan–Moriの法は17、97、998244353から選びます。');
  const limit=p===17n?8:p===97n?16:4194304;
  if(source==='rational'){if(!b[0])throw Error('分母の定数項は非0が必要です。');if(Math.max(trim(a).length,trim(b).length)>limit)throw Error(`P/Qの有効長は${limit}以下です。`);}
  else if(n>=BigInt(a.length)&&a.length>=limit)throw Error('漸化式の次数がNTT作業容量を超えます。');
 }
 return {mode,a,b,n,p,source,matrix};
}
function run(raw){
 const config=parse(raw),{mode,a,b,n,p}=config,frames=[];
 function emit(title,description,rows=[],formula='',extra={}){frames.push(structuredClone({title,description,rows:rows.map(([name,values])=>({name,values})),formula,result:null,...extra}));}
 function done(result,extra={}){emit('計算の完了','指定した法で正規化した結果です。',[['結果',result]],'',{result,...extra});return {config,frames,result,...extra};}
 emit('入力と係数の順序','c[0]は直前、c[1]は2つ前の項に掛ける係数です。',[['初期値／観測',a],['係数c',b]],'a[t] = Σ c[j] a[t-1-j]');
 function combine(left,right,c,trace=true){
  const d=c.length,r=product(left,right,p);if(trace)emit('Kitamasa: 係数を掛ける','初期項の重みを多項式とみなして積を作ります。',[['左',left],['右',right],['積',r]],'重み = 左*右 mod F(X)',{stage:'multiply'});
  for(let i=r.length-1;i>=d;--i){const value=r[i];for(let j=0;j<d;++j)r[i-1-j]=(r[i-1-j]+value*c[j])%p;r[i]=0n;if(trace)emit(`Kitamasa: X^${i}を還元`,'X^dを漸化式の係数で置き換え、高い次数を消します。',[['還元中',r],['c',c]],`X^d ≡ c[0]X^(d-1)+…+c[d-1]`,{stage:'reduce',active:i,degree:d,left,right});}
  return r.slice(0,d);
 }
 function kitamasa(initial,c,index,trace=true){
  const d=c.length;let k=index,acc=Array(d).fill(0n),base=Array(d).fill(0n),bit=0,exponent=0n;acc[0]=1n%p;if(d===1)base[0]=c[0];else base[1]=1n%p;
  if(trace)emit('Kitamasa: 単位の重み','X^nを特性多項式で割った剰余を二分累乗します。',[['累積r',acc],['基底X',base]],'a[n] = Σ weight[i] a[i]',{stage:'weights',exponent,weights:acc});
  while(k){if(k&1n){acc=combine(acc,base,c,trace);exponent+=1n<<BigInt(bit);if(trace)emit(`Kitamasa: bit ${bit}を採用`,'累積の剰余はX^processedの初期項に対する重みです。',[['重み',acc],['初期値',initial]],`processed=${exponent}`,{stage:'weights',exponent,weights:acc});}k>>=1n;if(k)base=combine(base,base,c,trace);bit++;}
  return {weights:acc,value:acc.reduce((s,x,i)=>(s+x*initial[i])%p,0n)};
 }
 if(mode==='matrix'){
  if(config.source!=='custom'&&n<BigInt(a.length))return done([a[Number(n)]],{initialLookup:true});
  let matrix,state,k;
  if(config.source==='custom'){matrix=config.matrix;state=a;k=n;}
  else{matrix=Array.from({length:b.length},()=>Array(b.length).fill(0n));matrix[0]=[...b];for(let i=1;i<b.length;++i)matrix[i][i-1]=1n%p;state=[...a].reverse();k=n-BigInt(a.length-1);}
  let bit=0,processed=0n;const start=[...state],transition=structuredClone(matrix);
  emit('行列: 状態と遷移を作る','列ベクトルの左から行列を掛けます。通常の漸化式では新しい項を先頭に置きます。',[['状態',state]],'s(k)=T^k s(0)',{matrix,bit,processed,start,transition,stage:'matrix'});
  while(k&&matrix.length){
   if(k&1n){state=apply(matrix,state,p);processed+=1n<<BigInt(bit);emit(`行列: bit ${bit}を採用`,'この2冪回の遷移を状態へ適用します。',[['更新後の状態',state]],`適用した回数=${processed}`,{matrix,bit,processed,start,transition,state,stage:'matrix-apply'});}
   k>>=1n;if(k){matrix=matrixProduct(matrix,matrix,p);emit('行列: 遷移を二乗','T^(2^bit)から、2倍の回数を進める遷移を作ります。',[['状態',state]],`次はbit ${bit+1}`,{matrix,bit:bit+1,processed,start,transition,stage:'matrix-square'});}bit++;
  }return done(config.source==='custom'?state:[state[0]]);
 }
 if(mode==='kitamasa'){
  if(n<BigInt(a.length))return done([a[Number(n)]],{initialLookup:true});const r=kitamasa(a,b,n);return done([r.value],{weights:r.weights});
 }
 if(mode==='berlekamp'){
  let C=[1n],B=[1n],L=0,m=1,last=1n;
  for(let index=0;index<a.length;++index){let delta=a[index];const terms=[];for(let j=1;j<=L;++j){const value=C[j]*a[index-j]%p;delta=(delta+value)%p;terms.push({j,index:index-j,c:C[j],sample:a[index-j],value});}
   emit(`BM: 観測${index}の不一致量`,'C[0]=1の接続多項式で、観測値を説明できるか確かめます。',[['観測',a],['C',C],['保存B',B]],'delta = s[i] + Σ C[j]s[i-j]',{stage:'discrepancy',index,L,m,last,delta,C,B,terms,active:index});
   if(!delta){m++;emit('BM: 不一致なし','現在の接続式でこの観測を説明できます。保存した失敗点からの距離だけ増やします。',[['C',C]],'delta=0',{stage:'bm-stable',index,L,m,C});continue;}
   const saved=[...C],factor=delta*inverse(last,p)%p,shifted=Array(m).fill(0n).concat(B.map(x=>factor*x%p));C=pre(C,Math.max(C.length,shifted.length)).map((x,i)=>norm(x-(shifted[i]||0n),p));const grows=2*L<=index;
   if(grows){L=index+1-L;B=saved;last=delta;m=1;}else m++;
   emit(grows?'BM: 次数を増やして補正':'BM: 同じ次数で補正','以前の不一致を打ち消す式をずらして引き、処理した観測すべてを満たす接続式へ直します。',[['旧C',saved],['引く式',shifted],['新C',C]],'C ← C - (delta/last) X^m B',{stage:'bm-corrected',index,L,m,last,delta,C,B,factor});
  }
  const c=C.slice(1,L+1).map(x=>norm(-x,p));emit('BM: 漸化式の係数へ変換','c[j]=-C[j+1]。末尾0も次数の一部なので残します。',[['接続C',pre(C,L+1)],['係数c',c]],`推定次数L=${L}`,{stage:'bm-result',L,C,c});
  const value=n<BigInt(a.length)?a[Number(n)]:c.length?kitamasa(a.slice(0,c.length),c,n,false).value:0n;
  return done(c,{target:value,inferred:true});
 }
 if(mode==='bostan'){
  let P,Q,k=n;
  if(config.source==='rational'){P=trim(a);Q=trim(b);}
  else{
   if(n<BigInt(a.length))return done([a[Number(n)]],{initialLookup:true});Q=[1n,...b.map(x=>norm(-x,p))];P=trim(product(a,Q,p,a.length));
   emit('母関数: P/Qへ変換','Q=1-c[0]x-…とすると、Q*Aのd次以上は漸化式で0になります。',[['初期項A',a],['分母Q',Q],['分子P',P]],'A(x)=P(x)/Q(x), P=(initial*Q) mod x^d',{stage:'generating',P,Q});Q=trim(Q);
  }
  while(k&&P.length){const negative=Q.map((x,i)=>i%2?norm(-x,p):x),s=product(P,negative,p),t=product(Q,negative,p);emit(`Bostan–Mori: k=${k}`,'Q(-x)を掛けると分母は偶数次だけになり、項番号を半分にできます。',[['P',P],['Q',Q],['Q(-x)',negative],['分子の積',s],['分母の積',t]],'P/Q = P*Q(-x) / (Q(x)Q(-x))',{stage:'bostan-product',k,P,Q,denominatorProduct:t});const parity=Number(k&1n);P=trim(s.filter((_,i)=>i%2===parity));Q=trim(t.filter((_,i)=>!(i%2)));k>>=1n;emit('Bostan–Mori: 偶奇を抽出',`分子は${parity?'奇数':'偶数'}次、分母は偶数次を選びます。`,[['新P',P],['新Q',Q]],`次のk=${k}`,{stage:'bostan-reduced',k,P,Q});}
  return done([P.length?P[0]*inverse(Q[0],p)%p:0n]);
 }
 throw Error('unknown mode');
}
const api={MODES,defaults,scenario,parse,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.LinearRecurrenceModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
