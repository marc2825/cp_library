(function(root){
'use strict';
const MODES=['gcd','extended','pow','inverse','sieve','spf','factor','divisors'];
const MIN=-(1n<<63n),MAX=(1n<<63n)-1n,UMAX=(1n<<64n)-1n;
const clone=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?String(v):v));
const abs=x=>x<0n?-x:x;
function big(value,label,lo=MIN,hi=MAX){
 if(typeof value==='number'&&!Number.isSafeInteger(value))throw new Error(`${label}は十進文字列で入力してください。`);
 const s=String(value??'').trim();if(!/^-?\d+$/.test(s))throw new Error(`${label}は十進整数にしてください。`);
 const x=BigInt(s);if(x<lo||x>hi)throw new Error(`${label}は ${lo}〜${hi} の範囲にしてください。`);return String(x);
}
function small(value,label,lo,hi){return Number(big(value,label,BigInt(lo),BigInt(hi)));}
function normalize(input){
 const mode=input.mode||'gcd';if(!MODES.includes(mode))throw new Error('未対応のモードです。');const c={mode};
 if(['gcd','extended','inverse','pow'].includes(mode))c.a=big(input.a,'a');
 if(mode==='gcd'||mode==='extended')c.b=big(input.b,'b');
 if(mode==='pow'){c.exponent=big(input.exponent,'指数',0n);c.modulus=big(input.modulus,'法',1n);}
 if(mode==='inverse')c.modulus=big(input.modulus,'法',1n);
 if(mode==='sieve'||mode==='spf')c.limit=small(input.limit,'上限',0,80);
 if(mode==='spf')c.query=small(input.query,'分解する数（0は分解なし）',0,c.limit);
 if(mode==='factor'||mode==='divisors')c.n=big(input.n,'n',1n,10000n);
 return c;
}
function defaults(mode){return {mode,a:mode==='inverse'?'3':mode==='pow'?'-2':'84',b:'30',exponent:'13',modulus:mode==='pow'?'1000':'10',limit:40,query:36,n:mode==='factor'?'360':'36'};}
function scenario(mode,kind){
 const c=defaults(mode);if(kind==='basic')return c;
 if(kind==='zero'){
  if(mode==='gcd'||mode==='extended'){c.a='0';c.b='0';}
  if(mode==='pow'){c.a='0';c.exponent='0';c.modulus='1';}
  if(mode==='inverse'){c.a='0';c.modulus='1';}
  if(mode==='sieve'||mode==='spf'){c.limit=0;c.query=0;}
  if(mode==='factor'||mode==='divisors')c.n='1';
 }else if(kind==='negative'){c.a='-30';c.b='18';c.modulus='11';}
 else if(kind==='non-coprime'){c.a='6';c.b='10';c.modulus='10';}
 else if(kind==='boundary'){
  if(mode==='gcd'||mode==='extended'){c.a=String(MIN);c.b=String(MAX);}
  if(mode==='pow'){c.a=String(MAX-1n);c.exponent=String(MAX);c.modulus=String(MAX);}
  if(mode==='inverse'){c.a=String(MIN);c.modulus=String(MAX);}
  if(mode==='sieve'||mode==='spf'){c.limit=80;c.query=72;}
  if(mode==='factor'||mode==='divisors')c.n='9409';
 }else if(kind==='prime'){c.a='89';c.b='55';c.n='97';c.limit=49;c.query=49;}
 else throw new Error('未対応の例です。');return c;
}
function run(input){
 const c=normalize(input),frames=[];
 const save=frame=>frames.push(clone({phase:'計算',result:'未確定',extra:'',...frame}));
 if(['gcd','extended','inverse'].includes(c.mode)){
  const a=BigInt(c.a),b=BigInt(c.mode==='inverse'?c.modulus:c.b);
  let r0=abs(a),r1=abs(b),x0=a<0n?-1n:1n,y0=0n,x1=0n,y1=b<0n?-1n:1n,history=[];
  const frame=(title,description,more={})=>save({phase:'互除法',title,description,r0,r1,x0,y0,x1,y1,history,...more});
  frame('絶対値から始める','各余りを元の符号付きa,bの線形結合として持ちます。GCDは非負です。');
  while(r1){
   const q=r0/r1,r=r0%r1;history.push({dividend:r0,divisor:r1,q,remainder:r});
   const x=x0-q*x1,y=y0-q*y1;
   r0=r1;r1=r;x0=x1;x1=x;y0=y1;y1=y;
   frame(`商 ${q}、余り ${r}`,`前の余り − 商 × 次の余り。係数も同じ引き算で更新します。`,{q});
  }
  if(!a&&!b){x0=0n;y0=0n;}
  const lcm=(!a||!b)?0n:abs(a)/r0*abs(b);
  let result=String(r0),extra=`LCM = ${lcm>UMAX?'範囲超過（ULLONG_MAX超）':lcm}`;
  if(c.mode==='extended')extra=`x = ${x0}, y = ${y0}`;
  if(c.mode==='inverse'){result=r0===1n?String((x0%b+b)%b):'なし';extra=`gcd(a,m) = ${r0}`;}
  frame('結果',c.mode==='inverse'?(r0===1n?'係数xを [0,m) に正規化した値が逆元です。法1では代表元0を返します。':'GCDが1でないため逆元は存在しません。'):'最後の非零の余りがGCDです。LCMは先にGCDで割ってから積を求めます。',{phase:'完了',result,extra,lcm:lcm<=UMAX?String(lcm):null});
 }else if(c.mode==='pow'){
  const m=BigInt(c.modulus),a=BigInt(c.a),e=BigInt(c.exponent);
  let value=(a%m+m)%m,remaining=e,acc=1n%m,weight=1n,used=0n;const bits=[];
  const frame=(title,description,more={})=>save({phase:'二進展開',title,description,value,remaining,acc,weight,used,bits,...more});
  frame('初期値','底を [0,m) に正規化し、積を1 mod mにします。');
  while(remaining){
   const bit=remaining&1n,base=value,previous=acc;
   if(bit){acc=acc*value%m;used+=weight;}
   bits.push({weight,bit,base,previous,acc});remaining>>=1n;value=value*value%m;weight<<=1n;
   frame(`ビット ${bits.length-1}: ${bit}`,bit?'この桁を採用して積に掛けます。底を二乗し、指数を右へ1ビットずらします。':'この桁は掛けずに進みます。底を二乗し、指数を右へ1ビットずらします。');
  }
  frame('結果','処理した指数の合計が元の指数に一致しました。0^0は1、法1では0です。',{phase:'完了',result:String(acc),extra:`採用した指数 = ${used}`});
 }else if(c.mode==='sieve'){
  const n=c.limit,table=Array(n+1).fill(true);table[0]=false;if(n>=1)table[1]=false;
  const frame=(title,description,more={})=>save({phase:'篩',title,description,table,primes:[],marked:[],p:null,...more});
  frame('0と1を除外','緑の数は素数候補です。まだすべてが素数と確定したわけではありません。');
  for(let p=2;p*p<=n;++p)if(table[p]){
   const marked=[];for(let j=p*p;j<=n;j+=p){if(table[j])marked.push(j);table[j]=false;}
   frame(`素数 ${p} の倍数を消す`,`${p}²=${p*p}から消します。それ未満の合成数は小さい素因数で消えています。`,{p,marked});
  }
  const primes=table.flatMap((yes,i)=>yes?[i]:[]);
  frame('結果','√N以下の素数で篩うと、残った2以上の数がすべて素数です。',{phase:'完了',result:String(primes.length),extra:primes.join(', ')||'素数なし',primes});
 }else if(c.mode==='spf'){
  const n=c.limit,table=Array(n+1).fill(0),primes=[],factors=[],route=[];if(n>=1)table[1]=1;
  const frame=(title,description,more={})=>save({phase:'線形篩',title,description,table,primes,factors,route,i:null,products:[],remaining:null,...more});
  frame('未確定を0にする','表の0と1は特別扱いです。2以上の0は最小素因数がまだ未確定です。');
  for(let i=2;i<=n;++i){
   if(!table[i]){table[i]=i;primes.push(i);}
   const products=[];for(const p of primes){if(p>table[i]||p>Math.floor(n/i))break;table[i*p]=p;products.push({value:i*p,p});}
   frame(`i = ${i}, SPF = ${table[i]}`,`p ≤ SPF[i] の素数だけでi*pを生成します。同じ合成数を二度生成しません。`,{i,products});
  }
  let remaining=c.query;
  if(remaining>0)while(remaining>1){const before=remaining,p=table[remaining];let exponent=0;do{remaining/=p;++exponent;}while(remaining>1&&table[remaining]===p);factors.push([p,exponent]);route.push({before,p,exponent,after:remaining});frame(`最小素因数 ${p} で割る`,`${before} ÷ ${p}^${exponent} = ${remaining}`,{phase:'分解',remaining});}
  frame('結果',c.query===0?'分解する数0は、図解で分解なしを選ぶための値です。C++のfactorize(0)は拒否します。':c.query===1?'1の素因数分解は空です。':'表をたどるたびに残りの数が小さくなります。',{phase:'完了',result:c.query===0?'分解なし':factors.map(([p,e])=>`${p}^${e}`).join(' × ')||'1',extra:`素数 ${primes.length} 個`,remaining});
 }else if(c.mode==='factor'){
  let remaining=BigInt(c.n);const factors=[],trials=[];
  const frame=(title,description,more={})=>save({phase:'試し割り',title,description,remaining,factors,trials,p:null,...more});
  frame('2から試す','割れるだけ割り、その回数を指数として残します。');
  for(let p=2n;p<=remaining/p;p+=p===2n?1n:2n){const before=remaining;let exponent=0;while(remaining%p===0n){remaining/=p;++exponent;}if(exponent)factors.push([p,exponent]);trials.push({p,before,exponent,after:remaining});frame(`候補 ${p}`,exponent?`${p}で${exponent}回割り、残りは${remaining}。`:'割れないので次の候補へ進みます。',{p});}
  if(remaining>1n){const last=remaining;factors.push([last,1]);remaining=1n;frame(`残り ${last} は素数`,'残りの平方根以下の候補ではもう割れないため、そのまま最後の素因数です。');}
  frame('結果',BigInt(c.n)===1n?'1の素因数分解は空です。':'素数の冪を掛け戻すと元の整数になります。',{phase:'完了',result:factors.map(([p,e])=>`${p}^${e}`).join(' × ')||'1'});
 }else{
  const n=BigInt(c.n),small=[],large=[],trials=[];
  const frame=(title,description,more={})=>save({phase:'約数の対',title,description,small,large,trials,d:null,...more});
  frame('1から平方根まで','小さい約数dと大きい約数n/dを対で集めます。');
  for(let d=1n;d<=n/d;++d){const divides=n%d===0n;if(divides){small.push(d);if(d!==n/d)large.push(n/d);}trials.push({d,divides,partner:divides?n/d:null});frame(`d = ${d}`,divides?(d===n/d?'平方根なので一度だけ加えます。':`${d} と ${n/d} をそれぞれの列へ追加します。`):'割り切れないので追加しません。',{d});}
  const divisors=small.concat([...large].reverse());
  frame('結果','小さい列に、大きい列の逆順をつなぐと昇順になります。',{phase:'完了',result:String(divisors.length),extra:divisors.join(', '),divisors});
 }
 return {config:c,frames};
}
const api={MODES,normalize,defaults,scenario,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;root.NumberTheoryModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
