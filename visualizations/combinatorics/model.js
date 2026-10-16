(function(root){
'use strict';
const MODES=['factorial','binomial','permutation','combination','catalan','multiset','inclusion'];
const clone=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?String(v):v));
function integer(value,label,lo,hi){if(typeof value==='number'&&!Number.isSafeInteger(value))throw new Error(`${label}は十進文字列で入力してください。`);const s=String(value??'').trim();if(!/^-?\d+$/.test(s))throw new Error(`${label}は整数にしてください。`);const x=BigInt(s);if(x<lo||x>hi)throw new Error(`${label}は${lo}〜${hi}の範囲にしてください。`);return x;}
function normalize(input){const mode=input.mode||'factorial';if(!MODES.includes(mode))throw new Error('未対応のモードです。');
 const c={mode,modulus:String(integer(input.modulus??(mode==='inclusion'?0:1000000007),'法',mode==='inclusion'?0n:1n,(1n<<63n)-1n))};
 if(mode==='inclusion'){
  c.universe=Number(integer(input.universe??8,'全体集合の要素数',0n,12n));c.target=input.target||'union';if(!['union','complement'].includes(c.target))throw new Error('包除の対象を選んでください。');
  let lines=Array.isArray(input.sets)?input.sets:String(input.sets??'').trim().split('\n').filter(s=>s.trim());if(lines.length>6)throw new Error('集合は6個以内にしてください。');
  c.sets=lines.map(line=>{const values=Array.isArray(line)?line:String(line).trim()==='-'?[]:String(line).trim().split(/[\s,]+/).filter(Boolean);return [...new Set(values.map(x=>Number(integer(x,'集合の要素',0n,BigInt(c.universe-1)))))].sort((a,b)=>a-b);});
 }else{
  c.n=Number(integer(input.n??5,'n',-1n,12n));c.k=Number(integer(input.k??2,'k',-1n,12n));c.bound=Number(integer(input.bound??6,'前計算の上限',0n,24n));
  c.backend=['factorial','permutation'].includes(mode)?'factorial':input.backend||'factorial';if(!['factorial','pascal'].includes(c.backend))throw new Error('計算法を選んでください。');
  if(mode==='factorial'&&(c.n<0||c.n>c.bound))throw new Error('階乗のnは0〜前計算上限にしてください。');
 }
 return c;
}
function defaults(mode){return {mode,n:mode==='catalan'?4:mode==='multiset'?3:5,k:['permutation','combination'].includes(mode)?3:mode==='multiset'?4:2,bound:mode==='catalan'?8:6,backend:'factorial',modulus:mode==='inclusion'?'0':'1000000007',universe:8,sets:'0 1 2 3\n2 3 4 5\n3 5 6',target:'union'};}
function scenario(mode,kind){const c=defaults(mode);if(kind==='basic')return c;
 if(kind==='empty'){c.n=0;c.k=0;c.bound=0;c.sets='';c.universe=0;}
 else if(kind==='composite'){c.modulus='12';c.backend='pascal';if(mode==='permutation'){c.n=2;c.k=1;c.bound=2;c.modulus='15';}}
 else if(kind==='nonunit'){c.modulus='7';c.bound=7;c.backend='factorial';}
 else if(kind==='mod-one')c.modulus='1';
 else if(kind==='large-mod')c.modulus='9223372036854775783';
 else if(kind==='out-of-range'){c.bound=2;c.n=5;c.k=2;}
 else if(kind==='overlap'){c.universe=3;c.sets='0 1 2\n0 1 2';c.target='complement';}
 else if(kind==='zero-types'){c.n=0;c.k=4;}
 else throw new Error('未対応の例です。');return c;
}
function inverse(a,m){let r0=a,r1=m,x0=1n,x1=0n;while(r1){const q=r0/r1;[r0,r1]=[r1,r0%r1];[x0,x1]=[x1,x0-q*x1];}return r0===1n?(x0%m+m)%m:null;}
function run(input){const c=normalize(input),m=BigInt(c.modulus),reduce=x=>m?(x%m+m)%m:x,frames=[],fact=[],invfact=[],pascal=[],terms=[];let phase='初期',stage='initial',i=null,selection=[],expression='',accumulator=null;
 const save=(title,description,more={})=>frames.push(clone({title,description,phase,stage,i,fact,invfact,pascal,terms,selection,expression,accumulator,result:'未確定',done:false,checkpoint:false,...more}));
 if(c.mode==='inclusion'){
  const weights=Array(c.universe).fill(0);let sum=0n;phase='包除';stage='inclusion';
  save('共通部分を数える',c.target==='union'?'非空の部分集合を順に数えます。奇数個の集合は加算、偶数個は減算です。':'mask=0を全体集合として含めます。空集合の項から始めて交互に加減算します。',{weights,intersection:[],mask:null,sum,checkpoint:true});
  for(let mask=c.target==='union'?1:0;mask<(1<<c.sets.length);++mask){let bitCount=0;for(let bits=mask;bits;bits&=bits-1)++bitCount;const sign=(bitCount%2===1)===(c.target==='union')?1:-1;
   const intersection=Array.from({length:c.universe},(_,x)=>x).filter(x=>c.sets.every((set,j)=>!(mask>>j&1)||set.includes(x)));
   for(const x of intersection)weights[x]+=sign;sum=reduce(sum+BigInt(sign*intersection.length));terms.push({mask,bitCount,sign,count:intersection.length,contribution:BigInt(sign*intersection.length),sum});
   save(`mask=${mask}: ${sign>0?'加える':'引く'}`,`${bitCount}個の集合の共通部分は${intersection.length}要素。${sign>0?'+':'−'}${intersection.length}で累計は${sum}です。`,{mask,intersection,weights,sum,sign});
  }
  phase='完了';stage='done';save('結果','各要素の係数が、求める集合に属すなら1、それ以外なら0になりました。',{done:true,checkpoint:true,result:String(sum),weights,sum,intersection:[],mask:null});return {config:c,frames};
 }
 const {bound,n,k}=c;
 save('前計算を用意する',`0〜${bound}までの表を作ります。法は${m}です。`,{checkpoint:true});
 if(c.backend==='factorial'){
  stage='factorial';phase='階乗';fact.push(1n%m);invfact.push(...Array(bound+1).fill(null));i=0;save('0! = 1','空の積は1です。法1では代表元0です。',{checkpoint:true});
  for(i=1;i<=bound;++i){fact.push(fact[i-1]*BigInt(i)%m);save(`${i}! を作る`,`${fact[i-1]} × ${i} mod ${m} = ${fact[i]}`);}
  stage='inverse-anchor';phase='逆元';i=bound;const anchor=inverse(fact[bound],m);
  if(anchor===null){if(c.mode!=='factorial')throw new Error(`${bound}!の逆元がありません。上限や法を変更するか、Pascalを選んでください。`);stage='done';phase='逆元なし';save('階乗だけを利用できます','factorials()は使えますが、FactorialTableの構築は逆元がないため拒否されます。',{done:true,checkpoint:true,result:String(fact[n]),inverseAvailable:false});return {config:c,frames};}
  invfact[bound]=anchor;save('最後の階乗の逆元を1回求める',`${fact[bound]} × ${anchor} ≡ 1 (mod ${m})`,{checkpoint:true});stage='inverse-factorial';phase='逆階乗';
  for(i=bound;i>0;--i){invfact[i-1]=invfact[i]*BigInt(i)%m;save(`逆階乗 ${i-1} を作る`,`${invfact[i]} × ${i} mod ${m} = ${invfact[i-1]}`);}
 }else{
  stage='pascal';phase='Pascal';for(i=0;i<=bound;++i){const row=Array(i+1).fill(1n%m);for(let j=1;j<i;++j)row[j]=(pascal[i-1][j-1]+pascal[i-1][j])%m;pascal.push(row);save(`Pascalの第${i}行`,'端は1。内側は左上と右上の和です。逆元を使いません。',{checkpoint:true});}
 }
 i=null;stage='query';phase='問い合わせ';
 const choose=(top,r)=>{if(top<0||r<0||r>top)return 0n;if(top>bound)throw new Error('二項係数のnが前計算上限を超えています。');return c.backend==='pascal'?pascal[top][r]:fact[top]*invfact[r]%m*invfact[top-r]%m;};
 const finish=(value,description)=>{stage='done';phase='完了';save('結果',description,{done:true,checkpoint:true,result:String(value)});return {config:c,frames};};
 if(c.mode==='factorial'){selection=[n];return finish(fact[n],`${n}! = ${fact[n]}、逆階乗は ${invfact[n]} です。`);}
 if(c.mode==='catalan'){
  if(n<0)return finish(0n,'負のnは0です。');if(n>Math.floor(bound/2))throw new Error('Catalan数は2*nまで前計算してください。');
  const all=choose(2*n,n),bad=choose(2*n,n+1);selection=[2*n,n];expression=`C(${2*n},${n}) − C(${2*n},${n+1})`;
  terms.push({label:'全ての配置',value:all});save('全ての配置',`n組の開閉括弧の配置はC(2n,n)で数えます。mod ${m}で${all}です。`,{checkpoint:true});terms.push({label:'違反する配置',value:bad});save('違反する配置を対応させる','最初の違反までを反転すると、開き括弧がn+1個の配置と一対一に対応します。');accumulator=reduce(all-bad);return finish(accumulator,'二項係数の差を取ります。n+1の逆元は使いません。');
 }
 let top=n,r=k;
 if(c.mode==='multiset'){
  if(n<0||k<0)return finish(0n,'負の引数は0です。');if(!k)return finish(1n%m,'0個選ぶ方法は1通り。0種類でも同じです。');if(!n)return finish(0n,'0種類から正の個数は選べません。');top=n+k-1;
  if(top>bound)throw new Error('重複組合せはn+k-1まで前計算してください。');selection=[top,k];expression=`H(${n},${k}) = C(${top},${k})`;save('星と仕切りを並べる',`${k}個の星と${n-1}本の仕切りから、星の位置を選びます。`,{checkpoint:true});
 }
 if(top<0||r<0||r>top)return finish(0n,'n<0、k<0、k>nのときは0です。');if(top>bound)throw new Error('nが前計算上限を超えています。');selection=[top,r];
 expression=c.mode==='permutation'?`P(${top},${r}) = ${top}! / ${top-r}!`:`C(${top},${r}) = ${top}! / (${r}! × ${top-r}!)`;
 if(c.backend==='pascal'){const value=choose(top,r);terms.push({label:`Pascal[${top}][${r}]`,value});save('表から取り出す','集合からk個を選ぶ方法を、Pascalの三角形の対応する場所から読みます。',{checkpoint:true});return finish(value,expression);}
 const factors=[{label:`${top}!`,value:fact[top]},...(c.mode==='permutation'?[]:[{label:`1/${r}!`,value:invfact[r]}]),{label:`1/${top-r}!`,value:invfact[top-r]}];accumulator=1n%m;
 for(const factor of factors){accumulator=accumulator*factor.value%m;terms.push(factor);save(`式へ代入: ${factor.label}`,`${factor.value}を掛けてmodを取ります。現在の積は${accumulator}です。`,{checkpoint:terms.length===1});}
 return finish(accumulator,expression);
}
const api={MODES,normalize,defaults,scenario,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;root.CombinatoricsModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
