(function(root){
'use strict';
const MODES=['divisor-zeta','divisor-mobius','multiple-zeta','multiple-mobius','dirichlet','gcd','lcm'];
const MIN=-(1n<<63n),MAX=(1n<<63n)-1n,clone=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?String(v):v));
function integer(value,label,lo=MIN,hi=MAX){if(typeof value==='number'&&!Number.isSafeInteger(value))throw new Error(`${label}は十進文字列にしてください。`);const s=String(value??'').trim();if(!/^-?\d+$/.test(s))throw new Error(`${label}は十進整数にしてください。`);const n=BigInt(s);if(n<lo||n>hi)throw new Error(`${label}は${lo}〜${hi}の範囲にしてください。`);return n;}
function values(input,label){const a=typeof input==='string'?input.trim().split(/[\s,]+/).filter(Boolean):input??[];if(!Array.isArray(a)||a.length>12)throw new Error(`${label}は12要素以内にしてください。`);return a.map(x=>integer(x,label));}
function normalize(input){const mode=input.mode||'divisor-zeta';if(!MODES.includes(mode))throw new Error('未対応のモードです。');const a=values(input.a,'配列A'),b=['dirichlet','gcd','lcm'].includes(mode)?values(input.b,'配列B'):[];
 const modulus=integer(input.modulus??0,'法（0は整数）',0n,2147483647n),zero=integer(input.zero??0,'添字0');const reduce=x=>modulus?(x%modulus+modulus)%modulus:x;
 const n=Math.max(a.length,b.length),focus=Number(integer(input.focus??1,'注目する添字',1n,12n));
 return {mode,a:a.map(x=>String(reduce(x))),b:b.map(x=>String(reduce(x))),modulus:String(modulus),zero:String(reduce(zero)),n,focus:n?Math.min(focus,n):0};
}
function defaults(mode){const c={mode,a:mode==='multiple-mobius'?'21 12 9 4 5 6':'1 2 3 4 5 6',b:'6 5 4 3 2 1',zero:'99',modulus:'0',focus:6};if(mode==='dirichlet'){c.a='1 1 1 1 1 1';c.b=c.a;}return c;}
function scenario(mode,kind){const c=defaults(mode);if(kind==='basic')return c;if(kind==='empty'){c.a='';c.b='';c.focus=1;}
 else if(kind==='prime-power'){c.a='1 0 0 0 0 0 0 0';c.b='0 1 0 1 0 0 0 1';c.focus=8;}
 else if(kind==='signed'){c.a='2 -1 3 0 -2 4';c.b='-1 2 0 3 1 -2';}
 else if(kind==='composite'){c.a='11 5 8 3 0 7';c.b='5 11 2 6 9 1';c.modulus='12';}
 else if(kind==='truncate'){c.a='0 1 0';c.b='0 0 1';c.focus=3;}
 else if(kind==='padded'){c.a='0 1 0 0 0 0';c.b='0 0 1 0 0 0';c.focus=6;}
 else if(kind==='boundary'){c.a='9223372036854775807 -9223372036854775808 1';c.b='1 0 -1';c.focus=2;}
 else throw new Error('未対応の例です。');return c;}
function primes(n){const result=[];for(let p=2;p<=n;++p){let yes=true;for(let d=2;d*d<=p;++d)if(p%d===0){yes=false;break;}if(yes)result.push(p);}return result;}
function run(input){const c=normalize(input),n=c.n,m=BigInt(c.modulus),conv=['dirichlet','gcd','lcm'].includes(c.mode),reduce=x=>m?(x%m+m)%m:x;
 let a=Array(n+1).fill(0n),b=Array(n+1).fill(0n),h=Array(n+1).fill(0n);c.a.forEach((x,i)=>a[i+1]=BigInt(x));c.b.forEach((x,i)=>b[i+1]=BigInt(x));if(!conv)a[0]=BigInt(c.zero);
 const frames=[],history=[],ps=primes(n);let stage='initial',phase='初期',active='a',processed=[],p=null,k=null,operation=null;
 const save=(title,description,more={})=>{const output=conv?h:a;frames.push(clone({title,description,stage,phase,active,processed,p,k,operation,a,b,h,history,output,current:c.focus?(active==='a'?a:active==='b'?b:h)[c.focus]:null,result:'未確定',done:false,checkpoint:false,...more}));};
 save('正の添字を用意する',conv?'短い入力を0で補います。添字0は入力に使わず、結果の0もゼロにします。':'正の添字1〜Nを変換します。添字0の値はそのまま保持します。',{checkpoint:true});
 function transform(target,divisor,inverse,nextStage,label){
  stage=nextStage;phase=label;active=target;processed=[];p=null;k=null;operation=null;const row=target==='a'?a:target==='b'?b:h;
  save(label+'を始める',inverse?'各更新を逆順に引いて戻します。素数もkも順方向の変換と逆にたどります。':divisor?'kの小さい方から、f[k]をf[k*p]へ加えます。':'kの大きい方から、f[k*p]をf[k]へ加えます。',{checkpoint:true});
  for(const prime of inverse?[...ps].reverse():ps){p=prime;const bound=Math.floor(n/p),ascending=divisor!==inverse;
   for(let step=0;step<bound;++step){k=ascending?step+1:bound-step;const source=divisor?k:k*p,targetIndex=divisor?k*p:k,before=row[targetIndex],value=row[source];row[targetIndex]=reduce(inverse?before-value:before+value);operation={kind:inverse?'subtract':'add',source,target:targetIndex,before,value,after:row[targetIndex]};history.push({stage,p,k,...operation});save(`p=${p}, k=${k}: ${source} → ${targetIndex}`,`f[${targetIndex}] = ${before} ${inverse?'−':'+'} ${value} = ${row[targetIndex]}${m?' (mod '+m+')':''}`);}
   processed.push(p);p=null;k=null;operation=null;save(`素数 ${prime} の処理が完了`,'この素数の指数に沿う更新が終わりました。',{checkpoint:true});
  }
 }
 if(!conv){const divisor=c.mode.startsWith('divisor'),inverse=c.mode.endsWith('mobius');transform('a',divisor,inverse,'transform',(divisor?'約数':'倍数')+(inverse?'メビウス':'ゼータ'));}
 else if(c.mode==='dirichlet'){
  stage='direct';phase='積へ集計';active='h';save('i*jへ加える','i+jではなく、添字の積へ集計します。出力上限Nを超える組は省きます。',{checkpoint:true});
  for(let i=1;i<=c.a.length;++i)for(let j=1;j<=c.b.length&&i*j<=n;++j){const target=i*j,before=h[target],value=reduce(a[i]*b[j]);h[target]=reduce(before+value);operation={kind:'pair',i,j,target,before,value,after:h[target]};history.push({stage,...operation});save(`(${i},${j}) → ${target}`,`c[${target}] += a[${i}] × b[${j}] = ${a[i]} × ${b[j]}`);}
 }else{
  const divisor=c.mode==='lcm',kind=divisor?'約数':'倍数';transform('a',divisor,false,'a-zeta','Aの'+kind+'ゼータ');transform('b',divisor,false,'b-zeta','Bの'+kind+'ゼータ');
  stage='product';phase='要素積';active='h';processed=[];p=null;k=null;operation=null;save('変換後の値を掛ける','同じ添字の集計値どうしを掛けます。まだ「ちょうど」の集計にはなっていません。',{checkpoint:true});
  for(let i=1;i<=n;++i){h[i]=reduce(a[i]*b[i]);operation={kind:'multiply',target:i,before:0n,value:h[i],after:h[i]};history.push({stage,...operation});save(`添字 ${i} の要素積`,`${a[i]} × ${b[i]} = ${h[i]}${m?' (mod '+m+')':''}`);}
  transform('h',divisor,true,'h-mobius',kind+'メビウス');
 }
 stage='done';phase='完了';p=null;k=null;operation=null;const output=conv?h:a;
 save('結果',n?`添字${c.focus}の結果は${output[c.focus]}です。${conv?'入力の添字0は使っていません。':'添字0は'+output[0]+'のままです。'}`:'正の添字の範囲は空です。',{done:true,checkpoint:true,result:n?String(output[c.focus]):'空'});
 return {config:c,frames};
}
const api={MODES,normalize,defaults,scenario,run};if(typeof module!=='undefined'&&module.exports)module.exports=api;root.ArithmeticConvolutionModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
