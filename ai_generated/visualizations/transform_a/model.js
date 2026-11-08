(function(root){
'use strict';
const specs={
 karatsuba:['Karatsuba畳み込み',{a:[1,2,3,4],b:[2,0,1,3]},'係数列を二分し、3つの再帰積から通常の積を組み立てます。'],
 fwht:['FWHT / Walsh–Hadamard変換',{f:[1,2,3,4],modulus:17,inverse:false},'自然順のbutterflyと、逆変換の各段の2除算を追います。'],
 bitwise:['AND / OR / XOR Convolution',{a:[1,2,3,4],b:[2,1,0,3],operation:'xor',modulus:17},'and / or / xorを切り替え、変換→各点積→逆変換を確認します。'],
 ranked:['Ranked Zeta Transform',{f:[1,2,3,4],modulus:17},'rankごとの層を作り、各ビットの部分集合和を積み重ねます。'],
 inverse:['逆行列',{a:[[0,2],[1,3]],modulus:17},'[A|I]の行操作を追い、右側から逆行列を取り出します。'],
 gf2:['GF(2)上のBitset Gauss',{a:[[1,1,0],[0,1,1]],b:[1,0],variables:3},'bit jを列jとして表示し、行XOR・特解・自由変数を追います。'],
 intersection:['線形空間の交差',{u:[[1,0,0],[0,1,0]],v:[[0,1,0],[0,0,1]],dimension:3,modulus:3},'独立な行基底から[Uᵀ|-Vᵀ]の零空間を作ります。'],
 charpoly:['特性多項式',{a:[[1,2,0],[0,3,4],[5,6,7]],modulus:17},'相似変形でHessenberg化し、主小行列の多項式を更新します。']
};
const paths={karatsuba:'convolution/karatsuba',fwht:'set_function/fwht',bitwise:'set_function/bitwise_convolution',ranked:'set_function/ranked_zeta_transform',inverse:'linear_algebra/inverse',gf2:'linear_algebra/gf2_gauss',intersection:'linear_algebra/space_intersection',charpoly:'linear_algebra/characteristic_polynomial'};
const clone=x=>JSON.parse(JSON.stringify(x));
const integer=(x,lo,hi,label)=>{if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`${label}: ${lo}〜${hi}の整数を指定してください`);return x;};
function build(mode,input){
 if(!Object.hasOwn(specs,mode)||!input||typeof input!=='object'||Array.isArray(input))throw Error('入力オブジェクトとモードを確認してください');
 const frames=[];let result;
 const emit=(title,note,grid=[],active=[],extra={})=>frames.push(clone({title,note,grid,active,columns:['行','内部状態'],rows:grid.length?grid.map((r,i)=>[i,r.join(' ')]):[['状態',note]],...extra}));
 const array=(a,max=16)=>{if(!Array.isArray(a)||a.length>max)throw Error(`配列は${max}項以下です`);return a.map(x=>integer(x,-100,100,'係数'));};
 const table=a=>{const f=array(a);if(!f.length||(f.length&(f.length-1)))throw Error('正の2冪長が必要です');return f;};
 const modulus=integer(input.modulus??(mode==='gf2'?2:0),0,101,'modulus');
 const mod=x=>modulus?((x%modulus)+modulus)%modulus:(x===0?0:x);
 const prime=p=>p>=2&&Array.from({length:Math.max(0,p-2)},(_,i)=>i+2).every(d=>p%d!==0);
 const inv=(x,p=modulus)=>{for(let i=1;i<p;i++)if(mod(x*i)===1)return i;throw Error('逆元がありません');};
 const field=()=>{if(!prime(modulus))throw Error('行列には2〜101の素数modulusが必要です');};
 const matrix=(a,cols,max=6)=>{if(!Array.isArray(a)||a.length>max)throw Error(`行数は${max}以下です`);return a.map(row=>{const v=array(row,max);if(v.length!==cols)throw Error('行列の列数が一致しません');return v.map(mod);});};
 function wht(f,inverse,label){
  if(inverse&&f.length>1&&modulus&&modulus%2===0)throw Error('逆FWHTには2の逆元が必要です');
  const bits=[];for(let bit=1;bit<f.length;bit*=2)bits.push(bit);if(inverse)bits.reverse();
  for(const bit of bits){for(let base=0;base<f.length;base+=2*bit)for(let j=0;j<bit;j++){
   const l=base+j,h=l+bit,x=f[l],y=f[h];let u=mod(x+y),v=mod(x-y);
   if(inverse){if(modulus){const half=inv(2);u=mod(u*half);v=mod(v*half);}else{if(u%2||v%2)throw Error('整数の逆FWHTが割り切れません');u/=2;v/=2;}}
   f[l]=u;f[h]=v;emit(`${label}: butterfly`,`bit=${bit}、(${l},${h}): (${x},${y}) → (${u},${v})`,[f],[[0,l],[0,h]],{bit});
  }emit(`${label}: ビット段完了`,`bit=${bit}`,[f],[],{stage:true,bit,inverse});}
 }
 function zeta(f,subset,inverse,label){
  const bits=[];for(let bit=1;bit<f.length;bit*=2)bits.push(bit);if(inverse)bits.reverse();
  for(const bit of bits){for(let s=0;s<f.length;s++)if(s&bit){const target=subset?s:s^bit,source=subset?s^bit:s;f[target]=mod(f[target]+(inverse?-1:1)*f[source]);emit(label,`bit=${bit}、${source}から${target}へ${inverse?'減算':'加算'}`,[f],[[0,target],[0,source]]);} }
 }
 function reduce(a,columns,label){
  let rank=0;const pivots=[];
  for(let col=0;col<columns&&rank<a.length;col++){
   let pivot=rank;while(pivot<a.length&&a[pivot][col]===0)pivot++;if(pivot===a.length)continue;
   [a[rank],a[pivot]]=[a[pivot],a[rank]];emit(`${label}: 行交換`,`pivot列${col}、行${rank}と${pivot}`,a,[[rank,col]],{coefficientColumns:columns});
   const scale=inv(a[rank][col]);a[rank]=a[rank].map(x=>mod(x*scale));emit(`${label}: 正規化`,`行${rank}のpivotを1にする`,a,[[rank,col]],{coefficientColumns:columns});
   for(let row=0;row<a.length;row++)if(row!==rank&&a[row][col]){const factor=a[row][col];a[row]=a[row].map((x,j)=>mod(x-factor*a[rank][j]));emit(`${label}: 行消去`,`行${row}から${factor}×行${rank}を引く`,a,[[row,col],[rank,col]],{coefficientColumns:columns});}
   pivots.push(col);rank++;
  }return{rank,pivots};
 }
 function kernel(reduced,columns,pivots){const basis=[];for(let j=0;j<columns;j++)if(!pivots.includes(j)){const v=Array(columns).fill(0);v[j]=1;pivots.forEach((p,i)=>v[p]=mod(-reduced[i][j]));basis.push(v);}return basis;}
 if(mode==='karatsuba'){
  const a=array(input.a),b=array(input.b);
  function rec(a,b,depth){const n=a.length;if(n<=2){const c=Array(2*n).fill(0);for(let i=0;i<n;i++)for(let j=0;j<n;j++)c[i+j]+=a[i]*b[j];emit('小さい積を直接計算',`深さ${depth}、長さ${n}`,[a,b,c],[],{depth});return c;}
   const h=n/2;emit('係数列を二分',`深さ${depth}、下位${h}項と上位${h}項`,[a,b],[],{depth});
   const lo=rec(a.slice(0,h),b.slice(0,h),depth+1),hi=rec(a.slice(h),b.slice(h),depth+1),sum=rec(a.slice(0,h).map((x,i)=>x+a[h+i]),b.slice(0,h).map((x,i)=>x+b[h+i]),depth+1),middle=sum.map((x,i)=>x-lo[i]-hi[i]),c=Array(2*n).fill(0);
   for(let i=0;i<n;i++){c[i]+=lo[i];c[i+h]+=middle[i];c[i+n]+=hi[i];}
   emit('3積から組み立てる',`深さ${depth}、z0 + x^h z1 + x^(2h) z2`,[lo,middle,hi,c],[],{depth});return c;
  }
  if(!a.length||!b.length)result=[];else{let n=1;while(n<Math.max(a.length,b.length))n*=2;result=rec([...a,...Array(n-a.length).fill(0)],[...b,...Array(n-b.length).fill(0)],0).slice(0,a.length+b.length-1);}
 }else if(mode==='fwht'){
  if(input.inverse!==undefined&&typeof input.inverse!=='boolean')throw Error('inverseはtrue/falseです');result=table(input.f).map(mod);emit('入力','自然順の係数',[result]);wht(result,!!input.inverse,input.inverse?'逆FWHT':'FWHT');
 }else if(mode==='bitwise'){
  const a=table(input.a).map(mod),b=table(input.b).map(mod),op=input.operation;if(a.length!==b.length||!['and','or','xor'].includes(op))throw Error('同じ長さとand/or/xorが必要です');
  emit('入力',`添字を${op}で集計`,[a,b]);
  if(op==='xor'){wht(a,false,'aのFWHT');wht(b,false,'bのFWHT');}else{zeta(a,op==='or',false,'aのzeta');zeta(b,op==='or',false,'bのzeta');}
  result=a.map((x,i)=>mod(x*b[i]));emit('各点積','変換後の同じ添字を掛ける',[a,b,result]);if(op==='xor')wht(result,true,'逆FWHT');else zeta(result,op==='or',true,'Möbius');
 }else if(mode==='ranked'){
  const f=table(input.f).map(mod),bits=Math.log2(f.length),pop=s=>s.toString(2).replace(/0/g,'').length;
  result=Array.from({length:bits+1},()=>Array(f.length).fill(0));f.forEach((x,s)=>result[pop(s)][s]=x);emit('rank層へ配置','rank=popcount(S)だけに係数を置く',result);
  for(let bit=1;bit<f.length;bit*=2){for(let k=0;k<=bits;k++)for(let s=0;s<f.length;s++)if(s&bit)result[k][s]=mod(result[k][s]+result[k][s^bit]);emit('rank別zetaのビット段',`bit=${bit}、各層で部分集合和を更新`,result,[],{stage:true,bit});}
 }else if(mode==='inverse'){
  field();const n=integer(input.a?.length,0,4,'行数'),a=matrix(input.a,n),r=a.map((row,i)=>[...row,...Array.from({length:n},(_,j)=>+(i===j))]);emit('拡大行列 [A|I]','右側にも同じ行操作を適用',r);
  const {rank}=reduce(r,n,'逆行列');result=rank===n?r.map(row=>row.slice(n)):null;emit(rank===n?'逆行列を取得':'特異行列',rank===n?'左側が単位行列になった':'必要なpivotがない',r);
 }else if(mode==='gf2'){
  if(modulus!==2)throw Error('GF(2)のmodulusは2です');const cols=integer(input.variables??input.a?.[0]?.length??0,0,6,'variables'),a=matrix(input.a,cols),b=array(input.b,6);if(b.length!==a.length||b.some(x=>x!==0&&x!==1)||input.a.some(row=>row.some(x=>x!==0&&x!==1)))throw Error('Aとbには0/1を指定してください');
  const r=a.map((row,i)=>[...row,b[i]]);emit('二値の拡大行列','右端はRHS。表示の列jがbit j',r);const {rank,pivots}=reduce(r,cols,'GF(2)');const consistent=r.slice(rank).every(row=>row[cols]===0),particular=Array(cols).fill(0);pivots.forEach((p,i)=>particular[p]=r[i][cols]);result={rank,has_solution:consistent,particular:consistent?particular:[],nullspace_basis:consistent?kernel(r,cols,pivots):[]};
 }else if(mode==='intersection'){
  field();const d=integer(input.dimension??input.u?.[0]?.length??input.v?.[0]?.length??0,0,4,'dimension'),u=matrix(input.u,d,4),v=matrix(input.v,d,4);
  const ur=reduce(u,d,'Uを基底へ').rank,vr=reduce(v,d,'Vを基底へ').rank;u.length=ur;v.length=vr;emit('独立な行基底','UとVの従属行を除く',[...u,...v]);
  const eq=Array.from({length:d},(_,j)=>[...u.map(row=>row[j]),...v.map(row=>mod(-row[j]))]);emit('[Uᵀ|-Vᵀ]','αU=βVの係数を解く',eq);const {pivots}=reduce(eq,ur+vr,'係数の零空間'),coefficients=kernel(eq,ur+vr,pivots);result=coefficients.map(c=>Array.from({length:d},(_,j)=>mod(u.reduce((sum,row,i)=>sum+c[i]*row[j],0))));emit('交差空間の基底','零空間のU側係数を生成ベクトルへ戻す',result);
 }else if(mode==='charpoly'){
  field();const n=integer(input.a?.length,0,4,'行数'),h=matrix(input.a,n);emit('入力行列','相似変形はdet(xI−A)を保つ',h);
  for(let col=0;col+2<n;col++){
   let pivot=col+1;while(pivot<n&&!h[pivot][col])pivot++;if(pivot===n)continue;
   [h[col+1],h[pivot]]=[h[pivot],h[col+1]];for(let i=0;i<n;i++)[h[i][col+1],h[i][pivot]]=[h[i][pivot],h[i][col+1]];emit('行と列を同時に交換',`pivotを(${col+1},${col})へ`,h);
   for(let i=col+2;i<n;i++){const factor=mod(h[i][col]*inv(h[col+1][col]));h[i][col]=0;for(let j=col+1;j<n;j++)h[i][j]=mod(h[i][j]-factor*h[col+1][j]);for(let j=0;j<n;j++)h[j][col+1]=mod(h[j][col+1]+factor*h[j][i]);emit('行消去と逆向きの列操作',`(${i},${col})を0にする相似変形`,h,[[i,col]],{similarity:true});}
  }
  const p=[[1]];for(let size=1;size<=n;size++){p[size]=Array(size+1).fill(0);for(let j=0;j<size;j++){p[size][j+1]=mod(p[size][j+1]+p[size-1][j]);p[size][j]=mod(p[size][j]-h[size-1][size-1]*p[size-1][j]);}let product=1;for(let i=size-1;i>0;i--){product=mod(product*h[i][i-1]);const factor=mod(h[i-1][size-1]*product);for(let j=0;j<i;j++)p[size][j]=mod(p[size][j]-factor*p[i-1][j]);}emit('主小行列の特性多項式',`P_${size}(x)、係数は定数項から昇冪順`,h,[],{polynomial:p[size],size,columns:['次数','係数'],rows:p[size].map((x,j)=>[j,x])});}result=p[n];
 }
 if(!frames.length)emit('初期状態',specs[mode][2]);emit('完了',JSON.stringify(result),Array.isArray(result)?(Array.isArray(result[0])?result:[result]):[],[],{result});return{frames,result};
}
const api={specs,paths,build};if(typeof module!=='undefined')module.exports=api;root.TransformAModels=api;
})(typeof globalThis!=='undefined'?globalThis:this);
