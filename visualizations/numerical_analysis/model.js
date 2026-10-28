(function(root){
'use strict';
const modes=typeof module==='object'?require('./modes.js'):root.NumericalModes;
const Rational=(typeof module==='object'?require('../computational_geometry/model.js'):root.GeometryModel).Rational;
const fmt=x=>Number.isFinite(x)?String(Number(x.toPrecision(6))):String(x);
const finite=x=>{if(typeof x!=='number'||!Number.isFinite(x))throw Error('有限な数値を入力してください');return x;};
const bounded=x=>{finite(x);if(Math.abs(x)>1e6)throw Error('この図解では絶対値1000000以下です');return x;};
const integer=(x,lo,hi)=>{if(!Number.isInteger(x)||x<lo||x>hi)throw Error(`整数 ${lo} … ${hi} を指定してください`);return x;};
const vector=(v,max=64)=>{if(!Array.isArray(v)||v.length>max)throw Error(`配列は${max}要素までです`);return v.map(finite);};
const polynomial=p=>{p=vector(p,9).map(bounded);return x=>p.reduceRight((a,c)=>a*x+c,0);};
const curve=(f,a,b)=>({curves:[Array.from({length:101},(_,i)=>{const x=a+(b-a)*i/100;return[x,f(x)];})],segments:[[[a,0],[b,0]]]});
const binary=x=>{const data=new DataView(new ArrayBuffer(8));data.setFloat64(0,x);const bits=data.getBigUint64(0),exponent=Number((bits>>52n)&2047n);let n=(bits&((1n<<52n)-1n))+(exponent?1n<<52n:0n),e=(exponent||1)-1023-52;if(bits>>63n)n=-n;return e>=0?new Rational(n<<BigInt(e)):new Rational(n,1n<<BigInt(-e));};
const zero=new Rational(0),one=new Rational(1),two=new Rational(2);
function trace(id,input){
 const mode=modes.find(m=>m.id===id);if(!mode)throw Error('未知のモード');
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('入力はJSONオブジェクトです');
 const x=structuredClone(input),steps=[];let result=[],meta={status:'converged'};
 const frame=(title,detail,scene={},columns=[],rows=[],values=[])=>steps.push({title,detail,scene,columns,rows,values});
 const finish=(values,scene={})=>{result=values.map(v=>typeof v==='number'?fmt(finite(v)):String(v));frame('停止と結果',`status: ${meta.status}。精度・適用条件はAPIの説明を参照してください。`,scene,['出力'],result.map(v=>[v]),[{label:'停止理由',value:meta.status}]);return{steps,result,meta};};
 frame(mode.title,'入力と条件を確認し、次へ進んで途中の状態を追います。');
 if(id==='quadratic'){
  const a=bounded(x.a),b=bounded(x.b),c=bounded(x.c),f=t=>a*t*t+b*t+c;
  let roots=[];
  if(!a){if(b)roots=[-c/b];else meta.all_real=!c;}
  else if(!c)roots=b?[-b/a,0].sort((a,b)=>a-b):[0];
  else{const A=binary(a),B=binary(b),C=binary(c),D=B.mul(B).sub(new Rational(4).mul(A).mul(C));
   frame('判別式の符号',`二進値を有理数化: D=${D}`,{},['係数','値'],[['a',a],['b',b],['c',c]]);
   if(D.cmp(zero)===0)roots=[-b/(2*a)];else if(D.cmp(zero)>0){const q=-(b+(b<0?-1:1)*Math.sqrt(Number(D)))/2;roots=[q/a,c/q].sort((a,b)=>a-b);frame('相殺を避けるq',`q=${fmt(q)}。根はq/aとc/qです。`);}}
  meta.roots=roots;const range=Math.max(2,...roots.map(v=>Math.min(100,Math.abs(v)*1.2)));
  return finish(roots,{...curve(f,-range,range),points:roots.map(v=>[v,0])});
 }
 if(['bisection','brent','newton'].includes(id)){
  const f=polynomial(x.coefficients),df=polynomial(x.coefficients.slice(1).map((c,i)=>c*(i+1)));
  const tolerance=finite(x.tolerance);if(tolerance<0)throw Error('許容誤差は非負です');
  const limit=integer(x.max_iterations,0,160),lower=bounded(x.lower),upper=bounded(x.upper);if(lower>=upper)throw Error('lower < upper としてください');
  const base=curve(f,lower,upper);let answer;
  if(id==='newton'){
   let current=bounded(x.start);meta.status='max_iterations';
   for(let i=0;i<=limit;i++){
    const y=f(current);answer=current;if(!Number.isFinite(y)){meta.status='non_finite';break;}
    if(Math.abs(y)<=tolerance){meta.status='converged';break;}if(i===limit)break;
    const derivative=df(current);if(!derivative){meta.status='zero_derivative';break;}const next=current-y/derivative;
    if(!Number.isFinite(next)){meta.status='non_finite';break;}
    frame(`Newton ${i+1}`,`x=${fmt(current)}, f=${fmt(y)}, f′=${fmt(derivative)}`,{...base,points:[[current,y],[next,0]],segments:[...base.segments,[[current,y],[next,0]]]},['x','f(x)','次のx'],[[fmt(current),fmt(y),fmt(next)]]);
    if(current===next){meta.status='stagnation';break;}current=next;
   }
  }else{
   let a=lower,b=upper,fa=f(a),fb=f(b);answer=a;
   if(fa===0||Math.abs(fa)<=tolerance)answer=a;
   else if(fb===0||Math.abs(fb)<=tolerance)answer=b;
   else if(Math.sign(fa)===Math.sign(fb))meta.status='invalid_bracket';
   else{
    if(Math.abs(fa)<Math.abs(fb)){[a,b]=[b,a];[fa,fb]=[fb,fa];}
    let c=a,fc=fa,d=c,bisected=true;meta.status='max_iterations';
    for(let i=0;i<=limit;i++){
     answer=b;meta.bracket=[Math.min(a,b),Math.max(a,b)];
     if(Math.abs(fb)<=tolerance||Math.abs(a-b)<=tolerance*(1+Math.max(Math.abs(a),Math.abs(b)))){meta.status='converged';break;}if(i===limit)break;
     let s=(a+b)/2,useBisection=true,label='二分';
     if(id==='brent'){
      let candidate;if(fa!==fc&&fb!==fc)candidate=a*fb*fc/((fa-fb)*(fa-fc))+b*fa*fc/((fb-fa)*(fb-fc))+c*fa*fb/((fc-fa)*(fc-fb));else candidate=b-fb*(b-a)/(fb-fa);
      const guard=(3*a+b)/4,previous=bisected?Math.abs(b-c):Math.abs(c-d);
      if(Number.isFinite(candidate)&&candidate>Math.min(guard,b)&&candidate<Math.max(guard,b)&&Math.abs(candidate-b)<previous/2&&previous>tolerance){s=candidate;useBisection=false;label=fa!==fc&&fb!==fc?'逆二次補間':'割線補間';}
     }
     if(s===a||s===b){meta.status='stagnation';break;}const fs=f(s);
     frame(`${label} ${i+1}`,`符号を確かめて区間を更新。幅=${fmt(Math.abs(a-b))}`,{...base,points:[[a,fa],[b,fb],[s,fs]],segments:[...base.segments,[[a,0],[a,fa]],[[b,0],[b,fb]],[[s,0],[s,fs]]]},['左端','右端','候補','f(候補)'],[[fmt(Math.min(a,b)),fmt(Math.max(a,b)),fmt(s),fmt(fs)]]);
     d=c;c=b;fc=fb;bisected=useBisection;if(Math.sign(fa)!==Math.sign(fs)){b=s;fb=fs;}else{a=s;fa=fs;}
     if(Math.abs(fa)<Math.abs(fb)){[a,b]=[b,a];[fa,fb]=[fb,fa];}
    }
   }
  }
  meta.value=answer;return finish([answer],{...base,points:[[answer,0]]});
 }
 if(['simpson','adaptive_simpson'].includes(id)){
  let a=bounded(x.lower),b=bounded(x.upper);const sign=a>b?-1:1;if(a>b)[a,b]=[b,a];
  const f=x.function==='exp'?Math.exp:polynomial(x.coefficients);if(x.function==='exp'&&b>20)throw Error('expの上端は20以下です');
  const base=curve(f,a,b),panels=[];const panelScene=()=>({...base,segments:base.segments.concat(panels.flatMap(p=>[[[p.a,0],[p.a,f(p.a)]],[[p.b,0],[p.b,f(p.b)]]]))});
  if(id==='simpson'){
   const n=integer(x.subdivisions,2,64);if(n%2)throw Error('分割数は偶数です');let total=0;
   for(let i=0;i<n;i+=2){const l=a+(b-a)*i/n,r=a+(b-a)*(i+2)/n,m=(l+r)/2,value=(r-l)*(f(l)+4*f(m)+f(r))/6;total+=value;panels.push({a:l,b:r});frame(`Simpson区間 ${i/2+1}`,'端点・中点の重み1:4:1で積分します。',panelScene(),['左端','中点','右端','区間積分','累積'],[[fmt(l),fmt(m),fmt(r),fmt(value),fmt(sign*total)]]);}
   meta.value=sign*total;return finish([sign*total],panelScene());
  }
  const tolerance=finite(x.tolerance);if(tolerance<0)throw Error('許容誤差は非負です');const depth=integer(x.max_depth,0,16);
  const make=(a,b,d)=>{const m=(a+b)/2,S=(b-a)*(f(a)+4*f(m)+f(b))/6,T=(b-a)*(f(a)+4*f((a+m)/2)+2*f(m)+4*f((m+b)/2)+f(b))/12,delta=(T-S)/15;return{a,b,d,value:T+delta,error:Math.abs(delta)};};
  panels.push(make(a,b,0));let total,error;meta.status='max_iterations';
  for(let k=0;k<=256;k++){
   total=panels.reduce((s,p)=>s+p.value,0);error=panels.reduce((s,p)=>s+p.error,0);
   frame(`全体の誤差 ${k}`,`推定誤差合計=${fmt(error)}。これは厳密な上界ではありません。`,panelScene(),['左端','右端','深さ','推定誤差'],panels.map(p=>[fmt(p.a),fmt(p.b),p.d,fmt(p.error)]),[{label:'積分',value:fmt(sign*total)},{label:'推定誤差',value:fmt(error)}]);
   if(error<=tolerance*(1+Math.abs(total))){meta.status='converged';break;}if(k===256)break;
   let index=-1;for(let i=0;i<panels.length;i++)if(panels[i].d<depth&&(index<0||panels[i].error>panels[index].error))index=i;
   if(index<0){meta.status='max_depth';break;}const p=panels[index],m=(p.a+p.b)/2;panels.splice(index,1,make(p.a,m,p.d+1),make(m,p.b,p.d+1));
  }
  meta.value=sign*total;meta.error=error;return finish([sign*total],panelScene());
 }
 if(id==='summation'){
  const v=vector(x.values),bars=v.map((a,i)=>[[i,0],[i,Math.sign(a)*Math.log10(1+Math.abs(a))]]);let k=0,c=0,s=0,correction=0,plain=0;
  for(let i=0;i<v.length;i++){const a=v[i],y=a-c,t=k+y;c=(t-k)-y;k=t;const u=s+a;correction+=Math.abs(s)>=Math.abs(a)?(s-u)+a:(a-u)+s;s=u;plain+=a;
   frame(`加算 ${i+1}`,`棒の高さは符号付きlog10(1+|x|)。入力=${fmt(a)}`,{segments:bars.slice(0,i+1)},['通常','Kahan','Neumaier補正','Neumaier合計'],[[fmt(plain),fmt(k),fmt(correction),fmt(s+correction)]]);}
  meta.kahan=k;meta.compensated=s+correction;return finish([k,s+correction],{segments:bars});
 }
 if(id==='log_sum_exp'){
  const v=vector(x.values),m=Math.max(...v);if(!v.length)return finish(['-Infinity']);let sum=0,skip=false;
  for(let i=0;i<v.length;i++){const term=Math.exp(v[i]-m);if(v[i]===m&&!skip)skip=true;else sum+=term;frame(`指数の寄与 ${i+1}`,`最大値${fmt(m)}を引いてexpを計算。`,{segments:v.slice(0,i+1).map((a,j)=>[[j,0],[j,Math.exp(a-m)]])},['x','x−max','exp','最大項以外の和'],[[v[i],fmt(v[i]-m),fmt(term),fmt(sum)]]);}
  meta.value=m+Math.log1p(sum);return finish([meta.value]);
 }
 if(id==='tolerance'){
  const a=finite(x.a),b=finite(x.b),absolute=finite(x.absolute),relative=finite(x.relative);if(absolute<0||relative<0)throw Error('許容誤差は非負です');
  const scale=Math.max(Math.abs(a),Math.abs(b)),error=Math.abs(a-b),threshold=absolute+relative*scale,accepted=a===b||(scale>0&&(Number.isFinite(error)?error/scale:Math.abs(a/scale-b/scale))<=absolute/scale+relative);
  const displayError=scale?Math.abs(a/scale-b/scale):0,displayThreshold=scale?absolute/scale+relative:absolute,denominator=Math.max(displayError,displayThreshold,Number.MIN_VALUE),scene={segments:[[[0,0],[Number.isFinite(denominator)?displayError/denominator:0,0]],[[0,1],[Number.isFinite(denominator)?displayThreshold/denominator:1,1]]]};
  frame('差と許容範囲を比較','上が許容誤差、下が実際の差。同一尺度で正規化しています。',scene,['絶対差','絶対許容','相対許容の寄与','合計許容'],[[fmt(error),fmt(absolute),fmt(relative*scale),fmt(threshold)]]);meta.accepted=accepted;return finish([Number(accepted)],scene);
 }
 if(id==='polynomial_roots'){
  const input=vector(x.coefficients,9).map(v=>new Rational(integer(v,-1000000,1000000))),tolerance=finite(x.tolerance);if(tolerance<1e-12||tolerance>1)throw Error('図解の許容誤差は1e-12 … 1です');
  const limit=integer(x.max_subdivisions,0,1000),trim=p=>{while(p.length&&!p.at(-1).n)p.pop();return p;},derivative=p=>p.slice(1).map((c,i)=>c.mul(new Rational(i+1)));
  const divide=(a,b)=>{a=a.slice();const q=Array.from({length:Math.max(0,a.length-b.length+1)},()=>zero);while(a.length>=b.length){const k=a.length-b.length,c=a.at(-1).div(b.at(-1));q[k]=c;for(let j=0;j<b.length;j++)a[j+k]=a[j+k].sub(c.mul(b[j]));trim(a);}return[trim(q),a];};
  const normalize=(p,positive=false)=>p.length?p.map(c=>c.div(positive&&p.at(-1).n<0n?zero.sub(p.at(-1)):p.at(-1))):p;
  const gcd=(a,b)=>{while(b.length)[a,b]=[b,normalize(divide(a,b)[1])];return normalize(a);};
  const evalp=(p,t)=>p.reduceRight((s,c)=>s.mul(t).add(c),zero);trim(input);meta.all_real=!input.length;
  if(input.length<=1){meta.count=0;return finish([]);}
  const p=normalize(divide(input,gcd(input,derivative(input)))[0]),chain=[p,derivative(p)];
  while(chain.at(-1).length)chain.push(normalize(divide(chain.at(-2),chain.at(-1))[1].map(c=>zero.sub(c)),true));chain.pop();
  const variations=t=>{let prior=0,count=0;for(const p of chain){const s=evalp(p,t).cmp(zero);if(s){if(prior&&prior!==s)count++;prior=s;}}return count;};
  const abs=q=>q.n<0n?zero.sub(q):q;let bound=one;for(const c of p.slice(0,-1))if(abs(c).add(one).cmp(bound)>0)bound=abs(c).add(one);bound=bound.add(one);
  const left=zero.sub(bound),stack=[{a:left,b:bound,va:variations(left),vb:variations(bound)}],intervals=[],tol=binary(tolerance);let subdivisions=0;
  const base=curve(polynomial(x.coefficients),-Number(bound),Number(bound));meta.count=stack[0].va-stack[0].vb;
  frame('平方因子除去とSturm列','有理数の剰余列で符号変化数を数えます。',base,['列','係数（昇べき順）'],chain.map((p,i)=>[i,p.map(String).join(', ')]));
  const emit=(a,b,count)=>{if(count)intervals.push({lower:a.toString(),upper:b.toString(),count,value:Number(a.add(b).div(two))});};
  while(stack.length){const w=stack.pop(),count=w.va-w.vb;if(!count)continue;
   if(count===1&&!evalp(p,w.b).n){emit(w.b,w.b,1);continue;}
   const scale=abs(w.a).cmp(abs(w.b))>0?abs(w.a):abs(w.b);
   if(count===1&&w.b.sub(w.a).cmp(tol.add(tol.mul(scale)))<=0){emit(w.a,w.b,1);continue;}
   if(subdivisions===limit){meta.status='max_iterations';emit(w.a,w.b,count);for(const s of stack)emit(s.a,s.b,s.va-s.vb);break;}
   const m=w.a.add(w.b).div(two),vm=variations(m);subdivisions++;
   if(subdivisions<=100)frame(`Sturm分割 ${subdivisions}`,`V(a)−V(b)=${count}。区間(a,b]の根数を保持します。`,{...base,points:[[Number(w.a),0],[Number(m),0],[Number(w.b),0]]},['a','中点','b','左の根数','右の根数'],[[w.a.toString(),m.toString(),w.b.toString(),w.va-vm,vm-w.vb]]);
   if(vm>w.vb)stack.push({a:m,b:w.b,va:vm,vb:w.vb});if(w.va>vm)stack.push({a:w.a,b:m,va:w.va,vb:vm});
  }
  intervals.sort((a,b)=>a.value-b.value);meta.intervals=intervals;meta.subdivisions=subdivisions;
  frame('根区間と個数','count>1は未分離です。valueは区間中点の近似です。',base,['下端','上端','count'],intervals.map(p=>[p.lower,p.upper,p.count]));
  return finish(intervals.map(p=>p.value),{...base,points:intervals.map(p=>[p.value,0])});
 }
 if(id==='tridiagonal'){
  const d=vector(x.diagonal,12).map(bounded),l=vector(x.lower,11).map(bounded),u=vector(x.upper,11).map(bounded),rhs=vector(x.rhs,12).map(bounded),n=d.length;
  if(l.length!==Math.max(0,n-1)||u.length!==l.length||rhs.length!==n)throw Error('三重対角の次元が一致しません');
  const A=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?d[i]:i===j+1?l[j]:j===i+1?u[i]:0));
  const scene=()=>({cells:A.flatMap((row,i)=>row.map((value,j)=>({x:j,y:n-i-1,value}))),points:[[0,0],[Math.max(1,n),Math.max(1,n)]]});
  meta.swaps=[];
  for(let i=0;i<n;i++){
   const swap=i+1<n&&Math.abs(A[i+1][i])>Math.abs(A[i][i]);meta.swaps.push(swap);
   if(swap){[A[i],A[i+1]]=[A[i+1],A[i]];[rhs[i],rhs[i+1]]=[rhs[i+1],rhs[i]];}
   if(!A[i][i]){meta.status='singular';return finish([],scene());}
   if(i+1<n){const m=A[i+1][i]/A[i][i];for(let j=i;j<Math.min(n,i+3);j++)A[i+1][j]-=m*A[i][j];rhs[i+1]-=m*rhs[i];}
   frame(`消去 ${i+1}${swap?'・隣接行を交換':''}`,'行交換で生じる第2上対角も使います。緑が非零成分です。',scene(),['行',...Array.from({length:n},(_,i)=>`A${i}`),'右辺'],A.map((r,j)=>[j,...r.map(fmt),fmt(rhs[j])]));
  }
  const answer=Array(n).fill(0);for(let i=n-1;i>=0;i--){let value=rhs[i];for(let j=i+1;j<Math.min(n,i+3);j++)value-=A[i][j]*answer[j];answer[i]=value/A[i][i];}
  meta.solution=answer;return finish(answer,scene());
 }
 throw Error('モード未対応');
}
const api={modes,trace};if(typeof module==='object')module.exports=api;else root.NumericalModel=api;
})(typeof globalThis==='object'?globalThis:this);
