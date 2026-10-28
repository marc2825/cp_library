(function(root){'use strict';
const modes=typeof module==='object'?require('./modes.js'):root.OptimizationModes;
const Rational=(typeof module==='object'?require('../computational_geometry/model.js'):root.GeometryModel).Rational;
const fmt=x=>String(Number(x.toPrecision(8))),qfmt=x=>Math.abs(x-Math.round(x))<1e-7?`${Math.round(x)}/1`:fmt(x);
const finite=x=>{if(typeof x!=='number'||!Number.isFinite(x)||Math.abs(x)>1000)throw Error('有限な数値・絶対値1000以下を入力してください');return x;};
const integer=(x,lo,hi)=>{if(!Number.isInteger(x)||x<lo||x>hi)throw Error(`整数${lo}〜${hi}を指定してください`);return x;};
const array=(v,max=8)=>{if(!Array.isArray(v)||v.length>max)throw Error(`配列は${max}要素までです`);return v.map(finite);};
const dot=(a,b)=>a.reduce((s,x,i)=>s+x*b[i],0),eps=1e-9;
function lp(A,b,c,limit=256,record=()=>{}){
 const m=A.length,n=c.length,N=n+2*m,t=A.map((row,i)=>{const a=Array(N).fill(0),s=b[i]<0?-1:1;row.forEach((v,j)=>a[j]=s*v);a[n+i]=s;if(s<0)a[n+m+i]=1;return a;}),rhs=b.map(Math.abs),basis=b.map((v,i)=>n+(v<0?m:0)+i);
 let pivots=0;const primal=()=>{const x=Array(n).fill(0);basis.forEach((j,i)=>{if(j<n)x[j]=rhs[i];});return x;};
 const snapshot=(phase,entering,leaving)=>record({phase,entering,leaving,pivots,x:primal(),basis:basis.slice(),table:t.map((r,i)=>[basis[i],rhs[i],...r])});
 const pivot=(r,j,phase)=>{if(pivots===limit)return false;const d=t[r][j];t[r]=t[r].map(x=>x/d);rhs[r]/=d;for(let i=0;i<t.length;i++)if(i!==r){const factor=t[i][j];t[i]=t[i].map((x,k)=>x-factor*t[r][k]);rhs[i]-=factor*rhs[r];t[i][j]=0;}t[r][j]=1;const old=basis[r];basis[r]=j;pivots++;snapshot(phase,j,old);return true;};
 let entering=-1;
 const optimize=(cost,allowed,phase)=>{for(;;){entering=-1;const basic=new Set(basis);for(let j=0;j<allowed;j++)if(!basic.has(j)&&cost[j]-basis.reduce((s,k,i)=>s+cost[k]*t[i][j],0)>eps){entering=j;break;}if(entering<0)return'optimal';let row=-1,best=0;for(let i=0;i<t.length;i++)if(t[i][entering]>eps){const ratio=Math.max(0,rhs[i])/t[i][entering];if(row<0||ratio<best-eps||(Math.abs(ratio-best)<=eps&&basis[i]<basis[row])){row=i;best=ratio;}}if(row<0)return'unbounded';if(!pivot(row,entering,phase))return'iteration_limit';}};
 snapshot('I',-1,-1);let cost=Array(N).fill(0);for(let j=n+m;j<N;j++)cost[j]=-1;
 let status=optimize(cost,N,'I');if(status!=='optimal')return{status,x:[],value:0,pivots};if(basis.reduce((s,j,i)=>s+cost[j]*rhs[i],0)<-eps)return{status:'infeasible',x:[],value:0,pivots};
 for(let i=0;i<t.length;){if(basis[i]<n+m){i++;continue;}rhs[i]=0;const j=t[i].findIndex((v,j)=>j<n+m&&Math.abs(v)>eps);if(j<0){t.splice(i,1);rhs.splice(i,1);basis.splice(i,1);}else{if(!pivot(i,j,'I cleanup'))return{status:'iteration_limit',x:[],value:0,pivots};i++;}}
 cost=Array(N).fill(0);c.forEach((x,j)=>cost[j]=x);snapshot('II',-1,-1);status=optimize(cost,n+m,'II');const x=primal();return{status,x,value:dot(c,x),pivots};
}
function problem(x){const c=array(x.c,6),b=array(x.b,16);if(!Array.isArray(x.A)||x.A.length!==b.length)throw Error('Aの行数とbが一致しません');const A=x.A.map(row=>{row=array(row,6);if(row.length!==c.length)throw Error('Aとcの次元が一致しません');return row;});return{A,b,c};}
function region(A,b,nonnegative=true){let p=nonnegative?[[0,0],[6,0],[6,6],[0,6]]:[[-6,-6],[6,-6],[6,6],[-6,6]];for(let k=0;k<A.length;k++){const q=[];for(let i=0;i<p.length;i++){const a=p[i],z=p[(i+1)%p.length],u=dot(A[k],a)-b[k],v=dot(A[k],z)-b[k];if(u<=eps)q.push(a);if((u< -eps&&v>eps)||(u>eps&&v< -eps)){const t=u/(u-v);q.push(a.map((x,j)=>x+t*(z[j]-x)));}}p=q;}return{polygons:p.length?[p]:[],points:[],bounds:nonnegative?[[-1,-1],[6,6]]:[[-6,-6],[6,6]]};}
function trace(id,input){const mode=modes.find(m=>m.id===id);if(!mode)throw Error('未知のモード');if(!input||typeof input!=='object')throw Error('入力はJSONオブジェクトです');const x=structuredClone(input),steps=[];let meta={status:'optimal'};
 const frame=(title,detail,scene={},columns=[],rows=[],values=[])=>steps.push({title,detail,scene,columns,rows,values});
 const finish=(result,scene={})=>{result=result.map(String);frame('結果と停止理由',`status: ${meta.status}。モデルとC++の条件は説明を参照してください。`,scene,['出力'],result.map(v=>[v]),[{label:'状態',value:meta.status}]);return{steps,result,meta};};
 frame(mode.title,mode.principle);
 if(id==='linear_program'){
  const {A,b,c}=problem(x),limit=integer(x.max_pivots,0,256),scene=c.length===2?region(A,b):{};
  const result=lp(A,b,c,limit,s=>{const columns=['基底','右辺',...Array.from({length:c.length+2*A.length},(_,i)=>`x${i}`)];frame(`Phase ${s.phase} / pivot ${s.pivots}`,s.entering<0?'基底と目的を確認します。':`x${s.entering}が入り、x${s.leaving}が出ます。`,{...scene,points:(scene.points||[]).concat(s.x.length===2?[s.x]:[])},columns,s.table.map(r=>r.map(fmt)));});
  meta=result;return finish([result.status,qfmt(result.value),...result.x.map(qfmt)],scene);
 }
 if(id==='low_dimensional_lp'){
  const {A,b,c}=problem(x);if(c.length!==2)throw Error('図解は2次元です');const limit=integer(x.max_bases,0,256),rows=A.concat([[1,0],[-1,0],[0,1],[0,-1]]),rhs=b.map(v=>[v,0]).concat([[0,1],[0,1],[0,1],[0,1]]),scene=region(A,b,false);
  const sign=v=>Math.abs(v[1])>eps?Math.sign(v[1]):Math.abs(v[0])>eps?Math.sign(v[0]):0;let best=null,count=0,status='infeasible';
  outer:for(let i=0;i<rows.length;i++)for(let j=i+1;j<rows.length;j++){
   if(count===limit){status='iteration_limit';break outer;}count++;const det=rows[i][0]*rows[j][1]-rows[i][1]*rows[j][0];if(Math.abs(det)<eps)continue;
   const solve=k=>[(rhs[i][k]*rows[j][1]-rows[i][1]*rhs[j][k])/det,(rows[i][0]*rhs[j][k]-rhs[i][k]*rows[j][0])/det],u=solve(0),v=solve(1);
   const feasible=rows.every((row,k)=>sign([dot(row,u)-rhs[k][0],dot(row,v)-rhs[k][1]])<=0);if(!feasible)continue;const objective=[dot(c,u),dot(c,v)];
   frame(`基底 ${i}, ${j}`,`x=u+Mv、目的=${fmt(objective[0])} + M*${fmt(objective[1])}`,{...scene,points:scene.points.concat([u])},['成分','定数u','Mの係数v'],u.map((a,i)=>[i,fmt(a),fmt(v[i])]));
   if(!best||sign([objective[0]-best.objective[0],objective[1]-best.objective[1]])>0)best={u,v,objective};status='optimal';if(objective[1]>eps){status='unbounded';break outer;}
  }
  meta={status,bases:count};let value=0;if(best){let M=1;A.forEach((row,i)=>{const slope=dot(row,best.v);if(slope< -eps)M=Math.max(M,(dot(row,best.u)-b[i])/(-slope));});meta.x=best.u.map((u,j)=>u+M*best.v[j]);value=dot(c,meta.x);}meta.value=value;return finish([status,qfmt(value)],scene);
 }
 const partition=(groups,capacities)=>{groups=array(groups).map(g=>integer(g,0,7));capacities=array(capacities).map(v=>integer(v,0,8));if(groups.some(g=>g>=capacities.length))throw Error('groupが容量配列の範囲外です');return s=>{const count=Array(capacities.length).fill(0);return s.every(e=>++count[groups[e]]<=capacities[groups[e]]);};};
 if(id==='matroid_union'){
  const groups=array(x.groups),n=groups.length;if(!Array.isArray(x.capacities)||x.capacities.length>4)throw Error('マトロイドは4個までです');const oracles=x.capacities.map(c=>partition(groups,c)),assignment=Array(n).fill(-1);let cardinality=0;
  for(;;){const sets=oracles.map((_,j)=>assignment.flatMap((v,e)=>v===j?[e]:[])),parent=Array(n).fill(-1),queue=[];assignment.forEach((v,e)=>{if(v<0){parent[e]=-2;queue.push(e);}});let last=-1,target=-1;
   for(let head=0;head<queue.length&&last<0;head++){const e=queue[head];for(let j=0;j<oracles.length&&last<0;j++)if(assignment[e]!==j){if(oracles[j]([...sets[j],e])){last=e;target=j;break;}for(const v of sets[j])if(parent[v]===-1&&oracles[j]([...sets[j].filter(a=>a!==v),e])){parent[v]=e;queue.push(v);}}}
   if(last<0)break;const path=[];let color=target;for(let e=last;e>=0;e=parent[e]){path.push(e);const old=assignment[e];assignment[e]=color;color=old;}cardinality++;
   frame(`分配 ${cardinality}`,`交換路: ${path.reverse().join(' → ')}。終点は集合${target}へ入ります。`,{points:assignment.map((j,e)=>[e,j+1]),segments:assignment.flatMap((j,e)=>j>=0?[[[e,0],[e,j+1]]]:[])},['要素','所属'],assignment.map((j,e)=>[e,j<0?'未選択':j]));
  }meta={status:'optimal',cardinality,assignment};return finish([cardinality,Number(cardinality===n)]);
 }
 if(id==='weighted_matroid_intersection'){
  const weights=array(x.weights),n=weights.length;if(x.groups1.length!==n||x.groups2.length!==n)throw Error('要素数が一致しません');const first=partition(x.groups1,x.capacities1),second=partition(x.groups2,x.capacities2),inside=new Set(),bySize=[0];let best={weight:0,selected:[]};
  for(;;){const selected=[...inside].sort((a,b)=>a-b),outside=weights.flatMap((_,e)=>inside.has(e)?[]:[e]),arcs=[],source=n,sink=n+1;const candidate=(a,e)=>[...selected.filter(v=>v!==a),e];
   for(const e of outside){if(first(candidate(-1,e)))arcs.push([source,e,-weights[e]]);if(second(candidate(-1,e)))arcs.push([e,sink,0]);for(const a of selected){if(first(candidate(a,e)))arcs.push([a,e,-weights[e]]);if(second(candidate(a,e)))arcs.push([e,a,weights[a]]);}}
   const dist=Array(n+2).fill(Infinity),length=Array(n+2).fill(Infinity),parent=Array(n+2).fill(-1);dist[source]=length[source]=0;
   for(let pass=0;pass<n+2;pass++){let changed=false;for(const[a,b,c]of arcs)if(dist[a]!==Infinity&&(dist[a]+c<dist[b]||(dist[a]+c===dist[b]&&length[a]+1<length[b]))){dist[b]=dist[a]+c;length[b]=length[a]+1;parent[b]=a;changed=true;}if(!changed)break;}
   if(dist[sink]===Infinity)break;const path=[];for(let e=parent[sink];e!==source;e=parent[e]){if(e<0||path.length>n)throw Error('交換路が不正です');path.push(e);if(inside.has(e))inside.delete(e);else inside.add(e);}
   const set=[...inside].sort((a,b)=>a-b),weight=set.reduce((s,e)=>s+weights[e],0);bySize.push(weight);if(weight>=best.weight)best={weight,selected:set};
   frame(`要素数 ${set.length}`,`最小費用路: ${path.reverse().join(' → ')}、費用=${dist[sink]}。`,{points:weights.map((w,e)=>[e,inside.has(e)?1:0]),segments:path.slice(1).map((e,i)=>[[path[i],inside.has(path[i])?1:0],[e,inside.has(e)?1:0]])},['要素','重み','選択'],weights.map((w,e)=>[e,w,inside.has(e)?'選択':'外側']));
  }meta={status:'optimal',...best,by_size:bySize};return finish([best.weight,...best.selected]);
 }
 if(id==='fractional_programming'){
  if(!Array.isArray(x.points)||!x.points.length||x.points.length>16)throw Error('候補点は1〜16個です');const points=x.points.map(p=>{p=array(p,2);if(p.length!==2||p[1]<=0)throw Error('分子・正の分母を指定してください');return p;}),d=finite(x.denominator_lower_bound);if(d<=0||points.some(p=>p[1]<d))throw Error('分母下限が不正です');const limit=integer(x.max_iterations,0,256),start=finite(x.start);let lambda=start,ratio=0,status='iteration_limit';
  const oracle=t=>points.reduce((best,p)=>p[0]-t*p[1]>best[0]-t*best[1]?p:best,points[0]);
  for(let i=0;i<limit;i++){const p=oracle(lambda),residual=p[0]-lambda*p[1];ratio=p[0]/p[1];frame(`Dinkelbach ${i+1}`,`λ=${fmt(lambda)}、最大残差=${fmt(residual)}、次の比率=${fmt(ratio)}`,{points:points.map(p=>[p[1],p[0]]),segments:points.map(p=>[[p[1],lambda*p[1]],[p[1],p[0]]])},['分子','分母','f−λg'],points.map(p=>[...p,fmt(p[0]-lambda*p[1])]));if(Math.abs(residual)<1e-10){status='converged';break;}lambda=ratio;}
  let lower=finite(x.lower),upper=finite(x.upper);if(lower>upper)throw Error('比率の区間が逆です');if(oracle(lower)[0]-lower*oracle(lower)[1]<-eps||oracle(upper)[0]-upper*oracle(upper)[1]>eps)throw Error('最適比率を囲んでいません');let second=oracle(lower)[0]/oracle(lower)[1];
  for(let i=0;i<limit&&upper-lower>1e-10;i++){const mid=(lower+upper)/2,p=oracle(mid),residual=p[0]-mid*p[1];second=Math.max(second,p[0]/p[1]);frame(`比率二分 ${i+1}`,`残差=${fmt(residual)}。非負なら下端、負なら上端を中点へ移します。`,{segments:[[[lower,0],[upper,0]]],points:[[mid,0]]},['下端','中点','上端','候補比率'],[[fmt(lower),fmt(mid),fmt(upper),fmt(second)]]);if(residual>=0)lower=mid;else upper=mid;}
  meta={status,ratio,lower,upper};return finish([fmt(ratio),fmt(second)]);
 }
 const greedy=(weights,f)=>{const order=weights.map((_,i)=>i).sort((a,b)=>weights[b]-weights[a]||a-b),base=Array(weights.length).fill(0),chain=[f([])],prefix=[];for(const e of order){prefix.push(e);const value=f(prefix);base[e]=value-chain.at(-1);chain.push(value);}return{order,base,chain,value:dot(weights,base)};};
 if(id==='base_polyhedron'){
  const weights=array(x.weights);if(!Array.isArray(x.sets)||x.sets.length!==weights.length)throw Error('被覆集合の個数が一致しません');const sets=x.sets.map(s=>array(s,16).map(v=>integer(v,0,31))),f=s=>new Set(s.flatMap(e=>sets[e])).size,g=greedy(weights,f),prefix=[];
  for(let i=0;i<g.order.length;i++){prefix.push(g.order[i]);frame(`貪欲基 ${i+1}`,`prefixの被覆数=${g.chain[i+1]}。差を要素${g.order[i]}へ置きます。`,{segments:g.base.map((v,j)=>[[j,0],[j,prefix.includes(j)?v:0]])},['要素','重み','基の座標'],weights.map((w,e)=>[e,w,prefix.includes(e)?g.base[e]:'未確定']));}
  meta={status:'optimal',...g};return finish([qfmt(g.value),...g.base.map(qfmt)]);
 }
 if(id==='submodular_optimization'){
  const modular=array(x.modular),n=modular.length;if(!Array.isArray(x.edges)||x.edges.length>28)throw Error('辺は28本までです');const edges=x.edges.map(e=>{if(!Array.isArray(e)||e.length!==3)throw Error('辺は[u,v,w]');return[integer(e[0],0,n-1),integer(e[1],0,n-1),integer(e[2],0,1000)];}),limit=integer(x.max_iterations,0,256),f=s=>{const inSet=new Set(s);return s.reduce((v,e)=>v+modular[e],0)+edges.reduce((v,[a,b,w])=>v+(inSet.has(a)!==inSet.has(b)?w:0),0);};
  let g=greedy(Array(n).fill(0),f),value=0,selected=[],iterations=0;const incumbent=g=>{for(let i=0;i<g.chain.length;i++)if(g.chain[i]<value){value=g.chain[i];selected=g.order.slice(0,i);}};incumbent(g);const shift=g.base.reduce((v,a)=>v+Math.min(0,a),0),cuts=[g.base];let lower=shift,status='iteration_limit';
  const points=modular.map((_,i)=>[3*Math.cos(2*Math.PI*i/Math.max(1,n)),3*Math.sin(2*Math.PI*i/Math.max(1,n))]),scene={points,segments:edges.map(([a,b])=>[points[a],points[b]])};
  for(;;){frame(`下界と候補 ${iterations}`,`下界=${fmt(lower)}、候補=${fmt(value)}、集合={${selected.join(',')}}`,scene,['cut','基の座標'],cuts.map((s,i)=>[i,s.map(fmt).join(', ')]),[{label:'下界',value:fmt(lower)},{label:'候補',value:fmt(value)}]);if(Math.abs(lower-value)<1e-7){status='optimal';break;}if(iterations===limit)break;
   const A=Array.from({length:n},(_,i)=>Array.from({length:n+1},(_,j)=>i===j?1:0)).concat(cuts.map(s=>[...s,-1])),b=Array(n).fill(1).concat(cuts.map(()=>shift)),c=Array(n).fill(0).concat([-1]),solved=lp(A,b,c);iterations++;if(solved.status!=='optimal'){status='lp_limit';break;}lower=shift+solved.x[n];g=greedy(solved.x.slice(0,n),f);incumbent(g);cuts.push(g.base);
  }meta={status,value,lower,selected,iterations};return finish([qfmt(value),qfmt(lower)],scene);
 }
 if(id==='integer_program'){
  const {A,b,c}=problem(x),n=c.length,lower=array(x.lower,6).map(v=>integer(v,-100,100)),upper=array(x.upper,6).map(v=>integer(v,-100,100)),limit=integer(x.max_nodes,0,256);if(lower.length!==n||upper.length!==n)throw Error('整数上下限の次元が一致しません');
  const stack=[{lo:lower,hi:upper,bound:Infinity}];let best=null,nodes=0,status='infeasible';
  while(stack.length){if(best&&stack.at(-1).bound<=best.value+eps){stack.pop();continue;}if(nodes===limit){status='node_limit';break;}const node=stack.pop();if(node.lo.some((v,i)=>v>node.hi[i]))continue;nodes++;
   const rows=A.map(r=>r.slice()),rhs=b.map((v,i)=>v-dot(A[i],node.lo));for(let j=0;j<n;j++){const row=Array(n).fill(0);row[j]=1;rows.push(row);rhs.push(node.hi[j]-node.lo[j]);}const r=lp(rows,rhs,c);if(r.status==='infeasible')continue;if(r.status!=='optimal'){status='lp_limit';break;}const point=r.x.map((v,j)=>v+node.lo[j]),bound=r.value+dot(c,node.lo);
   frame(`分枝ノード ${nodes}`,`LP上界=${fmt(bound)}、整数候補=${best?fmt(best.value):'なし'}`,n===2?{...region(A.concat([[1,0],[-1,0],[0,1],[0,-1]]),b.concat([node.hi[0],-node.lo[0],node.hi[1],-node.lo[1]]),false),points:[point]}:{},['変数','下限','上限','LP解'],point.map((v,j)=>[j,node.lo[j],node.hi[j],fmt(v)]));
   if(best&&bound<=best.value+eps)continue;const branch=point.findIndex(v=>Math.abs(v-Math.round(v))>eps);if(branch<0){best={value:bound,solution:point.map(Math.round)};continue;}const floor=Math.floor(point[branch]),left={lo:node.lo.slice(),hi:node.hi.slice(),bound},right={lo:node.lo.slice(),hi:node.hi.slice(),bound};left.hi[branch]=floor;right.lo[branch]=floor+1;stack.push(right,left);
  }
  if(!stack.length&&status!=='lp_limit')status=best?'optimal':'infeasible';meta={status,nodes,...best};return finish(best?[qfmt(best.value),...best.solution]:[status]);
 }
 if(id==='omega_rational'){
  const z=new Rational(0),one=new Rational(1),parse=p=>array(p,7).map(v=>new Rational(integer(v,-1000,1000))),trim=p=>{while(p.length&&!p.at(-1).n)p.pop();return p;},p=trim(parse(x.numerator)),q=trim(parse(x.denominator));if(!q.length)throw Error('分母多項式は非零です');
  const divide=(a,b)=>{a=a.slice();const out=Array.from({length:Math.max(0,a.length-b.length+1)},()=>z);while(a.length>=b.length){const k=a.length-b.length,c=a.at(-1).div(b.at(-1));out[k]=c;for(let j=0;j<b.length;j++)a[j+k]=a[j+k].sub(c.mul(b[j]));trim(a);}return[trim(out),a];};
  let a=p,b=q;while(b.length){const r=divide(a,b)[1];frame('有理多項式のEuclid除算','剰余を繰り返し、共通因子を取り除きます。',{},['多項式','昇べき係数'],[['a',a.map(String).join(', ')],['b',b.map(String).join(', ')],['剰余',r.map(String).join(', ')]]);a=b;b=r;}
  const np=divide(p,a)[0],nq=divide(q,a)[0],scale=nq.at(-1);for(let i=0;i<np.length;i++)np[i]=np[i].div(scale);for(let i=0;i<nq.length;i++)nq[i]=nq[i].div(scale);
  const equal=np.length===2&&np[0].cmp(one)===0&&np[1].cmp(one)===0&&nq.length===1&&nq[0].cmp(one)===0;
  frame('最高次係数で比較','ωは全ての有理定数より大きく、1/ωは正の無限小です。',{},['分子','分母','符号','有限'],[[np.map(String).join(', '),nq.map(String).join(', '),np.length?np.at(-1).cmp(z):0,np.length<=nq.length?'はい':'いいえ']]);
  meta={status:'normalized',numerator:np.map(String),denominator:nq.map(String),sign:np.length?np.at(-1).cmp(z):0};return finish([Number(equal),1,'1/1']);
 }
 throw Error('未対応のモード');
}
const api={modes,trace,lp};if(typeof module==='object')module.exports=api;else root.OptimizationModel=api;
})(globalThis);
