(function(root){
  'use strict';
  const modes=[
    ['two_pointers','尺取り法',{values:[2,1,3,1,2],limit:4}],
    ['maximum_subarray','最大部分配列和',{values:[-2,3,-1,3,-5],allowEmpty:false}],
    ['dynamic_mex','動的MEX',{operations:[['add',0],['add',1],['add',1],['erase',1],['add',3],['erase',0]]}],
    ['meet_in_the_middle','半分全列挙',{values:[3,-2,5,1],target:6}],
    ['permutation','順列演算',{values:[2,0,1,4,3],exponent:-2}],
    ['unimodal_search','黄金分割・フィボナッチ探索',{method:'fibonacci',left:-10,right:11,center:3,iterations:40}],
    ['collection_hash','集合・多重集合ハッシュ',{multi:true,seed:'42',operations:[['add','1'],['add','1'],['add','2'],['erase','1']]}],
    ['permutation_tree','共通区間分解木',{values:[1,0,3,5,2,4]}],
    ['parentheses','括弧列と順序木',{pairs:3,rank:'3'}],
    ['integer_partitions','整数分割',{n:5,limit:20}],
    ['set_partitions','集合分割',{n:3,limit:20}],
    ['partial_persistent_search','過去版と最初の成立時刻',{initial:[0,0],updates:[[0,2],[1,5],[0,7]],index:0,threshold:4}],
    ['retroactive_operations','過去の操作を編集',{times:[10,20,30],initial:1,operations:[['set',10,2,1],['set',30,3,0],['set',20,1,4],['erase',10]],query:20}],
    ['special_integer_enumeration','特殊な整数の列挙',{kind:'palindrome',first:'90',last:'140',digits:[0,2],factors:['2','3'],limit:20}]
  ].map(([id,title,sample])=>({id,title,sample,doc:`basic_utility/${id}.md`}));
  function need(ok,message){if(!ok)throw Error(message);}
  function integer(x,low=-1000000,high=1000000){need(Number.isSafeInteger(x)&&low<=x&&x<=high,`${low}〜${high} の整数が必要です`);return x;}
  function list(x,limit=40){need(Array.isArray(x)&&x.length<=limit,`配列は${limit}要素までです`);return x;}
  function values(x,limit=40){return list(x,limit).map(v=>integer(v));}
  function big(x,low=-(1n<<63n),high=(1n<<63n)-1n){need((typeof x==='string'&&/^-?\d+$/.test(x)&&x.length<=80)||(typeof x==='number'&&Number.isSafeInteger(x)),'整数を10進文字列で入力してください');const v=BigInt(x);need(low<=v&&v<=high,'整数が範囲外です');return v;}
  function permutation(x,limit=40){const a=values(x,limit);need(new Set(a).size===a.length&&a.every(v=>0<=v&&v<a.length),'[0,n) の順列が必要です');return a;}
  const mask=(1n<<64n)-1n;
  function hash(x,seed){let z=((x&mask)^seed)+0x9e3779b97f4a7c15n;z&=mask;z=((z^(z>>30n))*0xbf58476d1ce4e5b9n)&mask;z=((z^(z>>27n))*0x94d049bb133111ebn)&mask;return z^(z>>31n);}
  function trace(mode,input){
    need(input&&typeof input==='object'&&!Array.isArray(input),'JSONオブジェクトが必要です');
    const steps=[];let result;
    const step=(title,detail,vals,columns=[],rows=[])=>steps.push({title,detail,values:Object.entries(vals).map(([label,value])=>({label,value:typeof value==='object'?JSON.stringify(value):String(value)})),columns:[...columns],rows:rows.map(row=>row.map(v=>typeof v==='object'?JSON.stringify(v):String(v)))});
    step('初期状態','入力を変えて、各段階の候補と確定した結果を比較できます。',input);
    if(mode==='two_pointers'){
      const a=values(input.values);a.forEach(v=>integer(v,0));const cap=integer(input.limit,0);
      let r=0,sum=0,count=0,best=[0,0];const right=[];
      for(let l=0;l<a.length;l++){
        while(r<a.length&&sum+a[r]<=cap){sum+=a[r++];step('右端を追加','条件を満たす間だけ右端を進めます。',{left:l,right:r,sum},['位置','値','区間内'],a.map((v,i)=>[i,v,l<=i&&i<r?'●':'']));}
        right.push(r);count+=r-l;if(r-l>best[1]-best[0])best=[l,r];
        step('左端ごとの最大区間','ここから始まる非空区間は right−left 個です。',{left:l,right:r,count});
        if(l<r)sum-=a[l];else ++r;
      }
      result={right,longest:best,count};
    }else if(mode==='maximum_subarray'){
      const a=values(input.values);need(typeof input.allowEmpty==='boolean','allowEmpty は真偽値です');let sum=0,left=0,best=input.allowEmpty?{sum:0,left:0,right:0}:null;
      a.forEach((v,i)=>{if(sum<0){sum=0;left=i;}sum+=v;if(!best||sum>best.sum||(sum===best.sum&&(left<best.left||(left===best.left&&i+1<best.right))))best={sum,left,right:i+1};step('右端を1つ進める','負の累積和を捨て、和0なら早い左端を残します。',{left,right:i+1,sum,best},['位置','値','候補'],a.map((x,j)=>[j,x,left<=j&&j<=i?'●':'']));});result=best;
    }else if(mode==='dynamic_mex'){
      const counts=new Map(),history=[];let size=0,mex=0;
      for(const operation of list(input.operations,80)){need(Array.isArray(operation)&&operation.length===2,'操作は [add/erase, 値]');const[op,x]=operation;integer(x);need(op==='add'||op==='erase','add または erase を指定してください');let c=counts.get(x)||0;
        if(op==='add'){counts.set(x,c+1);++size;}else if(c){--size;if(c===1)counts.delete(x);else counts.set(x,c-1);}
        mex=0;while(counts.has(mex))++mex;history.push(mex);step('頻度と最小の欠損','重複が1個残れば、その値はまだ存在します。',{operation,mex,size},['値','個数'],[...counts].sort((a,b)=>a[0]-b[0]));
      }result={mex,size,history};
    }else if(mode==='meet_in_the_middle'){
      const a=values(input.values,16),target=integer(input.target),split=Math.floor(a.length/2);
      const table=(data,offset)=>{let sums=[{sum:0,mask:0}];data.forEach((v,i)=>{sums=sums.concat(sums.map(s=>({sum:s.sum+v,mask:s.mask+2**(i+offset)}))).sort((a,b)=>a.sum-b.sum||a.mask-b.mask);step('半分の部分和を拡張','各要素を採用する列と採用しない列を整列します。',{side:offset?'後半':'前半',items:i+1,subsets:sums.length},['和','元添字mask'],sums.slice(0,32).map(s=>[s.sum,s.mask]));});return sums;};
      const left=table(a.slice(0,split),0),right=table(a.slice(split),split);let countEqual=0,countLeq=0,best=null;
      for(const x of left){let feasible=0;for(const y of right){const s=x.sum+y.sum;if(s===target)++countEqual;if(s<=target){++countLeq;++feasible;const candidate={sum:s,mask:x.mask+y.mask};if(!best||s>best.sum||(s===best.sum&&candidate.mask<best.mask))best=candidate;}}
        step('左右の和を組み合わせる','図では小さい表を走査。C++版は単調な境界を動かします。',{leftSum:x.sum,feasible,countLeq,countEqual,best});}
      result={best,countEqual,countLeq};
    }else if(mode==='permutation'){
      const p=permutation(input.values),exponent=big(input.exponent),seen=new Set(),cycles=[],inverse=[],power=[];
      p.forEach((v,i)=>inverse[v]=i);
      for(let i=0;i<p.length;i++)if(!seen.has(i)){const cycle=[];for(let v=i;!seen.has(v);v=p[v]){seen.add(v);cycle.push(v);}cycles.push(cycle);const shift=Number((exponent%BigInt(cycle.length)+BigInt(cycle.length))%BigInt(cycle.length));cycle.forEach((v,j)=>power[v]=cycle[(j+shift)%cycle.length]);step('サイクルを循環移動','指数をサイクル長で割った余りだけ進みます。',{cycle,shift},['元','p','逆','冪'],p.map((v,j)=>[j,v,inverse[j],power[j]??'未処理']));}
      result={inverse,cycles,sign:(p.length-cycles.length)%2?-1:1,power};
    }else if(mode==='unimodal_search'){
      const l=integer(input.left,-1000,1000),r=integer(input.right,-1000,1000),center=input.center;need(Number.isFinite(center)&&Math.abs(center)<=1000,'center は有限値 ±1000 以内です');let best=null,calls=0;
      const evaluate=x=>{const value=Math.abs(x-center);++calls;if(!best||value<best.value||(value===best.value&&x<best.position))best={position:x,value};step('関数を評価','目的関数は |x−center|。観測済みの最良点も保持します。',{position:x,value,evaluations:calls,best});return value;};
      if(input.method==='fibonacci'){
        need(l<r,'整数の探索区間 [left,right) は非空にしてください');const fib=[0,1,1,2];while(fib.at(-1)<r-l+1)fib.push(fib.at(-1)+fib.at(-2));let k=fib.length-1,a=l-1;
        const sample=x=>x>=r?Infinity:evaluate(x);
        if(r-l===1)evaluate(l);else{let x=a+fib[k-2],y=a+fib[k-1],fx=sample(x),fy=sample(y);while(k>3){--k;if(fx<=fy){y=x;fy=fx;if(k>3){x=a+fib[k-2];fx=sample(x);}}else{a=x;x=y;fx=fy;if(k>3){y=a+fib[k-1];fy=sample(y);}}step('フィボナッチ長を縮小','一方の観測値を再利用し、仮想領域は評価しません。',{left:a+1,right:a+fib[k],order:k,evaluations:calls});}}
      }else{
        need(input.method==='golden','method は fibonacci / golden');need(l<=r,'閉区間の端点が逆順です');const iterations=integer(input.iterations,0,100);let a=l,b=r;evaluate(a);
        if(a<b){evaluate(b);const ratio=(Math.sqrt(5)-1)/2;let x=b-ratio*(b-a),y=a+ratio*(b-a),fx=evaluate(x),fy=evaluate(y);
          for(let i=0;i<iterations&&a<x&&x<y&&y<b;i++){if(fx<=fy){b=y;y=x;fy=fx;x=b-ratio*(b-a);fx=evaluate(x);}else{a=x;x=y;fx=fy;y=a+ratio*(b-a);fy=evaluate(y);}step('黄金比で区間を縮小','入力端点も最小値の候補です。',{left:a,right:b,width:b-a,evaluations:calls});}}
      }result=best;
    }else if(mode==='collection_hash'){
      need(typeof input.multi==='boolean','multi は真偽値です');const seed=big(input.seed,0n,mask),counts=new Map();let size=0,a=0n,b=0n;
      for(const operation of list(input.operations,80)){need(Array.isArray(operation)&&operation.length===2,'操作は [add/erase, 整数文字列]');const[op,raw]=operation,key=big(raw).toString(),x=BigInt(key);need(op==='add'||op==='erase','add または erase を指定してください');const c=counts.get(key)||0;
        const changed=op==='add'?(input.multi||c===0):c>0;
        if(changed){const h1=hash(x,seed),h2=hash(x,seed^0x9e3779b97f4a7c15n);if(input.multi){a=(a+(op==='add'?h1:-h1))&mask;b=(b+(op==='add'?h2:-h2))&mask;}else{a^=h1;b^=h2;}if(op==='add'){counts.set(key,c+1);++size;}else{if(c===1)counts.delete(key);else counts.set(key,c-1);--size;}}
        step('可換hashの差分更新','同じseedで比較します。hash一致でも衝突の可能性は残ります。',{operation,changed,size,first:a.toString(),second:b.toString()},['値','個数'],[...counts]);
      }result={seed:seed.toString(),size,first:a.toString(),second:b.toString()};
    }else if(mode==='permutation_tree'){
      const p=permutation(input.values,12),common=[];
      for(let l=0;l<p.length;l++){let low=Infinity,high=-Infinity;for(let r=l+1;r<=p.length;r++){low=Math.min(low,p[r-1]);high=Math.max(high,p[r-1]);if(high-low===r-l-1)common.push([l,r]);}}
      step('共通区間を確認','図では全区間の最大−最小と長さを比較します。C++の構築は O(n log n) です。',{count:common.length},['左端','右端'],common);
      const overlap=(a,b)=>(a[0]<b[0]&&b[0]<a[1]&&a[1]<b[1])||(b[0]<a[0]&&a[0]<b[1]&&b[1]<a[1]);
      const strong=common.filter(a=>!common.some(b=>overlap(a,b))),nodes=[];
      for(const interval of strong){const[l,r]=interval;const inside=strong.filter(b=>l<=b[0]&&b[1]<=r&&(l!==b[0]||r!==b[1]));
        const children=inside.filter(b=>!inside.some(c=>c!==b&&c[0]<=b[0]&&b[1]<=c[1]));let type='leaf';
        if(children.length){const lows=children.map(([l,r])=>Math.min(...p.slice(l,r)));type=lows.every((v,i)=>!i||lows[i-1]<v)?'increasing':lows.every((v,i)=>!i||lows[i-1]>v)?'decreasing':'prime';}
        nodes.push([l,r,type]);step('強共通区間を節点にする','包含以外の交差がない共通区間だけが節点になります。',{interval,type},['子の左端','子の右端'],children);
      }result={count:common.length,nodes};
    }else if(mode==='parentheses'){
      const n=integer(input.pairs,0,30),dp=Array.from({length:n+1},()=>Array(n+1).fill(0n));for(let c=0;c<=n;c++)dp[0][c]=1n;
      for(let o=1;o<=n;o++)for(let c=o;c<=n;c++)dp[o][c]=dp[o-1][c]+(c>o?dp[o][c-1]:0n);
      let rank=big(input.rank,0n,10n**70n);need(rank<dp[n][n],'rank が括弧列の個数以上です');const original=rank;let text='',open=n,close=n;
      while(close){const starts=open?dp[open-1][close]:0n;if(rank<starts){text+='(';--open;}else{rank-=starts;text+=')';--close;}step('辞書順のブロックを選択','開き括弧で始まる補完数と順位を比較します。',{prefix:text,startsOpen:starts.toString(),remainingRank:rank.toString(),open,close});}
      const children=[[]],path=[0];for(const c of text){if(c==='('){const id=children.length;children.push([]);children[path.at(-1)].push(id);path.push(id);}else path.pop();}
      step('仮想根を持つ順序木','括弧1組が辺1本、根0は括弧で囲みません。',{text},['頂点','子'],children.map((row,i)=>[i,row]));result={count:dp[n][n].toString(),text,rank:original.toString(),children};
    }else if(mode==='integer_partitions'){
      const n=integer(input.n,0,100),limit=integer(input.limit,1,100),dp=Array(n+1).fill(0n);dp[0]=1n;for(let p=1;p<=n;p++)for(let s=p;s<=n;s++)dp[s]+=dp[s-p];const partitions=[];let parts=n?[n]:[];
      while(partitions.length<limit){partitions.push([...parts]);step('整数分割を出力','非増加の部分を、降順辞書順で列挙します。',{ordinal:partitions.length,partition:parts},['部分','値'],parts.map((v,i)=>[i,v]));let remainder=0;while(parts.length&&parts.at(-1)===1){parts.pop();++remainder;}if(!parts.length)break;--parts[parts.length-1];++remainder;const cap=parts.at(-1);while(remainder>cap){parts.push(cap);remainder-=cap;}parts.push(remainder);}
      result={count:dp[n].toString(),partitions,more:dp[n]>BigInt(partitions.length)};
    }else if(mode==='set_partitions'){
      const n=integer(input.n,0,20),limit=integer(input.limit,1,100),dp=Array(n+1).fill(0n);dp[0]=1n;for(let i=1;i<=n;i++){for(let k=i;k;k--)dp[k]=dp[k]*BigInt(k)+dp[k-1];dp[0]=0n;}const count=dp.reduce((a,b)=>a+b,0n),partitions=[],rgs=Array(n).fill(0);
      while(partitions.length<limit){partitions.push([...rgs]);const blocks=[];rgs.forEach((b,i)=>(blocks[b]??=[]).push(i));step('RGSを出力','初出のブロックに次の番号を割り当てるため重複しません。',{ordinal:partitions.length,rgs},['ブロック','要素'],blocks.map((b,i)=>[i,b]));let pos=n-1;while(pos>0&&rgs[pos]===1+Math.max(...rgs.slice(0,pos)))--pos;if(pos<=0)break;++rgs[pos];rgs.fill(0,pos+1);}
      result={count:count.toString(),partitions,more:count>BigInt(partitions.length)};
    }else if(mode==='partial_persistent_search'){
      const initial=values(input.initial,20),history=[[...initial]],index=integer(input.index,0,initial.length-1),threshold=integer(input.threshold);
      for(const update of list(input.updates,60)){need(Array.isArray(update)&&update.length===2,'更新は [添字,値]');const[i,v]=update;integer(i,0,initial.length-1);integer(v);const next=[...history.at(-1)];next[i]=v;history.push(next);step('最新版に代入','同じ値への代入も版番号を1進めます。',{time:history.length-1,index:i,value:v},['版','配列'],history.map((a,t)=>[t,a]));}
      const predicates=history.map(a=>a[index]>=threshold);need(!predicates.some((b,i)=>i&&predicates[i-1]&&!b),'この閾値条件は履歴上で単調ではありません');let l=0,r=history.length;
      while(l<r){const mid=Math.floor((l+r)/2);step('過去版を観測','false→true の境界を二分探索します。',{left:l,right:r,middle:mid,value:history[mid][index],satisfied:predicates[mid]},['版','対象値','条件'],history.map((a,t)=>[t,a[index],predicates[t]]));if(predicates[mid])r=mid;else l=mid+1;}
      result={time:history.length-1,first:l===history.length?null:l,history};
    }else if(mode==='retroactive_operations'){
      const times=[...new Set(values(input.times,30))].sort((a,b)=>a-b),initial=BigInt(integer(input.initial)),query=integer(input.query),operations=new Map(),history=[];
      const replay=predicate=>{let x=initial;for(const t of times)if(predicate(t)&&operations.has(t)){const[a,b]=operations.get(t);x=a*x+b;}return x.toString();};
      for(const action of list(input.operations,60)){
        need(Array.isArray(action)&&(action[0]==='set'?action.length===4:action[0]==='erase'&&action.length===2),'操作は [set,時刻,a,b] / [erase,時刻]');
        const time=integer(action[1]);need(times.includes(time),'操作の時刻を times に登録してください');
        if(action[0]==='set')operations.set(time,[BigInt(integer(action[2],-10,10)),BigInt(integer(action[3]))]);else operations.delete(time);
        const current=replay(()=>true);history.push(current);let x=initial;
        const rows=times.map(t=>{const op=operations.get(t);if(op)x=op[0]*x+op[1];return [t,op?`${op[0]}x + ${op[1]}`:'操作なし',x.toString()];});
        step('過去を編集して再評価','時刻順の合成を表示します。C++はセグメント木の祖先だけを更新します。',{action,current,before:replay(t=>t<query),through:replay(t=>t<=query)},['時刻','操作','直後の値'],rows);
      }
      result={size:operations.size,before:replay(t=>t<query),through:replay(t=>t<=query),current:replay(()=>true),history};
    }else if(mode==='special_integer_enumeration'){
      const kind=input.kind,first=big(input.first,0n,mask),last=big(input.last,0n,mask),limit=integer(input.limit,1,100),output=[];let more=false;
      need(first<=last,'範囲は first ≤ last にしてください');
      const emit=(x,detail)=>{if(output.length===limit){more=true;return false;}output.push(x.toString());step('整数を出力',detail,{kind,ordinal:output.length,value:x.toString()},['順番','整数'],output.map((v,i)=>[i+1,v]));return true;};
      if(kind==='palindrome'){
        const mirror=(prefix,length)=>{const text=prefix.toString();return BigInt(text+[...text.slice(0,length%2?-1:undefined)].reverse().join(''));};
        let stopped=false;
        for(let length=1;length<=20&&!stopped;length++){
          const power=10n**BigInt(Math.ceil(length/2));let lo=length===1?0n:power/10n,hi=power;
          while(lo<hi){const mid=(lo+hi)/2n;if(mirror(mid,length)>=first)hi=mid;else lo=mid+1n;}
          for(let prefix=lo;prefix<power;prefix++){const x=mirror(prefix,length);if(x>=last)break;if(!emit(x,`前半 ${prefix} を反転して ${length} 桁の回文を作ります。`)){stopped=true;break;}}
        }
      }else if(kind==='digits'){
        const digits=[...new Set(list(input.digits,20).map(d=>integer(d,0,9)))].sort((a,b)=>a-b),powers=Array.from({length:20},(_,i)=>10n**BigInt(i));
        const walk=(prefix,remaining)=>{
          if(prefix<first/powers[remaining]||prefix>(last-1n)/powers[remaining])return true;
          if(!remaining)return emit(prefix,'桁数順・数字順。範囲と交わらないprefixの枝は飛ばします。');
          for(const d of digits)if(!walk(prefix*10n+BigInt(d),remaining-1))return false;return true;
        };
        if(first<last){let stopped=first===0n&&digits.includes(0)&&!emit(0n,'0は数字0を許した場合だけ出力します。');
          for(let length=1;length<=20&&!stopped;length++)for(const d of digits)if(d&&!walk(BigInt(d),length-1)){stopped=true;break;}}
      }else if(kind==='products'){
        need(first===0n,'products は first=0、出力範囲 [1,last) です');
        const factors=[...new Set(list(input.factors,20).map(f=>big(f,2n,mask)))],frontier=new Set(last>1n?[1n]:[]);
        while(frontier.size){let x=null;for(const v of frontier)if(x===null||v<x)x=v;frontier.delete(x);
          if(!emit(x,'候補集合の最小値を取り出し、各因子を掛けた次候補を追加します。'))break;
          for(const f of factors)if(x*f<last)frontier.add(x*f);
          step('積の候補を追加','図は集合で候補を保持。C++は各因子のポインタで整列列をマージします。',{factors:factors.map(String)},['次候補'],[...frontier].sort((a,b)=>a<b?-1:1).slice(0,20).map(String).map(x=>[x]));
        }
      }else throw Error('kind は palindrome / digits / products');
      result={values:output,more};
    }else throw Error('未知のモードです');
    step('最終結果','入力を編集して再実行できます。',result&&typeof result==='object'?result:{result});return {steps,result};
  }
  const api={modes,trace};if(typeof module!=='undefined')module.exports=api;root.UtilitySearchModel=api;
})(typeof globalThis==='undefined'?this:globalThis);
