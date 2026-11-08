(function (root) {
  'use strict';
  const modes = [
    ['frontier_dp','P-01 · Frontier DP',{n:4,edges:[[0,1],[1,2],[2,3],[3,0]],source:0,target:3,kind:'cycle'}],
    ['bit_parallel_lcs','P-02 · Bit-parallel LCS',{a:'ABCBDAB',b:'BDCABA'}],
    ['myers_edit_distance','P-03 · Myers編集距離',{a:'kitten',b:'sitting'}],
    ['exact_cover','P-04 · Exact Cover',{primary:3,secondary:0,rows:[[0,1],[2],[0],[1,2]]}],
    ['a_star','P-05 · A*',{n:4,edges:[[0,1,5],[0,2,1],[1,3,10],[2,1,1]],source:0,target:3,h:[0,0,11,0]}],
    ['ida_star','P-06 · IDA*',{n:4,edges:[[0,1,5],[0,2,1],[1,3,10],[2,1,1]],source:0,target:3,h:[0,0,11,0]}],
    ['bidirectional_search','P-07 · 双方向BFS',{n:5,edges:[[0,1,1],[0,2,1],[1,3,1],[2,3,1],[3,4,1]],source:0,target:4}],
    ['sat_solver','P-08 · SAT / DPLL',{n:3,clauses:[[1,2],[-1,3],[-2,3],[-3,2]],assumptions:[-3]}],
    ['knapsack_reconstruction','P-09 · ナップサック復元',{capacity:5,items:[[0,3],[2,4],[3,5],[2,4]],weights:[2,3,2],target:5}],
    ['optimal_solution_count','P-10 · 最適解数',{sequence:[1,2,2,3],a:'aa',b:'a',capacity:2,items:[[1,1],[1,1],[2,2]]}],
    ['monge_matrix_product','P-20 · Monge行列積',{a:[[0,4,2],[3,0,5]],b:[[0,1,4],[1,0,1],[4,1,0]]}],
    ['weighted_slope_trick','P-21 · 重み付きSlope Trick',{operations:[['abs',1,2],['abs',5,3],['shift',-1,2],['reflect']],x:0}],
    ['dynamic_finite_state_dp','P-22 · 動的な有限状態DP',{states:2,matrices:[[[0,3],[2,1]],[[1,0],[4,2]]],updates:[[0,[[2,5],[2,1]]]],source:0,target:1}],
    ['small_weight_knapsack','P-23 · 少種類重量Knapsack',{capacity:7,items:[[2,5],[2,4],[2,1],[3,7],[3,6],[0,2]]}]
  ].map(([id,title,sample])=>({id,title,sample,doc:`dp_optimization/${id}.md`}));
  const copy=x=>JSON.parse(JSON.stringify(x));
  function integer(x,lo=0,hi=10000){if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`整数 ${lo}〜${hi} を指定してください`);return x;}
  function array(x,max=100){if(!Array.isArray(x)||x.length>max)throw Error(`配列（最大${max}件）が必要です`);return x;}
  function bytes(s){if(typeof s!=='string'||s.length>256||[...s].some(c=>c.charCodeAt(0)>255))throw Error('256文字以下の1バイト文字列を指定してください');return s;}
  function graph(input,weighted=false){const n=integer(input.n,1,10);array(input.edges,18).forEach(e=>{array(e,3);if(e.length!==(weighted?3:2))throw Error('辺の要素数が違います');integer(e[0],0,n-1);integer(e[1],0,n-1);if(weighted)integer(e[2],0,30);else if(e[0]===e[1])throw Error('自己ループは対象外です');});integer(input.source,0,n-1);integer(input.target,0,n-1);return n;}
  function items(input){integer(input.capacity,0,50);array(input.items,20).forEach(x=>{if(array(x,2).length!==2)throw Error('品物は[重さ,価値]');integer(x[0],0,50);integer(x[1],-100,100);});}
  function trace(id,input){
    if(!modes.some(m=>m.id===id))throw Error('不明なモード');
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトが必要です');
    const steps=[];
    function add(title,detail,columns=[],rows=[],values=[],state=null,plot=null){if(steps.length>=3000)throw Error('図解の探索量上限を超えました。入力を小さくしてください');steps.push(copy({title,detail,columns,rows,values,state,plot}));}
    add('入力を確認','入力を編集すると、この後の候補と結果も変わります。');
    const view=(active=[],values=[],selected=[])=>({n:input.n,edges:input.edges,active,values,selected,directed:id!=='frontier_dp'});
    let result;
    if(id==='frontier_dp'){
      const n=graph(input),edges=input.edges;if(!['path','cycle'].includes(input.kind))throw Error('kindはpath/cycle');
      const last=Array(n).fill(-1);edges.forEach(([u,v],i)=>{last[u]=last[v]=i;});
      const run=(degree,shown)=>{
        if(degree.some((d,v)=>d&&last[v]<0))return 0;
        let states=[{d:[],c:[],done:false,count:1}],frontier=[];
        for(let ei=0;ei<edges.length;ei++){
          const [u,v]=edges[ei],active=[...frontier];for(const x of [u,v])if(!active.includes(x))active.push(x);
          const ai=active.indexOf(u),bi=active.indexOf(v),next=new Map();
          for(const old of states)for(const take of [0,1]){
            const d=[...old.d],c=[...old.c];while(d.length<active.length){d.push(0);c.push(-1);}
            if(take){if(old.done||d[ai]===degree[u]||d[bi]===degree[v])continue;d[ai]++;d[bi]++;let ca=c[ai],cb=c[bi];if(ca<0&&cb<0)ca=active.length;if(ca<0)ca=cb;if(cb>=0&&ca!==cb)for(let i=0;i<c.length;i++)if(c[i]===cb)c[i]=ca;c[ai]=c[bi]=ca;}
            let valid=true;const nd=[],nc=[],remaining=new Map();
            active.forEach((x,j)=>{if(c[j]>=0&&!remaining.has(c[j]))remaining.set(c[j],false);if(last[x]===ei){if(d[j]!==degree[x])valid=false;}else{nd.push(d[j]);nc.push(c[j]);if(c[j]>=0)remaining.set(c[j],true);}});
            if(!valid)continue;const closed=[...remaining.values()].filter(x=>!x).length;let done=old.done;
            if(closed){if(closed!==1||remaining.size!==1||done)continue;done=true;}
            const labels=new Map();for(let j=0;j<nc.length;j++)if(nc[j]>=0){if(!labels.has(nc[j]))labels.set(nc[j],labels.size);nc[j]=labels.get(nc[j]);}
            const key=JSON.stringify([nd,nc,done]);if(next.has(key))next.get(key).count+=old.count;else next.set(key,{d:nd,c:nc,done,count:old.count});
          }
          states=[...next.values()];frontier=active.filter(x=>last[x]!==ei);
          if(shown)add(`辺 e${ei}: ${u} — ${v}`,`採用/不採用から同じ境界状態を統合。残る境界は [${frontier}]。`,['次数','正規化成分','完成','個数'],states.map(s=>[s.d.join(','),s.c.join(','),s.done,s.count]),[{label:'状態数',value:states.length}],view(frontier,[],[ei]));
        }
        return states.filter(s=>s.done).reduce((a,s)=>a+s.count,0);
      };
      const degree=Array(n).fill(2);degree[input.source]=degree[input.target]=1;
      const path=input.source===input.target?Number(n===1):run(degree,input.kind==='path'),cycle=run(Array(n).fill(2),input.kind==='cycle');result=[path,cycle];
    }else if(id==='bit_parallel_lcs'||id==='myers_edit_distance'){
      const a=bytes(input.a),b=bytes(input.b),m=b.length,mask=(1n<<BigInt(m))-1n,masks=new Map();for(let j=0;j<m;j++)masks.set(b[j],(masks.get(b[j])||0n)|(1n<<BigInt(j)));
      let row=0n,pv=mask,mv=0n,score=m;
      const bits=x=>(x&mask).toString(2).padStart(m||1,'0'),pop=x=>{let n=0;while(x){x&=x-1n;n++;}return n;};
      if(!m){result=[id==='bit_parallel_lcs'?0:a.length];}
      else{for(let i=0;i<a.length;i++){
        const eq=masks.get(a[i])||0n;
        if(id==='bit_parallel_lcs'){const x=row|eq,y=(row<<1n)|1n;row=x&~(x-y)&mask;add(`a[${i}] = ${JSON.stringify(a[i])}`,'右端が列bの先頭です。LCS表の増分位置を1bitで保持します。',['mask','bits'],[['一致',bits(eq)],['x',bits(x)],['y',bits(y)],['差分row',bits(row)]],[{label:'LCS長',value:pop(row)}]);}
        else {const xv=eq|mv,xh=(((eq&pv)+pv)^pv)|eq;let ph=mv|~(xh|pv),mh=pv&xh;const top=1n<<BigInt(m-1);if(ph&top)score++;if(mh&top)score--;ph=(ph<<1n)|1n;mh<<=1n;pv=(mh|~(xv|ph))&mask;mv=ph&xv;add(`文字 ${i+1}: ${JSON.stringify(a[i])}`,'±1の差分を二つのbit列で更新。列全体への編集距離です。',['mask','bits'],[['一致',bits(eq)],['positive',bits(pv)],['negative',bits(mv)]],[{label:'編集距離',value:score}]);}
      }result=[id==='bit_parallel_lcs'?pop(row):score];}
    }else if(id==='exact_cover'){
      const p=integer(input.primary,0,8),s=integer(input.secondary,0,8),rows=array(input.rows,16);const masks=rows.map(row=>{array(row,16);if(!row.length||new Set(row).size!==row.length)throw Error('行の列番号は空・重複不可');row.forEach(c=>integer(c,0,p+s-1));if(!row.some(c=>c<p))throw Error('各行はprimary列が必要です');return row.reduce((a,c)=>a|(1<<c),0);});
      let count=0,first=null;const target=(1<<p)-1;
      const dfs=(used,chosen)=>{
        if((used&target)===target){count++;if(first===null)first=[...chosen];add('全primary列を被覆',`行ID [${chosen}] が一解です。`,['行ID','選択'],rows.map((_,i)=>[i,chosen.includes(i)?'採用':'']),[{label:'解数',value:count}]);return;}
        let candidates=null,column=-1;for(let c=0;c<p;c++)if(!(used>>c&1)){const options=masks.map((m,i)=>i).filter(i=>(masks[i]>>c&1)&&!(masks[i]&used));if(candidates===null||options.length<candidates.length){candidates=options;column=c;}}
        add(`primary列 ${column} を選択`,`候補行 [${candidates}]。既に覆った列と交わる行を除きます。`,['行ID','列集合','候補'],rows.map((r,i)=>[i,r.join(','),candidates.includes(i)?'○':'']),[{label:'現在の行',value:chosen.join(',')||'空'}]);
        for(const row of candidates)dfs(used|masks[row],[...chosen,row]);
      };dfs(0,[]);result=[count,...(first||[])];
    }else if(['a_star','ida_star','bidirectional_search'].includes(id)){
      const n=graph(input,true),s=input.source,t=input.target,g=Array.from({length:n},()=>[]),rev=Array.from({length:n},()=>[]);input.edges.forEach(([u,v,w],i)=>{g[u].push({to:v,cost:w,id:i});rev[v].push({to:u,cost:w,id:i});});
      let answer=null;
      if(id==='bidirectional_search'){
        if(input.edges.some(e=>e[2]!==1))throw Error('双方向BFSは単位費用のみ');
        const dist=[Array(n).fill(-1),Array(n).fill(-1)],paths=[Array(n),Array(n)],front=[[s],[t]];dist[0][s]=dist[1][t]=0;paths[0][s]=[s];paths[1][t]=[t];if(s===t)answer={cost:0,path:[s]};
        while(!answer&&front[0].length&&front[1].length){const side=front[0].length<=front[1].length?0:1,next=[];let best=Infinity;for(const u of front[side])for(const e of (side?rev:g)[u])if(dist[side][e.to]<0){const v=e.to;dist[side][v]=dist[side][u]+1;paths[side][v]=[...paths[side][u],v];next.push(v);if(dist[1-side][v]>=0&&dist[0][v]+dist[1][v]<best){best=dist[0][v]+dist[1][v];answer={cost:best,path:[...paths[0][v],...paths[1][v].slice(0,-1).reverse()]};}}add(`${side?'終点':'始点'}側を一層展開`,'層の途中では終了せず、全合流候補から最短を選びます。',['状態','始点距離','終点距離'],dist[0].map((d,v)=>[v,d,dist[1][v]]),[],view(next,dist[side]));front[side]=next;}
      }else{
        const h=array(input.h,n);if(h.length!==n)throw Error('hの長さが違います');h.forEach(v=>integer(v,0,100));const d=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?0:Infinity));input.edges.forEach(([u,v,w])=>d[u][v]=Math.min(d[u][v],w));for(let k=0;k<n;k++)for(let i=0;i<n;i++)for(let j=0;j<n;j++)d[i][j]=Math.min(d[i][j],d[i][k]+d[k][j]);if(h[t]!==0||h.some((x,v)=>x>d[v][t]))throw Error('hは許容的な非負下界で、目標では0にしてください');
        if(id==='a_star'){
          const best=Array(n).fill(Infinity),queue=[{v:s,g:0,path:[s],selected:[],serial:0}];best[s]=0;let serial=1;
          while(queue.length){queue.sort((a,b)=>a.g+h[a.v]-b.g-h[b.v]||a.serial-b.serial);const cur=queue.shift();if(cur.g!==best[cur.v])continue;add(`状態 ${cur.v} を取り出す`,`g=${cur.g}, h=${h[cur.v]}, f=${cur.g+h[cur.v]}。改善なら再展開します。`,['状態','g','h','f'],best.map((x,v)=>[v,Number.isFinite(x)?x:'∞',h[v],Number.isFinite(x)?x+h[v]:'∞']),[],view([cur.v],best.map(x=>Number.isFinite(x)?x:'∞'),cur.selected));if(cur.v===t){answer={cost:cur.g,path:cur.path};break;}for(const e of g[cur.v])if(cur.g+e.cost<best[e.to]){best[e.to]=cur.g+e.cost;queue.push({v:e.to,g:best[e.to],path:[...cur.path,e.to],selected:[...cur.selected,e.id],serial:serial++});}}
        }else{
          let bound=h[s];while(!answer){let next=Infinity;add(`閾値 ${bound} で開始`,'経路上の循環を除去し、最小の閾値超過を記録します。');const dfs=(v,cost,path,selected)=>{const f=cost+h[v];add(`状態 ${v}: f=${f}`,f>bound?'閾値超過。次回候補へ。':'閾値以内の単純パスを展開。',['状態列','g','h','閾値'],[[path.join(' → '),cost,h[v],bound]],[],view(path,[],selected));if(f>bound){next=Math.min(next,f);return false;}if(v===t){answer={cost,path};return true;}for(const e of g[v])if(!path.includes(e.to)&&dfs(e.to,cost+e.cost,[...path,e.to],[...selected,e.id]))return true;return false;};dfs(s,0,[s],[]);if(!Number.isFinite(next))break;bound=next;}
        }
      }
      result=answer?[answer.cost,...answer.path]:['null'];
    }else if(id==='sat_solver'){
      const n=integer(input.n,0,12),clauses=array(input.clauses,40),assumptions=array(input.assumptions,12);for(const c of [...clauses,...assumptions.map(x=>[x])])array(c,20).forEach(x=>{integer(x,-n,n);if(!x)throw Error('literal 0は禁止です');});
      const solve=extra=>{const all=[...clauses,...extra.map(x=>[x])];let model=null;const occurrences=Array(n).fill(0);all.forEach(c=>c.forEach(x=>occurrences[Math.abs(x)-1]++));
        const dfs=values=>{for(;;){let changed=false;for(const clause of all){if(clause.some(x=>values[Math.abs(x)-1]===Number(x>0)))continue;const unknown=[...new Set(clause.filter(x=>values[Math.abs(x)-1]<0))];if(!unknown.length){add('矛盾して戻る','全literalが偽の節があります。',['変数','値'],values.map((x,v)=>[v+1,x<0?'未定':x]));return false;}if(unknown.length===1){const x=unknown[0];values[Math.abs(x)-1]=Number(x>0);changed=true;add('単位節を伝播',`literal ${x} を真にします。`,['変数','値'],values.map((x,v)=>[v+1,x<0?'未定':x]));}}if(!changed)break;}
          let v=-1;for(let i=0;i<n;i++)if(values[i]<0&&(v<0||occurrences[i]>occurrences[v]))v=i;if(v<0){model=values;return true;}for(const value of [1,0]){const next=[...values];next[v]=value;add(`変数 ${v+1} = ${value} を仮定`,'反対の値への分岐も残します。',['変数','値'],next.map((x,v)=>[v+1,x<0?'未定':x]));if(dfs(next))return true;}return false;};dfs(Array(n).fill(-1));return model;};
      const first=solve([]);add('仮定を加えて再求解',`一時的なliteral [${assumptions}] を追加。元のCNFは保持します。`);const second=solve(assumptions);result=[Number(first!==null),Number(second!==null)];
    }else if(id==='knapsack_reconstruction'){
      items(input);array(input.weights,20).forEach(w=>integer(w,0,50));integer(input.target,0,input.capacity);
      let dp=Array(input.capacity+1).fill(null);dp[0]={value:0,counts:[]};
      input.items.forEach(([weight,value],i)=>{const next=dp.map(x=>x?{value:x.value,counts:[...x.counts,0]}:null);for(let w=weight;w<=input.capacity;w++)if(dp[w-weight]){const candidate=dp[w-weight].value+value;if(next[w]===null||candidate>next[w].value)next[w]={value:candidate,counts:[...dp[w-weight].counts,1]};}dp=next;add(`品物 ${i} を処理`,'前の段階の選択履歴から、今回の採用/不採用を保持します。',['重さ','最大価値','個数'],dp.map((x,w)=>[w,x?.value??'未到達',x?.counts.join(',')??'']));});
      let best=dp[0];for(const x of dp)if(x&&x.value>best.value)best=x;
      const paths=Array(input.capacity+1).fill(null);paths[0]=[];input.weights.forEach((w,i)=>{for(let s=input.capacity;s>=w;s--)if(paths[s]===null&&paths[s-w]!==null)paths[s]=[...paths[s-w],i];});add('部分和の親を復元','新しく到達した和にだけ品物IDを記録します。',['和','品物ID'],paths.map((x,w)=>[w,x?.join(',')??'未到達']));result=[best.value,...best.counts,...(paths[input.target]||['null'])];
    }else if(id==='optimal_solution_count'){
      items(input);const seq=array(input.sequence,20);seq.forEach(x=>integer(x,-100,100));const a=bytes(input.a),b=bytes(input.b);if(a.length>20||b.length>20)throw Error('個数の図解は長さ20まで');
      const length=[],ways=[];for(let i=0;i<seq.length;i++){length[i]=1;ways[i]=1;for(let j=0;j<i;j++)if(seq[j]<seq[i]){if(length[j]+1>length[i]){length[i]=length[j]+1;ways[i]=ways[j];}else if(length[j]+1===length[i])ways[i]+=ways[j];}add(`LIS: 添字 ${i} で終わる解`,'同じ値でも異なる添字列を別の解として数えます。',['添字','値','長さ','個数'],seq.slice(0,i+1).map((x,j)=>[j,x,length[j],ways[j]]));}
      const best=length.length?Math.max(...length):0,lc=best?ways.reduce((sum,w,i)=>sum+(length[i]===best?w:0),0):1;
      let table=Array.from({length:b.length+1},()=>({length:0,count:1}));for(const x of a){let diagonal=table[0];for(let j=0;j<b.length;j++){const top=table[j+1],left=table[j],match=x===b[j],len=Math.max(top.length,left.length,match?diagonal.length+1:0);let count=0;if(top.length===len)count+=top.count;if(left.length===len)count+=left.count;if(diagonal.length===len)count-=diagonal.count;if(match&&diagonal.length+1===len)count+=diagonal.count;table[j+1]={length:len,count};diagonal=top;}add('LCSの一致位置を数える','上と左の重複を対角で除き、新しい一致pairを加えます。',['bのprefix長','LCS長','対応列の個数'],table.map((x,j)=>[j,x.length,x.count]));}
      let dp=Array(input.capacity+1).fill(null),counts=Array(input.capacity+1).fill(0);dp[0]=0;counts[0]=1;input.items.forEach(([w,v])=>{for(let j=input.capacity;j>=w;j--)if(dp[j-w]!==null){const value=dp[j-w]+v,count=counts[j-w];if(dp[j]===null||dp[j]<value){dp[j]=value;counts[j]=count;}else if(dp[j]===value)counts[j]+=count;}});const value=Math.max(...dp.filter(x=>x!==null)),count=counts.reduce((s,c,w)=>s+(dp[w]===value?c:0),0);add('Knapsackの同点解','重さ0・価値0の品物も採用/不採用を区別します。',['重さ','価値','個数'],dp.map((x,w)=>[w,x??'未到達',counts[w]]));result=[best,lc,table[b.length].length,table[b.length].count,value,count];
    }else if(id==='monge_matrix_product'){
      const a=array(input.a,10),b=array(input.b,10),k=b.length,m=k?array(b[0],10).length:0;for(const row of a){if(array(row,10).length!==k)throw Error('Aの列数とBの行数が不一致');row.forEach(x=>integer(x,-100,100));}for(const row of b){if(array(row,10).length!==m)throw Error('Bは長方形行列');row.forEach(x=>integer(x,-100,100));}for(let i=1;i<k;i++)for(let j=1;j<m;j++)if(b[i-1][j-1]+b[i][j]>b[i-1][j]+b[i][j-1])throw Error('BがMonge条件を満たしません');result=[];
      for(let i=0;i<a.length;i++){const row=[];for(let j=0;j<m;j++){let best=Infinity,arg=-1;const candidates=[];for(let q=0;q<k;q++){const value=a[i][q]+b[q][j];candidates.push([q,a[i][q],b[q][j],value]);if(value<best){best=value;arg=q;}}row.push(best);add(`C[${i}][${j}] = ${best}`,`最左の最小位置 k=${arg}。C++はSMAWKで評価を省略します。`,['k','A[i][k]','B[k][j]','合計'],candidates,[{label:'argmin',value:arg}]);}result.push(...row);}
    }else if(id==='weighted_slope_trick'){
      const ops=array(input.operations,30);integer(input.x,-50,50);let left=new Map(),right=new Map(),minimum=0,sl=0,sr=0;
      const insert=(map,x,w)=>{if(w)map.set(x,(map.get(x)||0)+w);},remove=(map,x,w)=>{const next=map.get(x)-w;if(next)map.set(x,next);else map.delete(x);};
      const hinge=(a,weight,isLeft)=>{integer(a,-100,100);integer(weight,0,100);const own=isLeft?left:right,other=isLeft?right:left,os=isLeft?sl:sr,ts=isLeft?sr:sl;while(weight&&other.size){const key=isLeft?Math.min(...other.keys()):Math.max(...other.keys()),x=key+ts;if(isLeft?x>=a:x<=a)break;const moved=Math.min(weight,other.get(key));minimum+=Math.abs(a-x)*moved;weight-=moved;remove(other,key,moved);insert(other,a-ts,moved);insert(own,x-os,moved);}insert(own,a-os,weight);};
      const evaluate=x=>minimum+[...left].reduce((s,[a,w])=>s+w*Math.max(a+sl-x,0),0)+[...right].reduce((s,[a,w])=>s+w*Math.max(x-a-sr,0),0);
      for(const op of ops){array(op,3);if(['abs','left','right'].includes(op[0])){if(op.length!==3)throw Error('hingeは[種類,座標,重み]');if(op[0]!=='right')hinge(op[1],op[2],true);if(op[0]!=='left')hinge(op[1],op[2],false);}else if(op[0]==='constant'){integer(op[1],-100,100);minimum+=op[1];}else if(op[0]==='prefix')right.clear();else if(op[0]==='suffix')left.clear();else if(op[0]==='shift'){integer(op[1],-20,20);integer(op[2],op[1],20);sl+=op[1];sr+=op[2];}else if(op[0]==='reflect'){[left,right]=[new Map([...right].map(([x,w])=>[-x,w])),new Map([...left].map(([x,w])=>[-x,w]))];[sl,sr]=[-sr,-sl];}else throw Error('未知のSlope操作');const plot=Array.from({length:41},(_,i)=>[i-20,evaluate(i-20)]);add(`操作: ${op.join(' ')}`,'同じbreakpointの重みをまとめて管理します。',['側','座標','重み'],[...[...left].map(([x,w])=>['左',x+sl,w]),...[...right].map(([x,w])=>['右',x+sr,w])],[{label:'最小値',value:minimum}],null,plot);}
      result=[minimum,left.size?Math.max(...left.keys())+sl:'-inf',right.size?Math.min(...right.keys())+sr:'inf',evaluate(input.x)];
    }else if(id==='dynamic_finite_state_dp'){
      const k=integer(input.states,1,5),matrices=copy(array(input.matrices,12)),updates=array(input.updates,20);integer(input.source,0,k-1);integer(input.target,0,k-1);
      const validate=m=>{if(array(m,k).length!==k)throw Error('行列の行数');m.forEach(row=>{if(array(row,k).length!==k)throw Error('行列の列数');row.forEach(x=>{if(x!==null)integer(x,-100,100);});});};matrices.forEach(validate);
      const solve=()=>{let dp=Array(k).fill(null),paths=Array(k).fill(null);dp[input.source]=0;paths[input.source]=[input.source];for(let p=0;p<matrices.length;p++){const next=Array(k).fill(null),np=Array(k).fill(null);for(let i=0;i<k;i++)if(dp[i]!==null)for(let j=0;j<k;j++)if(matrices[p][i][j]!==null){const cost=dp[i]+matrices[p][i][j];if(next[j]===null||cost<next[j]){next[j]=cost;np[j]=[...paths[i],j];}}dp=next;paths=np;add(`位置 ${p} の遷移を合成`,'行が遷移前、列が遷移後。nullは遷移できません。',['前状態',...Array.from({length:k},(_,i)=>`→${i}`)],matrices[p].map((row,i)=>[i,...row.map(x=>x??'不可')]),dp.map((x,i)=>({label:`状態${i}`,value:x??'未到達'})));}return {cost:dp[input.target]??'null',path:paths[input.target]||[]};};
      const first=solve();let last=first;for(const [p,m]of updates){integer(p,0,matrices.length-1);validate(m);matrices[p]=copy(m);add(`遷移 ${p} を更新`,'C++ではsegment treeの更新経路だけを再計算します。');last=solve();}result=[first.cost,last.cost,...last.path];
    }else if(id==='small_weight_knapsack'){
      items(input);const groups=new Map();let free=0;input.items.forEach(([w,v])=>{if(!w)free+=Math.max(0,v);else{if(!groups.has(w))groups.set(w,[]);groups.get(w).push(v);}});let dp=Array(input.capacity+1).fill(null);dp[0]=free;
      for(const [w,values]of [...groups].sort((a,b)=>a[0]-b[0])){values.sort((a,b)=>b-a);const prefix=[0];for(const x of values)prefix.push(prefix.at(-1)+x);const next=[...dp];for(let weight=0;weight<=input.capacity;weight++)for(let count=1;count<prefix.length&&count*w<=weight;count++)if(dp[weight-count*w]!==null){const value=dp[weight-count*w]+prefix[count];if(next[weight]===null||value>next[weight])next[weight]=value;}dp=next;add(`重さ ${w} のグループ`,'上位個数ごとの価値prefixは凹。C++は単調argmaxを使って候補を減らします。',['個数','価値prefix'],prefix.map((x,j)=>[j,x]),dp.map((x,w)=>({label:`重さ${w}`,value:x??'未到達'})));}result=[Math.max(...dp.filter(x=>x!==null))];
    }
    result=result.map(String);add('計算完了','サンプルはC++の使用例と照合済みです。変更入力はこの図解モデルで計算しています。',['出力位置','値'],result.map((x,i)=>[i,x]));return {steps,result};
  }
  const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.DPExtensionsModel=api;
})(typeof window==='undefined'?globalThis:window);
