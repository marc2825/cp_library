(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.GraphCombinatoricsModel=api;})(globalThis,function(){
  'use strict';
  const clone=x=>JSON.parse(JSON.stringify(x)),vertices=(mask,n)=>Array.from({length:n},(_,v)=>v).filter(v=>mask>>v&1),first=mask=>31-Math.clz32(mask&-mask),count=mask=>{let n=0;for(;mask;mask&=mask-1)++n;return n;};
  const validInt=(v,min,max,label)=>{if(!Number.isSafeInteger(v)||v<min||v>max)throw new Error(`${label}の範囲が不正です。`);};
  function trace(mode,input){
    if(!['triangles','cliques','maximum','independent','coloring','trees'].includes(mode))throw new Error('未知のモードです。');
    const s=clone(input),n=s.n;validInt(n,0,8,'頂点数');const seen=new Set(),adj=Array(n).fill(0),degree=Array(n).fill(0);
    s.edges.forEach(e=>{const[u,v]=e;validInt(u,0,n-1,'頂点');validInt(v,0,n-1,'頂点');if(mode!=='trees'){const key=[Math.min(u,v),Math.max(u,v)].join(',');if(u===v||seen.has(key))throw new Error('単純グラフ用です。自己ループ・重複辺は使えません。');seen.add(key);}else validInt(e[2]??1,-99,99,'重み');adj[u]|=1<<v;adj[v]|=1<<u;++degree[u];++degree[v];});
    if(s.stopAfter!=null)validInt(s.stopAfter,1,1000,'列挙上限');
    Object.assign(s,{mode,phase:'initial',current:[],candidates:[],excluded:[],best:[],solutions:[],colors:Array(n).fill(-1),greedyColor:Array(n).fill(-1),result:0,pruned:0,proof:null,activeEdge:-1,mask:0,order:[],bounds:[],stopped:false,groups:[],dp:[],choice:[],independent:[],matrix:[],laplacian:[],minorVertices:[],pivot:-1,row:-1,factor:0,determinant:1});
    const frames=[],frame=(title,detail,extra={})=>{Object.assign(s,extra,{title,detail});frames.push(clone(s));},all=(1<<n)-1;
    frame('初期状態','入力全体から計算履歴を作り直します。頂点は 0 始まりです。');
    const emit=group=>{s.solutions.push(group.slice().sort((a,b)=>a-b));s.result=s.solutions.length;frame('一つ見つかった',`{${group.join(', ')}} を記録します。`,{current:group.slice(),phase:'found'});if(s.stopAfter!=null&&s.solutions.length>=s.stopAfter){s.stopped=true;return false;}return true;};
    if(mode==='triangles'){
      s.degree=degree;s.forward=Array.from({length:n},()=>[]);s.oriented=s.edges.map(([u,v])=>{if(degree[u]>degree[v]||(degree[u]===degree[v]&&u>v))[u,v]=[v,u];s.forward[u].push(v);return[u,v];});s.marks=Array(n).fill(false);
      frame('次数・頂点番号で辺を向ける','(degree, vertex) が小さい方から大きい方へ向けます。各三角形に一つだけ最小の頂点ができます。',{phase:'orient'});
      let completed=true;
      outer:for(let u=0;u<n;++u){s.marks.fill(false);for(const v of s.forward[u])s.marks[v]=true;frame(`${u} の先をマーク`,`forward[${u}] = {${s.forward[u].join(', ')}}。`,{phase:'mark',current:[u],candidates:s.forward[u].slice()});
        for(const v of s.forward[u])for(const w of s.forward[v]){frame(`${u}→${v}→${w} を調べる`,s.marks[w]?'u→w もあり、三角形です。':'u→w がなく、三角形ではありません。',{phase:'check',current:[u,v,w]});if(s.marks[w]&&!emit([u,v,w])){completed=false;break outer;}}
      }
      frame(completed?'全三角形の列挙が完了':'コールバックで打切り',`${s.solutions.length} 個を取得しました。`,{phase:'done',current:[],candidates:[]});
    }else if(mode==='cliques'){
      if(!s.maximal){
        const forward=Array.from({length:n},(_,u)=>vertices(adj[u],n).filter(v=>v>u));
        function search(current,candidates){for(let i=0;i<candidates.length;++i){const v=candidates[i],next=current.concat(v),remaining=candidates.slice(i+1).filter(w=>adj[v]>>w&1);frame(`${v} を加える`,`残りの候補を ${v} の隣接先と交差させます。`,{phase:'search',current:next,candidates:remaining});if(!emit(next)||!search(next,remaining))return false;}return true;}
        let completed=!s.includeEmpty||emit([]);
        for(let v=0;v<n&&completed;++v){frame(`最小頂点 ${v} から開始`,'頂点を昇順で加えるので、同じクリークを重ねて列挙しません。',{phase:'search',current:[v],candidates:forward[v].slice()});completed=emit([v])&&search([v],forward[v]);}
      }else{
        function search(current,p,x){s.current=current.slice();s.candidates=vertices(p,n);s.excluded=vertices(x,n);frame('P と X を更新','P は追加できる候補、X は既に別の枝で処理した頂点です。',{phase:'search'});
          if(!(p|x))return current.length||s.includeEmpty?emit(current):true;
          let pivot=-1,score=-1;for(let pool=p|x;pool;pool&=pool-1){const v=first(pool),k=count(p&adj[v]);if(k>score){score=k;pivot=v;}}
          frame(`pivot = ${pivot}`,'P のうち pivot と隣接しない頂点だけを分岐します。',{phase:'pivot',pivot});
          for(let branches=p&~adj[pivot];branches;branches&=branches-1){const v=first(branches);if(!search(current.concat(v),p&adj[v],x&adj[v]))return false;p&=~(1<<v);x|=1<<v;}return true;
        }
        search([],all,0);
      }
      frame(s.stopped?'列挙上限で打切り':'クリーク列挙が完了',`${s.maximal?'極大なもの':'全てのクリーク'}を ${s.solutions.length} 個取得しました。最大とは、頂点数が最大のものを指します。`,{phase:'done',current:[],candidates:[],excluded:[]});
    }else if(mode==='maximum'||mode==='independent'){
      const graph=mode==='independent'?adj.map((mask,v)=>all&~mask&~(1<<v)):adj;s.searchAdj=graph;
      frame(mode==='independent'?'補グラフで最大クリークを探す':'貪欲彩色を上界として使う',mode==='independent'?'元グラフで辺がない二頂点を補グラフで結びます。頂点番号はそのままです。':'候補のグラフを c 色で塗れたら、その候補のクリークは高々 c 頂点です。',{phase:'prepare'});
      function search(current,p){const order=[],bounds=[],colors=Array(n).fill(-1);let color=0;
        for(let remaining=p;remaining;){++color;for(let available=remaining;available;){const v=first(available);remaining&=~(1<<v);available&=~(1<<v);available&=~graph[v];order.push(v);bounds.push(color);colors[v]=color-1;}}
        frame('候補を貪欲彩色する','同じ色の頂点は互いに隣接しません。色番号順に並べ、後ろから分岐します。',{phase:'bound',current:current.slice(),candidates:vertices(p,n),greedyColor:colors,order:order.slice(),bounds:bounds.slice(),proof:null});
        for(let i=order.length-1;i>=0;--i){if(current.length+bounds[i]<=s.best.length){++s.pruned;frame('上界で枝を刈る',`${current.length} + ${bounds[i]} ≤ ${s.best.length}。この枝では最良解を更新できません。`,{phase:'prune',current:current.slice(),candidates:vertices(p,n),greedyColor:colors,order:order.slice(),bounds:bounds.slice(),proof:{size:current.length,bound:bounds[i],bestSize:s.best.length}});return;}
          const v=order[i],next=current.concat(v),candidates=p&graph[v];frame(`${v} を選ぶ`,'現在の集合と全て隣接する候補だけに絞ります。',{phase:'search',current:next,candidates:vertices(candidates,n),greedyColor:colors,order:order.slice(),bounds:bounds.slice(),proof:null});
          if(candidates)search(next,candidates);else if(next.length>s.best.length){s.best=next.slice().sort((a,b)=>a-b);s.result=s.best.length;frame('最良解を更新',`${s.best.length} 頂点になりました。`,{phase:'best',current:next,candidates:[]});}p&=~(1<<v);
        }
      }
      search([],all);frame('最大解が確定',`${s.best.length} 頂点。残りの枝も上界で除外、または探索済みです。`,{phase:'done',current:s.best.slice(),candidates:[],proof:null});
    }else if(mode==='coloring'){
      const states=1<<n;s.dp=Array(states).fill(null);s.choice=Array(states).fill(0);s.independent=Array(states).fill(false);s.dp[0]=0;s.independent[0]=true;
      frame('部分集合の DP を用意','dp[S] は S を塗るための最小色数。最小頂点を含む独立集合 I を一色として取り除きます。',{phase:'prepare'});
      for(let mask=1;mask<states;++mask){const v=first(mask),bit=1<<v,rest=mask^bit;s.mask=mask;s.current=vertices(mask,n);s.independent[mask]=s.independent[rest]&&!(adj[v]&rest);
        if(s.independent[mask]){s.dp[mask]=1;s.choice[mask]=mask;frame(`${mask}: 一色で塗れる`,`{${s.current.join(', ')}} は独立集合です。`,{phase:'dp'});continue;}
        s.dp[mask]=n+1;const allowed=rest&~adj[v];for(let sub=allowed;;sub=(sub-1)&allowed){const group=sub|bit;if(s.independent[sub]&&s.dp[mask^group]+1<s.dp[mask]){s.dp[mask]=s.dp[mask^group]+1;s.choice[mask]=group;frame(`${mask}: dp を ${s.dp[mask]} へ`,`I={${vertices(group,n).join(', ')}} を一色にして、残り ${mask^group} の解へ足します。`,{phase:'dp',candidates:vertices(group,n)});}if(s.dp[mask]===2||!sub)break;}
        frame(`${mask} の最小色数が確定`,'独立でない集合は少なくとも二色必要です。二色が見つかれば、この状態の探索を止めます。',{phase:'dp',candidates:vertices(s.choice[mask],n)});
      }
      s.result=s.dp[states-1];let mask=states-1;while(mask){const group=vertices(s.choice[mask],n),color=s.groups.length;s.groups.push(group);for(const v of group)s.colors[v]=color;frame(`色 ${color} を復元`,`{${group.join(', ')}} を同じ色へ。`,{phase:'restore',current:group,mask});mask^=s.choice[mask];}
      frame('最小の彩色が確定',`${s.result} 色です。確率的な判定や整数のオーバーフローには依存しません。`,{phase:'done',current:[],candidates:[],mask:states-1});
    }else{
      const mod=s.mod??998244353;validInt(mod,2,2147483647,'素数 mod');for(let d=2;d<=mod/d;++d)if(mod%d===0)throw new Error('mod は素数にしてください。');const p=BigInt(mod),norm=x=>Number((BigInt(x)%p+p)%p),multiply=(a,b)=>Number(BigInt(a)*BigInt(b)%p);
      const power=(base,exponent)=>{let result=1;for(;exponent;exponent=Math.floor(exponent/2),base=multiply(base,base))if(exponent%2)result=multiply(result,base);return result;};s.mod=mod;
      const root=s.root??(n-1);if(n)validInt(root,0,n-1,'除く頂点');s.root=n?root:-1;s.laplacian=Array.from({length:n},()=>Array(n).fill(0));
      s.edges.forEach(([u,v,weight=1],id)=>{if(u===v){frame(`自己ループ #${id} を除外`,'全域木に自己ループは使えません。ラプラシアンにも加えません。',{phase:'laplacian',activeEdge:id});return;}const w=s.weighted?norm(weight):1,L=s.laplacian;L[u][u]=norm(L[u][u]+w);L[v][v]=norm(L[v][v]+w);L[u][v]=norm(L[u][v]-w);L[v][u]=norm(L[v][u]-w);frame(`元辺 #${id} を行列へ`,`対角へ +${w}、非対角へ −${w}（mod ${mod}）。多重辺は別々に寄与します。`,{phase:'laplacian',activeEdge:id});});
      s.minorVertices=Array.from({length:n},(_,v)=>v).filter(v=>v!==root);s.matrix=s.minorVertices.map(u=>s.minorVertices.map(v=>s.laplacian[u][v]));s.activeEdge=-1;
      frame(`頂点 ${n?root:'—'} の行・列を除く`,'残った余因子の行列式を、素数体上の消去法で求めます。',{phase:'minor'});
      if(!n)s.determinant=0;else for(let col=0;col<s.matrix.length;++col){let pivot=col;while(pivot<s.matrix.length&&!s.matrix[pivot][col])++pivot;s.pivot=col;
        if(pivot===s.matrix.length){s.determinant=0;frame('非零の pivot がない','行列式は 0（mod p）です。連結でも個数が p の倍数なら 0 になります。',{phase:'singular'});break;}
        if(pivot!==col){[s.matrix[pivot],s.matrix[col]]=[s.matrix[col],s.matrix[pivot]];s.determinant=norm(-s.determinant);frame(`行 ${pivot} と ${col} を交換`,'行交換で行列式の符号が反転します。',{phase:'swap'});}
        const value=s.matrix[col][col],inverse=power(value,mod-2);s.determinant=multiply(s.determinant,value);frame(`pivot ${col}: ${value}`,'非零 pivot の逆元を使って、下の行からこの列を消します。',{phase:'pivot',row:-1});
        for(let row=col+1;row<s.matrix.length;++row){const factor=multiply(s.matrix[row][col],inverse);for(let j=col;j<s.matrix.length;++j)s.matrix[row][j]=norm(BigInt(s.matrix[row][j])-BigInt(factor)*BigInt(s.matrix[col][j]));frame(`行 ${row} から ${factor} × 行 ${col} を引く`,'他行の倍を引いても行列式は変わりません。全て mod p で計算します。',{phase:'eliminate',row,factor});}
      }
      s.result=s.determinant;frame('行列木定理の答え',`${s.weighted?'全域木ごとの重みの積の和':'全域木の個数'} = ${s.result}（mod ${mod}）。${n===0?'空グラフは 0 と定義します。':n===1?'一頂点では空積・0 次の行列式が 1 です。':''}`,{phase:'done',row:-1});
    }
    return {frames,state:clone(s)};
  }
  return {trace};
});
