(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.AdvancedModels=api;})(typeof globalThis==='object'?globalThis:this,()=>{
  'use strict';
  const specs={
    beats:['Segment Tree Beats',{a:[8,3,7,1,9,4],ops:[['chmin',0,6,6],['add',1,5,2],['chmax',2,6,5]]},'第2最大・第2最小で、まとめて更新できる範囲を判断します。'],
    wavelet:['Wavelet Matrix',{a:[5,1,4,1,3,8],l:1,r:6,k:2},'安定分割した各bit列へ、rankを使って質問区間を写します。'],
    rollback:['Rollback DSU',{n:6,ops:[['merge',0,1],['save'],['merge',1,2],['merge',3,4],['undo'],['restore']]},'経路圧縮をせず、併合で変わった親とサイズを巻き戻します。'],
    connectivity:['Offline Dynamic Connectivity',{n:5,ops:[['add',0,1],['add',1,2],['query',0,2],['remove',1,2],['query',0,2],['add',2,3],['query',2,3]]},'有効期間を時間区間木に分配し、DFSの枝を出るときにDSUを戻します。'],
    mo:['Mo with Updates',{a:[1,2,1,3,2],ops:[['query',0,5],['set',2,4],['query',1,4],['set',1,4],['query',0,3]]},'L・R・更新時刻Tを動かし、区間に入る値だけを集計します。'],
    persistent:['Persistent Segment Tree',{a:[0,0,0,0],ops:[[0,1,5],[1,2,7],[1,1,9],[2,0,4]]},'更新経路だけをコピーし、過去の版と残りのノードを共有します。'],
    'persistent-kth':['Persistent Order Statistics',{a:[5,1,4,1,3],l:1,r:5,k:2},'prefix rとprefix lの頻度差が区間の値の頻度になります。'],
    treap:['Implicit Treap',{a:[1,2,3,4,5],ops:[['reverse',1,5],['insert',2,9],['erase',0],['move',0,2,3]]},'部分木サイズを添字にし、split・mergeで列を切り貼りします。'],
    virtual:['Virtual Tree',{parent:[-1,0,0,1,1,2,5],important:[3,4,6]},'DFS順で隣接する重要頂点のLCAを加え、近い祖先へつなぎます。'],
    sack:['DSU on Tree',{parent:[-1,0,0,1,1,2,5],colors:[1,2,1,3,2,4,1]},'軽い子を片付け、重い子の集計を保持してから軽い子を再追加します。'],
    lct:['Link-Cut Tree',{n:5,values:[1,2,3,4,5],ops:[['link',0,1],['link',1,2],['link',1,3],['path',2,3],['cut',1,2],['link',2,4],['path',0,4]]},'パスを露出して集約します。この図は表現する森と論理パスを表示します。'],
    lichao:['Li Chao Tree',{xs:[-3,-1,0,2,4],lines:[[2,1],[-1,5],[0,0]],x:2},'中央で勝つ直線を残し、負けた直線を交点のある片側へ送ります。'],
    monotone:['Monotone Minima',{rows:5,cols:7,offsets:[2,1,0,1,2,3,4]},'中央行の最小位置で、上下の行が探索する列の範囲を絞ります。'],
    smawk:['SMAWK',{rows:5,cols:7,offsets:[2,1,0,1,2,3,4]},'候補列を減らし、奇数行を再帰し、偶数行を隣の解の間で調べます。'],
    minplus:['凸 Min-Plus Convolution',{a:[3,0,4,-1],b:[0,1,4,9]},'bの差分が単調なら、各出力位置の最小候補の添字も単調になります。'],
    primality:['64bit Miller–Rabin',{n:'341'},'n−1=2^s dとし、7基底の剰余二乗列を確かめます。'],
    rho:['64bit Pollard’s Rho',{n:'8051'},'x²+cの列を法nで動かし、差とnのGCDから因数を発見します。'],
    subset:['Subset Convolution',{a:[1,2,3,4,0,1,2,1],b:[5,6,7,8,1,0,1,0]},'rankごとにゼータ変換し、rankを畳み込み、メビウスで復元します。'],
    crt:['任意mod畳み込み',{a:[-1,2,3],b:[3,4],mod:1000000007},'3つのNTT素数の剰余をCRTの桁にし、目的の法で復元します。'],
    aho:['Aho–Corasick',{text:'shehe',patterns:['he','she','hers','']},'Trieのfailure linkで接尾辞へ戻り、出力linkで一致を報告します。'],
    eertree:['Eertree（回文木）',{text:'ababa'},'2つの根を使い、新文字で挟める最長回文suffixを探します。']
  };
  const copy=x=>JSON.parse(JSON.stringify(x,(_,v)=>typeof v==='bigint'?v.toString():v));
  function integer(x,lo,hi){if(!Number.isSafeInteger(x)||x<lo||x>hi)throw Error(`${lo}〜${hi}の整数が必要です`);return x;}
  function array(a,max=16){if(!Array.isArray(a)||a.length>max)throw Error(`配列は${max}要素までです`);return a.map(x=>integer(x,-1000000000,1000000000));}
  function ops(a){if(!Array.isArray(a)||a.length>40||a.some(x=>!Array.isArray(x)))throw Error('操作列は40個までです');return a;}
  function range(n,l,r){integer(l,0,n);integer(r,l,n);}
  function vertex(n,v){return integer(v,0,n-1);}
  function pow(a,b,m){let r=1n;for(;b;b>>=1n,a=a*a%m)if(b&1n)r=r*a%m;return r;}
  function gcd(a,b){while(b)[a,b]=[b,a%b];return a;}
  function uint(s){if(typeof s!=='string'||!/^\d+$/.test(s))throw Error('nは10進文字列で指定します');let n=BigInt(s);if(n<1n||n>18446744073709551615n)throw Error('nは1..UINT64_MAXです');return n;}
  function primality(n,emit=()=>{}){
    if(n<2n)return false;
    if(n%2n===0n)return n===2n;
    let d=n-1n,s=0;while(!(d&1n)){d>>=1n;s++;}
    for(const base of [2n,325n,9375n,28178n,450775n,9780504n,1795265022n]){
      let a=base%n;if(!a)continue;let x=pow(a,d,n),values=[x.toString()],ok=x===1n||x===n-1n;
      for(let r=1;!ok&&r<s;r++){x=x*x%n;values.push(x.toString());if(x===n-1n)ok=true;}
      emit(base,d,s,values,ok);if(!ok)return false;
    }return true;
  }
  function dsu(n){const parent=Array.from({length:n},(_,i)=>i),size=Array(n).fill(1),history=[];const leader=v=>{while(parent[v]!==v)v=parent[v];return v;};return{parent,size,history,leader,merge(a,b){a=leader(a);b=leader(b);if(a===b){history.push(null);return;}if(size[a]<size[b])[a,b]=[b,a];history.push([a,b]);parent[b]=a;size[a]+=size[b];},back(t){while(history.length>t){const e=history.pop();if(e){parent[e[1]]=e[1];size[e[0]]-=size[e[1]];}}}};}
  function forest(parent){array(parent);const n=parent.length;parent.forEach((p,v)=>{if(p!==-1)integer(p,0,v-1);});const children=Array.from({length:n},()=>[]),depth=Array(n).fill(0),order=[];parent.forEach((p,v)=>{if(p>=0){children[p].push(v);depth[v]=depth[p]+1;}});const dfs=v=>{order.push(v);children[v].forEach(dfs);};parent.forEach((p,v)=>{if(p===-1)dfs(v);});const ancestor=(u,v)=>{while(v!==-1){if(u===v)return true;v=parent[v];}return false;};const lca=(u,v)=>{while(depth[u]>depth[v])u=parent[u];while(depth[v]>depth[u])v=parent[v];while(u!==v&&u!==-1&&v!==-1){u=parent[u];v=parent[v];}return u===v?u:-1;};return{n,children,depth,order,ancestor,lca};}
  function path(adj,u,v){const p=Array(adj.length).fill(-1),q=[u];p[u]=u;for(const x of q)for(const y of adj[x])if(p[y]<0){p[y]=x;q.push(y);}if(p[v]<0)return[];const a=[];for(let x=v;;x=p[x]){a.push(x);if(x===u)break;}return a.reverse();}
  function build(mode,input){
    if(!specs[mode])throw Error('不明なモード');const p=copy(input),frames=[];
    const emit=(title,note,columns,rows,extra={})=>frames.push(copy({title,note,columns,rows,...extra}));
    let result=null;
    if(mode==='beats'){
      const a=array(p.a);emit('初期配列','区間は0始まりの半開区間です。',['i','値'],a.map((x,i)=>[i,x]),{values:a});
      for(const [kind,l,r,x] of ops(p.ops)){range(a.length,l,r);integer(x,-1000,1000);if(!['add','chmin','chmax'].includes(kind))throw Error('chmin/chmax/addを指定');
        const visit=(lo,hi)=>{if(r<=lo||hi<=l)return;const vals=a.slice(lo,hi),sorted=[...new Set(vals)].sort((x,y)=>x-y),max=sorted.at(-1),second=sorted.at(-2),min=sorted[0],next=sorted[1];
          if(kind==='chmin'&&max<=x||kind==='chmax'&&min>=x)return;
          const fast=l<=lo&&hi<=r&&(kind==='add'||kind==='chmin'&&(second===undefined||second<x)||kind==='chmax'&&(next===undefined||x<next));
          emit(`[${lo},${hi}) ${fast?'まとめて更新':'子へ降りる'}`,`${kind}(${l},${r},${x})。第2極値を越えると別の値も変わるので分割します。`,['最小','第2最小','最大','第2最大','最大個数'],[[min,next??'∞',max,second??'−∞',vals.filter(v=>v===max).length]],{values:a,active:[lo,hi]});
          if(fast){for(let i=lo;i<hi;i++)a[i]=kind==='add'?a[i]+x:kind==='chmin'?Math.min(a[i],x):Math.max(a[i],x);}else{const m=(lo+hi)>>1;visit(lo,m);visit(m,hi);}
        };if(a.length)visit(0,a.length);emit('操作完了','図では遅延値を配列へ展開して表示します。',['i','値'],a.map((x,i)=>[i,x]),{values:a});
      }result=a;emit('配列の集約',`sum=${a.reduce((x,y)=>x+y,0)}`,['i','値'],a.map((x,i)=>[i,x]),{values:a});
    }else if(mode==='wavelet'||mode==='persistent-kth'){
      const a=array(p.a);range(a.length,p.l,p.r);integer(p.k,-1,a.length);const sorted=[...new Set(a)].sort((x,y)=>x-y);let cur=a.map(x=>sorted.indexOf(x)),l=p.l,r=p.r,k=p.k,index=0;const levels=Math.ceil(Math.log2(Math.max(1,sorted.length)));
      if(mode==='persistent-kth'){
        const count=Array(sorted.length).fill(0),prefix=[count.slice()];for(const x of a){count[sorted.indexOf(x)]++;prefix.push(count.slice());}
        emit('prefixごとの頻度','左端と右端の版の差を使います。',['prefix',...sorted],prefix.map((x,i)=>[i,...x]));
        emit('2つの版の差',`prefix ${r} − prefix ${l}`,['値','右版','左版','差'],sorted.map((x,i)=>[x,prefix[r][i],prefix[l][i],prefix[r][i]-prefix[l][i]]));
      }
      const valid=k>=0&&k<r-l;
      for(let bit=levels-1;bit>=0;bit--){const ones=i=>cur.slice(0,i).filter(x=>x>>bit&1).length,z=cur.filter(x=>!(x>>bit&1)).length,ol=ones(l),or=ones(r),count=(r-l)-(or-ol),right=k>=count;
        emit(`bit ${bit} の安定分割`,valid?`0側は${count}個、k=${k}なので${right?'1':'0'}側へ。`:'kは区間外です。',['位置','圧縮値','bit'],cur.map((x,i)=>[i,x,x>>bit&1]),{values:cur.map(x=>sorted[x]),active:[l,r]});
        cur=cur.filter(x=>!(x>>bit&1)).concat(cur.filter(x=>x>>bit&1));if(valid){index=index*2+Number(right);if(right){k-=count;l=z+ol;r=z+or;}else{l-=ol;r-=or;}}
      }result=valid?sorted[index]:null;emit('k番目の値',result===null?'範囲外なのでnullopt。':String(result),['値'],[[result??'なし']]);
    }else if(mode==='rollback'){
      const n=integer(p.n,0,16),d=dsu(n);let saved=0;
      for(const [kind,u,v]of ops(p.ops)){if(kind==='merge'){vertex(n,u);vertex(n,v);d.merge(u,v);}else if(kind==='save')saved=d.history.length;else if(kind==='undo'){if(!d.history.length)throw Error('空履歴はundo不可');d.back(d.history.length-1);}else if(kind==='restore'){if(saved>d.history.length)throw Error('破棄したsnapshot');d.back(saved);}else throw Error('不明な操作');
        emit(kind,`履歴長=${d.history.length}、snapshot=${saved}`,['頂点','親','根','根のサイズ'],d.parent.map((x,i)=>[i,x,d.leader(i),d.size[d.leader(i)]]),{edges:d.parent.flatMap((x,i)=>x===i?[]:[[i,x]])});
      }result=d.parent.map((_,i)=>d.leader(i));
    }else if(mode==='connectivity'){
      const n=integer(p.n,0,16),operations=ops(p.ops),open=new Map(),intervals=[],answers=[],q=operations.length;const seg=Array.from({length:Math.max(1,q*4)},()=>[]);
      operations.forEach(([kind,u,v],t)=>{vertex(n,u);vertex(n,v);const e=[Math.min(u,v),Math.max(u,v)],key=e.join(',');if(kind==='add'){let s=open.get(key)||{start:t,count:0,e};s.count++;open.set(key,s);}else if(kind==='remove'){const s=open.get(key);if(!s)throw Error('存在しない辺の削除');if(!--s.count){intervals.push([s.start,t,e]);open.delete(key);}}else if(kind!=='query')throw Error('不明な操作');});
      for(const s of open.values())intervals.push([s.start,q,s.e]);emit('辺の有効期間','重複辺は個数が0になるまで有効です。',['辺','開始','終了'],intervals.map(([l,r,e])=>[e.join('—'),l,r]));
      const insert=(k,l,r,a,b,e)=>{if(b<=l||r<=a)return;if(a<=l&&r<=b){seg[k].push(e);return;}let m=(l+r)>>1;insert(k*2,l,m,a,b,e);insert(k*2+1,m,r,a,b,e);};if(q)for(const [l,r,e]of intervals)insert(1,0,q,l,r,e);
      const d=dsu(n),dfs=(k,l,r)=>{const saved=d.history.length;for(const [u,v]of seg[k])d.merge(u,v);emit(`時間区間 [${l},${r}) に入る`,'区間に共通する辺を併合します。',['頂点','親'],d.parent.map((x,i)=>[i,x]),{edges:seg[k]});if(r-l===1){const [kind,u,v]=operations[l];if(kind==='query'){const answer=d.leader(u)===d.leader(v);answers.push(answer);emit(`時刻${l}の質問`,`${u}と${v}: ${answer}`,['質問','結果'],[[`${u}—${v}`,answer]]);}}else{let m=(l+r)>>1;dfs(k*2,l,m);dfs(k*2+1,m,r);}d.back(saved);emit(`[${l},${r})から戻る`,'この枝の併合を巻き戻します。',['頂点','親'],d.parent.map((x,i)=>[i,x]));};if(q)dfs(1,0,q);result=answers;
    }else if(mode==='mo'){
      const a=array(p.a),working=a.slice(),updates=[],queries=[];for(const [kind,x,y]of ops(p.ops)){if(kind==='set'){vertex(a.length,x);integer(y,-1000,1000);updates.push({p:x,before:working[x],after:y});working[x]=y;}else if(kind==='query'){range(a.length,x,y);queries.push({l:x,r:y,t:updates.length,id:queries.length});}else throw Error('set/queryを指定');}
      const block=Math.max(1,Math.floor(Math.pow(Math.max(1,a.length),2/3)));queries.sort((x,y)=>Math.floor(x.l/block)-Math.floor(y.l/block)||Math.floor(x.r/block)-Math.floor(y.r/block)||x.t-y.t);let l=0,r=0,t=0;const answer=Array(queries.length);
      const frame=title=>emit(title,`L=${l}, R=${r}, T=${t}、異なる値=${new Set(a.slice(l,r)).size}`,['i','値','区間内'],a.map((x,i)=>[i,x,l<=i&&i<r]),{values:a,active:[l,r]});
      for(const q of queries){while(t<q.t){const u=updates[t++];a[u.p]=u.after;frame('時刻を進める');}while(t>q.t){const u=updates[--t];a[u.p]=u.before;frame('更新を戻す');}while(l>q.l){l--;frame('左に追加');}while(r<q.r){r++;frame('右に追加');}while(l<q.l){l++;frame('左を削除');}while(r>q.r){r--;frame('右を削除');}answer[q.id]=new Set(a.slice(l,r)).size;frame(`質問ID ${q.id}を保存`);}result=answer;
    }else if(mode==='persistent'){
      const initial=array(p.a),nodes=[null],roots=[];const make=(l,r,a)=>{if(l===r)return 0;const id=nodes.length;nodes.push({l,r,left:0,right:0,sum:0});if(r-l===1)nodes[id].sum=a[l];else{const m=(l+r)>>1;nodes[id].left=make(l,m,a);nodes[id].right=make(m,r,a);nodes[id].sum=nodes[nodes[id].left].sum+nodes[nodes[id].right].sum;}return id;};roots.push(make(0,initial.length,initial));
      for(const [version,pos,value]of ops(p.ops)){integer(version,0,roots.length-1);vertex(initial.length,pos);integer(value,-1000,1000);const created=[];const set=id=>{const old=nodes[id],k=nodes.length;nodes.push({...old});created.push(k);if(old.r-old.l===1)nodes[k].sum=value;else{if(pos<(old.l+old.r>>1))nodes[k].left=set(old.left);else nodes[k].right=set(old.right);nodes[k].sum=nodes[nodes[k].left].sum+nodes[nodes[k].right].sum;}return k;};roots.push(set(roots[version]));emit(`版${roots.length-1}を版${version}から分岐`,`新規ノード: ${created.join(', ')}。他の枝は共有。`,['ID','区間','左ID','右ID','和'],nodes.slice(1).map((x,i)=>[i+1,`[${x.l},${x.r})`,x.left,x.right,x.sum]),{edges:nodes.slice(1).flatMap((x,i)=>[x.left,x.right].filter(Boolean).map(c=>[i+1,c]))});}
      result=roots.map(r=>r?nodes[r].sum:0);
    }else if(mode==='treap'){
      const a=array(p.a);for(const [kind,x,y,z]of ops(p.ops)){if(kind==='insert'){integer(x,0,a.length);integer(y,-1000,1000);a.splice(x,0,y);}else if(kind==='erase'){vertex(a.length,x);a.splice(x,1);}else if(kind==='reverse'){range(a.length,x,y);a.splice(x,y-x,...a.slice(x,y).reverse());}else if(kind==='move'){range(a.length,x,y);integer(z,0,a.length-(y-x));const mid=a.splice(x,y-x);emit('split: 対象区間を取り出す','優先度の木ではなく論理列の分割を表示します。',['残り','取り出した列'],[[a.join(', '),mid.join(', ')]]);a.splice(z,0,...mid);}else throw Error('不明な操作');emit(kind+' 完了','反転では左右と順／逆方向の積を交換します。',['添字','値'],a.map((x,i)=>[i,x]),{values:a});}result=a;
    }else if(mode==='virtual'||mode==='sack'){
      const parent=array(p.parent),f=forest(parent);
      if(mode==='virtual'){
        if(!Array.isArray(p.important)||p.important.length>32)throw Error('重要頂点は32個まで');let important=[...new Set(p.important.map(v=>vertex(f.n,v)))].sort((a,b)=>f.order.indexOf(a)-f.order.indexOf(b)),all=important.slice();
        for(let i=1;i<important.length;i++){const v=f.lca(important[i-1],important[i]);if(v>=0)all.push(v);emit('隣接重要頂点のLCA',`${important[i-1]}, ${important[i]} → ${v}`,['重要頂点'],important.map(x=>[x]));}
        all=[...new Set(all)].sort((a,b)=>f.order.indexOf(a)-f.order.indexOf(b));const stack=[],parents=[],edges=[];
        for(const v of all){while(stack.length&&!f.ancestor(stack.at(-1),v))stack.pop();const p=stack.at(-1)??-1;parents.push(p);if(p>=0)edges.push([p,v]);stack.push(v);emit(`頂点${v}をつなぐ`,'最も近い残した祖先を選びます。',['頂点','親','重要'],all.slice(0,parents.length).map((x,i)=>[x,parents[i],important.includes(x)]),{edges});}result={vertices:all,parent:parents};
      }else{
        const colors=array(p.colors);if(colors.length!==f.n)throw Error('色は頂点数と同じ長さ');const sizes=Array(f.n).fill(1),heavy=Array(f.n).fill(-1),bag=new Set(),answer=Array(f.n);
        for(const v of [...f.order].reverse())for(const c of f.children[v]){sizes[v]+=sizes[c];if(heavy[v]===-1||sizes[c]>sizes[heavy[v]])heavy[v]=c;}
        const change=(v,add)=>{for(const w of f.order)if(f.ancestor(v,w)){if(add)bag.add(w);else bag.delete(w);}};
        const frame=(title,v)=>emit(title,`処理頂点${v}、重い子=${heavy[v]}`,['頂点','色','集計内'],colors.map((x,i)=>[i,x,bag.has(i)]),{values:[...bag].map(v=>colors[v]),edges:parent.flatMap((p,v)=>p<0?[]:[[p,v]])});
        const dfs=(v,keep)=>{for(const c of f.children[v])if(c!==heavy[v])dfs(c,false);if(heavy[v]>=0)dfs(heavy[v],true);for(const c of f.children[v])if(c!==heavy[v]){change(c,true);frame('軽い子の区間を再追加',v);}bag.add(v);answer[v]=new Set([...bag].map(x=>colors[x])).size;frame(`answer[${v}]=${answer[v]}`,v);if(!keep){change(v,false);frame('部分木をクリア',v);}};parent.forEach((p,v)=>{if(p<0)dfs(v,false);});result=answer;
      }
    }else if(mode==='lct'){
      const n=integer(p.n,0,16),values=array(p.values);if(values.length!==n)throw Error('値は頂点数と同じ長さ');const adj=Array.from({length:n},()=>new Set()),answers=[];
      for(const [kind,u,v]of ops(p.ops)){vertex(n,u);vertex(n,v);let route=path(adj,u,v),note='';if(kind==='link'){if(route.length)note='閉路になるので拒否';else{adj[u].add(v);adj[v].add(u);note='異なる木を接続';}}else if(kind==='cut'){if(adj[u].has(v)){adj[u].delete(v);adj[v].delete(u);note='直接の辺を削除';}else note='直接の辺がないので拒否';}else if(kind==='path'){const sum=route.length?route.reduce((s,x)=>s+values[x],0):null;answers.push(sum);note=route.length?`露出パス: ${route.join(' → ')}、和=${sum}`:'別成分なのでnullopt';}else throw Error('link/cut/pathを指定');
        emit(`${kind}(${u},${v})`,note,['頂点','値','パス内'],values.map((x,i)=>[i,x,kind==='path'&&route.includes(i)]),{edges:adj.flatMap((s,u)=>[...s].filter(v=>u<v).map(v=>[u,v]))});}result=answers;
    }else if(mode==='lichao'){
      const xs=[...new Set(array(p.xs))].sort((a,b)=>a-b);if(!xs.length||!xs.includes(p.x))throw Error('質問xは空でないxsに登録');const lines=ops(p.lines);const tree=Array(Math.max(1,xs.length*4)).fill(null),evalLine=(line,x)=>line[0]*x+line[1];
      const insert=(k,l,r,line)=>{if(!tree[k]){tree[k]=line;emit(`ノード${k}へ直線を格納`,`座標区間 [${l},${r})`,['ノード','傾き','切片'],tree.flatMap((x,i)=>x?[[i,...x]]:[]));return;}const m=(l+r)>>1,left=evalLine(line,xs[l])<evalLine(tree[k],xs[l]),mid=evalLine(line,xs[m])<evalLine(tree[k],xs[m]);if(mid)[line,tree[k]]=[tree[k],line];emit(`中央x=${xs[m]}で比較`,'中央で良い方を残します。',['ノード','保持a','保持b'],tree.flatMap((x,i)=>x?[[i,...x]]:[]));if(r-l>1){if(left!==mid)insert(k*2,l,m,line);else insert(k*2+1,m,r,line);}};
      for(const line of lines){if(line.length!==2)throw Error('直線は[a,b]');line.forEach(x=>integer(x,-10000,10000));insert(1,0,xs.length,line);}let k=1,l=0,r=xs.length,best=null;while(true){if(tree[k]){const y=evalLine(tree[k],p.x);best=best===null?y:Math.min(best,y);}emit(`x=${p.x}の候補を評価`,`現在の最小値=${best??'なし'}`,['ノード','区間','候補値'],[[k,`[${l},${r})`,tree[k]?evalLine(tree[k],p.x):'なし']]);if(r-l===1)break;const m=(l+r)>>1;if(p.x<xs[m]){k*=2;r=m;}else{k=k*2+1;l=m;}}result=best;
    }else if(mode==='monotone'||mode==='smawk'){
      const n=integer(p.rows,0,16),m=integer(p.cols,0,16),off=array(p.offsets);if(off.length!==m)throw Error('offsetsは列数と同じ長さ');const get=(r,c)=>(r-c)**2+off[c],matrix=Array.from({length:n},(_,r)=>Array.from({length:m},(_,c)=>get(r,c))),answer=Array(n).fill(-1);
      emit('Monge行列','(row−col)²+offset[col]。列ごとの加算はMonge性を保ちます。',['行',...off.map((_,i)=>`列${i}`)],matrix.map((a,r)=>[r,...a]));
      if(m&&mode==='monotone'){const solve=(top,bottom,left,right)=>{if(top>=bottom)return;const r=(top+bottom)>>1;let best=left;for(let c=left+1;c<right;c++)if(get(r,c)<get(r,best))best=c;answer[r]=best;emit(`中央行${r}の最小列=${best}`,`探索列 [${left},${right})。上の行は${best}以下、下の行は${best}以上。`,['行','最小列'],answer.map((x,i)=>[i,x]));solve(top,r,left,best+1);solve(r+1,bottom,best,right);};solve(0,n,0,m);}
      if(m&&mode==='smawk'){const solve=(rs,cs)=>{if(!rs.length)return;const reduced=[];for(const c of cs){while(reduced.length&&get(rs[reduced.length-1],c)<get(rs[reduced.length-1],reduced.at(-1))){const old=reduced.pop();emit(`列${old}を候補から除外`,`列${c}が比較行で厳密に良いため。`,['行候補','列候補'],[[rs.join(', '),reduced.join(', ')]]);}if(reduced.length<rs.length)reduced.push(c);}emit('列のreduce完了','行数以下の候補列で奇数行を再帰します。',['行','候補列'],[[rs.join(', '),reduced.join(', ')]]);solve(rs.filter((_,i)=>i%2),reduced);let start=0;for(let i=0;i<rs.length;i+=2){let end=reduced.length-1;if(i+1<rs.length)end=reduced.indexOf(answer[rs[i+1]]);let best=reduced[start];for(let j=start+1;j<=end;j++)if(get(rs[i],reduced[j])<get(rs[i],best))best=reduced[j];answer[rs[i]]=best;start=end;emit(`行${rs[i]}を補間`,'隣接する奇数行の最小位置の間だけを比較。',['行','最小列'],answer.map((x,i)=>[i,x]));}};solve(Array.from({length:n},(_,i)=>i),Array.from({length:m},(_,i)=>i));}result=answer;
    }else if(mode==='minplus'){
      const a=array(p.a),b=array(p.b);for(let j=2;j<b.length;j++)if(b[j]-b[j-1]<b[j-1]-b[j-2])throw Error('bは差分が非減少の凸列です');const out=[],arg=[];
      if(a.length&&b.length){const solve=(top,bottom,left,right)=>{if(top>=bottom)return;const k=(top+bottom)>>1,lo=Math.max(left,k-b.length+1),hi=Math.min(right,k+1),rows=[];let best=lo,value=a[lo]+b[k-lo];for(let i=lo;i<hi;i++){const v=a[i]+b[k-i];rows.push([i,k-i,a[i],b[k-i],v]);if(v<value){value=v;best=i;}}out[k]=value;arg[k]=best;emit(`出力k=${k}、最小i=${best}`,'単調な最小位置を次の探索境界にします。',['i','j','a[i]','b[j]','和'],rows);solve(top,k,left,best+1);solve(k+1,bottom,best,right);};solve(0,a.length+b.length-1,0,a.length);}result=out;
    }else if(mode==='primality'){
      const n=uint(p.n);result=primality(n,(a,d,s,values,ok)=>emit(`基底 ${a}`,`n−1=2^${s}×${d}、${ok?'通過':'合成数の証拠'}`,['二乗段階','剰余'],values.map((x,i)=>[i,x])));emit('素数判定結果',result?'prime':'composite',['n','結果'],[[n.toString(),result]]);
    }else if(mode==='rho'){
      const n=uint(p.n);if(n>1000000000n)throw Error('図のRhoは10^9以下。C++はuint64全域。');if(n===1n)result=[];else if(primality(n))result=[n.toString()];else if(n%2n===0n)result=['2',(n/2n).toString()];else{let factor=0n;for(let c=1n;c<=16n&&!factor;c++){let x=2n,y=2n,d=1n;const step=z=>(z*z+c)%n;for(let i=0;i<300&&d===1n;i++){x=step(x);y=step(step(y));d=gcd(x>y?x-y:y-x,n);emit(`c=${c}、反復${i+1}`,'図はFloydの2速度。C++はBrentの倍増とGCDのバッチ。',['x','y','gcd(|x−y|,n)'],[[x.toString(),y.toString(),d.toString()]]);}if(d>1n&&d<n)factor=d;}result=factor?[factor.toString(),(n/factor).toString()]:null;}emit('分割結果','1回の非自明な分割を表示します。因数が素数とは限りません。',['因数'],result?result.map(x=>[x]):[['再試行が必要']]);
    }else if(mode==='subset'){
      const a=array(p.a),b=array(p.b);a.concat(b).forEach(x=>integer(x,-1000,1000));if(!a.length||a.length!==b.length||(a.length&(a.length-1)))throw Error('同じ正の2冪長（16以下）が必要');const n=a.length,k=Math.log2(n),rank=s=>s.toString(2).replaceAll('0','').length,x=Array.from({length:k+1},()=>Array(n).fill(0)),y=x.map(r=>r.slice());for(let s=0;s<n;s++){x[rank(s)][s]=a[s];y[rank(s)][s]=b[s];}
      for(let bit=1;bit<n;bit*=2){for(let d=0;d<=k;d++)for(let s=0;s<n;s++)if(s&bit){x[d][s]+=x[d][s^bit];y[d][s]+=y[d][s^bit];}emit(`rank別ゼータ bit=${bit}`,'同じrankで部分集合の寄与を集計。',['rank',...a.map((_,i)=>i)],x.map((r,i)=>[i,...r]));}
      const out=Array(n).fill(0);for(let d=0;d<=k;d++){const h=Array(n).fill(0);for(let s=0;s<n;s++)for(let i=0;i<=d;i++)h[s]+=x[i][s]*y[d-i][s];emit(`rank=${d}の畳み込み`,'ここでは重なる集合の寄与も含みます。',['mask','rank積'],h.map((v,s)=>[s,v]));for(let bit=n/2;bit>=1;bit/=2)for(let s=0;s<n;s++)if(s&bit)h[s]-=h[s^bit];for(let s=0;s<n;s++)if(rank(s)===d)out[s]=h[s];emit(`rank=${d}を復元`,'メビウス変換後に|S|=rankだけを採用。',['mask','復元後','採用'],h.map((v,s)=>[s,v,rank(s)===d]));}result=out;
    }else if(mode==='crt'){
      const a=array(p.a).map(BigInt),b=array(p.b).map(BigInt),m=BigInt(integer(p.mod,1,2000000000)),norm=x=>(x%m+m)%m,p0=167772161n,p1=469762049n,p2=1224736769n;const x=a.map(norm),y=b.map(norm),out=Array(a.length&&b.length?a.length+b.length-1:0).fill(0n);for(let i=0;i<x.length;i++)for(let j=0;j<y.length;j++)out[i+j]+=x[i]*y[j];
      result=out.map((v,k)=>{const r0=v%p0,r1=v%p1,r2=v%p2,t1=((r1-r0+p1)%p1)*pow(p0,p1-2n,p1)%p1,low=r0+p0*t1,t2=((r2-low%p2+p2)%p2)*pow(p0*p1%p2,p2-2n,p2)%p2,answer=(low%m+(p0*p1%m)*t2)%m;emit(`係数${k}のCRT`,'図は直接積で剰余を生成。C++は3法でNTTします。',['r0','r1','r2','t1','t2','modでの値'],[[r0,r1,r2,t1,t2,answer]]);return Number(answer);});
    }else if(mode==='aho'){
      if(typeof p.text!=='string'||p.text.length>32||!Array.isArray(p.patterns)||p.patterns.length>16||p.patterns.some(s=>typeof s!=='string'||s.length>16)||p.patterns.join('').length>128)throw Error('text32、pattern各16・16個・合計128文字まで');const nodes=[{next:{},fail:0,out:[]}],queue=[];
      p.patterns.forEach((s,id)=>{let v=0;for(const c of s){if(nodes[v].next[c]===undefined){nodes[v].next[c]=nodes.length;nodes.push({next:{},fail:0,out:[]});}v=nodes[v].next[c];}nodes[v].out.push(id);});
      const go=(v,c)=>{while(v&&nodes[v].next[c]===undefined)v=nodes[v].fail;return nodes[v].next[c]??0;};for(const v of Object.values(nodes[0].next))queue.push(v);
      for(const v of queue){for(const [c,w]of Object.entries(nodes[v].next)){nodes[w].fail=go(nodes[v].fail,c);queue.push(w);}emit(`failureを構築: node${v}`,'最長の適合する接尾辞へ戻ります。',['ノード','failure','直接出力ID'],nodes.map((x,i)=>[i,x.fail,x.out.join(', ')]),{edges:nodes.flatMap((x,i)=>i?[[i,x.fail]]:[])});}
      const counts=Array(p.patterns.length).fill(0);let v=0,end=0;const report=()=>{let w=v;for(;;){for(const id of nodes[w].out)counts[id]++;if(!w)break;w=nodes[w].fail;}};report();for(const c of p.text){v=go(v,c);end++;report();emit(`文字${end}: ${c} → node${v}`,'図は一致時にfailure鎖を走査。C++のscanは出力linkを使います。',['pattern ID','pattern','累積回数'],p.patterns.map((x,i)=>[i,x||'空',counts[i]]));}result=counts;
    }else if(mode==='eertree'){
      if(typeof p.text!=='string'||p.text.length>32)throw Error('textは32文字まで');const text=Array.from(p.text),nodes=[{len:-1,link:0,end:-1,next:{},suffix:0},{len:0,link:0,end:-1,next:{},suffix:0}];let last=1;
      for(let pos=0;pos<text.length;pos++){const c=text[pos],find=start=>{while(pos-1-nodes[start].len<0||text[pos-1-nodes[start].len]!==c)start=nodes[start].link;return start;};const v=find(last);if(nodes[v].next[c]===undefined){const len=nodes[v].len+2,link=len===1?1:nodes[find(nodes[v].link)].next[c];nodes[v].next[c]=nodes.length;nodes.push({len,link,end:pos,next:{},suffix:0});}last=nodes[v].next[c];nodes[last].suffix++;emit(`文字${pos+1}: ${c}`,'新しい異なる回文は高々1個です。',['ID','長さ','suffix link','最初の回文','最長suffix回数'],nodes.map((x,i)=>[i,x.len,x.link,i<2?'根':text.slice(x.end-x.len+1,x.end+1).join(''),x.suffix]),{edges:nodes.flatMap((x,i)=>i>1?[[i,x.link]]:[])});}
      const counts=nodes.map(x=>x.suffix);for(let i=nodes.length-1;i>=2;i--)counts[nodes[i].link]+=counts[i];result=nodes.slice(2).map((x,i)=>({text:text.slice(x.end-x.len+1,x.end+1).join(''),count:counts[i+2]}));emit('出現回数をsuffix linkへ集約','各ノードの最長suffix回数を、短い回文へ伝えます。',['回文','総出現回数'],result.map(x=>[x.text,x.count]));
    }
    emit('完了','入力を変えて履歴を作り直せます。',['結果'],[[typeof result==='object'?JSON.stringify(result):String(result)]]);
    return {frames,result:copy(result)};
  }
  return {specs,build};
});
