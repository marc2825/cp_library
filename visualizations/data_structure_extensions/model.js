(function(root){
  'use strict';
  const modes=[
    {id:'double_ended_priority_queue',title:'D-02 · 両端優先度キュー',doc:'ordered_structure/double_ended_priority_queue.md',sample:{values:[4,1,4,9],operations:['pop_max','pop_min']}},
    {id:'static_range_mex',title:'D-03 · 区間 MEX',doc:'static_query/static_range_mex.md',sample:{values:[0,1,0,3,2],queries:[[0,5],[1,4],[2,2]]}},
    {id:'range_majority',title:'D-04 · 区間の過半数',doc:'static_query/range_majority.md',sample:{values:[2,1,2,2,3,1],queries:[[0,4],[0,6]]}},
    {id:'point_set_range_frequency',title:'D-05 · 点更新と区間頻度',doc:'range_query/point_set_range_frequency.md',sample:{values:[2,1,2,3],operations:[['count',0,4,2],['set',1,2],['set',3,100],['count',0,3,2]]}},
    {id:'persistent_leftist_heap',title:'D-06 · 永続 Leftist Heap',doc:'ordered_structure/persistent_leftist_heap.md',sample:{operations:[['push',0,4],['push',1,1],['push',1,7],['meld',2,3],['pop',4]]}},
    {id:'dynamic_segtree_merge_split',title:'D-07 · 動的セグ木の merge / split',doc:'range_query/dynamic_segtree_merge_split.md',sample:{size:10,operations:[['set',0,2,5],['set',1,8,7],['split',2,5],['set',3,2,9],['merge',5,4]]}},
    {id:'segment_tree_cover',title:'D-08 · 標準ノード分解',doc:'range_query/segment_tree_cover.md',sample:{size:8,left:1,right:7}},
    {id:'segment_tree_over_time',title:'D-09 · 時間軸セグ木',doc:'offline_query/segment_tree_over_time.md',sample:{vertices:3,times:4,edges:[[0,3,0,1],[1,2,1,2]]}},
    {id:'rollback_disjoint_set_union_with_data',title:'D-10 · 付随データも Rollback',doc:'union_find/rollback_disjoint_set_union_with_data.md',sample:{weights:[2,3,7],operations:[['save'],['merge',0,1],['rollback',0]]}}
  ];
  const integer=(x,l=-1000000,r=1000000)=>{if(!Number.isSafeInteger(x)||x<l||x>r)throw Error(`整数 ${l}..${r} を指定してください`);return x;};
  const list=(a,limit=64)=>{if(!Array.isArray(a)||a.length>limit)throw Error(`配列は ${limit} 要素以下です`);return a;};
  const values=a=>list(a,32).map(x=>integer(x));
  const operations=a=>list(a,40);
  const pair=(q,n)=>{list(q,2);if(q.length!==2)throw Error('[l,r] を指定してください');return [integer(q[0],0,n),integer(q[1],q[0],n)];};
  function layout(n){let base=1;while(base<n)base*=2;return base;}
  function cover(n,l,r){const base=layout(n),a=[],b=[];for(l+=base,r+=base;l<r;l=Math.floor(l/2),r=Math.floor(r/2)){if(l%2)a.push(l++);if(r%2)b.push(--r);}return a.concat(b.reverse());}
  function interval(n,k){let width=layout(n),level=1;while(2*level<=k){level*=2;width/=2;}const l=(k-level)*width;return [Math.min(n,l),Math.min(n,l+width)];}
  function trace(id,input){
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSON object を指定してください');
    const steps=[];
    function add(title,detail,columns=[],rows=[],metrics={}){steps.push({title,detail,columns,rows,values:Object.entries(metrics).map(([label,value])=>({label,value:String(value)}))});}
    const arrayRows=a=>a.map((v,i)=>[i,v]);
    let result;
    add('入力を確認','入力を変更して、各段階の値を確かめられます。',[],[],{モード:id});
    if(id==='double_ended_priority_queue'){
      const a=values(input.values).sort((a,b)=>a-b);
      const show=title=>add(title,'比較順に並んだ多重集合。左右の端だけを参照・削除します。',['順位','値'],arrayRows(a),{個数:a.length,最小:a[0]??'空',最大:a.at(-1)??'空'});
      show('初期集合');
      for(const op of operations(input.operations)){
        if(Array.isArray(op)&&op[0]==='push'){a.push(integer(op[1]));a.sort((a,b)=>a-b);}
        else if(op==='pop_min'||op==='pop_max'){if(!a.length)throw Error('空集合からは削除できません');op==='pop_min'?a.shift():a.pop();}
        else if(op==='clear')a.length=0;
        else throw Error('push / pop_min / pop_max / clear を指定してください');
        show(JSON.stringify(op));
      }
      result={min:a[0]??null,max:a.at(-1)??null,size:a.length};
    }else if(id==='static_range_mex'){
      const a=values(input.values),versions=[Array(a.length+1).fill(-1)];
      for(let i=0;i<a.length;i++){const next=versions.at(-1).slice();if(0<=a[i]&&a[i]<=a.length)next[a[i]]=i;versions.push(next);add(`prefix ${i+1} を保存`,'値ごとの最終出現位置。過去の prefix を残します。',['値','最終位置'],arrayRows(next),{版:i+1});}
      result=[];
      for(const q of operations(input.queries)){
        const [l,r]=pair(q,a.length),last=versions[r];let x=0,y=a.length+1;
        while(y-x>1){const m=Math.floor((x+y)/2),min=Math.min(...last.slice(x,m));add(`[${l},${r}) の探索`,'左半分の最終出現位置の最小値が l 未満なら、欠ける値は左にあります。',['値','最終位置'],last.slice(x,y).map((v,i)=>[x+i,v]),{左端:l,左半分の最小:min,進む方向:min<l?'左':'右'});if(min<l)y=m;else x=m;}
        result.push(x);add('MEX を発見',`区間 [${l},${r}) に ${x} は現れません。`,[],[],{MEX:x});
      }
    }else if(id==='range_majority'){
      const a=values(input.values),base=layout(a.length),tree=Array.from({length:2*base},()=>({value:null,count:0}));
      const combine=(a,b)=>!a.count?b:!b.count?a:a.value===b.value?{value:a.value,count:a.count+b.count}:a.count>b.count?{value:a.value,count:a.count-b.count}:{value:b.value,count:b.count-a.count};
      a.forEach((x,i)=>tree[base+i]={value:x,count:1});for(let k=base-1;k;k--)tree[k]=combine(tree[2*k],tree[2*k+1]);
      add('区間ごとの相殺候補','同じ値は加算し、異なる値は相殺します。候補の結合はモノイドではありません。',['ノード','候補','残票'],tree.slice(1).map((v,k)=>[k+1,v.count?v.value:'なし',v.count]),{長さ:a.length});
      result=[];
      for(const q of operations(input.queries)){
        const [l,r]=pair(q,a.length);let x=base+l,y=base+r,left={count:0},right={count:0};
        while(x<y){if(x%2)left=combine(left,tree[x++]);if(y%2)right=combine(tree[--y],right);x=Math.floor(x/2);y=Math.floor(y/2);}
        const candidate=combine(left,right),count=candidate.count?a.slice(l,r).filter(x=>x===candidate.value).length:0,answer=count>(r-l)/2?candidate.value:null;
        result.push(answer);add(`[${l},${r}) を検証`,'候補の実際の出現回数が半分を超える場合だけ採用します。',['位置','値'],a.slice(l,r).map((v,i)=>[l+i,v]),{候補:candidate.count?candidate.value:'なし',実際の頻度:count,過半数:answer??'なし'});
      }
    }else if(id==='point_set_range_frequency'){
      const a=values(input.values),answers=[],base=layout(a.length);
      for(const op of operations(input.operations)){
        list(op,4);
        if(op[0]==='set'){
          const p=integer(op[1],0,a.length-1),x=integer(op[2]);a[p]=x;const rows=[];
          for(let k=base+p;k;k=Math.floor(k/2)){const [l,r]=interval(a.length,k),counts=new Map();for(const v of a.slice(l,r))counts.set(v,(counts.get(v)||0)+1);rows.push([k,`[${l},${r})`,JSON.stringify([...counts].sort((a,b)=>a[0]-b[0]))]);}
          add(`a[${p}] ← ${x}`,'根までの頻度表だけを更新します。0個のキーは削除します。',['ノード','区間','値と頻度'],rows,{変更位置:p});
        }else if(op[0]==='count'){
          const [l,r]=pair([op[1],op[2]],a.length),x=integer(op[3]),nodes=cover(a.length,l,r);let count=0;
          const rows=nodes.map(k=>{const [a0,b0]=interval(a.length,k),c=a.slice(a0,b0).filter(v=>v===x).length;count+=c;return [k,`[${a0},${b0})`,c];});answers.push(count);
          add(`値 ${x} の頻度`,'重複しない標準ノードの頻度を加算します。',['ノード','区間','頻度'],rows,{答え:count});
        }else throw Error('set / count を指定してください');
      }
      result={values:a,answers};
    }else if(id==='persistent_leftist_heap'){
      const nodes=[{rank:0,size:0}],roots=[0];
      const make=n=>{nodes.push(n);return nodes.length-1;};
      function meld(a,b){if(!a||!b)return a||b;if(nodes[a].value>nodes[b].value)[a,b]=[b,a];const n={...nodes[a]},right=meld(n.right,b);n.right=right;if(nodes[n.left].rank<nodes[n.right].rank)[n.left,n.right]=[n.right,n.left];n.rank=nodes[n.right].rank+1;n.size=1+nodes[n.left].size+nodes[n.right].size;if(n.size>1000000)throw Error('図解の heap 要素数上限は 1000000 です');return make(n);}
      for(const op of operations(input.operations)){
        list(op,3);const a=roots[integer(op[1],0,roots.length-1)];let next;
        if(op[0]==='push')next=meld(a,make({value:integer(op[2]),left:0,right:0,rank:1,size:1}));
        else if(op[0]==='pop'){if(!a)throw Error('空 heap は pop できません');next=meld(nodes[a].left,nodes[a].right);}
        else if(op[0]==='meld')next=meld(a,roots[integer(op[2],0,roots.length-1)]);
        else throw Error('push / pop / meld を指定してください');
        roots.push(next);
        add(`版 ${roots.length-1} を作成`,'この図は最小優先。右鎖をコピーし、rank(左) ≥ rank(右) を保ちます。古い root は残ります。',['ID','値','左','右','rank'],nodes.slice(1).map((n,i)=>[i+1,n.value,n.left,n.right,n.rank]),{root:next,要素数:nodes[next].size,版数:roots.length});
      }
      result={tops:roots.map(k=>k?nodes[k].value:null),sizes:roots.map(k=>nodes[k].size)};
    }else if(id==='dynamic_segtree_merge_split'){
      const n=integer(input.size,0,32),versions=[new Map()];
      for(const op of operations(input.operations)){
        list(op,4);const a=versions[integer(op[1],0,versions.length-1)];
        if(op[0]==='set'){const p=integer(op[2],0,n-1),v=integer(op[3]),next=new Map(a);next.set(p,v);versions.push(next);}
        else if(op[0]==='split'){const p=integer(op[2],0,n);versions.push(new Map([...a].filter(([k])=>k<p)),new Map([...a].filter(([k])=>k>=p)));}
        else if(op[0]==='merge'){const b=versions[integer(op[2],0,versions.length-1)],next=new Map(a);for(const [k,v]of b)next.set(k,(next.get(k)||0)+v);versions.push(next);}
        else throw Error('set / split / merge を指定してください');
        add(JSON.stringify(op),'永続版の論理状態です。split は座標を詰めず、merge は同じ座標の値を加算します。',['版','座標 → 値','区間全体の和'],versions.map((v,i)=>[i,JSON.stringify([...v].sort((a,b)=>a[0]-b[0])),[...v.values()].reduce((a,b)=>a+b,0)]),{版数:versions.length});
      }
      result={totals:versions.map(v=>[...v.values()].reduce((a,b)=>a+b,0))};
    }else if(id==='segment_tree_cover'){
      const n=integer(input.size,0,32),[l,r]=pair([input.left,input.right],n),nodes=cover(n,l,r),intervals=nodes.map(k=>interval(n,k));
      add('完全二分木の番号','根は1、葉は base+p。padding は n で切り詰めます。',['ノード','区間','選択'],Array.from({length:2*layout(n)-1},(_,i)=>{const k=i+1,[a,b]=interval(n,k);return[k,`[${a},${b})`,nodes.includes(k)?'●':''];}),{base:layout(n),対象:`[${l},${r})`});
      nodes.forEach((k,i)=>add(`標準ノード ${k}`,'左から右へ、重なりのない区間を採用します。',['ノード','左端','右端'],nodes.slice(0,i+1).map((k,j)=>[k,...intervals[j]]),{採用数:i+1}));
      result={nodes,intervals};
    }else if(id==='segment_tree_over_time'){
      const n=integer(input.vertices,0,16),times=integer(input.times,0,16),base=layout(times),buckets=Array.from({length:2*base},()=>[]),parent=Array.from({length:n},(_,i)=>i),sizes=Array(n).fill(1),answer=[];
      const leader=v=>{while(parent[v]!==v)v=parent[v];return v;};
      for(const edge of operations(input.edges)){list(edge,4);const [l,r]=pair(edge.slice(0,2),times),a=integer(edge[2],0,n-1),b=integer(edge[3],0,n-1);for(const k of cover(times,l,r))buckets[k].push([a,b]);}
      add('有効期間を標準ノードに登録','イベントは、その期間を覆うノードで一度ずつ適用します。',['ノード','区間','辺'],buckets.map((v,k)=>[k,JSON.stringify(interval(times,k||1)),JSON.stringify(v)]).filter((_,k)=>k&&buckets[k].length),{時刻数:times});
      function visit(k,l,r){if(l>=times)return;const old=parent.slice(),oldSizes=sizes.slice();for(const [u,v]of buckets[k]){let a=leader(u),b=leader(v);if(a===b)continue;if(sizes[a]<sizes[b])[a,b]=[b,a];parent[b]=a;sizes[a]+=sizes[b];}add(`node ${k} に入る`,'snapshot を保存して、このノードの辺を適用します。',['頂点','代表元'],parent.map((_,i)=>[i,leader(i)]),{区間:`[${l},${Math.min(r,times)})`,適用辺:buckets[k].length});if(r-l===1){answer.push(parent.filter((v,i)=>v===i).length);add(`時刻 ${l} を訪問`,'葉には現在有効な辺だけが残ります。',[],[],{成分数:answer.at(-1)});}else{const m=(l+r)/2;visit(2*k,l,m);visit(2*k+1,m,r);}parent.splice(0,parent.length,...old);sizes.splice(0,sizes.length,...oldSizes);add(`node ${k} から戻る`,'rollback で入る前の状態に戻し、隣の区間へ進みます。',['頂点','代表元'],parent.map((_,i)=>[i,leader(i)]),{復元済み:'yes'});}
      if(times)visit(1,0,base);result=answer;
    }else if(id==='rollback_disjoint_set_union_with_data'){
      const weights=values(input.weights),n=weights.length;let parent=weights.map((_,i)=>i),sizes=Array(n).fill(1),sums=weights.slice(),components=n;
      const snapshots=[],history=[];const leader=v=>{while(parent[v]!==v)v=parent[v];return v;};
      for(const op of operations(input.operations)){
        list(op,3);
        if(op[0]==='save')snapshots.push({parent:parent.slice(),sizes:sizes.slice(),sums:sums.slice(),components});
        else if(op[0]==='merge'){let a=leader(integer(op[1],0,n-1)),b=leader(integer(op[2],0,n-1));if(a!==b){if(sizes[a]<sizes[b])[a,b]=[b,a];sums[a]+=sums[b];parent[b]=a;sizes[a]+=sizes[b];components--;}}
        else if(op[0]==='rollback'){const i=integer(op[1],0,snapshots.length-1),s=snapshots[i];parent=s.parent.slice();sizes=s.sizes.slice();sums=s.sums.slice();components=s.components;snapshots.length=i+1;}
        else throw Error('save / merge / rollback を指定してください');
        history.push(components);add(JSON.stringify(op),'成分の親と、callback が更新した付随データを同じ保存点へ戻します。',['頂点','親','根のサイズ','保存された和'],parent.map((p,i)=>[i,p,p===i?sizes[i]:'—',sums[i]]),{成分数:components,保存点数:snapshots.length});
      }
      result={components,sums,history};
    }else throw Error('未知のモードです');
    add('最終結果','入力を変更して、別の区間や分岐も確かめられます。',[],[],{手順数:steps.length});
    return {steps,result};
  }
  const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.DataStructureExtensionsModel=api;
})(typeof globalThis==='undefined'?this:globalThis);
