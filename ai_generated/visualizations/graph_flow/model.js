(function(root,factory){const api=factory();if(typeof module==="object"&&module.exports)module.exports=api;else root.GraphFlowModel=api;})(globalThis,function(){
  "use strict";
  const clone=x=>JSON.parse(JSON.stringify(x));
  const integer=(v,min,label)=>{if(!Number.isSafeInteger(v)||v<min)throw new Error(`${label}の範囲が不正です。`);};
  function trace(mode,input){
    if(!["dinic","cut","matching"].includes(mode))throw new Error("未知のモードです。");
    const frames=[],s=clone(input),frame=(title,detail,extra={})=>{Object.assign(s,extra,{title,detail});frames.push(clone(s));};
    s.mode=mode;s.phase="initial";s.round=0;s.work=[];s.path=[];s.delta=0;s.head=0;
    if(mode!=="matching"){
      integer(s.n,2,"頂点数");integer(s.source,0,"始点");integer(s.sink,0,"終点");
      if(s.source>=s.n||s.sink>=s.n||s.source===s.sink)throw new Error("始点と終点は範囲内の異なる頂点にしてください。");
      let sum=0;s.edges.forEach(e=>{if(e.length!==3)throw new Error("辺は u v capacity です。");const[u,v,c]=e;integer(u,0,"頂点");integer(v,0,"頂点");integer(c,0,"容量");if(u>=s.n||v>=s.n)throw new Error("頂点が範囲外です。");sum+=c;});
      if(!Number.isSafeInteger(sum))throw new Error("容量の合計が安全な整数の範囲外です。");
      if(s.limit!=null)integer(s.limit,0,"流量上限");
      const limit=mode==="cut"||s.limit==null?sum:s.limit,adj=Array.from({length:s.n},()=>[]),arcs=[];
      s.edges.forEach(([u,v,c],id)=>{arcs.push({from:u,to:v,cap:c},{from:v,to:u,cap:0});adj[u].push(2*id);adj[v].push(2*id+1);});
      s.flows=s.edges.map(()=>0);s.total=0;s.level=Array(s.n).fill(-1);s.next=Array(s.n).fill(0);s.sourceSide=Array(s.n).fill(false);s.cutEdges=[];s.cutCapacity=null;s.exhausted=false;
      const residual=id=>arcs[id].cap+(id%2?s.flows[id>>1]:-s.flows[id>>1]);
      frame("残余ネットワークを用意", "元辺ごとに正方向 c−f と逆方向 f の残余容量を持ちます。逆辺は、既に流した分を取り消すための辺です。");
      while(s.total<limit){
        ++s.round;s.level.fill(-1);s.level[s.source]=0;s.work=[s.source];s.path=[];s.phase="bfs";s.head=0;
        frame(`第 ${s.round} 段階: BFS`,"残余容量が正の辺を辿り、始点からの辺数を level に記録します。");
        for(let head=0;head<s.work.length;++head){s.head=head;const v=s.work[head];
          for(const id of adj[v])if(residual(id)>0&&s.level[arcs[id].to]<0){const to=arcs[id].to;s.level[to]=s.level[v]+1;s.work.push(to);frame(`${to} の level = ${s.level[to]}`,`残余辺 #${id>>1}${id%2?" の逆向き":""} で発見しました。`,{activeArc:id});}
        }
        s.work=[];s.activeArc=-1;
        if(s.level[s.sink]<0){frame("終点へ到達できない","残余ネットワークに増加路がなく、現在の流れが最大です。");break;}
        s.next.fill(0);s.phase="dfs";
        frame("レベルが一つ増える辺だけを使う",`終点の level は ${s.level[s.sink]}。current arc は次に調べる隣接辺の位置です。`);
        while(s.total<limit){
          const vertices=[s.source],path=[],bottleneck=[limit-s.total];let pushed=0;
          while(vertices.length){
            const v=vertices[vertices.length-1];s.work=vertices.slice();s.path=path.slice();
            if(v===s.sink){pushed=bottleneck[bottleneck.length-1];s.delta=pushed;frame("増加路が見つかった",`経路上の最小残余容量と残りの流量上限から ${pushed} 流せます。`,{phase:"path"});
              for(const id of path)s.flows[id>>1]+=id%2?-pushed:pushed;
              s.total+=pushed;s.phase="augment";frame(`${pushed} 流して残余容量を更新`,`累計 ${s.total}。正方向なら f を増やし、逆方向なら f を減らします。`);s.phase="dfs";break;
            }
            while(s.next[v]<adj[v].length){const id=adj[v][s.next[v]];if(residual(id)>0&&s.level[arcs[id].to]===s.level[v]+1)break;++s.next[v];}
            if(s.next[v]===adj[v].length){s.level[v]=-1;vertices.pop();bottleneck.pop();if(path.length){path.pop();++s.next[vertices[vertices.length-1]];}
              s.work=vertices.slice();s.path=path.slice();frame(`${v} から先へ進めない`,"この段階で使える枝が尽きた頂点を外し、親の次の候補へ進みます。",{delta:0});
            }else{const id=adj[v][s.next[v]];path.push(id);vertices.push(arcs[id].to);bottleneck.push(Math.min(bottleneck[bottleneck.length-1],residual(id)));s.work=vertices.slice();s.path=path.slice();frame("増加路を伸ばす",`辺 #${id>>1}${id%2?" を逆向きに":" を正方向に"}辿ります。`,{activeArc:id,delta:0});}
          }
          if(!pushed)break;
        }
      }
      s.phase="reach";s.path=[];s.activeArc=-1;s.work=[s.source];s.sourceSide[s.source]=true;
      frame("残余辺で始点側 S を調べる", "level の値ではなく、現在の残余容量が正の全ての辺で到達範囲を探索します。",{delta:0});
      for(let head=0;head<s.work.length;++head)for(const id of adj[s.work[head]])if(residual(id)>0&&!s.sourceSide[arcs[id].to]){s.sourceSide[arcs[id].to]=true;s.work.push(arcs[id].to);frame(`${arcs[id].to} を始点側へ`,"到達できる頂点が S。残りが T です。");}
      s.exhausted=!s.sourceSide[s.sink];s.work=[];s.phase="done";
      if(s.exhausted){s.cutCapacity=0;s.edges.forEach(([u,v,c],id)=>{if(s.sourceSide[u]&&!s.sourceSide[v]){s.cutEdges.push(id);s.cutCapacity+=c;}});
        frame("最大流と最小カットが確定",`最大流 ${s.total} = カット容量 ${s.cutCapacity}。元辺の S→T だけを復元します（容量 0 の辺も含みます）。`);
      }else frame("流量上限で停止",`累計 ${s.total}。終点はまだ到達可能なので、この S/T 分割は最小カットではありません。`);
    }else{
      integer(s.leftSize,0,"左の頂点数");integer(s.rightSize,0,"右の頂点数");
      const adj=Array.from({length:s.leftSize},()=>[]);
      s.edges.forEach(e=>{if(e.length!==2)throw new Error("辺は left right です。");const[u,v]=e;integer(u,0,"左頂点");integer(v,0,"右頂点");if(u>=s.leftSize||v>=s.rightSize)throw new Error("頂点が範囲外です。");adj[u].push(v);});
      s.leftMatch=Array(s.leftSize).fill(-1);s.rightMatch=Array(s.rightSize).fill(-1);s.distance=Array(s.leftSize).fill(-1);s.next=Array(s.leftSize).fill(0);s.cardinality=0;s.shortest=null;
      frame("空のマッチングから開始", "左と右は別々の番号です。一つの頂点は高々一つの相手と組になります。");
      const l=s.leftMatch,r=s.rightMatch;
      function pathEdges(vertices,via,end=-1){const result=[];for(let i=0;i<via.length;++i){result.push({left:vertices[i],right:via[i],matched:false},{left:vertices[i+1],right:via[i],matched:true});}if(end>=0)result.push({left:vertices[vertices.length-1],right:end,matched:false});return result;}
      while(true){
        ++s.round;s.distance.fill(-1);s.work=[];s.path=[];s.phase="bfs";s.head=0;s.shortest=null;s.head=0;let shortest=s.leftSize+1;
        for(let u=0;u<s.leftSize;++u)if(l[u]<0){s.distance[u]=0;s.work.push(u);}
        frame(`第 ${s.round} 段階: 未マッチの左から BFS`,"未採用辺を左→右、採用辺を右→左に辿る交互路を探します。distance は左頂点の層番号です。");
        for(let head=0;head<s.work.length;++head){s.head=head;const u=s.work[head];if(s.distance[u]>=shortest)continue;
          for(const v of adj[u]){const mate=r[v];if(mate<0){shortest=s.distance[u]+1;s.shortest=shortest;frame(`未マッチの R${v} へ届く`, `最短交互路の長さは左→右の ${shortest} 区間です。`);}
            else if(s.distance[mate]<0){s.distance[mate]=s.distance[u]+1;s.work.push(mate);frame(`R${v} の相手 L${mate} へ`,"既存の組を辿り、組み替えの候補を次の層へ入れます。");}
          }
        }
        s.work=[];if(shortest===s.leftSize+1)break;
        s.shortest=shortest;s.next.fill(0);s.phase="dfs";
        for(let root=0;root<s.leftSize;++root){if(l[root]>=0||s.distance[root]!==0)continue;
          const vertices=[root],via=[];
          while(vertices.length){const u=vertices[vertices.length-1];let descended=false,augmented=false;s.work=vertices.slice();
            while(s.next[u]<adj[u].length){const v=adj[u][s.next[u]++],mate=r[v];
              if(mate<0&&s.distance[u]+1===shortest){s.path=pathEdges(vertices,via,v);frame("交互路が見つかった", "橙が追加する辺、青破線が外す辺。全体を反転すると組が一つ増えます。",{phase:"path"});
                l[u]=v;r[v]=u;for(let i=via.length-1;i>=0;--i){l[vertices[i]]=via[i];r[via[i]]=vertices[i];}++s.cardinality;
                frame("交互路を反転",`${s.cardinality} 組になりました。既存の組も必要に応じて組み替えます。`,{phase:"augment"});s.phase="dfs";augmented=true;break;
              }
              if(mate>=0&&s.distance[mate]===s.distance[u]+1){via.push(v);vertices.push(mate);s.work=vertices.slice();s.path=pathEdges(vertices,via);frame("交互路を伸ばす",`L${u}→R${v}→L${mate}。採用済みの辺を戻ります。`);descended=true;break;}
            }
            if(augmented)break;if(descended)continue;s.distance[u]=-1;vertices.pop();if(via.length)via.pop();s.work=vertices.slice();s.path=pathEdges(vertices,via);frame(`L${u} の候補が尽きた`,"この段階で使える最短交互路がなく、探索を戻します。");
          }
        }
      }
      s.phase="done";s.work=[];s.path=[];s.pairs=l.flatMap((v,u)=>v<0?[]:[[u,v]]);
      frame("二部最大マッチングが確定",`${s.cardinality} 組。未マッチの左から未マッチの右への交互路がありません。`);
    }
    return {frames,state:clone(s)};
  }
  return {trace};
});
