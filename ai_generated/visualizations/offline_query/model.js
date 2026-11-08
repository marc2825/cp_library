(function(root,factory){const api=factory();if(typeof module==="object"&&module.exports)module.exports=api;else root.OfflineQueryModel=api;})(globalThis,function(){
  "use strict";
  const clone=x=>JSON.parse(JSON.stringify(x));
  const lower=(a,x)=>{let l=0,r=a.length;while(l<r){const m=(l+r)>>1;if(a[m]<x)l=m+1;else r=m;}return l;};
  function trace(mode,input){
    const s={mode,answers:[],activeId:-1,activePoint:-1,cursor:null,range:null,title:"",detail:"",...clone(input)};
    const frames=[];
    const frame=(title,detail,extra={})=>{Object.assign(s,extra,{title,detail});frames.push(clone(s));};
    if(mode==="mo"){
      const q=s.queries.length,n=s.array.length;
      s.width=s.width||Math.max(1,Math.floor(n/Math.sqrt(Math.max(1,q))));
      s.order=Array.from({length:q},(_,i)=>i).sort((i,j)=>{
        const a=Math.floor(s.queries[i][0]/s.width),b=Math.floor(s.queries[j][0]/s.width);
        return a-b||((a%2?-1:1)*(s.queries[i][1]-s.queries[j][1]))||i-j;
      });
      s.l=0;s.r=0;s.frequency={};s.distinct=0;s.answers=Array(q).fill(null);s.done=[];
      frame("空区間 [0, 0) から始める",`ブロック幅 ${s.width}。左端のブロックを揃え、右端を交互の向きに並べます。`);
      const change=(i,delta,side)=>{
        const x=s.array[i],old=s.frequency[x]||0;
        s.frequency[x]=old+delta;
        if(old===0&&delta===1)++s.distinct;
        if(old===1&&delta===-1)--s.distinct;
        frame(`${side}から${delta===1?"追加":"削除"}: a[${i}] = ${x}`,`頻度 ${old} → ${old+delta}。異なる値は ${s.distinct} 種類。`,{activePoint:i});
      };
      for(const id of s.order){
        const [l,r]=s.queries[id];s.activeId=id;
        frame(`Q${id} の [${l}, ${r}) へ移動`,"まず区間を広げ、その後に不要な端を削除します。途中で要素のない区間から削除しません。",{activePoint:-1});
        while(s.l>l)change(--s.l,1,"左");
        while(s.r<r){const i=s.r++;change(i,1,"右");}
        while(s.l<l){const i=s.l++;change(i,-1,"左");}
        while(s.r>r)change(--s.r,-1,"右");
        s.answers[id]=s.distinct;s.done.push(id);
        frame(`Q${id} の答えを保存`,`${s.distinct} 種類。計算する順番を変えても、答えは登録 ID の位置に戻します。`,{activePoint:-1});
      }
      frame("すべての区間を処理しました","配列の途中を更新する Mo や、木上の Mo はこの基本形には含めません。",{activeId:-1});
    }else if(mode==="pbs"){
      const m=s.edges.length,q=s.queries.length;
      s.low=Array(q).fill(-1);s.high=Array(q).fill(m+1);s.mid=Array(q).fill(null);s.answers=Array(q).fill(null);s.round=0;s.t=0;
      s.parent=Array.from({length:s.n},(_,i)=>i);
      const leader=x=>{while(s.parent[x]!==x)x=s.parent[x];return x;};
      frame("未成立 -1、成立 M+1 を番兵にする",`実際に調べる状態は 0..${m}。${m+1} は最後まで不成立という答えです。番兵自体は判定しません。`);
      while(true){
        const buckets=Array.from({length:m+1},()=>[]);let last=-1;
        s.mid=s.low.map((l,i)=>s.high[i]-l>1?Math.floor((l+s.high[i])/2):null);
        for(let i=0;i<q;++i)if(s.mid[i]!==null){buckets[s.mid[i]].push(i);last=Math.max(last,s.mid[i]);}
        if(last<0)break;
        ++s.round;s.t=0;s.parent=Array.from({length:s.n},(_,i)=>i);
        frame(`ラウンド ${s.round}: reset()`,"各 Q の mid を同じ時刻のバケツへ集めます。毎回、辺を 1 本も加えていない状態へ戻します。",{activeId:-1});
        for(let t=0;t<=last;++t){
          if(t){const [u,v]=s.edges[t-1];s.parent[leader(u)]=leader(v);s.t=t;
            frame(`apply(${t-1}): 辺 ${u}–${v}`,`最初の ${t} 本を追加した状態です。時刻が同じ問い合わせは、この状態を共有します。`,{activeId:-1});}
          for(const id of buckets[t]){
            const [u,v]=s.queries[id],ok=leader(u)===leader(v);s.activeId=id;
            if(ok)s.high[id]=t;else s.low[id]=t;
            frame(`check(Q${id}) = ${ok}`,`${u} と ${v} は${ok?"連結":"未連結"}。${ok?"high":"low"} を ${t} に更新します。状態 0 も調べるため、同じ頂点の答えは 0 です。`);
          }
        }
      }
      s.answers=s.high.slice();
      frame("最初に成立する更新数が確定",`答え ${m+1} は「該当なし」。並列は複数の探索をまとめる意味で、スレッドは使いません。`,{activeId:-1});
    }else{
      s.points=s.points||[];s.ys=[...new Set(s.points.map(p=>p[1]))].sort((a,b)=>a-b);
      s.bit=Array(s.ys.length+1).fill(0);s.frequency=Array(s.ys.length).fill(0);s.activePoints=[];
      const add=(point,delta)=>{const i=lower(s.ys,point[1]);s.frequency[i]+=delta;for(let j=i+1;j<s.bit.length;j+=j&-j)s.bit[j]+=delta;};
      const sum=y=>{let value=0;for(let i=lower(s.ys,y);i>0;i-=i&-i)value+=s.bit[i];return value;};
      if(mode==="sweep"){
        s.answers=Array(s.queries.length).fill(0);s.finished=false;
        const events=[];
        s.queries.forEach(([l,d,r,u],id)=>{if(l<r&&d<u){events.push({x:l,d,u,id,sign:-1},{x:r,d,u,id,sign:1});}});
        events.sort((a,b)=>a.x-b.x);
        const order=s.points.map((_,i)=>i).sort((i,j)=>s.points[i][0]-s.points[j][0]);let p=0;
        frame("x 方向に走査し、y 方向を BIT で数える","長方形ごとに右境界の寄与を加え、左境界の寄与を引きます。空・逆向きの長方形は 0 です。");
        for(const e of events){
          s.cursor=e.x;s.activeId=e.id;s.range=[e.x,e.d,e.x,e.u];
          frame(`x < ${e.x} の状態を準備`,"境界と同じ x の点はまだ追加しません。左辺は含み、右辺は含まない条件になります。",{activePoint:-1});
          while(p<order.length&&s.points[order[p]][0]<e.x){
            const i=order[p++];s.activePoints.push(i);add(s.points[i],s.points[i][2]);
            frame(`点 P${i} を BIT へ追加`,`y=${s.points[i][1]} の重み ${s.points[i][2]} を加えます。`,{activePoint:i});
          }
          const value=sum(e.u)-sum(e.d);s.answers[e.id]+=e.sign*value;
          frame(`Q${e.id}: ${e.sign>0?"＋":"−"}${value} を反映`,`y ∈ [${e.d}, ${e.u}) の和 ${value}。現在の部分和は ${s.answers[e.id]} です。`,{activePoint:-1});
        }
        s.finished=true;
        frame("長方形ごとの和が確定","負の重み・同一座標の点も、そのまま加算できます。",{activeId:-1,activePoint:-1});
      }else if(mode==="cdq"){
        const initial=s.points.slice(),events=[],points=[],queries=[];
        const point=(p,time)=>{const i=points.length;points.push(p);events.push({x:p[0],y:p[1],w:p[2],point:i,id:-1,time});};
        initial.forEach((p,i)=>point(p,i));
        s.operations.forEach((op,i)=>{
          const time=initial.length+i;
          if(op[0]==="add")point(op.slice(1),time);
          else{
            const [l,d,r,u]=op.slice(1),id=queries.length;queries.push([l,d,r,u]);
            if(l<r&&d<u)for(const [x,y,sign]of[[r,u,1],[l,u,-1],[r,d,-1],[l,d,1]])events.push({x,y,id,sign,time});
          }
        });
        s.points=points;s.queries=queries;s.events=events;s.answers=Array(queries.length).fill(0);s.finished=false;
        s.ys=[...new Set(points.map(p=>p[1]))].sort((a,b)=>a-b);s.bit=Array(s.ys.length+1).fill(0);s.frequency=Array(s.ys.length).fill(0);
        const order=events.map((_,i)=>i);s.interval=[0,0,events.length];
        frame("時刻順に並べ、問い合わせを 4 つに分解","F(x,y)=Σ(px<x, py<y) として +F(r,u)−F(l,u)−F(r,d)+F(l,d)。後の更新は前の問い合わせへ影響しません。");
        function cdq(l,r,depth){
          if(r-l<=1)return;
          const m=(l+r)>>1;s.interval=[l,m,r];s.depth=depth;
          frame(`[${l}, ${r}) を ${m} で分割`,"まず左右を解き、その後で左の更新 → 右の問い合わせを数えます。イベント番号は展開後の時刻順です。",{activeId:-1,activePoint:-1,range:null,cursor:null});
          cdq(l,m,depth+1);cdq(m,r,depth+1);s.interval=[l,m,r];s.depth=depth;
          frame(`左 [${l}, ${m}) → 右 [${m}, ${r})`,"各半分を x 順に見て、左側の更新だけを BIT に入れます。同じ x は除外します。",{activeId:-1,activePoint:-1,range:null,cursor:null});
          let p=l;const inserted=[];
          for(let j=m;j<r;++j){
            const e=events[order[j]];s.cursor=e.x;
            while(p<m&&events[order[p]].x<e.x){
              const a=events[order[p++]];
              if(a.id<0){add(points[a.point],a.w);inserted.push(a.point);s.activePoints.push(a.point);
                frame(`左の P${a.point} を追加`,`x=${a.x} < ${e.x}。この分割での寄与だけを保持します。`,{activePoint:a.point,activeId:-1});}
            }
            if(e.id>=0){const value=sum(e.y);s.answers[e.id]+=e.sign*value;s.range=[null,null,e.x,e.y];
              frame(`右の Q${e.id}: ${e.sign>0?"＋":"−"}${value}`,`F(${e.x}, ${e.y}) の部分寄与です。左の過去の更新かつ x<${e.x}, y<${e.y} を満たす点を数えます。`,{activeId:e.id,activePoint:-1});}
          }
          for(const i of inserted){add(points[i],-points[i][2]);s.activePoints.splice(s.activePoints.indexOf(i),1);
            frame(`P${i} の更新を巻き戻す`,"入れた重みの逆元だけを加え、次の分割へ BIT の状態を持ち越しません。",{activePoint:i,activeId:-1});}
          const merged=order.slice(l,r).sort((a,b)=>events[a].x-events[b].x);order.splice(l,r-l,...merged);
          frame(`[${l}, ${r}) を x 順にマージ`,"要素は元の時刻区間の内側に留まります。更新と問い合わせの各組は、別々の半分に分かれる分割で一度だけ数えられます。",{activePoint:-1,activeId:-1,range:null,cursor:null});
        }
        cdq(0,events.length,0);s.finished=true;
        frame("時刻を含む長方形和が確定","BIT は 0 に戻りました。図は分割・走査・巻き戻しを示す基本形です。",{activeId:-1,activePoint:-1,range:null,cursor:null});
      }else throw new Error("unknown mode");
    }
    return {frames,answers:s.answers.slice()};
  }
  return {trace,lower};
});
