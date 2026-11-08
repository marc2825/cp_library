/* Pure offline model; input arrays, candidate scans, FIFO extrema and two-stack folds. */
(function(root,factory){const api=factory();if(typeof module==="object"&&module.exports)module.exports=api;else root.StaticQueryModel=api;})(typeof globalThis!=="undefined"?globalThis:this,function(){
  "use strict";
  const modes=["sparse","stack","queue","window","swag","swag_deque"];
  const clone=x=>JSON.parse(JSON.stringify(x));
  function integer(x){if(!Number.isSafeInteger(x))throw new Error("安全に表示できる整数を入力してください。");return x===0?0:x;}
  const better=(s,x,y)=>s.order==="max"?x>y:x<y;
  const best=(s,x,y)=>better(s,y,x)?y:x;
  const identity=s=>s.aggregation==="concat"?"":0;
  const op=(s,x,y)=>s.aggregation==="concat"?x+y:integer(x+y);
  function makeTable(s){
    const table=s.a.length?[s.a.slice()]:[];
    for(let width=2;width<=s.a.length;width*=2){
      const previous=table.at(-1),half=width/2,row=[];
      for(let i=0;i+width<=s.a.length;++i)row.push(best(s,previous[i],previous[i+half]));
      table.push(row);
    }
    return table;
  }
  function create(mode,a,options={}){
    if(!modes.includes(mode))throw new Error("モードが不正です。");
    if(!Array.isArray(a)||a.length>16)throw new Error("初期配列は 0〜16 要素です。");
    a.forEach(integer);
    const s={mode,a:a.slice(),order:options.order??"min",strict:options.strict??true,direction:options.direction??"previous",aggregation:options.aggregation??"concat",k:options.k??3,cursor:0,count:0,candidates:[],answers:Array(a.length).fill(null),items:[],left:0,outputs:[],front:[],back:[],transferring:false,next:0,range:null,blocks:[]};
    s.rebuildSource=null;
    if(!["min","max"].includes(s.order)||!["previous","next"].includes(s.direction)||!["sum","concat"].includes(s.aggregation)||typeof s.strict!=="boolean")throw new Error("選択肢が不正です。");
    integer(s.k);if(s.k<=0)throw new Error("窓幅 k は正の整数です。");
    if(mode==="sparse")s.table=makeTable(s);
    return s;
  }
  function logical(s){
    if(s.mode==="swag_deque")return (s.rebuildSource??s.front.slice().reverse().concat(s.back)).map(x=>x.value);
    if(s.mode==="swag")return (s.transferring?s.back.concat(s.front.slice().reverse()):s.front.slice().reverse().concat(s.back)).map(x=>x.value);
    if(s.mode==="queue"||s.mode==="window")return s.items.slice(s.left);
    if(s.mode==="stack")return s.direction==="previous"?s.a.slice(0,s.count):s.a.slice(s.a.length-s.count);
    return s.a.slice();
  }
  function fold(s){
    if(s.rebuildSource)return s.rebuildSource.reduce((value,item)=>op(s,value,item.value),identity(s));
    if(s.front.length&&s.back.length)return s.transferring?op(s,s.back.at(-1).aggregate,s.front.at(-1).aggregate):op(s,s.front.at(-1).aggregate,s.back.at(-1).aggregate);
    if(s.front.length)return s.front.at(-1).aggregate;
    if(s.back.length)return s.back.at(-1).aggregate;
    return identity(s);
  }
  function run(initial,operation){
    const s=clone(initial),request=clone(operation),frames=[];
    let result=null;
    const emit=(title,detail,active=[])=>frames.push({state:clone(s),title,detail,active,result:null,stable:false});
    const allowed={sparse:["build","query"],stack:["step"],queue:["step","push","pop","query"],window:["step"],swag:["step","push","pop","query"],swag_deque:["step","push_front","push_back","pop_front","pop_back","clear","query"]};
    if(!allowed[s.mode].includes(request.type))throw new Error("このモードでは使えない操作です。");
    if(request.type==="step"&&s.cursor>=s.a.length)throw new Error("すべての入力を処理しました。初期化するともう一度試せます。");
    if(["push","push_front","push_back"].includes(request.type))integer(request.value);
    if(["pop","pop_front","pop_back"].includes(request.type)&&logical(s).length===0)throw new Error("空のキューからは削除できません。");
    if(request.type==="query"&&s.mode==="queue"&&logical(s).length===0)throw new Error("空のキューには最良値がありません。");
    if(s.mode==="sparse"&&request.type==="query"){
      integer(request.l);integer(request.r);
      if(request.l<0||request.l>request.r||request.r>s.a.length)throw new Error("0 ≤ l ≤ r ≤ n を満たしてください。");
    }
    emit("操作を開始","オレンジは参照・変更している要素です。次へ／戻るで変化を追えます。");frames[0].stable=true;
    function enqueue(value){
      const index=s.items.length;
      while(s.candidates.length&&better(s,value,s.candidates.at(-1).value)){
        const old=s.candidates.pop();
        emit(`候補 ${old.index} を後ろから取り除く`,`新しい ${value} が ${old.value} より良く、後から期限切れになるため、古い候補は不要です。`,[old.index]);
      }
      s.items.push(value);s.candidates.push({index,value});
      emit(`添字 ${index}・値 ${value} を追加`,"同値の候補は残し、最も古い添字を先に返します。",[index]);
    }
    function dequeue(){
      const expired=s.left++;
      if(s.candidates[0].index===expired){s.candidates.shift();emit(`候補 ${expired} が期限切れ`,"論理的な左端と最良候補の添字が一致したので、候補も前から取り除きます。",[expired]);}
      else emit(`論理要素 ${expired} を削除`,"この要素は候補から既に消えています。最良候補を消す必要はありません。",[expired]);
    }
    function pushSwag(value){
      const x=s.aggregation==="concat"?String(value):value,index=s.next++;
      const aggregate=s.back.length?op(s,s.back.at(-1).aggregate,x):x;
      s.back.push({index,value:x,aggregate});
      emit(`back に ${JSON.stringify(x)} を追加`,"back の累積は op(以前の積, 新しい値)。追加順に連結・集約します。",[index]);
    }
    function popSwag(){
      if(!s.front.length){
        s.transferring=true;
        while(s.back.length){
          const item=s.back.pop(),aggregate=s.front.length?op(s,item.value,s.front.at(-1).aggregate):item.value;
          s.front.push({...item,aggregate});
          emit(`${JSON.stringify(item.value)} を back → front へ`,"front の累積は op(移動した値, 以前の積)。スタックの向きが逆でも、列の順序を保ちます。",[item.index]);
        }
        s.transferring=false;
      }
      const item=s.front.pop();emit(`先頭 ${JSON.stringify(item.value)} を削除`,"front の頂上が論理キューの先頭です。残った累積はすでに用意されています。",[item.index]);
    }
    function pushDeque(value,left){
      const x=s.aggregation==="concat"?String(value):value,index=s.next++,stack=left?s.front:s.back;
      const aggregate=stack.length?(left?op(s,x,stack.at(-1).aggregate):op(s,stack.at(-1).aggregate,x)):x;
      stack.push({index,value:x,aggregate});
      emit(`${left?"先頭":"末尾"}に ${JSON.stringify(x)} を追加`,left?"front は op(新しい値, 以前の積)。":"back は op(以前の積, 新しい値)。",[index]);
    }
    function popDeque(left){
      if(!(left?s.front:s.back).length){
        const source=s.front.slice().reverse().concat(s.back),count=left?Math.ceil(source.length/2):Math.floor(source.length/2);
        s.rebuildSource=source;s.front=[];s.back=[];
        emit("半分ずつ再構築",`削除する側が空なので、front に ${count} 個、back に ${source.length-count} 個を割り当てます。`);
        const append=(item,left)=>{const stack=left?s.front:s.back,aggregate=stack.length?(left?op(s,item.value,stack.at(-1).aggregate):op(s,stack.at(-1).aggregate,item.value)):item.value;
          stack.push({...item,aggregate});emit(`${left?"front":"back"} の累積を再構築`,"元の論理列を保持しながら、左右の新しいスタックを作ります。",[item.index]);};
        for(let i=count-1;i>=0;--i)append(source[i],true);
        for(let i=count;i<source.length;++i)append(source[i],false);
        s.rebuildSource=null;emit("再構築完了","全要素を片側へ移すと、両端の交互削除で再構築が続いてしまいます。半分ずつ残すのが要点です。");
      }
      const item=(left?s.front:s.back).pop();
      emit(`${left?"先頭":"末尾"} ${JSON.stringify(item.value)} を削除`,"対応するスタックの頂上を除けば、残りの累積をそのまま使えます。",[item.index]);
    }
    if(s.mode==="sparse"){
      if(request.type==="build"){
        s.table=[];s.range=null;s.blocks=[];
        if(s.a.length){s.table.push(s.a.slice());emit("段 0: 長さ 1 の区間","各要素をそのまま格納します。",s.a.map((_,i)=>`0:${i}`));}
        for(let width=2;width<=s.a.length;width*=2){
          const previous=s.table.at(-1),row=[];
          for(let i=0;i+width<=s.a.length;++i)row.push(best(s,previous[i],previous[i+width/2]));
          s.table.push(row);const k=s.table.length-1;
          emit(`段 ${k}: 長さ ${width} の区間`,`前段の 2 つの長さ ${width/2} の区間を組み合わせます。`,row.map((_,i)=>`${k}:${i}`));
        }
      }else{
        const {l,r}=request;s.range=[l,r];s.blocks=[];
        if(l===r){result=s.order==="min"?"∞":"−∞";emit("空区間は単位元","log2(0) を計算せず、e() を返します。");}
        else{
          const k=Math.floor(Math.log2(r-l)),width=2**k;
          s.blocks=[[l,l+width],[r-width,r]];
          emit(`長さ ${r-l}: 段 ${k} を選ぶ`,`幅 ${width} の 2 区間 [${l},${l+width}) と [${r-width},${r}) で覆います。`,[`${k}:${l}`,`${k}:${r-width}`]);
          result=best(s,s.table[k][l],s.table[k][r-width]);
          emit(`prod(${l}, ${r}) = ${result}`,"重複部分の積を v とすると op(v,v)=v。min/max なので、二度使っても変わりません。",[`${k}:${l}`,`${k}:${r-width}`]);
        }
      }
    }else if(s.mode==="stack"){
      const i=s.direction==="previous"?s.cursor:s.a.length-1-s.cursor,x=s.a[i];
      while(s.candidates.length){
        const old=s.candidates.at(-1),qualifies=s.strict?better(s,old.value,x):!better(s,x,old.value);
        if(qualifies)break;
        s.candidates.pop();emit(`候補 ${old.index} を取り除く`,`${old.value} は今回の条件を満たしません。${s.strict?"厳密な比較では等値も取り除きます。":"等値は今回の条件を満たすため残ります。"}`,[old.index,i]);
      }
      result=s.candidates.length?s.candidates.at(-1).index:-1;s.answers[i]=result;
      emit(`添字 ${i} の最近傍 = ${result}`,result===-1?"該当する候補はありません。":"残った候補の末尾が最も近い適格な要素です。",result===-1?[i]:[i,result]);
      s.candidates.push({index:i,value:x});++s.cursor;++s.count;
      emit(`添字 ${i}・値 ${x} を積む`,"各要素は一度入って高々一度出ます。全体では O(n) です。",[i]);
    }else if(s.mode==="queue"||s.mode==="window"){
      if(request.type==="step"||request.type==="push"){
        if(s.mode==="window"&&s.items.length-s.left===s.k)dequeue();
        enqueue(request.type==="step"?s.a[s.cursor++]:request.value);
        if(s.mode==="window"&&s.items.length-s.left===s.k){
          const item=s.candidates[0];s.outputs.push({l:s.left,r:s.items.length,index:item.index,value:item.value});result=item.value;
          emit(`窓 [${s.left}, ${s.items.length}): 最良値 ${result}`,`最も左の最良値は添字 ${item.index}。次は窓を 1 つ右へ動かします。`,[item.index]);
        }
      }else if(request.type==="pop")dequeue();
      else{result=s.candidates[0].value;emit(`best = ${result}, index = ${s.candidates[0].index}`,"候補 deque の先頭が現在の最良値です。",[s.candidates[0].index]);}
    }else if(s.mode==="swag_deque"){
      if(request.type==="step")pushDeque(s.a[s.cursor++],false);
      else if(request.type==="push_front"||request.type==="push_back")pushDeque(request.value,request.type==="push_front");
      else if(request.type==="pop_front"||request.type==="pop_back")popDeque(request.type==="pop_front");
      else if(request.type==="clear"){s.front=[];s.back=[];emit("全要素を削除","空の積は単位元です。この状態から再び追加できます。");}
      else{result=fold(s);emit(`prod() = ${result===""?"空文字 ε":result}`,"front の積 → back の積の順に合成します。",s.front.concat(s.back).map(x=>x.index));}
    }else{
      if(request.type==="step"||request.type==="push")pushSwag(request.type==="step"?s.a[s.cursor++]:request.value);
      else if(request.type==="pop")popSwag();
      else{
        result=fold(s);
        emit(`prod() = ${result===""?"空文字 ε":result}`,"op(front の積, back の積) の順にまとめます。空なら単位元を返します。",s.front.concat(s.back).map(x=>x.index));
      }
    }
    emit("操作完了",s.mode==="window"&&s.outputs.length===0?`まだ幅 ${s.k} の窓がありません。` : "この状態から次の操作を続けられます。");
    const last=frames.at(-1);last.result=result;last.stable=true;
    if(result!==null){last.title=frames.at(-2)?.title??"操作完了";last.detail=frames.at(-2)?.detail??last.detail;last.active=frames.at(-2)?.active??[];}
    if(s.mode==="stack"){
      const i=s.direction==="previous"?s.cursor-1:s.a.length-s.cursor;
      last.title=`添字 ${i} の最近傍 = ${result}`;
      last.detail="答えを記録し、新しい要素を候補に積みました。次の入力へ進めます。";
    }
    return {state:s,frames,result};
  }
  return {modes,create,run,logical,fold};
});
