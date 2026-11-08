/* Pure offline teaching model. Ordered rows show logical order, not a vendor's tree. */
(function(root,factory){const api=factory();if(typeof module==="object"&&module.exports)module.exports=api;else root.OrderedStructureModel=api;})(typeof globalThis!=="undefined"?globalThis:this,function(){
  "use strict";
  const modes=["set","multiset","map","compression","statistics"],clone=x=>JSON.parse(JSON.stringify(x));
  function integer(x){if(!Number.isSafeInteger(x))throw new Error("安全に表示できる整数を入力してください。");return x===0?0:x;}
  const before=(s,x,y)=>s.reverse?x>y:x<y;
  const sort=(s,a)=>a.slice().sort((x,y)=>s.reverse?y-x:x-y);
  const unique=(s,a)=>sort(s,[...new Set(a)]);
  function create(mode,initial,options={}){
    if(!modes.includes(mode))throw new Error("モードが不正です。");
    if(!Array.isArray(initial)||initial.length>16)throw new Error("初期配列は 0〜16 要素です。");
    initial.forEach(integer);
    const universe=options.universe??initial;universe.forEach(integer);
    if(universe.length>16)throw new Error("登録候補は 16 要素までです。");
    const s={mode,reverse:options.reverse??false,initial:initial.slice(),values:[],entries:[],coordinates:[],working:initial.slice(),frequency:[],bit:[],total:0,building:false};
    if(mode==="set")s.values=unique(s,initial);
    else if(mode==="multiset")s.values=sort(s,initial);
    else if(mode==="map")s.entries=unique(s,initial).map(key=>({key,value:integer(key*10)}));
    else{
      s.coordinates=unique(s,mode==="compression"?initial:universe);
      if(mode==="statistics"){
        s.frequency=Array(s.coordinates.length).fill(0);
        for(const x of initial){const i=s.coordinates.indexOf(x);if(i<0)throw new Error("初期要素も登録候補に含めてください。");++s.frequency[i];}
        s.bit=[0,...s.frequency];s.total=initial.length;
        for(let i=1;i<s.bit.length;++i){const p=i+(i&-i);if(p<s.bit.length)s.bit[p]+=s.bit[i];}
      }
    }
    return s;
  }
  function run(initial,operation){
    const s=clone(initial),request=clone(operation),frames=[];
    const emit=(title,detail,active=[],bitActive=[],region=null,tone="normal")=>frames.push({state:clone(s),title,detail,active,bitActive,region,tone,result:null,stable:false});
    const allowed={set:["insert","erase","erase_all","count","predecessor","successor","pop_first","pop_last"],multiset:["insert","erase","erase_all","count","predecessor","successor","pop_first","pop_last"],map:["insert","assign","erase","lookup","bracket","predecessor","successor"],compression:["build","index","lower","upper"],statistics:["insert","erase","erase_all","count","rank","rank_le","range","kth","predecessor","successor","clear"]};
    if(!allowed[s.mode].includes(request.type))throw new Error("このモードでは使えない操作です。");
    if(!["build","pop_first","pop_last","clear"].includes(request.type))integer(request.type==="kth"?request.k:request.x);
    if(request.type==="range")integer(request.high);
    if(s.mode==="map"&&["insert","assign"].includes(request.type))integer(request.value);
    if(s.mode==="statistics"&&["insert","erase"].includes(request.type))integer(request.count??1);
    emit("操作を開始","比較に使う場所、変更する頻度、BIT のブロックを順に表示します。");frames[0].stable=true;
    const finish=(value,title,detail,active=[],bitActive=[],tone="normal")=>{
      const result={kind:request.type,value};emit(title,detail,active,bitActive,null,tone);
      frames.at(-1).result=clone(result);frames.at(-1).stable=true;
      return {state:s,frames,result};
    };
    function bound(a,x,upper=false){
      let l=0,r=a.length;
      while(l<r){
        const mid=Math.floor((l+r)/2),skip=upper?!before(s,x,a[mid]):before(s,a[mid],x);
        emit(`${upper?"upper":"lower"}_bound: 候補位置 ${mid}`,`値 ${a[mid]} を比較し、${skip?"右側":"左側"}へ探索範囲を絞ります。順序を示す概念図です。`,[mid],[],[l,r]);
        if(skip)l=mid+1;else r=mid;
      }
      return l;
    }
    function prefix(r){
      let total=0;
      for(let i=r;i>0;i-=i&-i){total=integer(total+s.bit[i]);emit(`prefix(${r}): 累計 ${total}`,`BIT[${i}]=${s.bit[i]} が覆う [${i-(i&-i)},${i}) の頻度を加えます。`,[],[i]);}
      return total;
    }
    function add(i,delta){
      s.frequency[i]+=delta;s.total+=delta;
      for(let p=i+1;p<s.bit.length;p+=p&-p){s.bit[p]=integer(s.bit[p]+delta);emit(`BIT[${p}] += ${delta}`,`座標 ${s.coordinates[i]} の頻度を変更。BIT のブロックは一つずつ更新されます。`,[i],[p]);}
    }
    function kth(k){
      if(k<0||k>=s.total)return {value:null,index:-1};
      let p=0,w=k+1,step=1;
      while(step<=s.coordinates.length/2)step*=2;
      for(;step>0;step=Math.floor(step/2)){
        const next=p+step;if(next>s.coordinates.length)continue;
        const take=s.bit[next]<w;
        emit(`幅 ${step}: ${take?"進む":"保留"}`,`残り目標 ${w} と BIT[${next}]=${s.bit[next]} を比較。目標未満ならブロックを飛ばします。`,[],[next]);
        if(take){w-=s.bit[next];p=next;}
      }
      return {value:s.coordinates[p],index:p};
    }
    const x=request.x,type=request.type;
    if(s.mode==="compression"){
      if(type==="build"){
        s.building=true;s.working=sort(s,s.initial);s.coordinates=[];
        emit("まず比較器順にソート","元の入力は保存し、作業用の列を並べ替えます。");
        for(const value of s.working){
          if(s.coordinates.at(-1)===value){emit(`重複 ${value} をまとめる`,"同じ値には同じ添字を割り当てます。",[s.coordinates.length-1]);continue;}
          s.coordinates.push(value);emit(`${value} → 添字 ${s.coordinates.length-1}`,"異なる値だけを順に登録します。",[s.coordinates.length-1]);
        }
        s.building=false;
        return finish(s.coordinates.length,"座標圧縮が完了","添字は順序を表します。元の値の距離を表すものではありません。");
      }
      const i=bound(s.coordinates,x,type==="upper"),value=type==="index"?(s.coordinates[i]===x?i:null):i;
      return finish(value,`${type}(${x}) = ${value===null?"未登録":value}`,type==="index"?"登録値の添字だけを返します。":"未登録の閾値でも境界位置を返します。末尾の位置は M です。",i<s.coordinates.length?[i]:[]);
    }
    if(s.mode==="statistics"){
      if(type==="clear"){s.frequency.fill(0);s.bit.fill(0);s.total=0;return finish(true,"個数を初期化","登録座標は保持し、頻度と BIT を 0 にしました。");}
      if(type==="kth"){
        const found=kth(request.k);
        return finish(found.value,`kth(${request.k}) = ${found.value===null?"該当なし":found.value}`,"0 始まりの k 番目は、累積和が k+1 以上になる最初の座標です。",found.index<0?[]:[found.index]);
      }
      const i=bound(s.coordinates,x);
      if(["insert","erase","erase_all","count"].includes(type)){
        const registered=i<s.coordinates.length&&s.coordinates[i]===x,current=registered?s.frequency[i]:0;
        if(type==="count")return finish(current,`count(${x}) = ${current}`,"現在の頻度です。未登録値は 0 とします。",registered?[i]:[]);
        if(type==="erase_all"){
          if(registered)add(i,-current);
          return finish(current,`${current} 個を全削除`,"登録座標自体は削除しません。",registered?[i]:[]);
        }
        const count=request.count??1;
        const valid=registered&&count>=0&&(type==="erase"?count<=current:count<=Number.MAX_SAFE_INTEGER-s.total);
        if(!valid)return finish(false,"更新できません",!registered?"挿入・削除する値は事前登録が必要です。":count<0?"個数は非負にします。":"個数不足、または図解で安全に扱える総数を超えています。内容は変更しません。",[],[],"danger");
        add(i,type==="insert"?count:-count);
        return finish(true,`${x} を ${count} 個${type==="insert"?"追加":"削除"}`,"頻度を非負に保つことで累積和の探索が使えます。",[i]);
      }
      if(type==="range"){
        const j=bound(s.coordinates,request.high),count=i<j?prefix(j)-prefix(i):0;
        return finish(count,`count_range(${x}, ${request.high}) = ${count}`,"2 つの境界位置に対する累積和の差を取ります。閾値の登録は不要です。");
      }
      const through=type==="rank_le"||(type==="predecessor"&&request.inclusive)||(type==="successor"&&!request.inclusive);
      const cut=through?bound(s.coordinates,x,true):i,rank=prefix(cut);
      if(type==="rank"||type==="rank_le")return finish(rank,`${type==="rank"?"order_of_key":"count_less_equal"}(${x}) = ${rank}`,"圧縮後の境界より前にある頻度を BIT で足します。");
      const found=kth(type==="predecessor"?rank-1:rank);
      return finish(found.value,`${type}(${x}) = ${found.value===null?"該当なし":found.value}`,"頻度の累積順位を k 番目の検索へ変換します。",found.index<0?[]:[found.index]);
    }
    const map=s.mode==="map",keys=map?s.entries.map(e=>e.key):s.values;
    if(type==="pop_first"||type==="pop_last"){
      const i=type==="pop_first"?0:keys.length-1,value=keys.length?keys[i]:null;
      if(keys.length)s.values.splice(i,1);
      return finish(value,`${type}() = ${value===null?"該当なし":value}`,"空なら何も削除しません。順序の先頭／末尾から 1 個だけ取り出します。");
    }
    const i=bound(keys,x),exists=i<keys.length&&keys[i]===x;
    if(type==="predecessor"||type==="successor"){
      let p;
      if(type==="predecessor")p=(request.inclusive?bound(keys,x,true):i)-1;
      else p=request.inclusive?i:bound(keys,x,true);
      const value=p<0||p>=keys.length?null:map?clone(s.entries[p]):keys[p];
      return finish(value,`${type}(${x})`,value===null?"該当する要素はありません。C++ の補助関数は end() を返します。":"比較器順で最も近い適格な要素です。",p>=0&&p<keys.length?[p]:[]);
    }
    if(type==="count")return finish(keys.filter(v=>v===x).length,`count(${x})`,"同値の個数です。標準 multiset の count は個数に比例する時間も必要です。");
    if(type==="erase"||type==="erase_all"){
      const count=exists?(type==="erase"?1:keys.filter(v=>v===x).length):0;
      (map?s.entries:s.values).splice(i,count);
      return finish(type==="erase"?!!count:count,type==="erase"?"1 個だけ削除":"同値を全削除",exists?`${count} 個を削除しました。`:"該当値がないため内容は変わりません。");
    }
    if(map){
      if(type==="lookup")return finish(exists?s.entries[i].value:null,`find(${x})`,"見つからなくても新しいキーを挿入しません。",exists?[i]:[]);
      if(type==="bracket"){
        if(!exists)s.entries.splice(i,0,{key:x,value:0});
        return finish(s.entries[i].value,`map[${x}]`,exists?"既存の値を返します。":"不在キーを値 0 で挿入してから返します。",[i]);
      }
      if(exists){if(type==="assign")s.entries[i].value=request.value;}
      else s.entries.splice(i,0,{key:x,value:request.value});
      return finish(type==="assign"?(exists?"updated":"inserted"):!exists,type==="assign"?"insert_or_assign":"try_emplace",exists?(type==="assign"?"既存の対応値を更新します。":"既存の対応値を保持します。新しい値は使いません。"):"新しいキーと値を追加します。",[i]);
    }
    const inserted=s.mode==="multiset"||!exists;
    if(inserted)s.values.splice(i,0,x);
    return finish(inserted,`insert(${x})`,inserted?"順序を保って追加します。":"set は同値の要素を追加せず、元の内容を保ちます。",[i]);
  }
  return {modes,create,run};
});
