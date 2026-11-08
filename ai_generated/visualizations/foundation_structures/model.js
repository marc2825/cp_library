(function(root){
  'use strict';
  const mask=(1n<<64n)-1n,mod=101n;
  const modes=[
    {id:'seeded_hash',title:'seedと複合キーハッシュ',doc:'basic_utility/seeded_hash.md',sample:{seed:'42',key:{tuple:['-7',{sequence:['1','2','3']}]}}},
    {id:'hash_map',title:'Hash Mapの衝突と削除',doc:'basic_utility/hash_map.md',sample:{seed:'0',constantHash:true,operations:[['set','1','7'],['set','9','3'],['erase','1'],['get','9'],['set','17','8']]}},
    {id:'csr_graph',title:'CSRの辺順と逆arc',doc:'graph_basic/csr_graph.md',sample:{n:3,directed:false,edges:[[0,1,5],[1,1,2],[0,1,7]]}},
    {id:'monoid_catalog',title:'順序付きMonoidの集約',doc:'range_query/monoid_catalog.md',sample:{kind:'affine',values:[['2','1'],['3','4']],x:'5'}},
    {id:'polynomial_add_sum',title:'多項式の加算とmoment',doc:'range_query/polynomial_add_sum.md',sample:{values:['1','2','3','4'],coefficients:['0','0','1'],l:0,r:4}}
  ];
  function integer(x){if(typeof x!=='string'||!/^[-+]?\d{1,40}$/.test(x))throw Error('整数は40桁以内の10進文字列です。');return BigInt(x);}
  function signed(x){const v=integer(x);if(v<-(1n<<63n)||v>=(1n<<63n))throw Error('符号付き64bitの範囲です。');return v;}
  function seedValue(x){const v=integer(x);if(v<0n||v>mask)throw Error('seedはuint64の範囲です。');return v;}
  function mix(x){x=(x+0x9e3779b97f4a7c15n)&mask;x=((x^(x>>30n))*0xbf58476d1ce4e5b9n)&mask;x=((x^(x>>27n))*0x94d049bb133111ebn)&mask;return x^(x>>31n);}
  const norm=x=>(x%mod+mod)%mod;
  function inv(x){let a=norm(x),b=mod,s=1n,t=0n;while(b){const q=a/b;[a,b]=[b,a-q*b];[s,t]=[t,s-q*t];}if(a!==1n)throw Error('逆元がありません。');return norm(s);}
  const product=(a,b)=>[norm(a[0]*b[0]+a[1]*b[2]),norm(a[0]*b[1]+a[1]*b[3]),norm(a[2]*b[0]+a[3]*b[2]),norm(a[2]*b[1]+a[3]*b[3])];
  const json=x=>JSON.stringify(x,(_,v)=>typeof v==='bigint'?String(v):v);
  function trace(mode,input){
    if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください。');
    const steps=[];let result;
    const step=(title,detail,values,extra={})=>steps.push({title,detail,values:Object.entries(values).map(([label,value])=>({label,value:typeof value==='object'?json(value):String(value)})),...extra});
    if(mode==='seeded_hash'){
      const seed=seedValue(input.seed);let nodes=0;
      function hash(key,depth=0){
        if(++nodes>60||depth>8)throw Error('図解は60要素・深さ8までです。');
        if(typeof key==='string'){const value=signed(key),h=mix((value&mask)^seed);step('整数を混ぜる','seedをXORし、SplitMix64の1出力を使います。',{value,seed,hash:h});return h;}
        if(!key||typeof key!=='object')throw Error('keyは整数文字列、tuple、sequenceです。');
        const tuple=Array.isArray(key.tuple),values=tuple?key.tuple:key.sequence;
        if(!Array.isArray(values)||values.length>20)throw Error('tuple/sequenceは20要素以内です。');
        let state=mix(seed^(tuple?0x7465706c65n:0x736571n));
        for(const value of values){const child=hash(value,depth+1);state=mix(state^child);step('要素を順番に結合','順番を入れ替えると、中間状態も変わります。',{child,state});}
        state=mix(state^BigInt(values.length));step('長さを混ぜる','空の列や、末尾の要素も区別します。',{length:values.length,hash:state});return state;
      }
      result=String(hash(input.key));
    }else if(mode==='hash_map'){
      const seed=seedValue(input.seed),ops=input.operations;
      if(typeof input.constantHash!=='boolean'||!Array.isArray(ops)||ops.length>40)throw Error('constantHashは真偽値、operationsは40件以内です。');
      let slots=[],size=0,used=0;const queries=[];
      const hash=key=>input.constantHash?7n:mix((BigInt(key)&mask)^seed);
      const picture=active=>({slots:slots.map(x=>x===null||x===false?x:[...x]),active});
      function locate(key){
        if(!slots.length)return -1;let at=Number(hash(key)&BigInt(slots.length-1));
        for(let seen=0;seen<slots.length;seen++){
          step('slotを調べる','削除済みでも探索を続け、空なら不在と分かります。',{key,slot:at},picture(at));
          if(slots[at]===null)return -1;if(slots[at]!==false&&slots[at][0]===key)return at;at=(at+1)%slots.length;
        }return -1;
      }
      function rebuild(n){const old=slots;slots=Array(n).fill(null);for(const item of old)if(item){let at=Number(hash(item[0])&BigInt(n-1));while(slots[at])at=(at+1)%n;slots[at]=item;}used=size;step('テーブルを再構築','生存する要素だけを移し、削除印を除去します。',{capacity:n,size},picture(-1));}
      for(const op of ops){
        if(!Array.isArray(op)||!['set','erase','get'].includes(op[0])||op.length!==(op[0]==='set'?3:2))throw Error('操作はset/key/value、erase/key、get/keyです。');
        const key=String(signed(op[1])),at=locate(key);
        if(op[0]==='get'){queries.push(at<0?'none':slots[at][1]);step('検索結果','見つかった値を返します。',{key,result:queries.at(-1)},picture(at));}
        else if(op[0]==='erase'){if(at>=0){slots[at]=false;size--;}step('削除印を置く','後ろにある衝突キーへの探索経路を残します。',{key,erased:at>=0},picture(at));}
        else{
          const value=String(signed(op[2]));
          if(at>=0)slots[at]=[key,value];else{
            if(!slots.length)rebuild(8);else if(size===slots.length/2)rebuild(slots.length*2);else if(used>=slots.length*3/4)rebuild(slots.length);
            let pos=Number(hash(key)&BigInt(slots.length-1));while(slots[pos])pos=(pos+1)%slots.length;
            if(slots[pos]===null)used++;slots[pos]=[key,value];size++;
          }
          step('挿入・更新','生存負荷率は1/2以内に保ちます。',{key,value,size,capacity:slots.length},picture(-1));
        }
      }
      result={entries:slots.filter(Boolean).sort((a,b)=>BigInt(a[0])<BigInt(b[0])?-1:1),queries};
    }else if(mode==='csr_graph'){
      const n=input.n,edges=input.edges,directed=input.directed;
      if(!Number.isInteger(n)||n<0||n>12||typeof directed!=='boolean'||!Array.isArray(edges)||edges.length>40)throw Error('nは0〜12、edgesは40辺以内、directedは真偽値です。');
      const offsets=Array(n+1).fill(0);
      for(const edge of edges){if(!Array.isArray(edge)||edge.length!==3||!edge.every(Number.isSafeInteger)||edge[0]<0||edge[0]>=n||edge[1]<0||edge[1]>=n)throw Error('辺は[from,to,cost]で有効な頂点を指定します。');offsets[edge[0]+1]++;if(!directed)offsets[edge[1]+1]++;}
      step('次数を数える','無向辺は二方向、自己ループも2arcとして数えます。',{counts:offsets.slice(1)});
      for(let i=0;i<n;i++)offsets[i+1]+=offsets[i];
      const next=[...offsets],arcs=Array(offsets[n]).fill(null);
      step('累積和で区切る','頂点vの範囲は[offsets[v],offsets[v+1])です。',{offsets},{arcs:[...arcs]});
      edges.forEach(([u,v,w],idx)=>{const a=next[u]++;if(directed)arcs[a]=[u,v,w,idx,-1];else{const b=next[v]++;arcs[a]=[u,v,w,idx,b];arcs[b]=[v,u,w,idx,a];}step('入力順にarcを配置','idxは論理辺番号、revは平坦なarc配列の番号です。',{edge:idx,offsets},{arcs:arcs.map(a=>a&&[...a])});});
      result={offsets,arcs};
    }else if(mode==='monoid_catalog'){
      const kind=input.kind,values=input.values;
      if(!['min','max','subarray','matrix','affine','fractional','hash'].includes(kind)||!Array.isArray(values)||values.length>20)throw Error('kindはmin/max/subarray/matrix/affine/fractional/hash、valuesは20要素以内です。');
      let state=kind==='affine'?[1n,0n]:kind==='matrix'||kind==='fractional'?[1n,0n,0n,1n]:kind==='hash'?[0n,1n,0]:kind==='subarray'?[0n,0n,0n,0n,0]:[0n,0];
      step('単位元','空区間の状態から左→右へ結合します。',{state});
      for(const value of values){
        if(kind==='min'||kind==='max'){const x=signed(value);if(!state[1]||(kind==='min'?x<state[0]:x>state[0]))state=[x,1];else if(x===state[0])state[1]++;}
        else if(kind==='subarray'){const x=signed(value),[sum,prefix,suffix,best,n]=state;state=n?[sum+x,prefix>sum+x?prefix:sum+x,suffix+x>x?suffix+x:x,[best,x,suffix+x].reduce((a,b)=>a>b?a:b),n+1]:[x,x,x,x,1];}
        else if(kind==='hash'){state=[norm(state[0]*7n+signed(value)),norm(state[1]*7n),state[2]+1];}
        else{
          const length=kind==='affine'?2:4;if(!Array.isArray(value)||value.length!==length)throw Error('affineは[a,b]、matrix/fractionalは[a,b,c,d]です。');
          const a=value.map(x=>norm(signed(x)));
          if(kind==='affine')state=[norm(a[0]*state[0]),norm(a[0]*state[1]+a[1])];
          else if(kind==='fractional'){if(norm(a[0]*a[3]-a[1]*a[2])===0n)throw Error('特異な一次分数変換です。');state=product(a,state);}
          else state=product(state,a);
        }
        step('次の要素を結合','行列はA*B、関数は先に左、次に右を適用します。',{value,state});
      }
      if(kind==='min'||kind==='max')result={value:String(state[0]),count:state[1]};
      else if(kind==='subarray')result={sum:String(state[0]),best:String(state[3]),length:state[4]};
      else if(kind==='matrix')result=state.map(String);
      else if(kind==='hash')result={hash:String(state[0]),power:String(state[1]),length:state[2]};
      else if(kind==='affine')result={a:String(state[0]),b:String(state[1]),atX:String(norm(state[0]*signed(input.x)+state[1]))};
      else{const x=input.x===null?null:norm(signed(input.x)),[a,b,c,d]=state,den=x===null?c:norm(c*x+d),num=x===null?a:norm(a*x+b);result={matrix:state.map(String),atX:den===0n?'infinity':String(norm(num*inv(den)))};}
    }else if(mode==='polynomial_add_sum'){
      const {values,coefficients,l,r}=input;
      if(!Array.isArray(values)||values.length>20||!Array.isArray(coefficients)||!coefficients.length||coefficients.length>6||!Number.isInteger(l)||!Number.isInteger(r)||l<0||l>r||r>values.length)throw Error('20要素・次数5以内、0<=l<=r<=nを指定してください。');
      const a=values.map(signed),c=coefficients.map(signed),moments=c.map(()=>0n);
      for(let i=l;i<r;i++){let power=1n;for(let k=0;k<c.length;k++){moments[k]+=power;power*=BigInt(i);}}
      const before=a.slice(l,r).reduce((a,b)=>a+b,0n);
      step('座標のmomentを集約','k=0は要素数、k=1は添字の和です。',{l,r,moments,before});
      let added=0n;for(let k=0;k<c.length;k++){added+=c[k]*moments[k];step('係数との内積','係数[k]×moment[k]を和へ加算します。',{degree:k,coefficient:c[k],moment:moments[k],added});}
      for(let i=l;i<r;i++){let power=1n;for(const coefficient of c){a[i]+=coefficient*power;power*=BigInt(i);}}
      result={values:a.map(String),sum:String(a.reduce((a,b)=>a+b,0n))};
      step('区間和を更新','座標のmomentは変わりません。',{rangeSum:before+added,values:a});
    }else throw Error('未知のモードです。');
    step('最終結果','入力を変え、構造や演算の変化を比較できます。',{result});return{steps,result};
  }
  const api={modes,trace,mix};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.FoundationStructuresModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
