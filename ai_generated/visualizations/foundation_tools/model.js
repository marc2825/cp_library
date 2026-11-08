(function(root){
'use strict';
const modes=[
 {id:'big_rational',title:'有理数の正規化と四則演算',doc:'basic_utility/big_rational.md',sample:{a:'6/-8',b:'1/6',op:'+'}},
 {id:'arena',title:'Arenaのブロックと共有ノード',doc:'basic_utility/arena.md',sample:{blockSize:2,operations:[['add',10,null],['add',20,0],['add',30,0]]}},
 {id:'bit_operations',title:'固定popcountの次のmask',doc:'basic_utility/bit_operations.md',sample:{n:5,k:2,limit:16}},
 {id:'grid',title:'グリッドの回転と近傍',doc:'basic_utility/grid.md',sample:{grid:['abc','def'],operation:'rotate',turns:1,origin:[0,1],neighbors:4}},
 {id:'rollback_array',title:'配列の変更ログと巻き戻し',doc:'basic_utility/rollback_array.md',sample:{values:[10,20],operations:[['save','root'],['set',0,17],['save','branch'],['set',1,99],['rollback','root']]}},
 {id:'radix_sort',title:'安定なbyteごとの整列',doc:'basic_utility/radix_sort.md',sample:{values:['3','-1','3','0'],bits:16,signed:true}}
];
const abs=x=>x<0n?-x:x;
function integer(x){if(typeof x!=='string'||!/^[-+]?\d{1,80}$/.test(x))throw Error('整数は80桁以内の10進文字列です。');return BigInt(x);}
function gcd(a,b){a=abs(a);b=abs(b);while(b)[a,b]=[b,a%b];return a;}
function rational(text){if(typeof text!=='string')throw Error('有理数は整数またはn/dの文字列です。');const parts=text.split('/');if(parts.length>2)throw Error('slashが多すぎます。');return normalize(integer(parts[0]),parts.length===2?integer(parts[1]):1n);}
function normalize(n,d){if(d===0n)throw Error('分母が0です。');if(d<0n){n=-n;d=-d;}const g=gcd(n,d);return [n/g,d/g];}
const serialize=([n,d])=>`${n}/${d}`;
function trace(mode,input){
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください。');
 const steps=[];let result;
 const step=(title,detail,values,columns=[],rows=[])=>steps.push({title,detail,values:Object.entries(values).map(([label,value])=>({label,value:Array.isArray(value)?JSON.stringify(value):String(value)})),columns:[...columns],rows:rows.map(row=>row.map(String))});
 if(mode==='big_rational'){
  const a=rational(input.a),b=rational(input.b),[n,d]=a,[m,e]=b;
  step('各分数を正規化','分母を正にし、GCDで約分します。',{a:serialize(a),b:serialize(b)});
  let numerator,denominator;
  if(input.op==='+'||input.op==='-'){
   const g=gcd(d,e),left=d/g,right=e/g;
   numerator=n*right+(input.op==='+'?m:-m)*left;denominator=left*e;
   step('共通分母を作る','分母のGCDを先に除きます。',{g,left,right,numerator,denominator});
  }else if(input.op==='*'||input.op==='/'){
   if(input.op==='/'&&m===0n)throw Error('0では割れません。');
   const [p,q]=input.op==='*'?b:normalize(e,m),g=gcd(n,q),h=gcd(p,d);
   numerator=(n/g)*(p/h);denominator=(d/h)*(q/g);
   step('交差する項を約分','掛ける前に分子と相手の分母を約分します。',{g,h,numerator,denominator});
  }else throw Error('opは+、-、*、/です。');
  result={value:serialize(normalize(numerator,denominator))};
 }else if(mode==='arena'){
  const {blockSize,operations}=input;
  if(!Number.isInteger(blockSize)||blockSize<1||blockSize>8||!Array.isArray(operations)||operations.length>40)throw Error('blockSizeは1〜8、操作は40件までです。');
  let nodes=[];
  step('空のArena','ノードIDは0から始まります。',{size:0,capacity:0});
  for(const op of operations){
   if(!Array.isArray(op))throw Error('操作は配列です。');
   if(op[0]==='clear'&&op.length===1){nodes=[];step('全ノードを破棄','実装では逆構築順に破棄し、全ブロックを解放します。',{size:0,capacity:0});}
   else if(op[0]==='add'&&op.length===3&&Number.isSafeInteger(op[1])&&(op[2]===null||(Number.isInteger(op[2])&&op[2]>=0&&op[2]<nodes.length))){
    const id=nodes.length;nodes.push({id,block:Math.floor(id/blockSize),slot:id%blockSize,value:op[1],previous:op[2]});
    step('ノードを構築','既存ノードのIDと位置を保ち、新しいノードを追加します。',{id,capacity:Math.ceil(nodes.length/blockSize)*blockSize},['ID','block','slot','value','previous'],nodes.map(x=>[x.id,x.block,x.slot,x.value,x.previous===null?'なし':x.previous]));
   }else throw Error('操作はadd/value/既存IDまたはnull、clearです。');
  }
  result={nodes,capacity:Math.ceil(nodes.length/blockSize)*blockSize};
 }else if(mode==='bit_operations'){
  const {n,k,limit}=input;
  if(!Number.isInteger(n)||n<0||n>64||!Number.isInteger(k)||k<0||k>n||!Number.isInteger(limit)||limit<1||limit>128)throw Error('0≤k≤n≤64、limitは1〜128です。');
  let value=(1n<<BigInt(k))-1n,more=false;const masks=[];
  for(let i=0;i<limit;i++){
   masks.push(String(value));step('現在のmask','1の個数を変えずに、次に大きいmaskを探します。',{value,binary:value.toString(2).padStart(n||1,'0'),popcount:k});
   if(value===0n)break;
   const lowest=value&-value,raised=value+lowest;
   if(raised>=(1n<<64n))break;
   const next=raised|(((raised^value)/lowest)>>2n);
   step('繰り上げて下位を詰める','最下位の1を繰り上げ、残りの1を下位へ寄せます。',{lowest,raised,next});
   if(next>=(1n<<BigInt(n)))break;
   value=next;if(i===limit-1)more=true;
  }
  result={masks,more};
 }else if(mode==='grid'){
  const {grid,origin,neighbors}=input;
  if(!Array.isArray(grid)||grid.length>8||!grid.every(row=>typeof row==='string'&&/^[ -~]{0,8}$/.test(row))||grid.some(row=>row.length!==(grid[0]?.length||0)))throw Error('図解の盤面は8×8以内の矩形ASCII文字列です。');
  const h=grid.length,w=grid[0]?.length||0,near=[];
  if(origin!==null){
   if(!Array.isArray(origin)||origin.length!==2||!origin.every(Number.isInteger)||origin[0]<0||origin[0]>=h||origin[1]<0||origin[1]>=w||![4,8].includes(neighbors))throw Error('originは盤面内の座標またはnull、neighborsは4か8です。');
   const directions=neighbors===4?[[0,1],[1,0],[0,-1],[-1,0]]:[[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]];
   for(const [dr,dc]of directions){const r=origin[0]+dr,c=origin[1]+dc;if(r>=0&&r<h&&c>=0&&c<w)near.push([r,c]);}
  }
  step('入力と近傍','行・列で座標を指定し、盤面外の近傍を除きます。',{rows:h,cols:w,neighbors:near},Array.from({length:w},(_,i)=>i),grid.map(row=>[...row]));
  let transformed=[...grid];
  const transpose=rows=>Array.from({length:rows[0]?.length||0},(_,c)=>rows.map(row=>row[c]).join(''));
  if(input.operation==='transpose')transformed=transpose(transformed);
  else if(input.operation==='rotate'){
   if(!Number.isSafeInteger(input.turns))throw Error('turnsは整数です。');
   for(let i=0;i<((input.turns%4)+4)%4;i++)transformed=transpose(transformed).map(row=>[...row].reverse().join(''));
  }else throw Error('operationはrotateかtransposeです。');
  step('座標を写す','回転は時計回り90度単位、転置は行と列の交換です。',{operation:input.operation},Array.from({length:transformed[0]?.length||0},(_,i)=>i),transformed.map(row=>[...row]));
  result={transformed,neighbors:near};
 }else if(mode==='rollback_array'){
  if(!Array.isArray(input.values)||input.values.length>16||!input.values.every(Number.isSafeInteger)||!Array.isArray(input.operations)||input.operations.length>40)throw Error('値は16件、操作は40件までです。');
  const values=[...input.values],history=[],snapshots=new Map();let serial=0,epoch=0;
  const show=(title,detail)=>step(title,detail,{values:[...values],history:history.length},['変更番号','添字','旧値'],history.map(x=>[x.serial,x.index,x.old]));
  show('初期配列','変更前の値を記録します。');
  for(const op of input.operations){
   if(!Array.isArray(op))throw Error('操作は配列です。');
   if(op[0]==='save'&&op.length===2&&typeof op[1]==='string'){snapshots.set(op[1],{depth:history.length,tail:history.at(-1)?.serial||0,epoch});show('snapshotを保存','現在の履歴の位置と枝を記録します。');}
   else if(op[0]==='set'&&op.length===3&&Number.isInteger(op[1])&&op[1]>=0&&op[1]<values.length&&Number.isSafeInteger(op[2])){history.push({index:op[1],old:values[op[1]],serial:++serial});values[op[1]]=op[2];show('変更を記録','旧値をログへ積んでから、新しい値を反映します。');}
   else if(op[0]==='rollback'&&op.length===2){const s=snapshots.get(op[1]);if(!s||s.epoch!==epoch||s.depth>history.length||(s.depth&&history[s.depth-1].serial!==s.tail))throw Error('snapshotは現在の履歴の祖先ではありません。');while(history.length>s.depth){const x=history.pop();values[x.index]=x.old;show('旧値を復元','変更を逆順に取り消します。');}}
   else if(op[0]==='clear_history'&&op.length===1){history.length=0;epoch++;show('履歴を破棄','値は維持し、すべてのsnapshotを無効にします。');}
   else throw Error('操作はsave/name、set/index/value、rollback/name、clear_historyです。');
  }
  result={values,history:history.length};
 }else if(mode==='radix_sort'){
  const {bits,signed,values}=input;
  if(![8,16,32,64,128].includes(bits)||typeof signed!=='boolean'||!Array.isArray(values)||values.length>32)throw Error('幅は8/16/32/64/128、signedは真偽値、値は32件までです。');
  const width=BigInt(bits),mask=(1n<<width)-1n,signbit=1n<<(width-1n),lo=signed?-signbit:0n,hi=signed?signbit-1n:mask;
  let order=values.map((text,id)=>{const value=integer(text);if(value<lo||value>hi)throw Error('キーが指定した型の範囲外です。');return {id,value,key:(value&mask)^(signed?signbit:0n)};});
  step('キーを保存','符号付き整数は最上位bitを反転します。',{bits,signed},['元ID','値','変換キー'],order.map(x=>[x.id,x.value,x.key]));
  for(let byte=0;byte<bits/8;byte++){
   const buckets=Array.from({length:256},()=>[]);for(const item of order)buckets[Number((item.key>>BigInt(byte*8))&255n)].push(item);
   order=buckets.flat();step('1byteを安定に整列','同じbyte値の中では前の順序を保ちます。',{byte},['元ID','値','byte'],order.map(x=>[x.id,x.value,(x.key>>BigInt(byte*8))&255n]));
  }
  result={values:order.map(x=>String(x.value)),indices:order.map(x=>x.id)};
 }else throw Error('不明なモードです。');
 step('最終結果','入力を変えて境界条件や途中の状態を比較できます。',result);
 return {steps,result};
}
const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;else root.FoundationToolsModel=api;
})(typeof window==='undefined'?globalThis:window);
