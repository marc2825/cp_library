(function(root){
'use strict';
const modes=[
 {id:'ci',title:'CIの検証と成果物',doc:'development/ci.md',sample:{online:true,local:'success',unionfind:'success',range_sum:'failure',suffix_array:'waiting'}},
 {id:'stress',title:'差分テストと失敗の保存',doc:'development/stress.md',sample:{compare:'tokens',cases:[{seed:40,input:'1 0\n',fast:'1\n',naive:'1'},{seed:41,input:'2 0\n',fast:'1\n',naive:'2\n'}]}},
 {id:'random',title:'seedから作るテストデータ',doc:'development/random_data.md',sample:{seed:'42',kind:'forest',n:7,components:3,m:9,bound:10}},
 {id:'bundle',title:'includeの展開と重複排除',doc:'development/bundle.md',sample:{entry:'main.cpp',files:{'main.cpp':['a.hpp','b.hpp'],'a.hpp':['common.hpp'],'b.hpp':['common.hpp'],'common.hpp':[]}}}
];
const mask=(1n<<64n)-1n;
class Random{
 constructor(seed){this.state=BigInt(seed)&mask;}
 next(){this.state=(this.state+0x9e3779b97f4a7c15n)&mask;let z=this.state;z=((z^(z>>30n))*0xbf58476d1ce4e5b9n)&mask;z=((z^(z>>27n))*0x94d049bb133111ebn)&mask;return z^(z>>31n);}
 below(n){if(!Number.isSafeInteger(n)||n<1)throw Error('乱数の範囲が不正です。');const b=BigInt(n),limit=(1n<<64n)-(1n<<64n)%b;let x;do{x=this.next();}while(x>=limit);return Number(x%b);}
 shuffle(a){for(let i=a.length-1;i>0;i--){const j=this.below(i+1);[a[i],a[j]]=[a[j],a[i]];}}
 sample(a,k){a=[...a];if(k<0||k>a.length)throw Error('選ぶ個数が範囲外です。');for(let i=0;i<k;i++){const j=i+this.below(a.length-i);[a[i],a[j]]=[a[j],a[i]];}return a.slice(0,k);}
}
const range=n=>Array.from({length:n},(_,i)=>i);
function generate(input){
 const {seed,kind,n,components:c,m,bound}=input;
 if(typeof seed!=='string'||!/^[-+]?\d{1,30}$/.test(seed)||!Number.isInteger(n)||n<0||n>24)throw Error('seedは30桁以内の整数文字列、nは0〜24です。');
 const rng=new Random(seed);let edges=[];
 if(kind==='tree'||kind==='forest'){
  if(kind==='forest'&&!(n===0&&c===0)&&(!Number.isInteger(c)||c<1||c>n))throw Error('成分数が範囲外です。');
  const labels=range(n);rng.shuffle(labels);
  for(let i=1;i<n;i++)edges.push([labels[i],labels[rng.below(i)]]);rng.shuffle(edges);
  if(kind==='forest')edges=rng.sample(edges,n-c);
 }else if(kind==='dag'){
  if(!Number.isInteger(m)||m<0||m>n*(n-1)/2)throw Error('辺数が範囲外です。');
  const order=range(n);rng.shuffle(order);for(let i=0;i<n;i++)for(let j=i+1;j<n;j++)edges.push([order[i],order[j]]);
  edges=rng.sample(edges,m);
 }else if(kind==='polygon'){
  if(!Number.isInteger(bound)||bound<1||bound>100||n<3||n>2*bound+1)throw Error('多角形は3点以上、boundは1〜100です。');
  const points=rng.sample(range(2*bound+1).map(x=>x-bound),n).sort((a,b)=>a-b).map(x=>[x,x*x]);
  const start=rng.below(n);return {points:points.slice(start).concat(points.slice(0,start))};
 }else throw Error('kindはtree / forest / dag / polygonです。');
 return {n,edges,directed:kind==='dag'};
}
function trace(mode,input){
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください。');
 const steps=[];let result;
 const step=(title,detail,values,columns=[],rows=[])=>steps.push({title,detail,values:Object.entries(values).map(([label,value])=>({label,value:String(value)})),columns,rows:rows.map(r=>r.map(String))});
 if(mode==='ci'){
  if(typeof input.online!=='boolean')throw Error('onlineはtrue / falseです。');
  const names=['local',...(input.online?['unionfind','range_sum','suffix_array']:[])];
  for(const key of names)if(!['success','failure','waiting'].includes(input[key]))throw Error('結果はsuccess / failure / waitingです。');
  step('設定と依存を解決','固定版のツールで登録済みソースと依存関係を収集します。',{tests:names.length});
  const rows=[];for(const name of names){rows.push([name,input[name]]);step('テスト結果を記録','入力の状態を使った説明モデルです。ブラウザからCIは実行しません。',{test:name,status:input[name]},['test','status'],rows.map(x=>[...x]));}
  result={status:names.some(k=>input[k]==='failure')?'failure':names.some(k=>input[k]==='waiting')?'waiting':'success',tests:names.length,artifacts:['resolved.json','result.json','docs/']};
  step('結果と文書を成果物へ','失敗も表示し、対象外のコードを検証済みにしません。',{status:result.status},['artifact'],result.artifacts.map(x=>[x]));
 }else if(mode==='stress'){
  if(!['tokens','exact'].includes(input.compare)||!Array.isArray(input.cases)||input.cases.length<1||input.cases.length>20)throw Error('compareはtokens / exact、casesは1〜20件です。');
  let executed=0;result={status:'ok',executed:0};
  const bytesTokens=s=>s.split(/[ \t\n\r\v\f]+/).filter(Boolean);
  for(const c of input.cases){
   if(!Number.isSafeInteger(c.seed)||!['input','fast','naive'].every(k=>typeof c[k]==='string'))throw Error('seedは整数、input / fast / naiveは文字列です。');
   executed++;step('同じ入力を両解へ','生成器の出力を高速解と愚直解の標準入力に渡します。',{seed:c.seed},['input'],[[c.input]]);
   const status=c.fastStatus||'ok';if(!['ok','timeout','exit'].includes(status))throw Error('fastStatusはok / timeout / exitです。');
   const same=input.compare==='exact'?c.fast===c.naive:JSON.stringify(bytesTokens(c.fast))===JSON.stringify(bytesTokens(c.naive));
   step('終了状態と出力を比較','timeoutや異常終了は出力が一致していても失敗です。',{status,same},['fast','naive'],[[c.fast,c.naive]]);
   if(status!=='ok'||!same){result={status:status==='ok'?'mismatch':'fast_failed',seed:c.seed,executed};step('失敗を保存して停止','input.txt・両解のstdout/stderr・case.jsonを保存し、入力を固定して再実行できます。',{seed:c.seed,status:result.status});break;}
   result={status:'ok',executed};
  }
 }else if(mode==='random'){
  result=generate(input);step('seedで初期化','SplitMix64と定められた抽選順で再現します。',{seed:input.seed,kind:input.kind});
  const values=result.points||result.edges;
  for(let i=0;i<values.length;i++)step(result.points?'頂点を追加':'辺を追加','共有Python APIと同じ生成結果です。',{count:i+1},result.points?['x','y']:['from','to'],values.slice(0,i+1));
 }else if(mode==='bundle'){
  const {entry,files}=input;
  if(typeof entry!=='string'||!files||typeof files!=='object'||Object.keys(files).length>30)throw Error('entryと最大30ファイルを指定します。');
  const seen=new Set(),order=[];
  function visit(name){
   if(!Object.hasOwn(files,name)||!Array.isArray(files[name])||files[name].length>30)throw Error(`依存ファイルがありません: ${name}`);
   if(seen.has(name)){step('重複を省略','pragma once付きヘッダの無条件includeをモデル化しています。',{file:name});return;}
   seen.add(name);step('ファイルを展開','依存するファイルを元のinclude位置へ展開します。',{file:name},['expanded'],[...seen].map(x=>[x]));
   for(const dep of files[name])visit(dep);order.push(name);
  }
  visit(entry);result={order,files:seen.size};
 }else throw Error('モードが不明です。');
 step('最終結果','入力を変更して、手順と結果を比較できます。',{result:JSON.stringify(result)});
 return {steps,result};
}
const api={modes,Random,generate,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;root.DevelopmentModel=api;
})(typeof window==='undefined'?globalThis:window);
