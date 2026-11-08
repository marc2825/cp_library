(function(root){
'use strict';
const modes=[
 {id:'debug',title:'値の整形と提出時の無効化',doc:'basic_utility/debug.md',sample:{enabled:true,onlineJudge:false,maxDepth:4,maxItems:2,maxString:12,value:{kind:'tuple',items:[{kind:'string',value:'state\n'},{kind:'list',items:[{kind:'integer',value:'3'},{kind:'integer',value:'1'},{kind:'integer',value:'4'}]}]}}},
 {id:'interactive',title:'双方向通信とEOF',doc:'development_runtime/interactive.md',sample:{timeout:5,idleTimeout:2,maxBytes:100,events:[{time:0,from:'judge',stream:'stdout',text:'100\n'},{time:.1,from:'solver',stream:'stdout',text:'? 50\n'},{time:.2,from:'judge',stream:'stdout',text:'0\n'},{time:.3,from:'solver',stream:'stdout',text:'! 50\n'},{time:.4,from:'judge',stream:'stderr',text:'accepted\n'},{time:.5,from:'judge',stream:'exit',code:0},{time:.6,from:'solver',stream:'exit',code:0}]}},
 {id:'benchmark',title:'同じseedで設定を比較',doc:'development_runtime/benchmark.md',sample:{goal:'maximize',seeds:[0,1,2],configurations:[{name:'first',scores:[5,2,8]},{name:'best',scores:[8,6,9]},{name:'unstable',scores:[10,null,10]}]}}
];
const number=(x,min,max)=>typeof x==='number'&&Number.isFinite(x)&&x>=min&&x<=max;
function trace(mode,input){
 if(!input||typeof input!=='object'||Array.isArray(input))throw Error('JSONオブジェクトを指定してください。');
 const steps=[];let result;
 const step=(title,detail,values,columns=[],rows=[])=>steps.push({title,detail,values:Object.entries(values).map(([label,value])=>({label,value:String(value)})),columns,rows:rows.map(r=>r.map(String))});
 if(mode==='debug'){
  const {enabled,onlineJudge,maxDepth,maxItems,maxString}=input;
  if(typeof enabled!=='boolean'||typeof onlineJudge!=='boolean'||![maxDepth,maxItems,maxString].every(Number.isInteger)||!number(maxDepth,0,8)||!number(maxItems,0,32)||!number(maxString,0,512))throw Error('深さは0〜8、要素数0〜32、文字列長0〜512です。');
  const active=enabled&&!onlineJudge;
  step('マクロの有効条件','CP_ENABLE_DEBUGと提出用設定を確認します。',{enabled,onlineJudge,active});
  function format(value,depth){
   if(depth>maxDepth){step('深さを打ち切る','これ以上の入れ子は省略します。',{depth});return '...';}
   if(!value||typeof value!=='object')throw Error('valueはkind付きオブジェクトです。');
   let text;
   if(value.kind==='integer'){
    if(typeof value.value!=='string'||!/^[-+]?\d{1,40}$/.test(value.value))throw Error('整数は40桁以内の文字列です。');
    const x=BigInt(value.value);if(x<-(1n<<127n)||x>=(1n<<128n))throw Error('128bit整数の範囲外です。');text=String(x);
   }else if(value.kind==='boolean'){if(typeof value.value!=='boolean')throw Error('booleanの値が不正です。');text=String(value.value);}
   else if(value.kind==='string'){
    if(typeof value.value!=='string'||/[^\x00-\x7f]/.test(value.value))throw Error('図解の文字列はASCIIです。C++側は任意byteを扱えます。');
    const escaped=[...value.value.slice(0,maxString)].map(c=>c==='\n'?'\\n':c==='\r'?'\\r':c==='\t'?'\\t':c==='"'?'\\"':c==='\\'?'\\\\':c.charCodeAt(0)<32||c.charCodeAt(0)===127?'\\x'+c.charCodeAt(0).toString(16).padStart(2,'0'):c).join('');
    text='"'+escaped+(value.value.length>maxString?'...':'')+'"';
   }else if(value.kind==='tuple'||value.kind==='list'){
    if(!Array.isArray(value.items)||value.items.length>32)throw Error('itemsは最大32件です。');
    const parts=value.items.slice(0,maxItems).map(x=>format(x,depth+1));if(value.items.length>maxItems)parts.push('...');
    text=(value.kind==='tuple'?'(':'[')+parts.join(', ')+(value.kind==='tuple'?')':']');
   }else throw Error('kindはinteger / boolean / string / tuple / listです。');
   step('値を整形','型に応じた括弧・引用符・escapeを付けます。',{kind:value.kind,depth,text});return text;
  }
  result={enabled:active,output:active?format(input.value,0):'',evaluations:active?1:0};
  if(!active)step('引数を評価しない','無効マクロでは副作用も発生しません。',{evaluations:0});
 }else if(mode==='interactive'){
  const {timeout,idleTimeout,maxBytes,events}=input;
  if(!number(timeout,.001,100)||!number(idleTimeout,.001,100)||!Number.isInteger(maxBytes)||!number(maxBytes,1,10000)||!Array.isArray(events)||events.length>60)throw Error('timeout/idleTimeoutは正の100以下、maxBytesは1〜10000、eventsは60件までです。');
  let last=0,previous=0,bytes=0,status='waiting';const exits={},rows=[];
  step('2つのプロセスを接続','stdoutを相手のstdinへ送り、stderrは別に保存します。',{solver:'judge.stdinへ転送',judge:'solver.stdinへ転送',stderr:'別ログへ保存'});
  for(const e of events){
   if(!number(e.time,previous,100)||!['solver','judge'].includes(e.from)||!['stdout','stderr','exit'].includes(e.stream))throw Error('イベントは時刻順、fromはsolver/judge、streamはstdout/stderr/exitです。');
   previous=e.time;
   if(e.time>=timeout){status='timeout';break;}
   if(e.time-last>=idleTimeout){status='idle_timeout';break;}
   if(e.stream==='exit'){
    if(!Number.isInteger(e.code))throw Error('終了コードは整数です。');exits[e.from]=e.code;
    step('プロセスが終了','EOFは未送信分の転送後に相手へ伝えます。',{process:e.from,code:e.code});
    if(e.code!==0){status=e.from+'_exit';break;}
   }else{
    if(typeof e.text!=='string')throw Error('通信内容textは文字列です。');
    const size=new TextEncoder().encode(e.text).length;
    if(bytes+size>maxBytes){bytes=maxBytes;status='output_limit';break;}
    bytes+=size;last=e.time;rows.push([e.time,e.from,e.stream,e.stream==='stdout'?(e.from==='solver'?'judge.stdin':'solver.stdin'):'log',e.text]);
    step('byteを転送・記録','図解は指定時刻で到着する通信をモデル化します。',{bytes},['秒','送信元','stream','転送先','内容'],rows.map(r=>[...r]));
   }
  }
  if(status==='waiting'&&exits.solver===0&&exits.judge===0)status='ok';
  result={status,bytes};
  if(status!=='ok'&&status!=='waiting')step('セッションを停止','原因を保存し、両プロセスを終了させます。',{status,bytes});
 }else if(mode==='benchmark'){
  const {goal,seeds,configurations}=input;
  if(!['maximize','minimize'].includes(goal)||!Array.isArray(seeds)||seeds.length<1||seeds.length>30||!seeds.every(Number.isSafeInteger)||new Set(seeds).size!==seeds.length||!Array.isArray(configurations)||!configurations.length||configurations.length>10)throw Error('方向、重複のない1〜30 seed、1〜10設定を指定します。');
  const rows=[],names=new Set();let best=null;
  step('全設定で入力を共有','各seedを一度だけ生成します。',{seeds:seeds.join(', ')});
  for(const c of configurations){
   if(typeof c.name!=='string'||names.has(c.name)||!Array.isArray(c.scores)||c.scores.length!==seeds.length||!c.scores.every(x=>x===null||number(x,-1e9,1e9)))throw Error('設定名は一意、scoresはseedと同数の数値（±10億）またはnullです。');names.add(c.name);
   const values=c.scores.filter(x=>x!==null),sorted=[...values].sort((a,b)=>a-b),success=values.length;
   const row={name:c.name,success,failed:seeds.length-success,mean:success?values.reduce((a,b)=>a+b,0)/success:null,median:success?(sorted[Math.floor((success-1)/2)]+sorted[Math.floor(success/2)])/2:null,min:success?sorted[0]:null,max:success?sorted.at(-1):null,eligible:success===seeds.length};rows.push(row);
   step('設定ごとのスコアを集計','nullは失敗。成功分の統計と、全seedが揃うかを別々に記録します。',{name:c.name,eligible:row.eligible},['seed','score'],seeds.map((s,i)=>[s,c.scores[i]===null?'失敗':c.scores[i]]));
   if(row.eligible&&(!best||(goal==='maximize'?row.mean>best.mean:row.mean<best.mean)))best=row;
  }
  result={goal,seeds,configurations:rows,best:best?best.name:null};
  step('全seed成功の設定から選ぶ','同点は設定の記載順です。失敗を0点に置き換えません。',{best:result.best},['設定','成功','失敗','平均','比較対象'],rows.map(r=>[r.name,r.success,r.failed,r.mean,r.eligible]));
 }else throw Error('モードが不明です。');
 step('最終結果','入力を変えて手順と結果を比較できます。',{result:JSON.stringify(result)});return {steps,result};
}
const api={modes,trace};if(typeof module!=='undefined'&&module.exports)module.exports=api;root.DevelopmentRuntimeModel=api;
})(typeof window==='undefined'?globalThis:window);
