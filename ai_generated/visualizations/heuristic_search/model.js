/* Deterministic teaching models. Timer mode advances a simulated clock. */
(function(root){'use strict';
const MODES=['random','timer','annealing','beam','local'];
const names=['乱数生成器','Timer','Simulated Annealing','Beam Search','局所探索'];
const docs=['random','timer','simulated_annealing','beam_search','local_search'];
const MASK=(1n<<64n)-1n;
function defaults(mode='annealing'){return {mode,seed:mode==='annealing'?'5':'42',bound:'7',steps:mode==='beam'?'5':mode==='random'?'6':'60',scores:'8 5 2 6 9 7 3 0 4 8 5',start:mode==='local'?'0':'2',goal:'minimize',width:'3',startTemperature:'4',endTemperature:'0.05',cooling:'geometric',localMode:'best',limit:'1.9',ticks:'0 0.25 0.5 1 0.5'};}
function integer(raw,lo,hi,label){const text=String(raw).trim();if(!/^[+-]?\d+$/.test(text))throw Error(label+'は整数');const n=Number(text);if(!Number.isSafeInteger(n)||n<lo||n>hi)throw Error(label+'の範囲外');return n;}
function real(raw,lo,hi,label){if(!String(raw).trim())throw Error(label+'を指定');const n=Number(raw);if(!Number.isFinite(n)||n<lo||n>hi)throw Error(label+'の範囲外');return n;}
function uint64(raw,label){const text=String(raw).trim();if(!/^\d+$/.test(text))throw Error(label+'は非負整数');const n=BigInt(text);if(n>MASK)throw Error(label+'はuint64の範囲');return n;}
const tokens=s=>String(s).trim().split(/[\s,]+/).filter(Boolean);
class SplitMix {
 constructor(seed){this.state=seed;}
 raw(trace){const increment=this.state=(this.state+0x9e3779b97f4a7c15n)&MASK;let z=increment;
  z=((z^(z>>30n))*0xbf58476d1ce4e5b9n)&MASK;const mix1=z;
  z=((z^(z>>27n))*0x94d049bb133111ebn)&MASK;const mix2=z;
  z=z^(z>>31n);if(trace)trace({increment,mix1,mix2,value:z});return z;}
 uniform(bound){const threshold=(1n<<64n)%bound;for(;;){const x=this.raw();if(x>=threshold)return x%bound;}}
 real(){return Number(this.raw()>>11n)/2**53;}
}
function run(raw){const c={...defaults(raw.mode),...raw};if(!MODES.includes(c.mode))throw Error('未知のモード');const frames=[];
 const save=(title,description,state={},formula='')=>frames.push({title,description,formula,...structuredClone(state)});
 let input,result;
 if(c.mode==='random'){
  const seed=uint64(c.seed,'seed'),bound=uint64(c.bound,'bound');if(bound===0n)throw Error('boundは正');const count=integer(c.steps,0,16,'生成数');
  input={seed,bound,count};const random=new SplitMix(seed),outputs=[],threshold=(1n<<64n)%bound;let draws=0;
  const state=extra=>({outputs,draws,threshold,bound,...extra});save('seedから開始','SplitMix64は64ビットの状態を一つ保持します。同じseedなら同じ列です。',state({value:seed}),'state = seed');
  while(outputs.length<count){if(++draws>1000)throw Error('図解の生成回数上限');let trace;random.raw(t=>{trace=t;});
   save('状態を定数だけ進める','加算と乗算は2^64を法として計算します。',state({value:trace.increment}),'state += 0x9e3779b97f4a7c15');
   save('1回目の混合','右シフトとのXORの後に、奇数の定数を掛けます。',state({value:trace.mix1}),'z = (z XOR (z >> 30)) × 0xbf58476d1ce4e5b9');
   save('2回目の混合','さらに別のシフトと定数でビットを混合します。',state({value:trace.mix2}),'z = (z XOR (z >> 27)) × 0x94d049bb133111eb');
   const rejected=trace.value<threshold;if(!rejected)outputs.push({raw:trace.value,value:trace.value%bound,real:Number(trace.value>>11n)/2**53});
   save(rejected?'範囲の偏りを除くため棄却':'範囲へ写して採用',rejected?'threshold未満の値は捨てて、次のrawを生成します。':'残ったrawの個数はboundの倍数です。各剰余の個数が揃います。',state({value:trace.value,rejected}),'threshold = 2^64 mod bound; raw ≥ thresholdなら raw mod bound');
  }
  result={outputs,draws};save('結果',`${outputs.length}個を採用、raw生成は${draws}回です。real欄は同じrawの上位53ビットから計算した値です。`,state({value:outputs.length?outputs.at(-1).raw:seed}),'real = (raw >> 11) / 2^53');
 }else if(c.mode==='timer'){
  const limit=real(c.limit,0,20,'制限秒'),ticks=tokens(c.ticks).map(x=>real(x,0,20,'経過秒'));if(ticks.length>20)throw Error('時刻の更新は20個まで');input={limit,ticks};let elapsed=0;
  const snapshot=()=>({elapsed,remaining:Math.max(0,limit-elapsed),progress:limit===0?1:Math.min(1,elapsed/limit),expired:elapsed>=limit});
  save('計測を開始','この図では模擬時計を進めます。C++のTimerはsteady_clockの実時間を秒で測ります。',snapshot(),'elapsed = now − start');
  for(const advance of ticks){elapsed+=advance;save(`${advance}秒進める`,elapsed>=limit?'制限時刻ちょうどからexpiredがtrueになります。':'まだ時間予算が残っています。',snapshot(),'remaining = max(0, limit − elapsed)');}
  result=snapshot();save('結果','期限を超えてもremainingとprogressを範囲内に収めます。stopを確認する箇所で探索を終了します。',snapshot(),'expired ⇔ elapsed ≥ limit');
 }else{
  const scores=tokens(c.scores).map(x=>integer(x,-100,100,'評価値'));if(scores.length<1||scores.length>21)throw Error('評価値は1〜21個');
  const start=integer(c.start,0,scores.length-1,'開始位置'),steps=integer(c.steps,0,c.mode==='beam'?8:100,'反復数');
  if(!['minimize','maximize'].includes(c.goal))throw Error('最小化・最大化を指定');const better=(a,b)=>c.goal==='minimize'?a<b:a>b;
  input={scores,start,steps,goal:c.goal};const neighbors=v=>[v-1,v+1].filter(x=>x>=0&&x<scores.length);
  if(c.mode==='beam'){
   const width=integer(c.width,1,6,'幅');input.width=width;let frontier=[{pos:start,path:[start],score:scores[start]}],depth=0,evaluations=1,termination='depth_limit',parents=frontier,candidates=[];
   const state=()=>({frontier,parents,candidates,depth,evaluations});save('初期frontier','位置と経路を候補に保持し、同じ深さの評価値だけを比較します。',state(),'width = '+width);
   for(let d=0;d<steps;++d){parents=frontier;candidates=[];
    for(let p=0;p<parents.length;++p)for(const pos of neighbors(parents[p].pos)){candidates.push({pos,path:[...parents[p].path,pos],score:scores[pos],parent:p,kept:false});evaluations++;}
    if(!candidates.length){termination='exhausted';save('次の候補がない','最後の空でない完成済みfrontierを返します。',state(),'exhausted');break;}
    const order=candidates.map((x,i)=>i).sort((a,b)=>better(candidates[a].score,candidates[b].score)?-1:better(candidates[b].score,candidates[a].score)?1:a-b);
    for(const i of order.slice(0,width))candidates[i].kept=true;
    frontier=order.slice(0,width).map(i=>({...candidates[i]}));depth++;
    save(`深さ ${depth}：上位${width}候補を残す`,'同点は親の順、次に子の列挙順です。重複状態も別候補として残します。',state(),'全候補を安定ソート → 幅まで切り詰め');
   }
   result={frontier,depth,evaluations,termination};save('結果','幅で落とした候補の先に良い解があっても復活しません。最適解の保証はありません。',state(),'best = 最後に完成した深さの先頭');
  }else{
   let current=start,best=start,iterations=0,evaluations=1,accepted=0,termination='iteration_limit';
   const state=extra=>({current,best,iterations,evaluations,accepted,...extra});save('初期解',c.mode==='annealing'?'現在解と、これまでの最良解を別々に保持します。':'近傍のうち、厳密に改善する候補だけへ移ります。',state(),'score = 入力した位置の評価値');
   if(c.mode==='annealing'){
    const seed=uint64(c.seed,'seed'),startTemperature=real(c.startTemperature,0.000001,1000000,'開始温度'),endTemperature=real(c.endTemperature,0.000001,1000000,'終了温度');
    if(!['geometric','linear'].includes(c.cooling))throw Error('冷却方式を指定');input.seed=seed;input.startTemperature=startTemperature;input.endTemperature=endTemperature;const random=new SplitMix(seed);
    for(let i=0;i<steps;++i){const candidate=Math.max(0,Math.min(scores.length-1,current+(random.uniform(2n)===0n?-1:1)));evaluations++;
     const progress=steps<=1?0:i/(steps-1),temperature=c.cooling==='linear'?startTemperature*(1-progress)+endTemperature*progress:startTemperature*(endTemperature/startTemperature)**progress;
     const gain=(scores[candidate]-scores[current])*(c.goal==='maximize'?1:-1),probability=gain>=0?1:Math.exp(gain/temperature),draw=gain>=0?null:random.real(),take=gain>=0||draw<probability,from=current;
     if(take){current=candidate;accepted++;if(better(scores[current],scores[best]))best=current;}iterations++;
     save(`反復 ${iterations}：${take?'採用':'棄却'}`,gain>=0?'改善または同点は必ず採用します。乱数を追加消費しません。':`悪化幅${-gain}。確率${probability.toPrecision(4)}と乱数${draw.toPrecision(4)}を比較します。`,state({candidate,from,temperature,gain,probability,draw,take}),'P = exp(−悪化幅 / T)');
    }
   }else{
    if(!['first','best'].includes(c.localMode))throw Error('近傍の選び方を指定');
    for(let step=0;step<steps;++step){let chosen=current;const from=current,examined=[];
     for(const candidate of neighbors(current)){evaluations++;examined.push(candidate);if(better(scores[candidate],scores[chosen])){chosen=candidate;if(c.localMode==='first')break;}}
     iterations++;if(chosen===current){termination='local_optimum';save('局所最適で終了','列挙した近傍に厳密な改善がありません。同点への移動を拒否し、巡回を避けます。',state({from,examined}),'近傍に改善なし');break;}
     current=best=chosen;accepted++;save(`反復 ${iterations}：位置${chosen}へ改善`,c.localMode==='first'?'列挙順で最初の改善を採用しました。':'全近傍から最も良い改善を採用しました。同点は先の候補です。',state({from,examined}),'strict improvement');
    }
   }
   result={current,best,iterations,evaluations,accepted,termination};save('結果',termination==='local_optimum'?'この近傍に対する局所最適です。大域最適とは限りません。':'反復上限に到達。現在解から悪化した場合も、最良解を保持しています。',state(),'best score = '+scores[best]);
  }
 }
 return {config:c,input,frames,result};
}
const api={MODES,names,docs,defaults,run,SplitMix};if(typeof module!=='undefined')module.exports=api;else root.HeuristicSearchModel=api;
})(globalThis);
