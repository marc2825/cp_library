(function(){'use strict';
const M=HeuristicSearchModel,$=id=>document.getElementById(id),ids=Object.keys(M.defaults()).filter(k=>k!=='mode');
const state={mode:M.MODES.includes(location.hash.slice(1))?location.hash.slice(1):'annealing',data:null,index:0,playing:null};window.heuristicSearchState=state;
const fmt=x=>typeof x==='number'?Number(x.toPrecision(6)).toString():String(x);
const esc=x=>String(x).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function stop(){if(state.playing)clearInterval(state.playing);state.playing=null;$('play').textContent='再生';}
function read(){return {mode:state.mode,...Object.fromEntries(ids.map(id=>[id,$(id).value]))};}
function fill(c){for(const id of ids)$(id).value=c[id];}
function execute(){stop();try{state.data=M.run(read());state.index=0;$('input-error').textContent='';render();}catch(e){state.data=null;$('input-error').textContent=e.message;$('diagram').replaceChildren();$('result').textContent='';$('details').replaceChildren();for(const id of ['next','previous','finish','play'])$(id).disabled=true;}}
function mode(value){stop();state.mode=value;location.hash=value;fill(M.defaults(value));for(const e of document.querySelectorAll('[data-for]'))e.hidden=!e.dataset.for.split(' ').includes(value);for(const e of document.querySelectorAll('[data-mode]'))e.setAttribute('aria-pressed',String(e.dataset.mode===value));$('steps-label').textContent=value==='random'?'採用する生成数（16まで）':value==='beam'?'深さの上限（8まで）':'反復上限';$('doc-link').href='../../docs/heuristic_search/'+M.docs[M.MODES.indexOf(value)]+'.md';execute();}
function node(tag,attrs={},text){const e=document.createElementNS('http://www.w3.org/2000/svg',tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;$('diagram').append(e);return e;}
const text=(x,y,value,cls='')=>node('text',{x,y,class:cls},value);
const rect=(x,y,width,height,cls)=>node('rect',{x,y,width,height,rx:7,class:cls});
function table(head,rows){$('detail-head').innerHTML='<tr>'+head.map(x=>'<th>'+esc(x)+'</th>').join('')+'</tr>';$('details').innerHTML=rows.map(row=>'<tr>'+row.map(x=>'<td>'+esc(x)+'</td>').join('')+'</tr>').join('');}
function render(){if(!state.data)return;const {input,frames,result}=state.data,f=frames[state.index],mode=state.mode;const svg=$('diagram');svg.replaceChildren();svg.setAttribute('viewBox','0 0 820 340');$('mode-value').textContent=M.names[M.MODES.indexOf(mode)];$('step-title').textContent=f.title;$('description').textContent=f.description;$('formula').textContent=f.formula;$('progress').textContent=`${state.index+1} / ${frames.length}`;$('previous').disabled=state.index===0;$('next').disabled=$('finish').disabled=state.index===frames.length-1;$('play').disabled=frames.length<=1;
 $('conditions').textContent=mode==='random'?'uniformは[0,bound)、integerは[lo,hi]。図では同じrawから剰余とrealを表示します。':mode==='timer'?'単位は秒。C++はsteady_clock。時間制限はstopを確認する箇所で有効になります。':'評価関数は純粋関数、近傍は実行可能な状態を返します。終了条件と最適性の保証を区別してください。';
 if(mode==='random'){
  const hex=f.value.toString(16).padStart(16,'0');text(30,35,'64ビットの値を16進数で表示','heading');
  for(let i=0;i<16;i++){const x=30+(i%8)*94,y=65+Math.floor(i/8)*75;rect(x,y,82,55,f.rejected?'active':'kept');text(x+30,y+24,hex[i].toUpperCase(),'mono');text(x+14,y+43,parseInt(hex[i],16).toString(2).padStart(4,'0'),'mono');}
  text(30,260,'decimal: '+f.value,'mono');text(30,290,'threshold: '+f.threshold,'mono');text(30,315,`採用 ${f.outputs.length} / raw ${f.draws}`,'caption');
  $('metric-label').textContent='raw生成回数';$('metric').textContent=f.draws;$('state-label').textContent='採用数';$('state').textContent=f.outputs.length;
  $('result').textContent='結果: '+result.outputs.map(x=>x.value).join(', ');table(['raw','[0,bound)','同じrawのreal'],f.outputs.map(x=>[x.raw,x.value,fmt(x.real)]));
 }else if(mode==='timer'){
  text(30,35,'時間予算と模擬時計','heading');rect(30,95,750,40,'bar');if(f.progress>0)rect(30,95,750*f.progress,40,f.expired?'expired':'fill');
  text(30,80,`経過 ${fmt(f.elapsed)}秒 / 制限 ${fmt(input.limit)}秒`);text(30,180,`remaining = ${fmt(f.remaining)}秒`);text(30,215,`progress = ${fmt(f.progress)}`);text(30,250,`expired = ${f.expired}`,'heading');text(30,300,'0秒の制限は、開始時点からexpired = trueです。','caption');
  $('metric-label').textContent='経過秒';$('metric').textContent=fmt(f.elapsed);$('state-label').textContent='残り秒';$('state').textContent=fmt(f.remaining);$('result').textContent=`結果: elapsed=${fmt(result.elapsed)}, expired=${result.expired}`;table(['経過秒','残り秒','期限'],frames.slice(0,state.index+1).map(x=>[fmt(x.elapsed),fmt(x.remaining),x.expired]));
 }else if(mode==='beam'){
  const parents=f.parents,candidates=f.candidates,height=Math.max(340,candidates.length*44+110);svg.setAttribute('viewBox',`0 0 820 ${height}`);text(30,35,`深さ ${f.depth} / 幅 ${input.width}`,'heading');text(45,70,'親（前のfrontier）');text(485,70,'生成した子：緑を残す');
  const py=i=>105+i*44,cy=i=>105+i*44;
  for(let i=0;i<candidates.length;i++){const child=candidates[i];node('path',{d:`M 315 ${py(child.parent)+12} L 480 ${cy(i)+12}`,class:'wire'});}
  for(let i=0;i<parents.length;i++){rect(30,py(i)-2,285,32,'point');text(40,py(i)+19,`位置${parents[i].pos} / score ${parents[i].score}`);}
  for(let i=0;i<candidates.length;i++){const child=candidates[i];rect(480,cy(i)-2,310,32,child.kept?'kept':'discarded');text(490,cy(i)+19,`位置${child.pos} / score ${child.score} / ${child.kept?'保持':'除外'}`);}
  if(!candidates.length)text(485,110,'まだ子を展開していません。','caption');
  $('metric-label').textContent='評価回数';$('metric').textContent=f.evaluations;$('state-label').textContent='深さ / 候補数';$('state').textContent=f.depth+' / '+f.frontier.length;
  $('result').textContent=`結果: score=${result.frontier[0].score}, depth=${result.depth}, ${result.termination}`;table(['順位','位置','評価','経路'],f.frontier.map((x,i)=>[i+1,x.pos,x.score,x.path.join('→')]));
 }else{
  const scores=input.scores,min=Math.min(...scores),max=Math.max(...scores),px=i=>45+(scores.length===1?365:i*730/(scores.length-1)),py=v=>245-(v-min)*170/Math.max(1,max-min);
  text(30,30,`${input.goal==='minimize'?'小さい':'大きい'}評価値を目指す / 近傍は左右の位置`,'heading');node('path',{d:scores.map((v,i)=>(i?'L':'M')+' '+px(i)+' '+py(v)).join(' '),class:'curve'});
  for(let i=0;i<scores.length;i++){const y=py(scores[i]);if(i===f.best)node('circle',{cx:px(i),cy:y,r:13,class:'kept'});node('circle',{cx:px(i),cy:y,r:7,class:i===f.current?'active':'point'});text(px(i)-8,y-20,scores[i]);text(px(i)-8,275,i,'caption');}
  if(f.candidate!==undefined)text(30,310,`候補 ${f.candidate} / T ${fmt(f.temperature)} / ${f.take?'採用':'棄却'} / P ${fmt(f.probability)}`);else text(30,310,`現在 ${f.current} / 最良 ${f.best} / ${f.iterations}反復`);
  $('metric-label').textContent='評価回数';$('metric').textContent=f.evaluations;$('state-label').textContent='現在 / 最良の評価';$('state').textContent=scores[f.current]+' / '+scores[f.best];$('result').textContent=`結果: best=${result.best} (score=${scores[result.best]}), ${result.termination}`;
  table(['反復','現在位置','最良位置','採用数'],frames.slice(0,state.index+1).map(x=>[x.iterations,x.current,x.best,x.accepted]));
 }
}
for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>mode(button.dataset.mode));$('run').addEventListener('click',execute);$('basic').addEventListener('click',()=>{fill(M.defaults(state.mode));execute();});$('boundary').addEventListener('click',()=>{const c=M.defaults(state.mode);if(state.mode==='random'){c.seed='18446744073709551615';c.bound='9223372036854775809';c.steps='4';}else if(state.mode==='timer'){c.limit='0';c.ticks='0 0.1';}else if(state.mode==='beam'){c.scores='5';c.start='0';c.steps='3';}else if(state.mode==='local'){c.scores='2 2 2';c.start='1';}else{c.steps='0';}fill(c);execute();});$('previous').addEventListener('click',()=>{stop();if(state.index>0){state.index--;render();}});$('next').addEventListener('click',()=>{if(state.data&&state.index<state.data.frames.length-1){state.index++;render();}});$('finish').addEventListener('click',()=>{stop();if(state.data){state.index=state.data.frames.length-1;render();}});$('play').addEventListener('click',()=>{if(state.playing){stop();return;}if(!state.data)return;if(state.index===state.data.frames.length-1)state.index=0;$('play').textContent='停止';render();state.playing=setInterval(()=>{if(state.index>=state.data.frames.length-1){stop();return;}state.index++;render();},450);});window.addEventListener('hashchange',()=>{const value=location.hash.slice(1);if(M.MODES.includes(value)&&value!==state.mode)mode(value);});mode(state.mode);
})();
