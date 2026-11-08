(function () {
  'use strict';
  const model=window.SearchParsingModel,$=id=>document.getElementById(id);
  let state=null,index=0,timer=null;
  for(const mode of model.modes){const option=document.createElement('option');option.value=mode.id;option.textContent=mode.title;$('mode').append(option);}
  function stop(){if(timer!==null)clearInterval(timer);timer=null;$('play').textContent='再生';}
  function render(){
    if(!state)return;
    const step=state.steps[index];$('step-count').textContent=`${index+1} / ${state.steps.length}`;
    $('progress').max=state.steps.length-1;$('progress').value=index;
    $('step-title').textContent=step.title;$('detail').textContent=step.detail;$('values').replaceChildren();
    for(const value of (step.values||[])){
      const card=document.createElement('div'),label=document.createElement('span'),number=document.createElement('span');
      card.className='value-card';label.className='value-label';number.className='value-number';
      label.textContent=value.label;number.textContent=value.value;card.append(label,number);$('values').append(card);
    }
    drawState(step);
    $('diagram').replaceChildren();
    if(step.columns.length){
      const table=document.createElement('table'),head=document.createElement('tr');
      for(const value of step.columns){const cell=document.createElement('th');cell.textContent=String(value);head.append(cell);}table.append(head);
      for(const row of step.rows){const tr=document.createElement('tr');for(const value of row){const cell=document.createElement('td');cell.textContent=String(value);tr.append(cell);}table.append(tr);}
      $('diagram').append(table);
    }
    $('result').textContent=JSON.stringify(state.result,null,2);
    $('first').disabled=$('previous').disabled=index===0;$('next').disabled=$('finish').disabled=index===state.steps.length-1;
  }
  function drawState(step){
    const box=$('forest');box.replaceChildren();const scene=step.scene||{};
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    const W=Math.max(320,Math.min(780,box.clientWidth||640)),H=Math.min(480,Math.max(320,W*0.65)),pad=34;
    svg.setAttribute('viewBox',`0 0 ${W} ${H}`);svg.setAttribute('role','img');svg.setAttribute('aria-label','探索の状態と構文木・オートマトン');
    const p=[...(scene.bounds||[]),...(scene.curves||[]).flat(),...(scene.points||[]),...(scene.polygons||[]).flat(),...(scene.segments||[]).flat()];
    for(const [x,y,r]of scene.circles||[])p.push([x-r,y-r],[x+r,y+r]);
    if(!p.length)p.push([-1,-1],[5,4]);let minX=Math.min(...p.map(p=>p[0])),maxX=Math.max(...p.map(p=>p[0])),minY=Math.min(...p.map(p=>p[1])),maxY=Math.max(...p.map(p=>p[1]));
    const sx=(W-2*pad)/Math.max(1e-15,maxX-minX),sy=(H-2*pad)/Math.max(1e-15,maxY-minY);
    const cx=(minX+maxX)/2,cy=(minY+maxY)/2,X=x=>W/2+(x-cx)*sx,Y=y=>H/2-(y-cy)*sy;
    const el=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);return e;};
    for(let i=0;i<=5;i++){const x=pad+(W-2*pad)*i/5,y=pad+(H-2*pad)*i/5;el('line',{x1:x,y1:pad,x2:x,y2:H-pad,stroke:'#e5ece9'});el('line',{x1:pad,y1:y,x2:W-pad,y2:y,stroke:'#e5ece9'});}
    for(const points of scene.curves||[])el('polyline',{points:points.map(([x,y])=>`${X(x)},${Y(y)}`).join(' '),fill:'none',stroke:'#168175','stroke-width':2.5});
    for(const cell of scene.cells||[]){el('rect',{x:X(cell.x),y:Y(cell.y+1),width:Math.max(1,sx-2),height:Math.max(1,sy-2),fill:cell.value?'#bce0d5':'#eef2f0',stroke:'#fff'});el('text',{x:X(cell.x+.5),y:Y(cell.y+.5),'text-anchor':'middle','font-size':12,fill:'#163f49'},Number(cell.value.toPrecision(3)));}
    if(minX<=0&&maxX>=0)el('line',{x1:X(0),y1:pad,x2:X(0),y2:H-pad,stroke:'#98aba6','stroke-width':1.3});
    if(minY<=0&&maxY>=0)el('line',{x1:pad,y1:Y(0),x2:W-pad,y2:Y(0),stroke:'#98aba6','stroke-width':1.3});
    const colors=['#168175','#a66c10','#3372a4','#986788','#627a34'];
    (scene.polygons||[]).forEach((poly,i)=>el(poly.length>=3?'polygon':'polyline',{points:poly.map(([x,y])=>`${X(x)},${Y(y)}`).join(' '),fill:poly.length>=3?colors[i%colors.length]:'none','fill-opacity':0.13,stroke:colors[i%colors.length],'stroke-width':2}));
    (scene.circles||[]).forEach(([x,y,r],i)=>el('circle',{cx:X(x),cy:Y(y),r:Math.max(1,r*sx),fill:colors[i%colors.length],'fill-opacity':0.08,stroke:colors[i%colors.length],'stroke-width':2}));
    for(const arc of scene.arcs||[]){const [x,y,r]=arc.circle,poly=[];const count=Math.max(2,Math.ceil((arc.right-arc.left)*20));for(let i=0;i<=count;i++){const t=arc.left+(arc.right-arc.left)*i/count;poly.push([X(x+r*Math.cos(t)),Y(y+r*Math.sin(t))]);}el('polyline',{points:poly.map(p=>p.join(',')).join(' '),fill:'none',stroke:'#c5780a','stroke-width':5});}
    for(const [a,b]of scene.segments||[])el('line',{x1:X(a[0]),y1:Y(a[1]),x2:X(b[0]),y2:Y(b[1]),stroke:'#ad6a13','stroke-width':2.5});
    (scene.points||[]).forEach(([x,y],i)=>{el('circle',{cx:X(x),cy:Y(y),r:3.5,fill:'#163f49'});if((scene.points||[]).length<=32)el('text',{x:X(x)+6,y:Y(y)-7,'font-size':12,fill:'#163f49'},i);});
    el('text',{x:12,y:H-8,'font-size':11,fill:'#597570'},`x: ${Number(minX.toPrecision(3))} … ${Number(maxX.toPrecision(3))} / y: ${Number(minY.toPrecision(3))} … ${Number(maxY.toPrecision(3))}`);box.append(svg);
  }
  function run(){stop();try{state=model.trace($('mode').value,JSON.parse($('input').value));index=0;$('input-error').textContent='';render();}
    catch(error){$('input-error').textContent=error.message;}}
  function select(){
    stop();const selected=model.modes.find(x=>x.id===$('mode').value);
    $('title').textContent=selected.title;$('input-hint').textContent=selected.principle+' 要素は0始まりです。サンプルを編集して実行してください。';$('doc-link').href=`../../docs/${selected.doc}`;
    $('input').value=JSON.stringify(selected.sample,null,2);run();
  }
  $('mode').addEventListener('change',()=>{history.replaceState(null,'',`#${$('mode').value}`);select();});
  $('run').addEventListener('click',run);$('sample').addEventListener('click',select);
  for(const [id,move] of [['first',()=>0],['previous',()=>Math.max(0,index-1)],['next',()=>Math.min(state.steps.length-1,index+1)],['finish',()=>state.steps.length-1]]){
    $(id).addEventListener('click',()=>{stop();if(state){index=move();render();}});
  }
  $('play').addEventListener('click',()=>{
    if(timer!==null){stop();return;}if(!state)return;
    if(index===state.steps.length-1)index=0;render();$('play').textContent='停止';
    timer=setInterval(()=>{index=Math.min(index+1,state.steps.length-1);render();if(index===state.steps.length-1)stop();},650);
  });
  function fromHash(){const id=location.hash.slice(1);$('mode').value=model.modes.some(x=>x.id===id)?id:model.modes[0].id;select();}
  window.addEventListener('hashchange',fromHash);window.addEventListener('resize',()=>{if(state)drawState(state.steps[index]);});fromHash();
})();
