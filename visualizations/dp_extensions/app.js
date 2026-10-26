(function () {
  'use strict';
  const model=window.DPExtensionsModel,$=id=>document.getElementById(id);
  let state=null,index=0,timer=null;
  for(const mode of model.modes){const option=document.createElement('option');option.value=mode.id;option.textContent=mode.title;$('mode').append(option);}
  function stop(){if(timer!==null)clearInterval(timer);timer=null;$('play').textContent='再生';}
  function render(){
    if(!state)return;
    const step=state.steps[index];$('step-count').textContent=`${index+1} / ${state.steps.length}`;
    $('progress').max=state.steps.length-1;$('progress').value=index;
    $('step-title').textContent=step.title;$('detail').textContent=step.detail;$('values').replaceChildren();
    for(const value of step.values){
      const card=document.createElement('div'),label=document.createElement('span'),number=document.createElement('span');
      card.className='value-card';label.className='value-label';number.className='value-number';
      label.textContent=value.label;number.textContent=value.value;card.append(label,number);$('values').append(card);
    }
    if(step.plot)drawPlot(step.plot);else drawGraph(step.state);
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
  function drawPlot(points){
    const box=$('forest');box.replaceChildren();
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    svg.setAttribute('viewBox','0 0 560 300');svg.setAttribute('role','img');svg.setAttribute('aria-label','現在の凸関数の整数点での値');
    const low=Math.min(...points.map(p=>p[1])),high=Math.max(...points.map(p=>p[1]));
    const x=v=>45+(v-points[0][0])*480/(points.at(-1)[0]-points[0][0]||1),y=v=>250-(v-low)*210/(high-low||1);
    const element=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);return e;};
    element('line',{x1:45,y1:250,x2:525,y2:250,stroke:'#aebdce'});
    element('polyline',{points:points.map(p=>`${x(p[0])},${y(p[1])}`).join(' '),fill:'none',stroke:'#276ab0','stroke-width':3});
    for(const p of points)element('circle',{cx:x(p[0]),cy:y(p[1]),r:2.5,fill:'#276ab0'});
    element('text',{x:45,y:280,'text-anchor':'middle',fill:'#253e55'},points[0][0]);element('text',{x:525,y:280,'text-anchor':'middle',fill:'#253e55'},points.at(-1)[0]);
    element('text',{x:15,y:35,fill:'#253e55'},high);element('text',{x:15,y:250,fill:'#253e55'},low);box.append(svg);
  }
  function drawGraph(state){
    const box=$('forest');box.replaceChildren();if(!state)return;
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    svg.setAttribute('viewBox','0 0 560 330');svg.setAttribute('role','img');svg.setAttribute('aria-label','青い頂点と太い辺が現在の対象です');
    const positions=Array.from({length:state.n},(_,v)=>{const a=2*Math.PI*v/state.n-Math.PI/2;return [280+220*Math.cos(a),165+120*Math.sin(a)];});
    function element(tag,attrs,text){const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);return e;}
    const defs=element('defs',{}),marker=document.createElementNS(ns,'marker');marker.setAttribute('id','arrow');marker.setAttribute('markerWidth','8');marker.setAttribute('markerHeight','8');marker.setAttribute('refX','18');marker.setAttribute('refY','3');marker.setAttribute('orient','auto');const tip=document.createElementNS(ns,'path');tip.setAttribute('d','M0,0 L0,6 L7,3 z');tip.setAttribute('fill','#8298b2');marker.append(tip);defs.append(marker);
    const groups=new Map();
    state.edges.forEach(([u,v],id)=>{const key=`${Math.min(u,v)},${Math.max(u,v)}`;if(!groups.has(key))groups.set(key,[]);groups.get(key).push(id);});
    for(const [edgeId,edge] of state.edges.entries()){
      const [u,v,w]=edge,[x1,y1]=positions[u],[x2,y2]=positions[v],heavy=(state.selected||[]).includes(edgeId);
      const group=groups.get(`${Math.min(u,v)},${Math.max(u,v)}`),ordinal=group.indexOf(edgeId);
      const attrs={fill:'none',stroke:heavy?'#276ab0':'#aebdce','stroke-width':heavy?4:2};
      if(state.directed)attrs['marker-end']='url(#arrow)';
      let tx,ty;
      if(u===v){
        const radius=32+ordinal*18;
        element('path',{...attrs,d:`M ${x1-12} ${y1-14} C ${x1-radius} ${y1-radius*2} ${x1+radius} ${y1-radius*2} ${x1+12} ${y1-14}`});
        tx=x1;ty=y1-radius*1.5;
      }else{
        const length=Math.hypot(x2-x1,y2-y1),offset=(ordinal-(group.length-1)/2)*46*(u<v?1:-1);
        const cx=(x1+x2)/2-(y2-y1)/length*offset,cy=(y1+y2)/2+(x2-x1)/length*offset;
        element('path',{...attrs,d:`M ${x1} ${y1} Q ${cx} ${cy} ${x2} ${y2}`});
        tx=(x1+2*cx+x2)/4;ty=(y1+2*cy+y2)/4-5;
      }
      const label=edge.length===5?`e${edgeId}: [${w},${edge[3]}] c=${edge[4]}`:edge.length===4?`e${edgeId}: ${w} / label ${edge[3]}`:w===undefined?`e${edgeId}`:`e${edgeId}: ${w}`;
      element('text',{x:tx,y:ty,fill:'#253e55','font-size':12,'text-anchor':'middle'},label);
    }
    positions.forEach(([x,y],v)=>{const active=state.active.includes(v);element('circle',{cx:x,cy:y,r:19,fill:active?'#276ab0':'#edf2f8',stroke:'#8298b2'});element('text',{x,y:y+5,'text-anchor':'middle',fill:active?'white':'#253e55','font-size':15},v);element('text',{x,y:y+36,'text-anchor':'middle',fill:'#253e55','font-size':13},String(state.values[v]??''));});
    box.append(svg);
  }
  function run(){stop();try{state=model.trace($('mode').value,JSON.parse($('input').value));index=0;$('input-error').textContent='';render();}
    catch(error){$('input-error').textContent=error.message;}}
  function select(){
    stop();const selected=model.modes.find(x=>x.id===$('mode').value);
    $('title').textContent=selected.title;$('doc-link').href=`../../docs/${selected.doc}`;
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
  window.addEventListener('hashchange',fromHash);fromHash();
})();
