(function () {
  'use strict';
  const model=window.TreeExtensionsModel,$=id=>document.getElementById(id);
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
    drawForest(step.state);
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
  function drawForest(state){
    const box=$('forest');box.replaceChildren();if(!state)return;
    const ns='http://www.w3.org/2000/svg',svg=document.createElementNS(ns,'svg');
    svg.setAttribute('viewBox','0 0 560 330');svg.setAttribute('role','img');svg.setAttribute('aria-label','青い頂点が対象、太い辺がspineです');
    const positions=Array.from({length:state.n},(_,v)=>{const a=2*Math.PI*v/state.n-Math.PI/2;return [280+220*Math.cos(a),165+120*Math.sin(a)];});
    function element(tag,attrs,text){const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;svg.append(e);return e;}
    for(const [u,v,w]of state.edges){const [x1,y1]=positions[u],[x2,y2]=positions[v];const p=state.spine||[],i=p.indexOf(u),heavy=i>=0&&(p[i-1]===v||p[i+1]===v);element('line',{x1,y1,x2,y2,stroke:heavy?'#276ab0':'#aebdce','stroke-width':heavy?5:2});if(w!==undefined)element('text',{x:(x1+x2)/2,y:(y1+y2)/2,fill:'#253e55','font-size':14},w);}
    positions.forEach(([x,y],v)=>{const active=state.active.includes(v);element('circle',{cx:x,cy:y,r:19,fill:active?'#276ab0':'#edf2f8',stroke:'#8298b2'});element('text',{x,y:y+5,'text-anchor':'middle',fill:active?'white':'#253e55','font-size':15},v);element('text',{x,y:y+36,'text-anchor':'middle',fill:'#253e55','font-size':13},`値 ${state.values[v]}`);});
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
