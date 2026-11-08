'use strict';
const assert=require('node:assert/strict'),model=require('../../visualizations/geometry_extensions/model.js'),expected=require('../geometry_extensions/visualization_expected.json');
let checks=0,traces=0,frames=0,seed=20261028;
const rand=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const eq=(a,b)=>{assert.deepEqual(a,b);checks++;},near=(a,b)=>{assert.ok(Math.abs(a-b)<2e-5*(1+Math.abs(b)),`${a} != ${b}`);checks++;};
const trace=(id,input)=>{const before=structuredClone(input),t=model.trace(id,input);eq(input,before);traces++;frames+=t.steps.length;for(const s of t.steps){assert.ok(s.title&&s.detail&&s.scene&&s.columns&&s.rows);assert.ok(!/NaN|Infinity/.test(JSON.stringify(s.scene)));}return t;};
const result=(id,input)=>trace(id,input).result.map(Number),poly=(a,b,c,d)=>[[a,b],[c,b],[c,d],[a,d]];
const turn=(a,b,c)=>(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]),area=p=>Math.abs(p.reduce((a,q,i)=>{const r=p[(i+1)%p.length];return a+q[0]*r[1]-q[1]*r[0];},0))/2;
const gift=input=>{const p=[...new Map(input.map(q=>[q.join(','),q])).values()];if(p.length<2)return p;const start=p.reduce((a,b)=>a[0]<b[0]||(a[0]===b[0]&&a[1]<b[1])?a:b),h=[];let a=start;do{h.push(a);let b=p.find(q=>q!==a);for(const c of p){const d=turn(a,b,c);if(d<0||(d===0&&(c[0]-a[0])**2+(c[1]-a[1])**2>(b[0]-a[0])**2+(b[1]-a[1])**2))b=c;}a=b;assert.ok(h.length<=p.length);}while(a!==start);return h;};
const gridMeasure=rs=>{const cells=new Set();for(const [l,b,r,t]of rs)for(let x=l;x<r;x++)for(let y=b;y<t;y++)cells.add(`${x},${y}`);let perimeter=0;for(const cell of cells){const [x,y]=cell.split(',').map(Number);for(const [u,v]of [[1,0],[-1,0],[0,1],[0,-1]])if(!cells.has(`${x+u},${y+v}`))perimeter++;}return [cells.size,perimeter];};
for(const m of model.modes)eq(trace(m.id,structuredClone(m.sample)).result,expected[m.id]);
for(let trial=0;trial<120;trial++){
 const rs=Array.from({length:rand(5)},()=>{const xs=[rand(9)-4,rand(9)-4].sort((a,b)=>a-b),ys=[rand(9)-4,rand(9)-4].sort((a,b)=>a-b);return [xs[0],ys[0],xs[1],ys[1]];}),[a,p]=gridMeasure(rs);
 eq(result('rectangle_area',{rectangles:rs}),[a]);eq(result('rectangle_perimeter',{rectangles:rs}),[p]);eq(result('polygon_union_area',{polygons:rs.map(r=>poly(...r))}),[a]);
 const corners=Array.from({length:6},()=>[rand(6),rand(6)]),wanted=[];for(let i=1;i<=corners.length;i++)wanted.push(gridMeasure(corners.slice(0,i).map(q=>[0,0,...q]))[0]);eq(result('origin_rectangle_union',{rectangles:corners}),wanted);
 const points=Array.from({length:8},()=>[rand(9)-4,rand(9)-4]),h=gift(points),dt=trace('delaunay',{points}).meta.delaunay;near(dt.triangles.reduce((s,t)=>s+area(t.map(i=>dt.p[i])),0),area(h));
 if(h.length>2){const boundary=dt.p.filter(q=>h.some((a,i)=>{const b=h[(i+1)%h.length];return turn(a,b,q)===0&&(q[0]-a[0])*(q[0]-b[0])+(q[1]-a[1])*(q[1]-b[1])<=0;})).length;eq(dt.triangles.length,2*dt.p.length-boundary-2);}
 const vt=trace('voronoi',{points,box:[[-5,-5],[5,5]]});near(Number(vt.result[1]),100);for(let i=0;i<vt.meta.cells.length;i++)for(const q of vt.meta.cells[i]){const d=(s)=>(s[0]-q[0])**2+(s[1]-q[1])**2;assert.ok(dt.p.every(s=>d(s)+1e-7>=d(dt.p[i])));}
 if(h.length){eq(result('convex_farthest_neighbors',{points:h}),h.map(a=>Math.max(...h.map(b=>(a[0]-b[0])**2+(a[1]-b[1])**2))));}
 const triangle=[[0,0],[5,0],[0,5]],closed=points.filter(([x,y])=>x>=0&&y>=0&&x+y<=5).length,strict=points.filter(([x,y])=>x>0&&y>0&&x+y<5).length;eq(result('triangle_point_count',{points,triangle}),[closed,strict]);
 const operations=points.map(q=>['insert',q]).concat([['erase',points[0]],['insert',points[1]]]),remaining=points.slice(1).concat([points[1]]),actual=result('dynamic_convex_hull',{operations,direction:[1,2]});eq(actual[0],gift(remaining).length);eq(actual[1]+2*actual[2],Math.max(...remaining.map(q=>q[0]+2*q[1])));
 const w=1+rand(6),height=1+rand(6),x=rand(5)-2,y=rand(5)-2,box=poly(x,y,x+w,y+height);eq(result('rational_lattice',{points:box}),[(w+1)*(height+1),4]);eq(result('polygon_centroid',{points:box}),[x+w/2,y+height/2]);eq(result('polygon_triangulation',{points:box}),[2,w*height]);eq(result('convex_polygon_intersection',{a:box,b:poly(-10,-10,10,10)}),[4,w*height]);
 const r=1+rand(3),s=1+rand(3),d=rand(9),cs=[[0,0,r],[d,0,s]],overlap=result('circle_intersection_area',{circles:cs})[0],union=result('circle_union_area',{circles:cs})[0];near(overlap+union,r*r+s*s);near(result('circle_polygon_area',{circle:cs[0],points:poly(-5,-5,5,5)})[0],r*r);
 const count=d===0?(r===s?0:0):d>r+s?4:d===r+s?3:d>Math.abs(r-s)?2:d===Math.abs(r-s)?1:0;eq(result('circle_tangents',{circles:cs}),[Number(d===0&&r===s),count]);
 const centers=result('triangle_centers',{points:[[x,y],[x+w,y],[x,y+height]]});near(centers[0],Math.hypot(w,height)/2);near(centers[1],(w+height-Math.hypot(w,height))/2);eq(centers.slice(2),[x,y]);
 const q=points.slice(0,4).map(p=>p.map(BigInt)),[aa,bb,cc,dd]=q.map(p=>p.map((v,i)=>v-q[3][i])),cross=(a,b)=>a[0]*b[1]-a[1]*b[0],norm=a=>a[0]**2n+a[1]**2n,sign=n=>n>0n?1:n<0n?-1:0;eq(result('robust_predicates',{points:points.slice(0,4)}),[sign(cross(bb.map((v,i)=>v-aa[i]),cc.map((v,i)=>v-aa[i]))),sign(norm(aa)*cross(bb,cc)-norm(bb)*cross(aa,cc)+norm(cc)*cross(aa,bb))]);
}
for(let n=0;n<=4;n++){
 const points=[],edges=[],segments=[];for(let i=0;i<n;i++){const p=poly(2*i,0,2*i+1,1),base=points.length;points.push(...p);for(let j=0;j<4;j++){edges.push([base+j,base+(j+1)%4]);segments.push([p[j],p[(j+1)%4]]);}}
 eq(result('planar_embedding',{points,edges}),[n+1,4*n]);eq(result('segment_arrangement',{segments:segments.slice(0,12)}),[Math.min(n,3)*4,Math.min(n,3)*4]);
}
for(const [points,want]of [[[],[-1,0,0]],[[[1,2,3]],[0,0,0]],[[[0,0,0],[1,1,1],[2,2,2]],[1,0,0]],[[[0,0,0],[2,0,0],[0,3,0],[0,0,4]],[3,4,4]]])eq(result('geometry3d',{points}),want);
for(let k=1;k<=5;k++){const points=[];for(let x=0;x<2;x++)for(let y=0;y<2;y++)for(let z=0;z<2;z++)points.push([x*k+1,y*k+2,z*k-3]);eq(result('geometry3d',{points}),[3,12,k**3]);}
for(const points of [[],[[0,0]],[[0,0],[1,1],[2,2],[3,3]]])eq(result('polygon_triangulation',{points}),[0,0]);
eq(result('polygon_triangulation',{points:[[0,0],[2,0],[2,0],[4,0],[4,2],[0,2],[0,0]]}),[2,8]);
eq(result('rational_lattice',{points:[['1/2','1/2'],['5/2','1/2'],['5/2','5/2'],['1/2','5/2']]}),[4,4]);
eq(result('segment_arrangement',{segments:[[[0,0],[4,0]],[[1,0],[3,0]],[[2,0],[2,0]]]}),[5,4]);
for(const [id,input]of [['rectangle_area',{rectangles:[[2,0,1,1]]}],['circle_union_area',{circles:[[0,0,-1]]}],['triangle_centers',{points:[[0,0],[1,1],[2,2]]}],['planar_embedding',{points:[[0,0],[2,2],[0,2],[2,0]],edges:[[0,1],[2,3]]}],['polygon_triangulation',{points:[[0,0],[2,2],[0,2],[2,0]]}],['convex_farthest_neighbors',{points:[[0,0],[0,0]]}],['geometry3d',{points:[[0,0,33]]}]])assert.throws(()=>trace(id,input));
console.log(`geometry extensions model: ${checks} checks, ${traces} traces, ${frames} frames, 22 C++ fixtures and degenerate/error inputs passed`);
