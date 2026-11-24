/* Local sonar surface reconstruction. Metres, Y up, angles in degrees. */
(function (root) {
  'use strict';
  const RAD=Math.PI/180;
  const horizontal=(a,b)=>Math.hypot(a.x-b.x,a.z-b.z);
  function median(values) {
    if(!values.length)return 0;
    const sorted=values.slice().sort((a,b)=>a-b),mid=sorted.length>>1;
    return sorted.length%2?sorted[mid]:(sorted[mid-1]+sorted[mid])/2;
  }
  function atYaw(a,b,yaw) {
    if(yaw===a.yaw)return a;
    if(yaw===b.yaw)return b;
    const t=(yaw-a.yaw)/(b.yaw-a.yaw);
    return {x:a.x+t*(b.x-a.x),y:a.y+t*(b.y-a.y),z:a.z+t*(b.z-a.z),yaw};
  }
  function range(p){return Number.isFinite(p.distance)?p.distance*.01:1;}
  // Beam spacing grows with range. At short range, voxel thinning skips some beams.
  function closeAcross(a,b,maxAngle) {
    const angle=b.yaw-a.yaw;
    if(angle<=0||angle>maxAngle)return false;
    const footprint=Math.min(range(a),range(b))*RAD*angle;
    return horizontal(a,b)<=Math.min(.32,.035+2.35*footprint);
  }
  function rowEdges(raw) {
    // Select a measured median-range return at repeated angles, independent of arrival order.
    const groups=new Map();
    for(const p of raw){
      const bin=Math.round(p.yaw*2);
      if(!groups.has(bin))groups.set(bin,[]);
      groups.get(bin).push(p);
    }
    const sorted=Array.from(groups.values(),g=>g.sort((a,b)=>range(a)-range(b)||a.x-b.x||a.z-b.z||a.y-b.y)[g.length>>1])
      .sort((a,b)=>a.yaw-b.yaw);
    const gaps=sorted.slice(1).map((p,i)=>p.yaw-sorted[i].yaw).filter(n=>n<=12);
    const maxAngle=Math.min(13,Math.max(6.1,median(gaps)*2.5));
    const kept=sorted.filter((p,i)=>{
      const a=sorted[i-1],b=sorted[i+1];
      if(!a||!b||!closeAcross(a,b,maxAngle))return true;
      // An isolated range spike has agreeing flanks and two incompatible depth jumps.
      return closeAcross(a,p,maxAngle)||closeAcross(p,b,maxAngle);
    });
    function sameSurface(i){
      const a=kept[i-1],b=kept[i];
      if(!closeAcross(a,b,maxAngle))return false;
      // Split a sustained depth step even when its two lips happen to be close in XYZ.
      const left=kept[i-2],right=kept[i+1];
      if(!left||!right||a.yaw-left.yaw>maxAngle||right.yaw-b.yaw>maxAngle)return true;
      const slope=Math.max(Math.abs(range(a)-range(left))/(a.yaw-left.yaw),
        Math.abs(range(right)-range(b))/(right.yaw-b.yaw));
      return Math.abs(range(b)-range(a))<=.05+.018*Math.min(range(a),range(b))+2.5*slope*(b.yaw-a.yaw);
    }
    const edges=[];
    for(let start=0;start<kept.length;){
      let end=start+1;
      while(end<kept.length&&sameSurface(end))end++;
      const run=kept.slice(start,end);start=end;
      if(run.length<3)continue;
      const first=run[0],last=run.at(-1);
      if(last.yaw-first.yaw<3.8||horizontal(first,last)<.028)continue;
      const fitted=run.map((p,i)=>{
        const a=run[i-1],b=run[i+1];
        if(!a||!b)return p;
        const chord=atYaw(a,b,p.yaw);
        // Mild smoothing keeps corners and coherent curved surfaces.
        if(horizontal(p,chord)>Math.min(.024,.009+range(p)*.011))return p;
        return {...p,x:.6*p.x+.4*chord.x,y:.6*p.y+.4*chord.y,z:.6*p.z+.4*chord.z};
      });
      for(let i=1;i<run.length;i++)edges.push({a:fitted[i-1],b:fitted[i],
        sources:[run[i-1],run[i]],reach:Math.min(range(run[i-1]),range(run[i])),standalone:run.length>=5,joined:[]});
    }
    return edges;
  }
  function build(points) {
    const stations=new Map();
    for(const p of points){
      if(![p.x,p.y,p.z,p.yaw,p.pitch].every(Number.isFinite)||p.y<.055||(p.pitch<0&&p.y<.075))continue;
      const key=p.station||'snapshot';
      if(!stations.has(key))stations.set(key,new Map());
      const rows=stations.get(key),pitch=Math.round(p.pitch/2.5)*2.5;
      if(!rows.has(pitch))rows.set(pitch,[]);
      rows.get(pitch).push(p);
    }
    const vertices=[],uv=[],covered=new Set();
    let panels=0,bands=0;
    function quad(a,b,c,d) {
      // Fixed world-space UV axes avoid the texture sliding as new rays arrive.
      const useX=Math.abs(b.x-a.x)>=Math.abs(b.z-a.z);
      for(const p of [a,b,c,a,c,d]){
        vertices.push(p.x,p.y,p.z);uv.push((useX?p.x:p.z)*2,p.y*2);
      }
    }
    function compatible(lo,hi,yaw,step){
      const a=atYaw(lo.a,lo.b,yaw),b=atYaw(hi.a,hi.b,yaw),height=b.y-a.y;
      const reach=Math.min(lo.reach,hi.reach);
      const expected=Math.max(.012,reach*Math.sin(step*RAD));
      const maxHeight=step>6 ? .30 : .50;
      return height>.002&&height<=Math.min(maxHeight,.034+expected*1.7)&&
        horizontal(a,b)<=Math.min(.16,.028+.016*reach+height*.48);
    }
    function mark(edge,from,to) {
      edge.joined.push([from,to]);
      if(from<=edge.a.yaw+1e-6)covered.add(edge.sources[0]);
      if(to>=edge.b.yaw-1e-6)covered.add(edge.sources[1]);
    }
    function joinRows(lower,upper,step,between) {
      // Intersect continuous angular intervals. Staggered samples need no identical angles.
      between.sort((a,b)=>a.yaw-b.yaw);
      let i=0,j=0,mid=0;
      function contradicted(lo,hi,from,to){
        while(mid<between.length&&between[mid].yaw<from-1)mid++;
        for(let k=mid;k<between.length&&between[k].yaw<=to+1;k++){
          const p=between[k],yaw=Math.max(from,Math.min(to,p.yaw));
          const a=atYaw(lo.a,lo.b,yaw),b=atYaw(hi.a,hi.b,yaw);
          // Sparse intermediate returns can reveal a foreground object or an opening.
          if(Math.min(horizontal(a,p),horizontal(b,p))>.05+.025*range(p))return true;
        }
        return false;
      }
      while(i<lower.length&&j<upper.length){
        const lo=lower[i],hi=upper[j],from=Math.max(lo.a.yaw,hi.a.yaw),to=Math.min(lo.b.yaw,hi.b.yaw);
        if(to-from>1e-5&&compatible(lo,hi,from,step)&&compatible(lo,hi,to,step)&&
          !contradicted(lo,hi,from,to)){
          quad(atYaw(lo.a,lo.b,from),atYaw(lo.a,lo.b,to),atYaw(hi.a,hi.b,to),atYaw(hi.a,hi.b,from));
          panels++;mark(lo,from,to);mark(hi,from,to);
        }
        if(lo.b.yaw<=hi.b.yaw)i++;
        if(hi.b.yaw<=lo.b.yaw)j++;
      }
    }
    function band(edge,from,to){
      if(to-from<1e-5)return;
      const a=atYaw(edge.a,edge.b,from),b=atYaw(edge.a,edge.b,to);
      quad({...a,y:a.y-.012},{...b,y:b.y-.012},{...b,y:b.y+.012},{...a,y:a.y+.012});
      if(from===edge.a.yaw)covered.add(edge.sources[0]);
      if(to===edge.b.yaw)covered.add(edge.sources[1]);
      bands++;
    }
    for(const rows of stations.values()){
      const pitches=[...rows.keys()].sort((a,b)=>a-b);
      const lines=pitches.map(pitch=>rowEdges(rows.get(pitch)));
      let previous=-1;
      for(let i=0;i<lines.length;i++){
        if(!lines[i].length)continue;
        if(previous>=0&&pitches[i]-pitches[previous]<=10){
          const between=pitches.slice(previous+1,i).flatMap(p=>rows.get(p));
          joinRows(lines[previous],lines[i],pitches[i]-pitches[previous],between);
        }
        previous=i;
      }
      for(const line of lines)for(const edge of line){
        if(!edge.standalone)continue;
        let cursor=edge.a.yaw;
        for(const [from,to] of edge.joined.sort((a,b)=>a[0]-b[0])){
          band(edge,cursor,from);cursor=Math.max(cursor,to);
        }
        band(edge,cursor,edge.b.yaw);
      }
    }
    return {vertices,uv,panels,bands,covered};
  }

  // Deterministic, tileable stone grain for offline use.
  function stoneTexture(T) {
    const size=128, data=new Uint8Array(size*size*4);
    function noise(x,y,grid) {
      const gx=x*grid/size,gy=y*grid/size,ix=Math.floor(gx),iy=Math.floor(gy);
      function hash(i,j) {
        const h=Math.sin((i%grid)*127.1+(j%grid)*311.7+7)*43758.5453;
        return h-Math.floor(h);
      }
      const t=gx-ix,s=gy-iy;
      return (1-s)*((1-t)*hash(ix,iy)+t*hash(ix+1,iy))+s*((1-t)*hash(ix,iy+1)+t*hash(ix+1,iy+1));
    }
    for(let y=0;y<size;y++) for(let x=0;x<size;x++) {
      const broad=noise(x,y,6),grain=noise(x,y,64),vein=Math.abs(broad+0.22*noise(x,y,16)-0.56);
      const shade=85+broad*53+grain*35-30*Math.max(0,1-vein*24);
      const i=(x+y*size)*4; data[i]=shade*1.05;data[i+1]=shade;data[i+2]=shade*.94;data[i+3]=255;
    }
    const texture=new T.DataTexture(data,size,size,T.RGBAFormat);
    texture.wrapS=texture.wrapT=T.RepeatWrapping;texture.magFilter=T.LinearFilter;
    texture.generateMipmaps=true;texture.minFilter=T.LinearMipmapLinearFilter;texture.needsUpdate=true;
    return texture;
  }
  function layer(T,scale) {
    const texture=stoneTexture(T);
    const material=new T.MeshLambertMaterial({map:texture,color:0x988c83,side:T.DoubleSide,polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1});
    const mesh=new T.Mesh(new T.BufferGeometry(),material);
    mesh.name='estimated-stone-walls';
    mesh.scale.setScalar(scale);
    return {mesh, update(points) {
      const result=build(points),geometry=new T.BufferGeometry();
      geometry.setAttribute('position',new T.Float32BufferAttribute(result.vertices,3));
      geometry.setAttribute('uv',new T.Float32BufferAttribute(result.uv,2));
      geometry.computeVertexNormals();mesh.geometry.dispose();mesh.geometry=geometry;
      return result;
    }, dispose() {mesh.geometry.dispose();material.dispose();texture.dispose();}};
  }
  const api={build,layer};
  root.ScanWalls=api;
  if(typeof module==='object' && module.exports)module.exports=api;
})(typeof globalThis==='undefined'?this:globalThis);
