/* Reconstruct only contiguous, nearly straight sonar strips. Coordinates in metres, Y up. */
(function (root) {
  'use strict';
  function supportedEdges(points) {
    const edges = new Map();
    const sorted = [...points.values()].sort((a,b) => a.yaw-b.yaw);
    // Five independently aimed returns must support each local line.
    for (let i=0; i+4<sorted.length; i++) {
      const window=sorted.slice(i,i+5), a=window[0], b=window[4];
      const dx=b.x-a.x, dz=b.z-a.z, length=Math.hypot(dx,dz);
      if (length<0.09 || b.yaw-a.yaw<6) continue;
      const straight=window.every((p,j)=>{
        if (!j) return true;
        const q=window[j-1];
        return p.yaw-q.yaw<=5.1 && Math.hypot(p.x-q.x,p.z-q.z)<=0.32 &&
          (p.x-q.x)*dx+(p.z-q.z)*dz>0 &&
          Math.abs((p.x-a.x)*dz-(p.z-a.z)*dx)/length<=Math.min(0.024,length*0.09);
      });
      if (straight) for (let j=1;j<5;j++) edges.set(window[j-1].yaw,[window[j-1],window[j]]);
    }
    return edges;
  }

  function build(points) {
    const stations=new Map();
    for (const p of points) {
      if (p.y<0.055 || (p.pitch<0 && p.y<0.075)) continue;
      const key=p.station||'snapshot', angle=Math.round(p.pitch);
      if (!stations.has(key)) stations.set(key,new Map());
      const rows=stations.get(key);
      if (!rows.has(angle)) rows.set(angle,new Map());
      rows.get(angle).set(p.yaw,p);
    }
    const vertices=[],uv=[],covered=new Set();
    function quad(a,b,c,d) {
      const dx=b.x-a.x,dz=b.z-a.z,len=Math.hypot(dx,dz);
      for (const p of [a,b,c,a,c,d]) {
        vertices.push(p.x,p.y,p.z);
        uv.push((p.x*dx+p.z*dz)/len*2,p.y*2);
      }
    }
    let panels=0, bands=0;
    for (const rows of stations.values()) {
      const angles=[...rows.keys()].sort((a,b)=>a-b);
      const lines=angles.map(a=>supportedEdges(rows.get(a)));
      const joined=new Set();
      for (let i=1;i<angles.length;i++) {
        if (angles[i]-angles[i-1]>6) continue;
        for (const [yaw,lo] of lines[i-1]) {
          const hi=lines[i].get(yaw);
          if (!hi || hi[1].yaw!==lo[1].yaw) continue;
          const adjacent=lo.every((p,k)=>{
            const q=hi[k], height=q.y-p.y;
            return height>0.008 && height<=0.48 && Math.hypot(q.x-p.x,q.z-p.z)<=0.045+height*0.28;
          });
          if (!adjacent) continue;
          quad(lo[0],lo[1],hi[1],hi[0]);panels++;
          joined.add(lo);joined.add(hi);
          for(const p of [...lo,...hi])covered.add(p);
        }
      }
      for (const line of lines) for (const edge of line.values()) {
        if (joined.has(edge)) continue;
        const [a,b]=edge;
        quad({...a,y:a.y-.015},{...b,y:b.y-.015},{...b,y:b.y+.015},{...a,y:a.y+.015});
        bands++;covered.add(a);covered.add(b);
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
