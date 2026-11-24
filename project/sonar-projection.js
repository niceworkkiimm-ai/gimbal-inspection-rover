/* The echo supplies a range, not a bearing. Data and geometry are in metres except distance (cm). */
(function(root){
  'use strict';
  const rad=Math.PI/180;
  const nominal=[-30,30,-30,30];
  function validCalibration(v){
    return Array.isArray(v)&&v.length===4&&v.every(n=>Number.isFinite(n)&&Math.abs(n)<=85)&&
      [0,2].every(i=>Math.abs(v[i+1]-v[i])>=12&&Math.abs(v[i+1]-v[i])<=120);
  }
  function angles(yaw,pitch,v=nominal){
    const convert=(angle,i)=>Math.max(-90,Math.min(90,(v[i]+v[i+1])/2+
      Math.max(-90,Math.min(90,angle))*(v[i+1]-v[i])/60));
    return {yaw:convert(yaw,0),pitch:convert(pitch,2)};
  }
  function median(a){
    const s=a.slice().sort((x,y)=>x-y),i=s.length>>1;
    return s.length%2?s[i]:(s[i-1]+s[i])/2;
  }
  function ray(p){
    const r=p.distance*.01,h=p.head+p.yaw*rad,tilt=p.pitch*rad;
    const dx=Math.sin(h)*Math.cos(tilt),dy=Math.sin(tilt),dz=Math.cos(h)*Math.cos(tilt);
    return {p,r,dx,dy,dz,ox:p.x-r*dx,oz:p.z-r*dz};
  }
  function orientation(rays,heading){
    const nx=Math.sin(heading),nz=Math.cos(heading);
    return rays.map(ray=>{
      const cosine=Math.max(-1,Math.min(1,nx*ray.dx+nz*ray.dz));
      return {ray,cosine,incidence:Math.acos(cosine),origin:ray.ox*nx+ray.oz*nz};
    });
  }
  function solve(group,explicit){
    // Equal weight per bearing. Repeated passes at one angle supply no new angular evidence.
    const bins=new Map();
    for(const r of group){
      if(Math.abs(r.p.pitch)>6||Math.abs(r.p.yaw)>58)continue;
      const key=Math.round(r.p.yaw/3);
      if(!bins.has(key))bins.set(key,[]);
      bins.get(key).push(r);
    }
    const samples=Array.from(bins.values(),v=>v.sort((a,b)=>a.r-b.r)[v.length>>1]);
    if(samples.length<(explicit?4:12))return null;
    const head=group[0].p.head;
    let best=null,ambiguous=null;
    for(const offset of (explicit?[0]:[0,-12,12,-24,24])){
      const view=orientation(samples,head+offset*rad);
      const middle=view.filter(v=>Math.abs(v.ray.p.yaw-offset)<=6);
      if(middle.length<2)continue;
      // Anchor depth at near-normal incidence and include the displaced transducer origin.
      const level=median(middle.map(v=>v.origin+v.ray.r*v.cosine));
      const range=median(middle.map(v=>v.ray.r));
      const eps=.009+.012*range;
      if(middle.filter(v=>Math.abs(v.origin+v.ray.r*v.cosine-level)<=eps).length<middle.length*.7)continue;
      const base={heading:head+offset*rad,level,range,eps};
      if(explicit)return {...base,width:12*rad,explicit:true};
      const obs=view.filter(v=>Math.abs(v.ray.p.yaw-offset)<=52);
      if(obs.filter(v=>v.ray.p.yaw-offset<=-16).length<3||obs.filter(v=>v.ray.p.yaw-offset>=16).length<3)continue;
      const flatSpread=median(obs.map(v=>Math.abs(v.ray.r-range)));
      const far=obs.filter(v=>Math.abs(v.ray.p.yaw-offset)>=22);
      if(far.length<4)continue;
      if(flatSpread<eps&&far.filter(v=>Math.abs(v.ray.r-range)<eps).length>=far.length*.75){
        if(!ambiguous)ambiguous={...base,ambiguous:true};
        continue;
      }
      function errors(width){
        return obs.map(v=>Math.abs(v.ray.r-(level-v.origin)/Math.cos(Math.max(0,v.incidence-width))));
      }
      const centerErrors=errors(0);
      for(const degrees of [4,8,12]){
        const width=degrees*rad,err=errors(width);
        const inliers=obs.filter((v,i)=>err[i]<eps);
        if(inliers.length<obs.length*.84)continue;
        const flank=obs.map((v,i)=>({v,i})).filter(({v})=>v.incidence>=20*rad);
        const sides=[-1,1].every(sign=>flank.filter(({v,i})=>(v.ray.p.yaw-offset)*sign>0&&
          err[i]<eps&&centerErrors[i]-err[i]>eps*.55).length>=2);
        if(!sides||median(flank.map(({v})=>v.ray.r-range))<eps*1.5)continue;
        const loss=median(err)+eps*(1-inliers.length/obs.length);
        if(!best||loss<best.loss)best={...base,width,loss};
      }
    }
    return best||ambiguous;
  }

  function flatRangeRuns(group){
    // A wall can occupy just one side of a sweep. Look for repeatable constant-radius
    // runs locally, without combining measurements from opposite sides of a doorway.
    const rows=new Map(),ambiguous=new Set();
    for(const r of group){
      const pitch=Math.round(r.p.pitch/2.5),yaw=Math.round(r.p.yaw/2);
      if(!rows.has(pitch))rows.set(pitch,new Map());
      const row=rows.get(pitch);
      if(!row.has(yaw))row.set(yaw,[]);
      row.get(yaw).push(r);
    }
    for(const row of rows.values()){
      const bins=[...row].sort((a,b)=>a[0]-b[0]).map(([yaw,rays])=>({yaw:yaw*2,rays,r:median(rays.map(v=>v.r))}));
      for(let i=0;i<bins.length;i++){
        let j=i+1;
        while(j<bins.length&&bins[j].yaw-bins[j-1].yaw<=8&&bins[j-1].yaw-bins[i].yaw<30)j++;
        const last=j-1;
        if(last-i<6||bins[last].yaw-bins[i].yaw<30)continue;
        const window=bins.slice(i,j),r=median(window.map(b=>b.r)),tol=Math.max(.004,r*.010);
        if(window.filter(b=>Math.abs(b.r-r)<=tol).length<window.length*.85)continue;
        for(const bin of window)for(const ray of bin.rays)if(Math.abs(ray.r-r)<=tol)ambiguous.add(ray.p);
      }
    }
    return ambiguous;
  }
  function resolve(points,{raw=false}={}){
    const display=points.slice(),uncertain=new Set(),stations=new Map();
    let corrected=0,confirmed=0;
    if(raw)return {display,uncertain,corrected,confirmed};
    points.forEach((p,i)=>{
      if(!Number.isFinite(p.head)||!Number.isFinite(p.distance)||p.y<.075||p.distance<2)return;
      const groupKey=(p.frontWall===true?'reference:':'auto:')+p.station;
      if(!stations.has(groupKey))stations.set(groupKey,[]);
      stations.get(groupKey).push({...ray(p),i});
    });
    for(const group of stations.values()){
      const model=solve(group,group.some(r=>r.p.frontWall===true));
      const local=flatRangeRuns(group);
      for(const p of local)uncertain.add(p);
      if(!model)continue;
      if(model.explicit)confirmed++;
      for(const {ray:r,origin,cosine,incidence} of orientation(group,model.heading)){
        if(incidence>58*rad||cosine<=0)continue;
        const near=model.level-origin;
        if(near<=0)continue;
        const axisRange=near/cosine;
        if(model.ambiguous){
          if(Math.abs(r.r-model.range)<=model.eps*1.5)uncertain.add(r.p);
          continue;
        }
        const first=near/Math.cos(Math.max(0,incidence-model.width));
        const slack=model.eps;
        const consistent=model.explicit?r.r>=first-slack&&r.r<=axisRange+slack:Math.abs(r.r-first)<=slack;
        if(!consistent){
          if(model.explicit&&Math.abs(r.r-model.range)<slack&&r.r<first-slack)uncertain.add(r.p);
          continue;
        }
        // Estimated intersection of the *commanded centre ray* with the supported plane.
        // This is a surface estimate. Never substitute it for the sensor range.
        const shift=axisRange-r.r;
        if(Math.abs(shift)>.22*Math.max(.3,r.r)+slack)continue;
        display[r.i]={...r.p,x:r.p.x+shift*r.dx,y:r.p.y+shift*r.dy,z:r.p.z+shift*r.dz};
        uncertain.delete(r.p);corrected++;
      }
    }
    return {display,uncertain,corrected,confirmed};
  }
  const api={resolve,angles,validCalibration,nominal};
  root.SonarProjection=api;if(typeof module==='object'&&module.exports)module.exports=api;
})(typeof globalThis==='undefined'?this:globalThis);
