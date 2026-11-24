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
    if(samples.length<(explicit?4:12)){
      // Vertical-only scan. The centre return anchors a user-confirmed wall.
      const central=group.filter(r=>Math.abs(r.p.yaw)<=3&&Math.abs(r.p.pitch)<=2.5);
      if(!explicit||!central.length||new Set(group.map(r=>Math.round(r.p.pitch/5))).size<7)return null;
      const heading=central[0].p.head,view=orientation(central,heading);
      const range=median(central.map(r=>r.r)),level=median(view.map(v=>v.origin+v.ray.r*v.cosine));
      return {heading,level,range,eps:.009+.012*range,width:12*rad,explicit:true};
    }
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

  function flatRangeRuns(group,vertical=false){
    // A wall can occupy just one side of a sweep. Look for repeatable constant-radius
    // runs locally, without combining measurements from opposite sides of a doorway.
    const rows=new Map(),ambiguous=new Set();
    for(const r of group){
      const slice=vertical?Math.round(r.p.yaw/5):Math.round(r.p.pitch/2.5);
      const angle=Math.round((vertical?r.p.pitch:r.p.yaw)/2);
      if(!rows.has(slice))rows.set(slice,new Map());
      const row=rows.get(slice);
      if(!row.has(angle))row.set(angle,[]);
      row.get(angle).push(r);
    }
    for(const row of rows.values()){
      const bins=[...row].sort((a,b)=>a[0]-b[0]).map(([k,rays])=>({angle:k*2,rays,r:median(rays.map(v=>v.r))}));
      for(let i=0;i<bins.length;i++){
        let j=i+1;
        while(j<bins.length&&bins[j].angle-bins[j-1].angle<=8&&bins[j-1].angle-bins[i].angle<30)j++;
        const last=j-1;
        if(last-i<6||bins[last].angle-bins[i].angle<30)continue;
        const window=bins.slice(i,j),r=median(window.map(b=>b.r)),tol=Math.max(.004,r*.010);
        if(window.filter(b=>Math.abs(b.r-r)<=tol).length<window.length*.85)continue;
        for(const bin of window)for(const ray of bin.rays)if(Math.abs(ray.r-r)<=tol)ambiguous.add(ray.p);
      }
    }
    return ambiguous;
  }
  // Fit a vertical slice through one bearing. A fixed pitch zero and a sloping
  // surface can give the same slope, so automatic mode preserves that slope.
  function pitchSection(column,bearing,explicit){
    const nx=Math.sin(bearing),nz=Math.cos(bearing),byPitch=new Map();
    for(const r of column){
      const key=Math.round(r.p.pitch/5);
      if(!byPitch.has(key))byPitch.set(key,[]);
      byPitch.get(key).push(r);
    }
    if(byPitch.size<7||!byPitch.has(0))return null;
    const bins=new Map();
    for(const [key,rays] of byPitch){
      const range=median(rays.map(v=>v.r)),tol=.011+.024*range;
      const core=rays.filter(v=>Math.abs(v.r-range)<=tol);
      if(core.length<rays.length*.6)continue;
      bins.set(key,{range,rays,depth:median(core.map(v=>v.p.x*nx+v.p.z*nz)),
        height:median(core.map(v=>v.p.y)),pitch:median(core.map(v=>v.p.pitch))});
    }
    const anchor=bins.get(0);
    if(!anchor||Math.abs(anchor.pitch)>2.6)return null;
    const run=[anchor];
    for(const sign of [-1,1]){
      let prev=anchor;
      for(let k=sign;bins.has(k);k+=sign){
        const b=bins.get(k);
        // A shelf edge or an opening can start on the next raster line.
        if(Math.abs(prev.range-b.range)>.014+.05*Math.min(prev.range,b.range))break;
        if(sign<0)run.unshift(b);else run.push(b);
        prev=b;
      }
    }
    const low=run[0],high=run[run.length-1],span=high.pitch-low.pitch;
    if(run.length<7||span<29||high.height-low.height<.08)return null;
    // The station key is quantized. Reject profiles assembled while turning
    // or translating beyond the travel of the transducer around its pitch pivot.
    const first=anchor.rays[0];
    for(const b of run)for(const r of b.rays){
      if(Math.cos(first.p.head-r.p.head)<Math.cos(3*rad)||Math.hypot(r.ox-first.ox,r.oz-first.oz)>.047)return null;
    }
    const slopes=[];
    for(const a of run)for(const b of run){
      if(b.pitch-a.pitch<span*.4||b.height-a.height<.04)continue;
      slopes.push((b.depth-a.depth)/(b.height-a.height));
    }
    if(!slopes.length)return null;
    const slope=median(slopes);
    if(Math.abs(slope)>.72)return null;
    const at=median(run.map(b=>b.depth-slope*(b.height-anchor.height)));
    const residual=b=>b.depth-at-slope*(b.height-anchor.height);
    const errors=run.map(residual),tol=.01+.022*anchor.range;
    if(median(errors.map(Math.abs))>tol||errors.some(v=>Math.abs(v)>2.6*tol))return null;
    if(!explicit){
      // Constant range across elevation also occurs on a cylindrical reflector.
      const ranges=run.map(b=>b.range).sort((a,b)=>a-b);
      if(ranges[ranges.length-2]-ranges[1]<.009+.014*anchor.range)return null;
      const edge=Math.max(2,Math.round(run.length*.23));
      const middle=errors.slice(edge,-edge);
      const sag=median(middle)-(median(errors.slice(0,edge))+median(errors.slice(-edge)))/2;
      // Only compensate a shallow, outward bow on the scale of the scanned arc.
      const angularSag=anchor.range*(1-Math.cos(span*rad/2));
      if(sag<Math.max(.0028+.0015*anchor.range,.12*angularSag)||sag>1.35*angularSag)return null;
    }
    return {nx,nz,slope,at,anchor,low,high,run};
  }

  function compensatePitch(group,display,uncertain,reference){
    if(group.length<7)return 0;
    const columns=new Map(),models=new Map(),head=group[0].p.head;
    for(const r of group){
      const key=Math.round(r.p.yaw/5);
      if(!columns.has(key))columns.set(key,[]);
      columns.get(key).push(r);
    }
    for(const [key,column] of columns){
      if(column.length<7)continue;
      const section=pitchSection(column,head+key*5*rad,Boolean(reference));
      if(section)models.set(key,section);
    }
    let changed=0;
    for(const [key,m] of models){
      // In a raster, seek independent support at a neighbouring bearing. A
      // vertical-only sweep can instead provide a longer single-bearing profile.
      const supported=[models.get(key-1),models.get(key+1)].some(other=>other&&
        Math.abs(other.slope-m.slope)<.24&&Math.abs(other.anchor.range-m.anchor.range)<.025+.12*m.anchor.range&&
        Math.min(other.high.pitch,m.high.pitch)-Math.max(other.low.pitch,m.low.pitch)>=24);
      if(!reference&&!supported&&!(columns.size===1&&m.high.pitch-m.low.pitch>=39))continue;
      if(reference){
        const nx=Math.sin(reference.heading),nz=Math.cos(reference.heading);
        const anchorDepth=median(m.anchor.rays.map(v=>v.p.x*nx+v.p.z*nz));
        const matches=Math.abs(anchorDepth-reference.level)<.014+.025*m.anchor.range&&Math.abs(key*5)<=45;
        // A supported depth break takes priority over the horizontal fit, even
        // when a later echo happens to return to the earlier distance.
        const inside=new Set(m.run.flatMap(b=>b.rays));
        for(const r of columns.get(key))if(!matches||!inside.has(r))display[r.i]=r.p;
        if(!matches)continue;
      }
      for(const b of m.run)for(const r of b.rays){
        if(!reference&&(display[r.i]!==r.p||uncertain.has(r.p)))continue;
        if(Math.abs(r.r-b.range)>.009+.02*b.range)continue;
        let nx=m.nx,nz=m.nz,slope=m.slope,target=m.at+slope*(r.p.y-m.anchor.height);
        if(reference){
          nx=Math.sin(reference.heading);nz=Math.cos(reference.heading);slope=0;target=reference.level;
        }
        const gain=nx*r.dx+nz*r.dz-slope*r.dy;
        if(gain<.55)continue;
        const shift=(target-r.p.x*nx-r.p.z*nz)/gain;
        const limit=reference ? .025+.23*r.r : .018+.045*r.r;
        if(Math.abs(shift)>limit||r.p.y+shift*r.dy<.075)continue;
        display[r.i]={...r.p,x:r.p.x+shift*r.dx,y:r.p.y+shift*r.dy,z:r.p.z+shift*r.dz};
        if(reference)uncertain.delete(r.p);
        changed++;
      }
    }
    return changed;
  }


  // Old archives have no per-return time. Only infer travel for a contiguous
  // locked-beam run spanning multiple vehicle poses. Repeated, stable returns
  // at the same pose remain available even within such a run.
  function legacyTravelPoints(points){
    const omitted=new Set();
    for(let i=0;i<points.length;){
      const first=points[i],cmd=first.commands;
      if(!cmd||first.stationary!==undefined||!Number.isFinite(first.head)){i++;continue;}
      let j=i+1;
      while(j<points.length){
        const p=points[j];
        if(p.stationary!==undefined||!Number.isFinite(p.head)||!p.commands||
           p.commands[0]!==cmd[0]||p.commands[1]!==cmd[1])break;
        j++;
      }
      if(j-i>=4){
        const run=points.slice(i,j).map(ray),start=run[0];
        const spansTravel=new Set(run.map(r=>r.p.station)).size>=3&&run.some(r=>
          Math.hypot(r.ox-start.ox,r.oz-start.oz)>.12||Math.cos(r.p.head-start.p.head)<Math.cos(9*rad));
        if(spansTravel){
          for(let k=0;k<run.length;k++){
            const r=run[k];
            const repeat=[run[k-1],run[k+1]].some(v=>v&&Math.hypot(r.ox-v.ox,r.oz-v.oz)<=.003&&
              Math.cos(r.p.head-v.p.head)>=Math.cos(.5*rad)&&Math.abs(r.r-v.r)<=.012+.02*r.r);
            if(!repeat)omitted.add(r.p);
          }
        }
      }
      i=j;
    }
    return omitted;
  }

  function isolatedBelowFloor(points,excluded){
    const groups=new Map(),isolated=new Set();
    for(const p of points){
      if(p.pitch>=0||p.y>=-.05||excluded.has(p))continue;
      if(!groups.has(p.station))groups.set(p.station,[]);
      groups.get(p.station).push(p);
    }
    for(const group of groups.values()){
      const sorted=group.sort((a,b)=>a.distance-b.distance);
      let start=0;
      for(const p of sorted){
        const tolerance=4+.06*p.distance;
        while(sorted[start].distance<p.distance-tolerance)start++;
        let supported=false;
        for(let i=start;i<sorted.length&&sorted[i].distance<=p.distance+tolerance;i++){
          const q=sorted[i],yaw=Math.abs(q.yaw-p.yaw),pitch=Math.abs(q.pitch-p.pitch);
          if(Math.max(yaw,pitch)>=1&&yaw<=7&&pitch<=7){supported=true;break;}
        }
        if(!supported)isolated.add(p);
      }
    }
    return isolated;
  }

  function resolve(points,{raw=false,legacyTravel=false}={}){
    const display=points.slice(),uncertain=new Set(),stations=new Map();
    let pitchCorrected=0,confirmed=0;
    const excluded=!raw&&legacyTravel?legacyTravelPoints(points):new Set();
    const travel=excluded.size;
    const floor=raw?new Set():isolatedBelowFloor(points,excluded),belowFloor=floor.size;
    for(const p of floor)excluded.add(p);
    if(raw)return {display,uncertain,excluded,travel,belowFloor,corrected:0,pitchCorrected,confirmed};
    points.forEach((p,i)=>{
      if(excluded.has(p))return;
      if(!Number.isFinite(p.head)||!Number.isFinite(p.distance)||p.y<.075||p.distance<2)return;
      const groupKey=(p.frontWall===true?'reference:':'auto:')+p.station;
      if(!stations.has(groupKey))stations.set(groupKey,[]);
      stations.get(groupKey).push({...ray(p),i});
    });
    for(const group of stations.values()){
      const model=solve(group,group.some(r=>r.p.frontWall===true));
      const local=flatRangeRuns(group);
      for(const p of flatRangeRuns(group,true))local.add(p);
      for(const p of local)uncertain.add(p);
      if(!model){
        pitchCorrected+=compensatePitch(group,display,uncertain,null);
        continue;
      }
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
        uncertain.delete(r.p);
      }
      pitchCorrected+=compensatePitch(group,display,uncertain,model.explicit?model:null);
      if(model.explicit)for(const r of group)if(display[r.i]===r.p&&local.has(r.p))uncertain.add(r.p);
    }
    const corrected=display.reduce((count,p,i)=>count+Number(p!==points[i]),0);
    return {display,uncertain,excluded,travel,belowFloor,corrected,pitchCorrected,confirmed};
  }
  const api={resolve,angles,validCalibration,nominal};
  root.SonarProjection=api;if(typeof module==='object'&&module.exports)module.exports=api;
})(typeof globalThis==='undefined'?this:globalThis);
