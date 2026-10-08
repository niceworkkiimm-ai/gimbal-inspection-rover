/* V8.12 / 2026-09-21: original detailed gimbal model shared by live and saved views.
 * Pure display geometry; no hardware, wiring, route data or control changes.
 * Each call owns its geometry/materials so closing a saved view cannot affect the live model.
 */
(function(root){
  'use strict';
  function create(THREE){
    const VISUAL_DISTANCE_SCALE=4,PITCH_AXIS_TO_SENSOR_CM=6.5;
    const MM_TO_SCENE=VISUAL_DISTANCE_SCALE/10;
    const LOWER_BASE_HEIGHT_MM=107;
    const LOWER_BASE_BOTTOM_WIDTH_MM=197;
    const LOWER_BASE_TOP_WIDTH_MM=159;
    const LOWER_BASE_DEPTH_MM=145;
    const UPPER_BOX_WIDTH_MM=115;
    const UPPER_BOX_HEIGHT_MM=95;
    const UPPER_BOX_DEPTH_MM=115;
    const GIMBAL_HEIGHT_MM=135;
    const SENSOR_HEIGHT_ABOVE_BOX_MM=105;
    const SENSOR_AXIS_Y=(LOWER_BASE_HEIGHT_MM+UPPER_BOX_HEIGHT_MM+SENSOR_HEIGHT_ABOVE_BOX_MM)*MM_TO_SCENE;
    const machine=new THREE.Group(),shellMat=new THREE.MeshStandardMaterial({color:0x24362d,metalness:.5,roughness:.42}),panelMat=new THREE.MeshStandardMaterial({color:0x314b3e,metalness:.32,roughness:.5}),darkMat=new THREE.MeshStandardMaterial({color:0x101713,metalness:.24,roughness:.58}),metalMat=new THREE.MeshStandardMaterial({color:0x8da39a,metalness:.78,roughness:.25}),accentMat=new THREE.MeshStandardMaterial({color:0x32c885,emissive:0x0e5d3a,emissiveIntensity:.85,metalness:.18,roughness:.35}),boardMat=new THREE.MeshStandardMaterial({color:0x176c4a,metalness:.08,roughness:.55});
    const mm=value=>value*MM_TO_SCENE;
    function outline(mesh,color=0x496858){mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry,24),new THREE.LineBasicMaterial({color,transparent:true,opacity:.82})));return mesh;}
    function box(width,height,depth,material){return outline(new THREE.Mesh(new THREE.BoxGeometry(mm(width),mm(height),mm(depth)),material));}
    function trapezoidPrism(bottomWidth,topWidth,height,depth,material){
      const bw=mm(bottomWidth)/2,tw=mm(topWidth)/2,h=mm(height),d=mm(depth)/2,geometry=new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.Float32BufferAttribute([-bw,0,d,bw,0,d,bw,0,-d,-bw,0,-d,-tw,h,d,tw,h,d,tw,h,-d,-tw,h,-d],3));
      geometry.setIndex([0,3,2,0,2,1,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]);geometry.computeVertexNormals();
      return outline(new THREE.Mesh(geometry,material));
    }

    // Real-size enclosure: 107 mm trapezoid base plus the reduced 95 mm upper box.
    const lowerBase=trapezoidPrism(LOWER_BASE_BOTTOM_WIDTH_MM,LOWER_BASE_TOP_WIDTH_MM,LOWER_BASE_HEIGHT_MM,LOWER_BASE_DEPTH_MM,shellMat);machine.add(lowerBase);
    const lowerTop=box(LOWER_BASE_TOP_WIDTH_MM+4,4,LOWER_BASE_DEPTH_MM+4,metalMat);lowerTop.position.y=mm(LOWER_BASE_HEIGHT_MM);machine.add(lowerTop);
    [[-82,-55],[82,-55],[-82,55],[82,55]].forEach(([x,z])=>{const foot=box(15,5,15,darkMat);foot.position.set(mm(x),mm(2.5),mm(z));machine.add(foot);});

    const upperBottomY=LOWER_BASE_HEIGHT_MM,upperBox=box(UPPER_BOX_WIDTH_MM,UPPER_BOX_HEIGHT_MM,UPPER_BOX_DEPTH_MM,shellMat);upperBox.position.y=mm(upperBottomY+UPPER_BOX_HEIGHT_MM/2);machine.add(upperBox);
    const frontPanel=box(101,80,3,panelMat);frontPanel.position.set(0,mm(upperBottomY+UPPER_BOX_HEIGHT_MM/2),mm(UPPER_BOX_DEPTH_MM/2+1.8));machine.add(frontPanel);
    [22,47,72].forEach(y=>{const rail=box(88,3,3,metalMat);rail.position.set(0,mm(upperBottomY+y),mm(UPPER_BOX_DEPTH_MM/2+3.4));machine.add(rail);[-48,48].forEach(x=>{const screw=new THREE.Mesh(new THREE.CylinderGeometry(mm(3),mm(3),mm(2),16),metalMat);screw.rotation.x=Math.PI/2;screw.position.set(mm(x),mm(upperBottomY+y),mm(UPPER_BOX_DEPTH_MM/2+5));machine.add(screw);});});
    [-1,1].forEach(side=>{const sideRail=box(5,UPPER_BOX_HEIGHT_MM+4,UPPER_BOX_DEPTH_MM+5,metalMat);sideRail.position.set(mm(side*(UPPER_BOX_WIDTH_MM/2+3)),mm(upperBottomY+UPPER_BOX_HEIGHT_MM/2),0);machine.add(sideRail);});
    const upperCap=box(UPPER_BOX_WIDTH_MM+10,5,UPPER_BOX_DEPTH_MM+10,metalMat);upperCap.position.y=mm(upperBottomY+UPPER_BOX_HEIGHT_MM);machine.add(upperCap);

    // The yaw frame turns around Y; the sensor carrier pitches around X.
    const gimbalBaseY=mm(upperBottomY+UPPER_BOX_HEIGHT_MM),yawAssembly=new THREE.Group();yawAssembly.position.y=gimbalBaseY;machine.add(yawAssembly);
    const panDisk=new THREE.Mesh(new THREE.CylinderGeometry(mm(30),mm(34),mm(12),36),shellMat);panDisk.position.y=mm(6);yawAssembly.add(outline(panDisk));
    const panServo=box(44,34,38,darkMat);panServo.position.y=mm(29);yawAssembly.add(panServo);
    const pitchPivotAboveBoxMm=SENSOR_HEIGHT_ABOVE_BOX_MM-PITCH_AXIS_TO_SENSOR_CM*10;
    const pitchPivotLocalY=mm(pitchPivotAboveBoxMm),armHeight=GIMBAL_HEIGHT_MM-12;
    [-1,1].forEach(side=>{const arm=box(8,armHeight,20,metalMat);arm.position.set(mm(side*37),mm(10+armHeight/2),0);yawAssembly.add(arm);});
    const topBridge=box(82,8,20,metalMat);topBridge.position.y=mm(GIMBAL_HEIGHT_MM-4);yawAssembly.add(topBridge);
    const pitchAxle=new THREE.Mesh(new THREE.CylinderGeometry(mm(7),mm(7),mm(82),24),accentMat);pitchAxle.rotation.z=Math.PI/2;pitchAxle.position.y=pitchPivotLocalY;yawAssembly.add(outline(pitchAxle));
    const pitchServo=box(30,30,38,darkMat);pitchServo.position.set(mm(-43),pitchPivotLocalY,0);yawAssembly.add(pitchServo);

    const pitchAssembly=new THREE.Group();pitchAssembly.position.y=pitchPivotLocalY;yawAssembly.add(pitchAssembly);
    const sensorOffsetY=mm(PITCH_AXIS_TO_SENSOR_CM*10),sensorBoard=box(56,27,7,boardMat);sensorBoard.position.y=sensorOffsetY;pitchAssembly.add(sensorBoard);
    const sensorBracket=box(46,7,22,accentMat);sensorBracket.position.y=mm(25);pitchAssembly.add(sensorBracket);
    [-14,14].forEach(x=>{const probe=new THREE.Mesh(new THREE.CylinderGeometry(mm(8),mm(8),mm(14),24),metalMat);probe.rotation.x=Math.PI/2;probe.position.set(mm(x),sensorOffsetY,mm(10));pitchAssembly.add(outline(probe));});
    const sensorCenter=new THREE.Mesh(new THREE.SphereGeometry(mm(2.5),16,10),accentMat);sensorCenter.position.set(0,sensorOffsetY,mm(18));pitchAssembly.add(sensorCenter);
    return {machine,yawAssembly,pitchAssembly,sensorAxisY:SENSOR_AXIS_Y};
  }
  const api={create};
  if(typeof module==='object'&&module.exports)module.exports=api;
  root.GimbalModel=api;
})(typeof globalThis!=='undefined'?globalThis:this);
