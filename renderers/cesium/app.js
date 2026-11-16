const $=id=>document.getElementById(id);
const state={visible:0,pending:0,processing:0,failures:[],capturing:false};
window.mapReady=false;
const viewer=new Cesium.Viewer('view',{animation:false,timeline:false,baseLayerPicker:false,
  geocoder:false,homeButton:false,sceneModePicker:false,navigationHelpButton:false,fullscreenButton:false,
  infoBox:false,selectionIndicator:false,baseLayer:false,globe:false,skyBox:false,skyAtmosphere:false,
  contextOptions:{webgl:{preserveDrawingBuffer:true,antialias:true}},useBrowserRecommendedResolution:false});
viewer.resolutionScale=1;viewer.clock.shouldAnimate=false;
viewer.clock.currentTime=Cesium.JulianDate.fromIso8601('2022-06-21T10:00:00Z');
viewer.scene.backgroundColor=Cesium.Color.fromCssColorString('#8bb1c2');
viewer.scene.screenSpaceCameraController.enableCollisionDetection=false;
if(viewer.scene.sun)viewer.scene.sun.show=false;
if(viewer.scene.moon)viewer.scene.moon.show=false;
window.viewer=viewer;
const vector=a=>new Cesium.Cartesian3(...a);
const xyz=p=>[p.x,p.y,p.z];
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function api(path,options){const r=await fetch(path,options);const v=await r.json();if(!r.ok)throw new Error(v.error||r.statusText);return v;}
let tileset,settings;
async function waitTiles(timeout=240000){
  const start=performance.now();let stable=0;
  while(performance.now()-start<timeout){
    if(state.failures.length)throw new Error('3D tile request failed; preserve incomplete recording and retry after checking the map source');
    await new Promise(resolve=>viewer.scene.postRender.addEventListener(function once(){viewer.scene.postRender.removeEventListener(once);resolve();}));
    if(tileset.tilesLoaded&&state.pending===0&&state.processing===0&&state.visible>0)stable++;else stable=0;
    if(stable>=8)return;
    if(!state.capturing)$('loading').textContent=`3D 타일 로딩 · 요청 ${state.pending} · 처리 ${state.processing} · 화면 타일 ${state.visible}`;
    await pause(40);
  }
  throw new Error(`3D tile load timeout (pending=${state.pending}, processing=${state.processing}, visible=${state.visible})`);
}
async function init(){
  settings=await api('/api/config');
  $('credit').textContent=settings.map.attribution;
  tileset=await Cesium.Cesium3DTileset.fromUrl('/tiles/tileset.json',{
    maximumScreenSpaceError:6,
    dynamicScreenSpaceError:false,skipLevelOfDetail:false,preloadWhenHidden:false,
    cacheBytes:512*1024*1024,maximumCacheOverflowBytes:256*1024*1024});
  viewer.scene.primitives.add(tileset);window.tileset=tileset;
  tileset.loadProgress.addEventListener((pending,processing)=>{state.pending=pending;state.processing=processing;});
  tileset.tileFailed.addEventListener(error=>{state.failures.push(error);console.error('TILE FAILED',JSON.stringify(error));});
  viewer.scene.preUpdate.addEventListener(()=>state.visible=0);
  tileset.tileVisible.addEventListener(()=>state.visible++);
  const ic=settings.flight.initial;
  viewer.camera.setView({destination:Cesium.Cartesian3.fromDegrees(ic.longitude_deg,ic.latitude_deg,ic.altitude_h_sl_m+150),
    orientation:{heading:Cesium.Math.toRadians(30),pitch:Cesium.Math.toRadians(-55),roll:0}});
  await waitTiles();window.mapReady=true;
  $('loading').textContent='실측 3D 메시 준비됨 · 마우스로 회전 / 확대할 수 있습니다.';
  $('phase').textContent='취득 대기';$('record').disabled=false;
}

function setPose(row,origin,camera){
  const o=Cesium.Cartesian3.fromRadians(origin.longitude_rad,origin.latitude_rad,origin.ellipsoid_height_m);
  const transform=Cesium.Transforms.northEastDownToFixedFrame(o);
  const rEW=Cesium.Matrix4.getMatrix3(transform,new Cesium.Matrix3());
  const rWC=Cesium.Matrix3.fromQuaternion(new Cesium.Quaternion(row[5],row[6],row[7],row[4]));
  const rEC=Cesium.Matrix3.multiply(rEW,rWC,new Cesium.Matrix3());
  const destination=Cesium.Matrix4.multiplyByPoint(transform,vector(row.slice(1,4)),new Cesium.Cartesian3());
  const direction=Cesium.Matrix3.getColumn(rEC,2,new Cesium.Cartesian3());
  const up=Cesium.Cartesian3.negate(Cesium.Matrix3.getColumn(rEC,1,new Cesium.Cartesian3()),new Cesium.Cartesian3());
  viewer.camera.frustum=new Cesium.PerspectiveFrustum({fov:Cesium.Math.toRadians(camera.horizontal_fov_deg),aspectRatio:camera.width/camera.height,near:.5,far:15000});
  viewer.camera.setView({destination,orientation:{direction,up}});
  return {destination,rEC,camera};
}
function reprojection(pose){
  let error=0;const c=pose.camera,fx=c.width/(2*Math.tan(Cesium.Math.toRadians(c.horizontal_fov_deg)/2));
  for(const p of [[0,0,100],[10,5,100],[-20,10,100]]){
    const offset=Cesium.Matrix3.multiplyByVector(pose.rEC,vector(p),new Cesium.Cartesian3());
    const point=Cesium.Cartesian3.add(pose.destination,offset,new Cesium.Cartesian3());
    const pixel=Cesium.SceneTransforms.worldToWindowCoordinates(viewer.scene,point);
    if(!pixel)throw new Error('Projection test behind camera');
    // Cesium window coordinates refer to pixel edges; calibration uses center indices.
    const expected=[fx*p[0]/p[2]+(c.width-1)/2,fx*p[1]/p[2]+(c.height-1)/2];
    error=Math.max(error,Math.hypot(pixel.x-.5-expected[0],pixel.y-.5-expected[1]));
  }return error;
}
async function capture(){
  if(state.capturing)return;
  state.capturing=true;$('record').disabled=true;$('error').textContent='';
  viewer.scene.screenSpaceCameraController.enableInputs=false;
  try{
    tileset.maximumScreenSpaceError=settings.map.maximum_screen_space_error;
    const data=await api('/api/poses');
    for(let index=0;index<data.poses.length;index++){
      const row=data.poses[index],pose=setPose(row,data.origin,data.config.camera);
      $('phase').textContent=`3D 맵 렌더링 · 시뮬레이션 ${(row[0]/1e9).toFixed(1)} s`;
      await waitTiles();
      const projection_error_px=reprojection(pose);
      const meta={tiles_loaded:tileset.tilesLoaded,visible_tiles:state.visible,projection_error_px,
        camera_position_ecef:xyz(viewer.camera.positionWC),camera_direction_ecef:xyz(viewer.camera.directionWC),camera_up_ecef:xyz(viewer.camera.upWC)};
      const blob=await new Promise(resolve=>viewer.scene.canvas.toBlob(resolve,'image/png'));
      const result=await api(`/api/frame?t_ns=${row[0]}`,{method:'POST',headers:{'Content-Type':'image/png','X-Capture-Metadata':JSON.stringify(meta)},body:blob});
      $('counts').textContent=`영상 ${result.captured}/${data.poses.length} · IMU ${data.config.duration_s*data.config.imu_hz+1}`;
      $('progress').value=100*result.captured/data.poses.length;
      $('sample').textContent=`특징점 ${result.features} · PNG 저장 중 · 10 Hz 시뮬레이션 시각`;
    }
    $('phase').textContent='취득 완료 — 실제 3D 맵 영상 + JSBSim IMU';
    $('loading').textContent='완료 · 데이터셋에 PNG, IMU, GT, calibration 및 검증 보고서를 저장했습니다.';
  }catch(error){
    $('error').textContent=String(error);$('phase').textContent='취득 중단';
    await api('/api/fail',{method:'POST',body:JSON.stringify({error:String(error)})}).catch(()=>{});
    console.error(error);
  }finally{state.capturing=false;viewer.scene.screenSpaceCameraController.enableInputs=true;$('record').disabled=false;}
}
$('record').onclick=async()=>{
  try{
    const params={duration_s:+$('duration').value,mean_ned_mps:[+$('windN').value,+$('windE').value,0],
      turbulence:{model:$('turbulence').value,severity:1,windspeed_20ft_mps:2},
      gusts:[{start_s:5,rise_s:1,hold_s:2,fall_s:1,ned_mps:[0,+$('gust').value,0]}],
      gyro_white_std_rad_s:+$('gyro').value,accel_white_std_m_s2:+$('accel').value};
    $('record').disabled=true;await api('/api/start',{method:'POST',body:JSON.stringify(params)});
  }catch(error){$('error').textContent=String(error);$('record').disabled=false;}
};
setInterval(async()=>{
  try{const s=await api('/api/status');
    if(s.dataset)$('dataset').textContent=s.dataset;
    if(s.phase==='generating')$('phase').textContent='JSBSim 비행 · IMU/GT 생성 중';
    if(s.phase==='flight_ready'&&!state.capturing)capture();
    if(s.phase==='failed'){$('error').textContent=s.error;$('record').disabled=false;}
  }catch(error){console.error(error);}
},1000);
init().catch(error=>{$('error').textContent=String(error);$('phase').textContent='맵 로딩 실패';console.error(error);});
