'use strict';
(() => {
  const video=$('markerVideo'), canvas=$('markerCanvas');
  const ctx=canvas.getContext('2d',{willReadFrequently:true});
  let stream=null, timer=null, generation=0, detector=null;
  function status(text){$('scanStatus').textContent=text;}
  function clear(){ $('markerResults').replaceChildren(); }
  function configure(){
    const [bits,count]=$('dictionary').value.split(',').map(Number);
    const source=AR.DICTIONARIES[`ARUCO_${bits}X${bits}_1000`];
    // Exact code matching avoids guessing IDs from damaged/wrong-dictionary markers.
    AR.DICTIONARIES.KENNY={nBits:source.nBits,tau:1,codeList:source.codeList.slice(0,count)};
    detector=new AR.Detector({dictionaryName:'KENNY',maxHammingDistance:1});
    clear();
  }
  function stop(text='Camera stopped. Results cleared.'){
    ++generation;clearTimeout(timer);timer=null;
    if(stream)stream.getTracks().forEach(track=>track.stop());
    stream=null;video.srcObject=null;scannerBusy=false;
    $('cameraStart').disabled=false;$('cameraStop').disabled=true;
    clear();ctx.clearRect(0,0,canvas.width,canvas.height);buttons();status(text);
  }
  function detect(source,width,height){
    const scale=Math.min(1,640/Math.max(width,height));
    canvas.width=Math.round(width*scale);canvas.height=Math.round(height*scale);
    ctx.drawImage(source,0,0,canvas.width,canvas.height);
    const markers=detector.detect(ctx.getImageData(0,0,canvas.width,canvas.height));
    clear();
    const sx=width/canvas.width,sy=height/canvas.height;
    markers.forEach(marker=>{
      const corners=marker.corners.map(p=>({x:p.x*sx,y:p.y*sy}));
      // Intersect diagonals: this is the projected center even under perspective.
      const [a,b,c,d]=corners;
      const ux=c.x-a.x,uy=c.y-a.y,vx=d.x-b.x,vy=d.y-b.y;
      const denom=ux*vy-uy*vx;
      const t=((b.x-a.x)*vy-(b.y-a.y)*vx)/denom;
      const center={x:a.x+t*ux,y:a.y+t*uy};
      const row=document.createElement('div');row.className='marker-result';
      const title=document.createElement('strong');title.textContent=`ID / value: ${marker.id}`;
      const detail=document.createElement('div');
      detail.textContent=`Center: (${center.x.toFixed(1)}, ${center.y.toFixed(1)}) px · Corners: `+corners.map(p=>`(${p.x.toFixed(1)}, ${p.y.toFixed(1)})`).join(' · ');
      row.append(title,detail);$('markerResults').append(row);
      ctx.strokeStyle='#9ce4bc';ctx.lineWidth=3;ctx.beginPath();
      marker.corners.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));ctx.closePath();ctx.stroke();
      ctx.fillStyle='#ff7878';ctx.beginPath();ctx.arc(marker.corners[0].x,marker.corners[0].y,5,0,Math.PI*2);ctx.fill();
      ctx.fillStyle='#ff7878';ctx.font='bold 18px sans-serif';ctx.fillText(String(marker.id),marker.corners[0].x,marker.corners[0].y-8);
    });
    status(`${markers.length?markers.length+' marker(s) detected':'No markers detected — check dictionary, focus and lighting'}. Source ${width} × ${height} px; coordinates refer to this source image.`);
  }
  function frame(mine){
    if(mine!==generation||!stream)return;
    try{if(video.readyState>=2)detect(video,video.videoWidth,video.videoHeight);}
    catch(error){stop('Scan failed: '+error.message);return;}
    timer=setTimeout(()=>frame(mine),250);
  }
  $('cameraStart').addEventListener('click',async()=>{
    stop();disarm('Camera scanning. Driving stopped.');
    if(!window.isSecureContext||!navigator.mediaDevices?.getUserMedia){
      status('Live camera requires HTTPS. On http://192.168.4.1 use Take a photo / choose image below.');return;
    }
    scannerBusy=true;buttons();$('cameraStart').disabled=true;$('cameraStop').disabled=false;
    const mine=generation;status('Requesting rear camera permission…');
    try{
      const acquired=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:640},height:{ideal:480}}});
      if(mine!==generation){acquired.getTracks().forEach(track=>track.stop());return;}
      stream=acquired;video.srcObject=stream;
      stream.getVideoTracks()[0].addEventListener('ended',()=>{if(mine===generation)stop('Camera disconnected.');});
      await video.play();frame(mine);
    }catch(error){if(mine===generation)stop('Camera unavailable: '+error.message+'. Use Take a photo / choose image.');}
  });
  $('cameraStop').addEventListener('click',()=>stop());
  $('markerPhoto').addEventListener('click',()=>{stop();disarm('Photo scanning. Driving stopped.');});
  $('markerPhoto').addEventListener('change',async event=>{
    const file=event.target.files[0];if(!file)return;
    stop();disarm('Photo scanning. Driving stopped.');scannerBusy=true;buttons();
    const mine=generation,url=URL.createObjectURL(file),image=new Image();
    try{
      image.src=url;await image.decode();
      if(mine===generation)detect(image,image.naturalWidth,image.naturalHeight);
    }catch(error){if(mine===generation){clear();status('Image could not be scanned. Choose a JPEG or PNG photo.');}}
    finally{URL.revokeObjectURL(url);event.target.value='';if(mine===generation){scannerBusy=false;buttons();}}
  });
  $('dictionary').addEventListener('change',()=>{stop('Dictionary changed. Start camera or choose another photo.');configure();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stop('Camera stopped when page was hidden.');});
  window.addEventListener('pagehide',()=>stop());
  configure();
  if(!window.isSecureContext)status('On this HTTP page, use Take a photo / choose image. Live camera requires HTTPS.');
})();
