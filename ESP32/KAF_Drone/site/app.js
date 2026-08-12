let droneID="55";
let lastUpdateTime=0;
let motorcmds=[0,0,0,0];
let rlog=[];
let plog=[];
let rlogScale=[0.001,0.001,0.001,0.001,0.001,0.001];
let plogScale=1;

function root_updateinput(elem){
  elem.style.color="#ff007b";
  elem.style.backgroundColor="#5a002c";
}
function root_neuterinput(elem,bctype){
  currstyle=window.getComputedStyle(document.body);
  elem.style.color=currstyle.getPropertyValue("--fc");
  elem.style.backgroundColor=currstyle.getPropertyValue(bctype);
}
async function root_kill(){
  fetch("cmd?KILL=");
  await window.alert("Quadcopter Terminated!");
  location.reload();
}
async function root_hover(){
  await fetch("cmd?CHVR=");
}
async function root_savedata(){
  if((await fetch("cmd?CSAV=")).ok){
    window.alert("Quadrotor Data Saved to Disk.");
  }
}
async function root_manual(){
  if((await fetch("cmd?MANL=")).ok){
    window.location.replace("manual.html");
  }
}
async function root_dosafety(target){
  safety=target.innerText=="Enable Safety";
  if((await fetch("cmd?SCNM="+(safety?1:0)+"$"))){
    target.innerText=safety?"Disable Safety":"Enable Safety";
  }
}
async function root_upload(event,name){
  form=new FormData();
  form.append("f",event.target.files[0]);
  if((await fetch("upload?"+name,{method:"POST",body:form})).ok){
    await window.alert("File Uploaded");
  }
}
function root_drawcoords(canvas,cx,cy,rot,coords){
  canvas.beginPath();
  canvas.moveTo(cx,cy);
  len=coords.length/3;
  drawpos=Array(len*2);
  for(i=0;i<len;i++){
    drawpos[i]=rot[0]*coords[i]+rot[1]*coords[len+i]+rot[2]*coords[2*len+i]+cx;
    drawpos[len+i]=rot[3]*coords[i]+rot[4]*coords[len+i]+rot[5]*coords[2*len+i]+cy;
    canvas.lineTo(drawpos[i],drawpos[len+i]);
  }
  return drawpos;
}
function root_drawheathbar(canvas,name,value,color,x,y,w,h){
  xm=5;
  ym=40;
  fh=h-ym*2;
  canvas.fillStyle=color;
  canvas.strokeStyle="#00ffff";
  canvas.fillRect(x+xm,y+ym+fh*(1-value),w-2*xm,fh*value);
  canvas.strokeRect(x+xm,y+ym,w-2*xm,fh);
  canvas.font="26px Arial";
  canvas.fillStyle=canvas.strokeStyle;
  canvas.textAlign="center";
  canvas.textBaseline="middle";
  canvas.fillText(name,x+w/2,y+ym/2);
  canvas.fillText((value*100).toFixed(1)+"%",x+w/2,y+h-ym/2);
}

async function load_network(){
  resp=await fetch("cmd?GNET=");
  if(resp.ok){
    data=await resp.json();
    droneID=data.id.toString(16);
    document.getElementById("ipaddress").value=data.ip;
    document.getElementById("network").value=data.name;
    document.getElementById("password").value=data.password;
    document.getElementById("droneid").value=droneID;
    document.getElementById("wifiselect").value=data.ap?"AP Mode":"WiFi Network";
    homepage_resetnetwork();
  }
}
async function load_setpoints(){
  resp=await fetch("cmd?GSPT=");
  if(resp.ok){
    data=await resp.json();
    for(i=0;i<22;i++){
      elem=document.getElementById("s"+i.toString());
      elem.value=data.sp[i].toString();
      root_neuterinput(elem,"--b2");
    }
  }
}
async function load_armingbutton(armed){
  armstat=document.getElementById("armstat");
  armstat.innerText=armed?"DRONE ARMED":"DRONE DISARMED";
  armstat.style.color=armed?"#00ff00":"#ff0000";
  armstat.style.backgroundColor=armed?"#0a5f3a":"#800000";
}
async function load_joystick(id,event){
  joystick=document.getElementById("cn"+id);
  w2=joystick.clientWidth/2;
  h2=joystick.clientHeight/2;
  r=80;
  x=0;
  y=0;
  scale=Math.sqrt((w2*w2+h2*h2)/2)-r;
  if(event!=null){
    if(event.buttons!=0){
      x=event.offsetX-w2;
      y=event.offsetY-h2;
      sr=scale/Math.sqrt(x*x+y*y);
      if(sr<1){
        x=x*sr;
        y=y*sr;
      }
    }else if(id==1){
      return;
    }
  }
  joystick.width=joystick.clientWidth;
  joystick.height=joystick.clientHeight;
  canvas=joystick.getContext("2d");
  canvas.beginPath();
  canvas.arc(x+w2,y+h2,r,0,6.28318530718);
  canvas.fillStyle="#6c67ff";
  canvas.fill();
  canvas.beginPath();
  canvas.arc(x+w2,y+h2,r,0,6.28318530718);
  canvas.strokeStyle="#9794ff";
  canvas.lineWidth=2;
  canvas.stroke();
  motorcmds[id*2]=x/scale;
  motorcmds[id*2+1]=y/scale;
}
async function load_flight(){
  currentTime=Date.now();
  if(currentTime-lastUpdateTime<=15000){
    return;
  }
  lastUpdateTime=currentTime;
  resp=await fetch("cmd?GFLT=");
  if(resp.ok){
    data=await resp.json();
    waitTime=Date.now()-currentTime;
    load_armingbutton(data.armed);
    document.getElementById("modeselect").selectedIndex=data.mode;
    rotP=[0.707106781186548,0.707106781186547,0,0.5,-0.5,-0.707106781186547];
    ploglen3=plog.length/3;
    plog.splice(ploglen3*3,0,data.x[2]);
    plog.splice(ploglen3*2,0,data.x[1]);
    plog.splice(ploglen3,0,data.x[0]);
    plogScale=Math.max(plogScale,Math.sqrt(data.x[0]*data.x[0]+data.x[1]*data.x[1]+data.x[2]*data.x[2]));
    coords = [0,1,-1, 0,0, 0,0,0,
              0,0, 0, 0,1,-1,0,0,
              0,0, 0, 0,0, 0,0,1];
    coordnames=["0","+X","-X","","+Y","-Y","","+Z"];
    position=document.getElementById("position");
    position.width=position.clientWidth;
    position.height=position.clientHeight;
    pw2=position.width/2;
    ph2=position.height/2;
    pl=(pw2<ph2?pw2:ph2);
    rotC=[];
    for(i=0;i<6;i++){
      rotC[i]=pl*rotP[i];
    }
    canvas=position.getContext("2d");
    pos=root_drawcoords(canvas,pw2,ph2,rotC,coords);
    canvas.lineWidth=2;
    canvas.strokeStyle="#ffffff";
    canvas.stroke();
    canvas.font="13px Arial";
    canvas.fillStyle="#ffffff";
    canvas.textAlign="center";
    plogScaleName=plogScale.toPrecision(3);
    for(i=0;i<pos.length/2;i++){
      if(coords[i]!=0||coords[coords.length/3+i]!=0){
        canvas.textBaseline="top";
        canvas.fillText(coordnames[i],pos[i],pos[pos.length/2+i]+3);
        canvas.fillText(plogScaleName+"m",pos[i],pos[pos.length/2+i]+13);
      }else if(i==7){
        canvas.textBaseline="bottom";
        canvas.fillText(coordnames[i],pos[7],pos[15]-10);
        canvas.fillText(plogScaleName+"m",pos[7],pos[15]);
      }else{
        canvas.textBaseline="top";
        canvas.fillText(coordnames[i],pos[i],pos[pos.length/2+i]+3);
      }
    }
    for(i=0;i<6;i++){
      rotC[i]=pl*rotP[i]/plogScale;
    }
    pos=root_drawcoords(canvas,pw2,ph2,rotC,plog);
    canvas.lineWidth=2;
    canvas.strokeStyle="#ffff00";
    canvas.stroke();
    posX=pos[pos.length/2-1];
    posY=pos[pos.length-1];
    canvas.beginPath();
    canvas.arc(posX,posY,5,0,6.28318530718);
    canvas.fillStyle = "#00ffff";
    canvas.fill();
    canvas.font="13px Arial";
    canvas.fillStyle="#00ffff";
    canvas.textAlign="center";
    canvas.textBaseline="top";
    canvas.fillText("("+data.x[0].toPrecision(3)+","+data.x[1].toPrecision(3)+","+data.x[2].toPrecision(3)+")",posX,posY+10);
    canvas.textBaseline="bottom";
    canvas.fillText("T: "+data.step+"ms  D: "+waitTime+"ms",pw2,position.height);
    coords=[0,1,1,1,0,-1,-1,-1,0,-1,-1,-1,0, 1, 1, 1,0,0,
            0,1,1,1,0, 1, 1, 1,0,-1,-1,-1,0,-1,-1,-1,0,0,
            0,0,1,0,0, 0, 1, 0,0, 0, 1, 0,0, 0, 1, 0,0,1];
    coordidcs=[42,46,38,50];
    attitude=document.getElementById("attitude");
    attitude.width=attitude.clientWidth;
    attitude.height=attitude.clientHeight;
    aw2=attitude.width/2;
    ah2=attitude.height/2;
    al=(aw2<ah2?aw2:ah2)*0.57735026919;
    rotC=[];
    for(i=0;i<2;i++){
      for(j=0;j<3;j++){
        rotC[i*3+j]=al*(rotP[i*3]*data.r[j]+rotP[i*3+1]*data.r[j+3]+rotP[i*3+2]*data.r[j+6]);
      }
    }
    for(i=0;i<4;i++){
      coords[coordidcs[i]]=data.t[i]; 
    }
    canvas=attitude.getContext("2d");
    pos=root_drawcoords(canvas,aw2,ah2,rotC,coords);
    canvas.lineWidth=3;
    canvas.strokeStyle="#00ffff";
    canvas.stroke();
    canvas.font="13px Arial";
    canvas.fillStyle = "#00ffff";
    canvas.textAlign = "center";
    canvas.textBaseline="bottom";
    for(i=0;i<4;i++){
      id=coordidcs[i]-coords.length*2/3;
      canvas.fillText("M"+i+"="+data.t[i],pos[id],pos[pos.length/2+id]);
    }
    rlog.push([data.a[0],data.a[1],data.a[2],data.w[0],data.w[1],data.w[2]]);
    purgecount=rlog.length-document.getElementById("r0").clientWidth;
    if(purgecount>0){
      rlog.splice(0,purgecount);
    }
    units=["m/s2","rad/s"];
    measnames=["Accel X","Accel Y","Accel Z","Gyro X","Gyro Y","Gyro Z"];
    for(i=0;i<6;i++){
      current=rlog[rlog.length-1][i];
      current=current<0?-current:current;
      rlogScale[i]=rlogScale[i]>current?rlogScale[i]:current;
      rc=document.getElementById("r"+i);
      rc.width=rc.clientWidth;
      rc.height=rc.clientHeight;
      rh2=rc.height/2;
      canvas=rc.getContext("2d");
      canvas.beginPath();
      canvas.moveTo(0,rh2);
      canvas.lineTo(rc.width,rh2);
      canvas.lineWidth = 2;
      canvas.strokeStyle = "#9794ff";
      canvas.stroke();
      canvas.beginPath();
      canvas.moveTo(0,rh2);
      lx=0;
      ly=0;
      for(j=0;j<rlog.length;j++){
        lx=j;
        ly=-rlog[j][i]*rh2/rlogScale[i]+rh2;
        canvas.lineTo(j,ly);
      }
      canvas.lineWidth = 1;
      canvas.strokeStyle = "#ffffff";
      canvas.stroke();
      canvas.font="13px Arial";
      unit=units[Math.floor(i/3)];
      canvas.textBaseline="middle";
      canvas.textAlign = "end";
      canvas.fillStyle = "#ffffff";
      canvas.fillText(rlog[rlog.length-1][i].toPrecision(3)+unit,lx,ly);
      canvas.fillStyle = "#9794ff";
      scale=rlogScale[i].toPrecision(3);
      canvas.textBaseline="top";
      canvas.textAlign = "center";
      canvas.fillText(measnames[i],rc.width/2,2);
      canvas.textAlign = "start";
      canvas.fillText("+"+scale+unit,0,2);
      canvas.fillText("0"+unit,0,rh2+2);
      canvas.textBaseline="bottom";
      canvas.fillText("-"+scale+unit,0,rc.height);
      lastUpdateTime=0;
    }
  }
}
async function load_logging(){
  resp=await fetch("cmd?GLOG=");
  if(resp.ok){
    data=await resp.json();
    droneID=data.id.toString(16);
    periphs=document.getElementById("periphselect");
    for(i=0;i<data.periphs.length;i++){
      periphs.add(new Option(data.periphs[i]));
    }
  }
}
async function load_manual(){
  currentTime=Date.now();
  if(currentTime-lastUpdateTime<=15000){
    return;
  }
  lastUpdateTime=currentTime;
  resp=await fetch("cmd?GMAN="+motorcmds[0].toFixed(3)+"$"+motorcmds[1].toFixed(3)+"$"+motorcmds[2].toFixed(3)+"$"+motorcmds[3].toFixed(3)+"$");
  if(resp.ok){
    data=await resp.json();
    document.getElementById("arming").innerText=data.armed?"Disarm Drone":"Arm Drone";
    motorstat=document.getElementById("motorstat");
    w=motorstat.clientWidth;
    h=motorstat.clientHeight;
    motorstat.width=w;
    motorstat.height=h;
    canvas=motorstat.getContext("2d");
    welem=100;
    helem=50;
    h2=(h-helem)/2;
    root_drawheathbar(canvas,"Battery",data.battery/100,"#00d215",welem,helem,w-2*welem,h-helem);
    root_drawheathbar(canvas,"M0",data.motors[0],"#ffff00",w-welem,helem,welem,h2);
    root_drawheathbar(canvas,"M1",data.motors[1],"#ffff00",0,helem,welem,h2);
    root_drawheathbar(canvas,"M2",data.motors[2],"#ffff00",w-welem,h2+helem,welem,h2);
    root_drawheathbar(canvas,"M3",data.motors[3],"#ffff00",0,h2+helem,welem,h2);
    canvas.fillStyle=data.armed?"#00ff00":"#ff0000";
    canvas.strokeStyle=data.armed?"#0a5f3a":"#800000";
    canvas.fillRect(5,5,w-10,helem-10);
    canvas.strokeRect(5,5,w-10,helem-10);
    canvas.textAlign="center";
    canvas.textBaseline="middle";
    canvas.font="26px Arial";
    canvas.fillStyle=canvas.strokeStyle;
    canvas.fillText((data.armed?"DRONE ARMED":"DRONE DISARMED")+(data.manual?", MANUAL":""),w/2,helem/2);
    lastUpdateTime=0;
  }
}
async function load_calibration(){
  currentTime=Date.now();
  if(currentTime-lastUpdateTime<=15000){
    return;
  }
  lastUpdateTime=currentTime;
  resp=await fetch("cmd?GCAL=");
  if(resp.ok){
    data=await resp.json();
    for(i=0;i<3;i++){
      elem=document.getElementById("p"+i);
      if(elem!=document.activeElement){
        elem.value=data.x[i].toString();
        root_neuterinput(elem);
      }
      elem=document.getElementById("v"+i);
      if(elem!=document.activeElement){
        elem.value=data.v[i].toString();
        root_neuterinput(elem);
      }
    }
    lastUpdateTime=0;
  }
}
async function load_calibdata(){
  resp=await fetch("cmd?DCAL=");
  if(resp.ok) {
    flarr=new Float32Array((await(await resp.blob()).bytes()).buffer);
    for(i=0;i<75;i++){
      elem=document.getElementById("c"+i);
      elem.value=flarr[i].toString();
      elem.addEventListener("input",function(event){
        event.target.hasmodified=true;
        root_updateinput(event.target);
      });
      elem.addEventListener("focusout",function(event){
        if(event.target.hasmodified){
          if(calib_update(event)){
            event.target.hasmodified=false;
          }
        }
      });
    }
  }
}

async function homepage_disarm(){
  if((await fetch("cmd?SARM=0")).ok){
    window.alert("Quadcopter Motors "+(arming?"Armed":"Disarmed")+"!");
  }
}
async function homepage_resetnetwork(){
  root_neuterinput(document.getElementById("ipaddress"),"--b2");
  root_neuterinput(document.getElementById("network"),"--b2");
  root_neuterinput(document.getElementById("password"),"--b2");
  root_neuterinput(document.getElementById("droneid"),"--b2");
  root_neuterinput(document.getElementById("wifiselect"),"--b2");
}
async function homepage_setnetwork(){
  droneID=document.getElementById("droneid").value;
  command="cmd?SNET="+document.getElementById("ipaddress").value+"$"+
      document.getElementById("network").value+"$"+document.getElementById("password").value+"$"+
      droneID+"$"+(document.getElementById("wifiselect").value=="AP Mode"?1:0)+"$";
  if((await fetch(command)).ok){
    homepage_resetnetwork();
  }
}
async function homepage_armingstatus(){
  armed=document.getElementById("armstat").innerText=="DRONE DISARMED";
  if((await fetch("cmd?SARM="+(armed?1:0))).ok){
    load_armingbutton(armed);
  }
}
async function homepage_flightmode(){
  await fetch("cmd?SFLT="+document.getElementById("modeselect").selectedIndex+"$");
}
async function homepage_zeroout(){
  await fetch("cmd?CSX0=");
}
async function homepage_clearsetpoints(){
  for(i=0;i<22;i++){
    elem=document.getElementById("s"+i.toString());
    elem.value="0";
    root_updateinput(elem);
  }
  homepage_setsetpoints();
}
async function homepage_setsetpoints(){
  command="cmd?SSPT=";
  for(i=0;i<22;i++){
    command+=document.getElementById("s"+i.toString()).value+"$";
  }
  if((await fetch(command)).ok){
    for(i=0;i<22;i++){
      root_neuterinput(document.getElementById("s"+i.toString()),"--b2");
    }
  }
}
async function homepage_settrajectory(){
  trajtype=document.getElementById("trajselect");
  command="cmd?STRJ="+trajtype.selectedIndex.toString(16)+"$";
  trajtype.selectedIndex=0;
  for(i=1;i<=4;i++){
    elem=document.getElementById("p"+i.toString());
    command+=elem.value+"$";
    elem.value="";
  }
  if((await fetch(command)).ok){
    await load_setpoints();
    await load_flight();
  }
}
async function log_upload(event){
  root_upload(event,"PERI="+document.getElementById("periphselect").selectedIndex+"$");
}
async function log_selectdown(){
  periphdown=document.getElementById("periphdown");
  periphselect=document.getElementById("periphselect");
  periphdown.href="cmd?DPRP="+periphselect.selectedIndex+"$";
  periphdown.download="periph_"+periphselect.value+".bin";
}
async function log_coms(command){
  const logger=document.getElementById("commandlog");
  try{
    resp=await fetch("com?DATA="+command);
  }catch(e){
    resp=false;
  }
  if(logger){
    logger.innerText+="[GS->QD]:"+command+"\n";
    if(resp&&resp.ok){
      logger.innerText+="[QD->GS]:"+await resp.text()+"\n";
    }
  }
}
async function log_quick(command){
  log_coms(droneID+command);
}
async function log_comcmd(event){
  if(event.key=='Enter'){
    const strval=event.target.value;
    log_coms(strval.replaceAll(" ", ""));
    event.target.value="";
  }
}
async function manual_arming(target){
  arming=target.innerText=="Arm Drone";
  if((await fetch("cmd?SARM="+(arming?1:0))).ok){
    target.innerText=armed?"Disarm Drone":"Arm Drone";
  }
}
async function calib_update(event){
  if((await fetch("cmd?SCAL="+event.target.id+"$"+event.target.value)).ok){
    root_neuterinput(event.target);
    return true;
  }
  return false;
}
async function calib_resetdata(){
  fetch("cmd?CRST=");
  await window.alert("Reset Data!");
  location.reload();
}
async function calib_sensecalib(){
  resp=await fetch("cmd?SFLT=2$");
  if(resp.ok){
    await window.alert("Sensor Calibration Started. Click OK to Terminate.");
    resp=await fetch("cmd?SFLT=0$");
    if(resp.ok){
      location.reload();
    }
  }
}
async function calib_magcalib(){
  await window.alert("Not Implemented!");
}
async function calib_simpidtune(){
  await window.alert("Not Implemented!");
}
async function calib_physpidtune(){
  await window.alert("Not Implemented!");
}

console.log("KAF Drone Web Interface Started");
if(document.getElementById("homepage")){
  load_network();
  load_setpoints();
  setInterval(load_flight,50);
}else if(document.getElementById("logging")){
  load_logging();
  log_selectdown();
}else if(document.getElementById("calibration")){
  load_calibdata();
  setInterval(load_calibration,50);
}else if(document.getElementById("manual")){
  load_joystick(0,null);
  load_joystick(1,null);
  setInterval(load_manual,50);
}