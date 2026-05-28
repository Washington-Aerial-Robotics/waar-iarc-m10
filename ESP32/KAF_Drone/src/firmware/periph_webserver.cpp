#include "../core/communication.h"
#include "../core/flight.h"
#include "../core/firmware.h"
#include "../auxilary/common_data.h"
#include "../auxilary/commander.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#include <math.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WebServer.h>
#endif

#define WEBSERVER_COM_METHOD 4
#define WEBSERVER_PORT 80
#define SNFORMAT( I, ... ) snprintf( &webserver.scratch[I], sizeof( webserver.scratch ) - I - 1, __VA_ARGS__ )

extern COMS_BUFFERTYPE;
extern FLIGHT_BUFFERTYPE;
extern void peripheral_esp32KillCommand();
extern void peripheral_wifiNetwork( const bool rwflag, char* name, char* password, char* address, bool* ap );

const char app[] =
  "let droneID=\"55\";let lastUpdateTime=0;let motorcmds=[0,0,0,0];let rlog=[];let plog=[];let rlogScale="
  "[0.001,0.001,0.001,0.001,0.001,0.001];let plogScale=1;function root_updateinput(elem){elem.style.col"
  "or=\"#ff007b\";elem.style.backgroundColor=\"#5a002c\";}function root_neuterinput(elem,bctype){currstyle="
  "window.getComputedStyle(document.body);elem.style.color=currstyle.getPropertyValue(\"--fc\");elem.styl"
  "e.backgroundColor=currstyle.getPropertyValue(bctype);}async function root_kill(){fetch(\"cmd?KILL=\");"
  "await window.alert(\"Quadcopter Terminated!\");location.reload();}async function root_hover(){await fe"
  "tch(\"cmd?CHVR=\");}async function root_savedata(){if((await fetch(\"cmd?CSAV=\")).ok){window.alert(\"Qua"
  "drotor Data Saved to Disk.\");}}async function root_manual(){if((await fetch(\"cmd?MANL=\")).ok){window"
  ".location.replace(\"manual.html\");}}async function root_dosafety(target){safety=target.innerText==\"En"
  "able Safety\";if((await fetch(\"cmd?SCNM=\"+(safety?1:0)+\"$\"))){target.innerText=safety?\"Disable Safety"
  "\":\"Enable Safety\";}}async function root_upload(event,name){form=new FormData();form.append(\"f\",event"
  ".target.files[0]);if((await fetch(\"upload?\"+name,{method:\"POST\",body:form})).ok){await window.alert("
  "\"File Uploaded\");}}function root_drawcoords(canvas,cx,cy,rot,coords){canvas.beginPath();canvas.moveT"
  "o(cx,cy);len=coords.length/3;drawpos=Array(len*2);for(i=0;i<len;i++){drawpos[i]=rot[0]*coords[i]+rot"
  "[1]*coords[len+i]+rot[2]*coords[2*len+i]+cx;drawpos[len+i]=rot[3]*coords[i]+rot[4]*coords[len+i]+rot"
  "[5]*coords[2*len+i]+cy;canvas.lineTo(drawpos[i],drawpos[len+i]);}return drawpos;}function root_drawh"
  "eathbar(canvas,name,value,color,x,y,w,h){xm=5;ym=40;fh=h-ym*2;canvas.fillStyle=color;canvas.strokeSt"
  "yle=\"#00ffff\";canvas.fillRect(x+xm,y+ym+fh*(1-value),w-2*xm,fh*value);canvas.strokeRect(x+xm,y+ym,w-"
  "2*xm,fh);canvas.font=\"26px Arial\";canvas.fillStyle=canvas.strokeStyle;canvas.textAlign=\"center\";canv"
  "as.textBaseline=\"middle\";canvas.fillText(name,x+w/2,y+ym/2);canvas.fillText((value*100).toFixed(1)+\""
  "%\",x+w/2,y+h-ym/2);}async function load_network(){resp=await fetch(\"cmd?GNET=\");if(resp.ok){data=awa"
  "it resp.json();droneID=data.id.toString(16);document.getElementById(\"ipaddress\").value=data.ip;docum"
  "ent.getElementById(\"network\").value=data.name;document.getElementById(\"password\").value=data.passwor"
  "d;document.getElementById(\"droneid\").value=droneID;document.getElementById(\"wifiselect\").value=data."
  "ap?\"AP Mode\":\"WiFi Network\";homepage_resetnetwork();}}async function load_setpoints(){resp=await fet"
  "ch(\"cmd?GSPT=\");if(resp.ok){data=await resp.json();for(i=0;i<22;i++){elem=document.getElementById(\"s"
  "\"+i.toString());elem.value=data.sp[i].toString();root_neuterinput(elem,\"--b2\");}}}async function loa"
  "d_armingbutton(armed){armstat=document.getElementById(\"armstat\");armstat.innerText=armed?\"DRONE ARME"
  "D\":\"DRONE DISARMED\";armstat.style.color=armed?\"#00ff00\":\"#ff0000\";armstat.style.backgroundColor=arme"
  "d?\"#0a5f3a\":\"#800000\";}async function load_joystick(id,event){joystick=document.getElementById(\"cn\"+"
  "id);w2=joystick.clientWidth/2;h2=joystick.clientHeight/2;r=80;x=0;y=0;scale=Math.sqrt((w2*w2+h2*h2)/"
  "2)-r;if(event!=null){if(event.buttons!=0){x=event.offsetX-w2;y=event.offsetY-h2;sr=scale/Math.sqrt(x"
  "*x+y*y);if(sr<1){x=x*sr;y=y*sr;}}else if(id==1){return;}}joystick.width=joystick.clientWidth;joystic"
  "k.height=joystick.clientHeight;canvas=joystick.getContext(\"2d\");canvas.beginPath();canvas.arc(x+w2,y"
  "+h2,r,0,6.28318530718);canvas.fillStyle=\"#6c67ff\";canvas.fill();canvas.beginPath();canvas.arc(x+w2,y"
  "+h2,r,0,6.28318530718);canvas.strokeStyle=\"#9794ff\";canvas.lineWidth=2;canvas.stroke();motorcmds[id*"
  "2]=x/scale;motorcmds[id*2+1]=y/scale;}async function load_flight(){currentTime=Date.now();if(current"
  "Time-lastUpdateTime<=15000){return;}lastUpdateTime=currentTime;resp=await fetch(\"cmd?GFLT=\");if(resp"
  ".ok){data=await resp.json();waitTime=Date.now()-currentTime;load_armingbutton(data.armed);document.g"
  "etElementById(\"modeselect\").selectedIndex=data.mode;rotP=[0.707106781186548,0.707106781186547,0,0.5,"
  "-0.5,-0.707106781186547];ploglen3=plog.length/3;plog.splice(ploglen3*3,0,data.x[2]);plog.splice(plog"
  "len3*2,0,data.x[1]);plog.splice(ploglen3,0,data.x[0]);plogScale=Math.max(plogScale,Math.sqrt(data.x["
  "0]*data.x[0]+data.x[1]*data.x[1]+data.x[2]*data.x[2]));coords = [0,1,-1, 0,0, 0,0,0,0,0, 0, 0,1,-1,0"
  ",0,0,0, 0, 0,0, 0,0,1];coordnames=[\"0\",\"+X\",\"-X\",\"\",\"+Y\",\"-Y\",\"\",\"+Z\"];position=document.getElementB"
  "yId(\"position\");position.width=position.clientWidth;position.height=position.clientHeight;pw2=positi"
  "on.width/2;ph2=position.height/2;pl=(pw2<ph2?pw2:ph2);rotC=[];for(i=0;i<6;i++){rotC[i]=pl*rotP[i];}c"
  "anvas=position.getContext(\"2d\");pos=root_drawcoords(canvas,pw2,ph2,rotC,coords);canvas.lineWidth=2;c"
  "anvas.strokeStyle=\"#ffffff\";canvas.stroke();canvas.font=\"13px Arial\";canvas.fillStyle=\"#ffffff\";canv"
  "as.textAlign=\"center\";plogScaleName=plogScale.toPrecision(3);for(i=0;i<pos.length/2;i++){if(coords[i"
  "]!=0||coords[coords.length/3+i]!=0){canvas.textBaseline=\"top\";canvas.fillText(coordnames[i],pos[i],p"
  "os[pos.length/2+i]+3);canvas.fillText(plogScaleName+\"m\",pos[i],pos[pos.length/2+i]+13);}else if(i==7"
  "){canvas.textBaseline=\"bottom\";canvas.fillText(coordnames[i],pos[7],pos[15]-10);canvas.fillText(plog"
  "ScaleName+\"m\",pos[7],pos[15]);}else{canvas.textBaseline=\"top\";canvas.fillText(coordnames[i],pos[i],p"
  "os[pos.length/2+i]+3);}}for(i=0;i<6;i++){rotC[i]=pl*rotP[i]/plogScale;}pos=root_drawcoords(canvas,pw"
  "2,ph2,rotC,plog);canvas.lineWidth=2;canvas.strokeStyle=\"#ffff00\";canvas.stroke();posX=pos[pos.length"
  "/2-1];posY=pos[pos.length-1];canvas.beginPath();canvas.arc(posX,posY,5,0,6.28318530718);canvas.fillS"
  "tyle = \"#00ffff\";canvas.fill();canvas.font=\"13px Arial\";canvas.fillStyle=\"#00ffff\";canvas.textAlign="
  "\"center\";canvas.textBaseline=\"top\";canvas.fillText(\"(\"+data.x[0].toPrecision(3)+\",\"+data.x[1].toPrec"
  "ision(3)+\",\"+data.x[2].toPrecision(3)+\")\",posX,posY+10);canvas.textBaseline=\"bottom\";canvas.fillText"
  "(\"T: \"+data.step+\"ms  D: \"+waitTime+\"ms\",pw2,position.height);coords=[0,1,1,1,0,-1,-1,-1,0,-1,-1,-1,"
  "0, 1, 1, 1,0,0,0,1,1,1,0, 1, 1, 1,0,-1,-1,-1,0,-1,-1,-1,0,0,0,0,1,0,0, 0, 1, 0,0, 0, 1, 0,0, 0, 1, 0"
  ",0,1];coordidcs=[38,42,46,50];attitude=document.getElementById(\"attitude\");attitude.width=attitude.c"
  "lientWidth;attitude.height=attitude.clientHeight;aw2=attitude.width/2;ah2=attitude.height/2;al=(aw2<"
  "ah2?aw2:ah2)*0.57735026919;rotC=[];for(i=0;i<2;i++){for(j=0;j<3;j++){rotC[i*3+j]=al*(rotP[i*3]*data."
  "r[j]+rotP[i*3+1]*data.r[j+3]+rotP[i*3+2]*data.r[j+6]);}}for(i=0;i<4;i++){coords[coordidcs[i]]=data.t"
  "[i];}canvas=attitude.getContext(\"2d\");pos=root_drawcoords(canvas,aw2,ah2,rotC,coords);canvas.lineWid"
  "th=3;canvas.strokeStyle=\"#00ffff\";canvas.stroke();canvas.font=\"13px Arial\";canvas.fillStyle = \"#00ff"
  "ff\";canvas.textAlign = \"center\";canvas.textBaseline=\"bottom\";for(i=0;i<4;i++){id=coordidcs[i]-coords"
  ".length*2/3;canvas.fillText(\"M\"+i+\"=\"+data.t[i],pos[id],pos[pos.length/2+id]);}rlog.push([data.a[0],"
  "data.a[1],data.a[2],data.w[0],data.w[1],data.w[2]]);purgecount=rlog.length-document.getElementById(\""
  "r0\").clientWidth;if(purgecount>0){rlog.splice(0,purgecount);}units=[\"m/s2\",\"rad/s\"];measnames=[\"Acce"
  "l X\",\"Accel Y\",\"Accel Z\",\"Gyro X\",\"Gyro Y\",\"Gyro Z\"];for(i=0;i<6;i++){current=rlog[rlog.length-1][i]"
  ";current=current<0?-current:current;rlogScale[i]=rlogScale[i]>current?rlogScale[i]:current;rc=docume"
  "nt.getElementById(\"r\"+i);rc.width=rc.clientWidth;rc.height=rc.clientHeight;rh2=rc.height/2;canvas=rc"
  ".getContext(\"2d\");canvas.beginPath();canvas.moveTo(0,rh2);canvas.lineTo(rc.width,rh2);canvas.lineWid"
  "th = 2;canvas.strokeStyle = \"#9794ff\";canvas.stroke();canvas.beginPath();canvas.moveTo(0,rh2);lx=0;l"
  "y=0;for(j=0;j<rlog.length;j++){lx=j;ly=-rlog[j][i]*rh2/rlogScale[i]+rh2;canvas.lineTo(j,ly);}canvas."
  "lineWidth = 1;canvas.strokeStyle = \"#ffffff\";canvas.stroke();canvas.font=\"13px Arial\";unit=units[Mat"
  "h.floor(i/3)];canvas.textBaseline=\"middle\";canvas.textAlign = \"end\";canvas.fillStyle = \"#ffffff\";can"
  "vas.fillText(rlog[rlog.length-1][i].toPrecision(3)+unit,lx,ly);canvas.fillStyle = \"#9794ff\";scale=rl"
  "ogScale[i].toPrecision(3);canvas.textBaseline=\"top\";canvas.textAlign = \"center\";canvas.fillText(meas"
  "names[i],rc.width/2,2);canvas.textAlign = \"start\";canvas.fillText(\"+\"+scale+unit,0,2);canvas.fillTex"
  "t(\"0\"+unit,0,rh2+2);canvas.textBaseline=\"bottom\";canvas.fillText(\"-\"+scale+unit,0,rc.height);lastUpd"
  "ateTime=0;}}}async function load_logging(){resp=await fetch(\"cmd?GLOG=\");if(resp.ok){data=await resp"
  ".json();droneID=data.id.toString(16);periphs=document.getElementById(\"periphselect\");for(i=0;i<data."
  "periphs.length;i++){periphs.add(new Option(data.periphs[i]));}}}async function load_manual(){current"
  "Time=Date.now();if(currentTime-lastUpdateTime<=15000){return;}lastUpdateTime=currentTime;resp=await "
  "fetch(\"cmd?GMAN=\"+motorcmds[0].toFixed(3)+\"$\"+motorcmds[1].toFixed(3)+\"$\"+motorcmds[2].toFixed(3)+\"$"
  "\"+motorcmds[3].toFixed(3)+\"$\");if(resp.ok){data=await resp.json();document.getElementById(\"arming\")."
  "innerText=data.armed?\"Disarm Drone\":\"Arm Drone\";motorstat=document.getElementById(\"motorstat\");w=mot"
  "orstat.clientWidth;h=motorstat.clientHeight;motorstat.width=w;motorstat.height=h;canvas=motorstat.ge"
  "tContext(\"2d\");welem=100;helem=50;h2=(h-helem)/2;root_drawheathbar(canvas,\"Battery\",data.battery/100"
  ",\"#00d215\",welem,helem,w-2*welem,h-helem);root_drawheathbar(canvas,\"M0\",data.motors[0],\"#ffff00\",w-w"
  "elem,helem,welem,h2);root_drawheathbar(canvas,\"M1\",data.motors[1],\"#ffff00\",0,helem,welem,h2);root_d"
  "rawheathbar(canvas,\"M2\",data.motors[2],\"#ffff00\",w-welem,h2+helem,welem,h2);root_drawheathbar(canvas"
  ",\"M3\",data.motors[3],\"#ffff00\",0,h2+helem,welem,h2);canvas.fillStyle=data.armed?\"#00ff00\":\"#ff0000\";"
  "canvas.strokeStyle=data.armed?\"#0a5f3a\":\"#800000\";canvas.fillRect(5,5,w-10,helem-10);canvas.strokeRe"
  "ct(5,5,w-10,helem-10);canvas.textAlign=\"center\";canvas.textBaseline=\"middle\";canvas.font=\"26px Arial"
  "\";canvas.fillStyle=canvas.strokeStyle;canvas.fillText((data.armed?\"DRONE ARMED\":\"DRONE DISARMED\")+(d"
  "ata.manual?\", MANUAL\":\"\"),w/2,helem/2);lastUpdateTime=0;}}async function load_calibration(){currentT"
  "ime=Date.now();if(currentTime-lastUpdateTime<=15000){return;}lastUpdateTime=currentTime;resp=await f"
  "etch(\"cmd?GCAL=\");if(resp.ok){data=await resp.json();for(i=0;i<3;i++){elem=document.getElementById(\""
  "p\"+i);if(elem!=document.activeElement){elem.value=data.x[i].toString();root_neuterinput(elem);}elem="
  "document.getElementById(\"v\"+i);if(elem!=document.activeElement){elem.value=data.v[i].toString();root"
  "_neuterinput(elem);}}lastUpdateTime=0;}}async function load_calibdata(){resp=await fetch(\"cmd?DCAL=\""
  ");if(resp.ok) {flarr=new Float32Array((await(await resp.blob()).bytes()).buffer);for(i=0;i<75;i++){e"
  "lem=document.getElementById(\"c\"+i);elem.value=flarr[i].toString();elem.addEventListener(\"input\",func"
  "tion(event){event.target.hasmodified=true;root_updateinput(event.target);});elem.addEventListener(\"f"
  "ocusout\",function(event){if(event.target.hasmodified){if(calib_update(event)){event.target.hasmodifi"
  "ed=false;}}});}}}async function homepage_disarm(){if((await fetch(\"cmd?SARM=0\")).ok){window.alert(\"Q"
  "uadcopter Motors \"+(arming?\"Armed\":\"Disarmed\")+\"!\");}}async function homepage_resetnetwork(){root_ne"
  "uterinput(document.getElementById(\"ipaddress\"),\"--b2\");root_neuterinput(document.getElementById(\"net"
  "work\"),\"--b2\");root_neuterinput(document.getElementById(\"password\"),\"--b2\");root_neuterinput(documen"
  "t.getElementById(\"droneid\"),\"--b2\");root_neuterinput(document.getElementById(\"wifiselect\"),\"--b2\");}"
  "async function homepage_setnetwork(){droneID=document.getElementById(\"droneid\").value;command=\"cmd?S"
  "NET=\"+document.getElementById(\"ipaddress\").value+\"$\"+document.getElementById(\"network\").value+\"$\"+do"
  "cument.getElementById(\"password\").value+\"$\"+droneID+\"$\"+(document.getElementById(\"wifiselect\").value"
  "==\"AP Mode\"?1:0)+\"$\";if((await fetch(command)).ok){homepage_resetnetwork();}}async function homepage"
  "_armingstatus(){armed=document.getElementById(\"armstat\").innerText==\"DRONE DISARMED\";if((await fetch"
  "(\"cmd?SARM=\"+(armed?1:0))).ok){load_armingbutton(armed);}}async function homepage_flightmode(){await"
  " fetch(\"cmd?SFLT=\"+document.getElementById(\"modeselect\").selectedIndex+\"$\");}async function homepage"
  "_zeroout(){await fetch(\"cmd?CSX0=\");}async function homepage_clearsetpoints(){for(i=0;i<22;i++){elem"
  "=document.getElementById(\"s\"+i.toString());elem.value=\"0\";root_updateinput(elem);}homepage_setsetpoi"
  "nts();}async function homepage_setsetpoints(){command=\"cmd?SSPT=\";for(i=0;i<22;i++){command+=documen"
  "t.getElementById(\"s\"+i.toString()).value+\"$\";}if((await fetch(command)).ok){for(i=0;i<22;i++){root_n"
  "euterinput(document.getElementById(\"s\"+i.toString()),\"--b2\");}}}async function homepage_settrajector"
  "y(){trajtype=document.getElementById(\"trajselect\");command=\"cmd?STRJ=\"+trajtype.selectedIndex.toStri"
  "ng(16)+\"$\";trajtype.selectedIndex=0;for(i=1;i<=4;i++){elem=document.getElementById(\"p\"+i.toString())"
  ";command+=elem.value+\"$\";elem.value=\"\";}if((await fetch(command)).ok){await load_setpoints();await l"
  "oad_flight();}}async function log_upload(event){root_upload(event,\"PERI=\"+document.getElementById(\"p"
  "eriphselect\").selectedIndex+\"$\");}async function log_selectdown(){periphdown=document.getElementById"
  "(\"periphdown\");periphselect=document.getElementById(\"periphselect\");periphdown.href=\"cmd?DPRP=\"+peri"
  "phselect.selectedIndex+\"$\";periphdown.download=\"periph_\"+periphselect.value+\".bin\";}async function l"
  "og_coms(command){const logger=document.getElementById(\"commandlog\");try{resp=await fetch(\"com?DATA=\""
  "+command);}catch(e){resp=false;}if(logger){logger.innerText+=\"[GS->QD]:\"+command+\"\\n\";if(resp&&resp."
  "ok){logger.innerText+=\"[QD->GS]:\"+await resp.text()+\"\\n\";}}}async function log_quick(command){log_co"
  "ms(droneID+command);}async function log_comcmd(event){if(event.key=='Enter'){const strval=event.targ"
  "et.value;log_coms(strval.replaceAll(\" \", \"\"));event.target.value=\"\";}}async function manual_arming(t"
  "arget){arming=target.innerText==\"Arm Drone\";if((await fetch(\"cmd?SARM=\"+(arming?1:0))).ok){target.in"
  "nerText=armed?\"Disarm Drone\":\"Arm Drone\";}}async function calib_update(event){if((await fetch(\"cmd?S"
  "CAL=\"+event.target.id+\"$\"+event.target.value)).ok){root_neuterinput(event.target);return true;}retur"
  "n false;}async function calib_resetdata(){fetch(\"cmd?CRST=\");await window.alert(\"Reset Data!\");locat"
  "ion.reload();}async function calib_sensecalib(){resp=await fetch(\"cmd?SFLT=2$\");if(resp.ok){await wi"
  "ndow.alert(\"Sensor Calibration Started. Click OK to Terminate.\");resp=await fetch(\"cmd?SFLT=0$\");if("
  "resp.ok){location.reload();}}}async function calib_magcalib(){await window.alert(\"Not Implemented!\")"
  ";}async function calib_simpidtune(){await window.alert(\"Not Implemented!\");}async function calib_phy"
  "spidtune(){await window.alert(\"Not Implemented!\");}console.log(\"KAF Drone Web Interface Started\");if"
  "(document.getElementById(\"homepage\")){load_network();load_setpoints();setInterval(load_flight,50);}e"
  "lse if(document.getElementById(\"logging\")){load_logging();log_selectdown();}else if(document.getElem"
  "entById(\"calibration\")){load_calibdata();setInterval(load_calibration,50);}else if(document.getEleme"
  "ntById(\"manual\")){load_joystick(0,null);load_joystick(1,null);setInterval(load_manual,50);}";
const char calibration[] =
  "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"/><meta name=\"viewport\" content=\"width=dev"
  "ice-width, initial-scale=1.0\"/><title>Calibration Manager</title><link rel=\"icon\" href=\"icon.png\"><l"
  "ink rel=\"stylesheet\" href=\"styles.css\"/></head><body><header class=\"topbar hdr\"><a class=\"button\" hr"
  "ef=\"homepage.html\">Master Panel</a><h1 class=\"hdrl\" id=\"calibration\">KAF Drone Ground Control Calibr"
  "ation</h1><a class=\"button\" href=\"https://github.com/Washington-Aerial-Robotics/waar-iarc-m10/tree/F"
  "irmware-Kent/ESP32/KAF_Drone\">Source Code</a></header><main class=\"calib-layout\"><section class=\"lc1"
  "\"><div class=\"pnl\"><div class=\"pnl-title\">Data Management</div><div class=\"pnl-body\"><div class=\"lc1"
  "\"><a class=\"button button-thin\" href=\"cmd?DALL=\" download=\"info.bin\">Download Full Info</a><label fo"
  "r=\"upinfo\" class=\"button button-thin\">Upload Full Info</label><input type=\"file\" id=\"upinfo\" class=\""
  "fileselect\" oninput=\"root_upload(event,'INFO=')\"/><button class=\"button button-thin\" onclick=\"calib_"
  "resetdata()\">Reset Full Info</button><button class=\"button button-thin\" onclick=\"root_savedata()\">Sa"
  "ve Full Info</button><a class=\"button button-thin\" href=\"cmd?DCAL=\" download=\"calibration.bin\">Downl"
  "oad Calibration</a><label for=\"upcalb\" class=\"button button-thin\">Upload Calibration</label><input t"
  "ype=\"file\" id=\"upcalb\" class=\"fileselect\" oninput=\"root_upload(event,'CALB=')\"/></div></div></div><d"
  "iv class=\"pnl\"><div class=\"pnl-title\">Position</div><div class=\"pnl-body\"><div class=\"lcunits\"><labe"
  "l for=\"p0\">Pos X</label><input id=\"p0\" type=\"text\" class=\"field\" onfocus=\"root_updateinput(event.tar"
  "get)\" onfocusout=\"calib_update(event)\"/><label>m</label><label for=\"p1\">Pos Y</label><input id=\"p1\" "
  "type=\"text\" class=\"field\" onfocus=\"root_updateinput(event.target)\" onfocusout=\"calib_update(event)\"/"
  "><label>m</label><label for=\"p2\">Pos Z</label><input id=\"p2\" type=\"text\" class=\"field\" onfocus=\"root"
  "_updateinput(event.target)\" onfocusout=\"calib_update(event)\"/><label>m</label></div></div></div><div"
  " class=\"pnl\"><div class=\"pnl-title\">Sensor Filter Values</div><div class=\"pnl-body\"><div class=\"lcco"
  "ord\"><label>Sensor</label><label>Gain</label><label>Offset</label><label>Stdev</label><label>GYRO X<"
  "/label><input id=\"c3\" type=\"text\" class=\"field\"/><input id=\"c4\" type=\"text\" class=\"field\"/><input id"
  "=\"c5\" type=\"text\" class=\"field\"/><label>GYRO Y</label><input id=\"c6\" type=\"text\" class=\"field\"/><inp"
  "ut id=\"c7\" type=\"text\" class=\"field\"/><input id=\"c8\" type=\"text\" class=\"field\"/><label>GYRO Z</label"
  "><input id=\"c9\" type=\"text\" class=\"field\"/><input id=\"c10\" type=\"text\" class=\"field\"/><input id=\"c11"
  "\" type=\"text\" class=\"field\"/><label>ACCEL X</label><input id=\"c12\" type=\"text\" class=\"field\"/><input"
  " id=\"c13\" type=\"text\" class=\"field\"/><input id=\"c14\" type=\"text\" class=\"field\"/><label>ACCEL Y</labe"
  "l><input id=\"c15\" type=\"text\" class=\"field\"/><input id=\"c16\" type=\"text\" class=\"field\"/><input id=\"c"
  "17\" type=\"text\" class=\"field\"/><label>ACCEL Z</label><input id=\"c18\" type=\"text\" class=\"field\"/><inp"
  "ut id=\"c19\" type=\"text\" class=\"field\"/><input id=\"c20\" type=\"text\" class=\"field\"/><label>MAG X</labe"
  "l><input id=\"c21\" type=\"text\" class=\"field\"/><input id=\"c22\" type=\"text\" class=\"field\"/><input id=\"c"
  "23\" type=\"text\" class=\"field\"/><label>MAG Y</label><input id=\"c24\" type=\"text\" class=\"field\"/><input"
  " id=\"c25\" type=\"text\" class=\"field\"/><input id=\"c26\" type=\"text\" class=\"field\"/><label>MAG Z</label>"
  "<input id=\"c27\" type=\"text\" class=\"field\"/><input id=\"c28\" type=\"text\" class=\"field\"/><input id=\"c29"
  "\" type=\"text\" class=\"field\"/><label>BARO</label><input id=\"c30\" type=\"text\" class=\"field\"/><input id"
  "=\"c31\" type=\"text\" class=\"field\"/><input id=\"c32\" type=\"text\" class=\"field\"/><label>TEMP</label><inp"
  "ut id=\"c33\" type=\"text\" class=\"field\"/><input id=\"c34\" type=\"text\" class=\"field\"/><input id=\"c35\" ty"
  "pe=\"text\" class=\"field\"/><label>GPS LAT</label><input id=\"c36\" type=\"text\" class=\"field\"/><input id="
  "\"c37\" type=\"text\" class=\"field\"/><input id=\"c38\" type=\"text\" class=\"field\"/><label>GPS LNG</label><i"
  "nput id=\"c39\" type=\"text\" class=\"field\"/><input id=\"c40\" type=\"text\" class=\"field\"/><input id=\"c41\" "
  "type=\"text\" class=\"field\"/><label>GPS ALT</label><input id=\"c42\" type=\"text\" class=\"field\"/><input i"
  "d=\"c43\" type=\"text\" class=\"field\"/><input id=\"c44\" type=\"text\" class=\"field\"/><label>DW X</label><in"
  "put id=\"c45\" type=\"text\" class=\"field\"/><input id=\"c46\" type=\"text\" class=\"field\"/><input id=\"c47\" t"
  "ype=\"text\" class=\"field\"/><label>DW Y</label><input id=\"c48\" type=\"text\" class=\"field\"/><input id=\"c"
  "49\" type=\"text\" class=\"field\"/><input id=\"c50\" type=\"text\" class=\"field\"/><label>DW Z</label><input "
  "id=\"c51\" type=\"text\" class=\"field\"/><input id=\"c52\" type=\"text\" class=\"field\"/><input id=\"c53\" type="
  "\"text\" class=\"field\"/></div></div></div></section><section class=\"lc1\"><div class=\"pnl\"><div class=\""
  "pnl-title\">Calibration & Tuning</div><div class=\"pnl-body\"><div class=\"lc1\"><button class=\"button bu"
  "tton-thin\" onclick=\"calib_sensecalib()\">Sensor Calibration</button><button class=\"button button-thin"
  "\" onclick=\"calib_magcalib()\">Magnetometer Calibration</button><button class=\"button button-thin\" onc"
  "lick=\"calib_simpidtune()\">Simulation PID Tuning</button><button class=\"button button-thin\" onclick=\""
  "calib_physpidtune()\">Physical PID Tuning</button><label for=\"upsims\" class=\"button button-thin\">Set "
  "Drone System ID</label><input type=\"file\" id=\"upsims\" class=\"fileselect\" oninput=\"root_upload(event,"
  "'SIMS=')\"/><label for=\"upresp\" class=\"button button-thin\">Set PID Response Data</label><input type=\""
  "file\" id=\"upresp\" class=\"fileselect\" oninput=\"root_upload(event,'RESP=')\"/></div></div></div><div cl"
  "ass=\"pnl\"><div class=\"pnl-title\">Velocity</div><div class=\"pnl-body\"><div class=\"lcunits\"><label for"
  "=\"v0\">Veloc X</label><input id=\"v0\" type=\"text\" class=\"field\" onfocus=\"root_updateinput(event.target"
  ")\" onfocusout=\"calib_update(event)\"/><label>m/s</label><label for=\"v1\">Veloc Y</label><input id=\"v1\""
  " type=\"text\" class=\"field\" onfocus=\"root_updateinput(event.target)\" onfocusout=\"calib_update(event)\""
  "/><label>m/s</label><label for=\"v2\">Veloc Z</label><input id=\"v2\" type=\"text\" class=\"field\" onfocus="
  "\"root_updateinput(event.target)\" onfocusout=\"calib_update(event)\"/><label>m/s</label></div></div></d"
  "iv><div class=\"pnl\"><div class=\"pnl-title\">Control Constants</div><div class=\"pnl-body\"><div class=\""
  "lcunits\"><label for=\"c0\">Angle A</label><input id=\"c0\" type=\"text\" class=\"field\"/><label>Hz</label><"
  "label for=\"c1\">Pos A</label><input id=\"c1\" type=\"text\" class=\"field\"/><label>Hz</label><label for=\"c"
  "2\">Grav G</label><input id=\"c2\" type=\"text\" class=\"field\"/><label>m/s2</label></div></div></div><div"
  " class=\"pnl\"><div class=\"pnl-title\">PID Gain Values</div><div class=\"pnl-body\"><div class=\"lccoord\">"
  "<label>Controller</label><label>Kp</label><label>Ki</label><label>Kd</label><label>Position</label><"
  "input id=\"c54\" type=\"text\" class=\"field\"/><input id=\"c55\" type=\"text\" class=\"field\"/><input id=\"c56\""
  " type=\"text\" class=\"field\"/><label>Velocity</label><input id=\"c57\" type=\"text\" class=\"field\"/><input"
  " id=\"c58\" type=\"text\" class=\"field\"/><input id=\"c59\" type=\"text\" class=\"field\"/><label>Thrust</label"
  "><input id=\"c60\" type=\"text\" class=\"field\"/><input id=\"c61\" type=\"text\" class=\"field\"/><input id=\"c6"
  "2\" type=\"text\" class=\"field\"/><label>Attitude</label><input id=\"c63\" type=\"text\" class=\"field\"/><inp"
  "ut id=\"c64\" type=\"text\" class=\"field\"/><input id=\"c65\" type=\"text\" class=\"field\"/><label>W Rate X</l"
  "abel><input id=\"c66\" type=\"text\" class=\"field\"/><input id=\"c67\" type=\"text\" class=\"field\"/><input id"
  "=\"c68\" type=\"text\" class=\"field\"/><label>W Rate Y</label><input id=\"c69\" type=\"text\" class=\"field\"/>"
  "<input id=\"c70\" type=\"text\" class=\"field\"/><input id=\"c71\" type=\"text\" class=\"field\"/><label>W Rate "
  "Z</label><input id=\"c72\" type=\"text\" class=\"field\"/><input id=\"c73\" type=\"text\" class=\"field\"/><inpu"
  "t id=\"c74\" type=\"text\" class=\"field\"/></div></div></div><div class=\"pnl\"><div class=\"pnl-title\">Magn"
  "etometer Matrix</div><div class=\"pnl-body\"><div class=\"lccoord\"><label>-</label><label>i=1</label><l"
  "abel>i=2</label><label>i=3</label><label>j=1</label><input id=\"c75\" type=\"text\" class=\"field\"/><inpu"
  "t id=\"c76\" type=\"text\" class=\"field\"/><input id=\"c77\" type=\"text\" class=\"field\"/><label>j=2</label><"
  "input id=\"c78\" type=\"text\" class=\"field\"/><input id=\"c79\" type=\"text\" class=\"field\"/><input id=\"c80\""
  " type=\"text\" class=\"field\"/><label>j=3</label><input id=\"c81\" type=\"text\" class=\"field\"/><input id=\""
  "c82\" type=\"text\" class=\"field\"/><input id=\"c83\" type=\"text\" class=\"field\"/></div></div></div></secti"
  "on><section class=\"lc1\"><div class=\"graph graph-resp\"><canvas id=\"response\"></canvas></div></section"
  "></main><script src=\"app.js\"></script></body></html>";
const char homepage[] =
  "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"/><meta name=\"viewport\" content=\"width=dev"
  "ice-width, initial-scale=1.0\"/><title>Master Control Panel</title><link rel=\"icon\" href=\"icon.png\"><"
  "link rel=\"stylesheet\" href=\"styles.css\"/></head><body><header class=\"topbar hdr\"><a class=\"button\" h"
  "ref=\"homepage.html\">Master Panel</a><h1 class=\"hdrl\" id=\"homepage\">KAF Drone Ground Station Master P"
  "anel</h1><a class=\"button\" href=\"https://github.com/Washington-Aerial-Robotics/waar-iarc-m10/tree/Fi"
  "rmware-Kent/ESP32/KAF_Drone\">Source Code</a></header><main class=\"home-layout\"><section class=\"lc1\">"
  "<div class=\"pnl\"><div class=\"pnl-title\">Safety</div><div class=\"pnl-body\"><div class=\"lc3\"><button c"
  "lass=\"button\" onclick=\"homepage_disarm()\">Disarm Motors</button><button class=\"button\" onclick=\"root"
  "_kill(false)\">Kill</button><button class=\"button\" onclick=\"root_hover()\">Hover Drone</button><button"
  " class=\"button\" onclick=\"root_savedata()\">Save Data</button><button class=\"button\" onclick=\"root_man"
  "ual()\">Manual Override</button><button class=\"button\" onclick=\"root_dosafety(event.target)\">Enable S"
  "afety</button></div></div></div><div class=\"pnl\"><div class=\"pnl-title\">Control Panels</div><div cla"
  "ss=\"pnl-body\"><div class=\"lc1\"><a class=\"button button-thin\" href=\"manual.html\">Manual Control</a><a"
  " class=\"button button-thin\" href=\"logging.html\">Data Logging</a><a class=\"button button-thin\" href=\""
  "calibration.html\">Calibrations</a></div></div></div><div class=\"pnl\"><div class=\"pnl-title\">Connecti"
  "on</div><div class=\"pnl-body\"><div class=\"lc1\"><label for=\"ipaddress\">IP:</label><input id=\"ipaddres"
  "s\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\" value=\"0.0.0.0\", readonly/><la"
  "bel for=\"network\">Network:</label><input id=\"network\" type=\"text\" class=\"field\" oninput=\"root_update"
  "input(event.target)\" placeholder=\"Network Name\"/><label for=\"password\">Password:</label><input id=\"p"
  "assword\" type=\"password\" class=\"field\" oninput=\"root_updateinput(event.target)\" placeholder=\"Network"
  " Password\"/><label for=\"droneid\">Drone ID:</label><input id=\"droneid\" type=\"text\" class=\"field\" onin"
  "put=\"root_updateinput(event.target)\" placeholder=\"Drone ID Number\"/><label for=\"wifiselect\">Connecti"
  "on Type:</label><select id=\"wifiselect\" class=\"field\" onchange=\"root_updateinput(event.target)\"\"><op"
  "tion>AP Mode</option><option>WiFi Network</option></select><button class=\"button button-thin\" onclic"
  "k=\"homepage_setnetwork()\">Update</button></div></div></div></section><section class=\"lc1\"><div class"
  "=\"lcgraph\"><div class=\"graph\"><canvas id=\"attitude\"></canvas></div><div class=\"graph\"><canvas id=\"po"
  "sition\"></canvas></div><div class=\"graph graph-wide\"><canvas id=\"r0\"></canvas></div><div class=\"grap"
  "h graph-wide\"><canvas id=\"r3\"></canvas></div><div class=\"graph graph-wide\"><canvas id=\"r1\"></canvas>"
  "</div><div class=\"graph graph-wide\"><canvas id=\"r4\"></canvas></div><div class=\"graph graph-wide\"><ca"
  "nvas id=\"r2\"></canvas></div><div class=\"graph graph-wide\"><canvas id=\"r5\"></canvas></div></div></sec"
  "tion><section class=\"lc1\"><div class=\"pnl\"><div class=\"pnl-title\">Control Mode</div><div class=\"pnl-"
  "body\"><div class=\"lc1\"><button class=\"button\" id=\"armstat\" onclick=\"homepage_armingstatus()\">ARMED/U"
  "NARMED</button><label for=\"modeselect\">Flight Mode:</label><select id=\"modeselect\" class=\"field\" oni"
  "nput=\"homepage_flightmode()\"><option>None</option><option>Inactive</option><option>Sensor Calibratio"
  "n</option><option>Manual Actuation</option><option>Motor Setpoint Control</option><option>Accelerati"
  "on Control</option><option>Position Control</option><option>Trajectory Control</option></select><but"
  "ton class=\"button button-thin\" onclick=\"homepage_zeroout()\">Zero Out State Estimate</button></div></"
  "div></div><div class=\"pnl\"><div class=\"pnl-title\">Command Setpoints</div><div class=\"pnl-body\"><div "
  "class=\"lc5\"><input id=\"s0\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><inpu"
  "t id=\"s1\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s2\" type=\"t"
  "ext\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s3\" type=\"text\" class=\"field"
  "\" oninput=\"root_updateinput(event.target)\"/><input id=\"s4\" type=\"text\" class=\"field\" oninput=\"root_u"
  "pdateinput(event.target)\"/><input id=\"s5\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event."
  "target)\"/><input id=\"s6\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input "
  "id=\"s7\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s8\" type=\"tex"
  "t\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s9\" type=\"text\" class=\"field\" "
  "oninput=\"root_updateinput(event.target)\"/><input id=\"s10\" type=\"text\" class=\"field\" oninput=\"root_up"
  "dateinput(event.target)\"/><input id=\"s11\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event."
  "target)\"/><input id=\"s12\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input"
  " id=\"s13\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s14\" type=\""
  "text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s15\" type=\"text\" class=\"fie"
  "ld\" oninput=\"root_updateinput(event.target)\"/><input id=\"s16\" type=\"text\" class=\"field\" oninput=\"roo"
  "t_updateinput(event.target)\"/><input id=\"s17\" type=\"text\" class=\"field\" oninput=\"root_updateinput(ev"
  "ent.target)\"/><input id=\"s18\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><i"
  "nput id=\"s19\" type=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s20\" ty"
  "pe=\"text\" class=\"field\" oninput=\"root_updateinput(event.target)\"/><input id=\"s21\" type=\"text\" class="
  "\"field\" oninput=\"root_updateinput(event.target)\"/><button class=\"button button-thin\" onclick=\"homepa"
  "ge_clearsetpoints()\">CLR</button><button class=\"button button-thin\" onclick=\"load_setpoints()\">LOAD<"
  "/button><button class=\"button button-thin\" onclick=\"homepage_setsetpoints()\">SET</button></div></div"
  "></div><div class=\"pnl\"><div class=\"pnl-title\">Trajectories</div><div class=\"pnl-body\"><div class=\"l"
  "c1\"><select id=\"trajselect\" class=\"field\"><option>-- Select --</option><option>Launch</option><optio"
  "n>Land</option><option>Position Lock</option><option>Return Home</option><option>Glide Point</option"
  "><option>Circle Path</option></select><div class=\"lcfield\"><label for=\"p1\">Param 1:</label><input id"
  "=\"p1\" type=\"text\" class=\"field\", placeholder=\"Trajectory Parameter 1\"/><label for=\"p2\">Param 2:</lab"
  "el><input id=\"p2\" type=\"text\" class=\"field\", placeholder=\"Trajectory Parameter 2\"/><label for=\"p3\">P"
  "aram 3:</label><input id=\"p3\" type=\"text\" class=\"field\", placeholder=\"Trajectory Parameter 3\"/><labe"
  "l for=\"p4\">Param 4:</label><input id=\"p4\" type=\"text\" class=\"field\", placeholder=\"Trajectory Paramet"
  "er 4\"/></div><button class=\"button button-thin\" onclick=\"homepage_settrajectory()\">Execute</button><"
  "/div></div></div></section></main><script src=\"app.js\"></script></body></html>";
const char icon[] =
  "\x89\x50\x4E\x47\x0D\x0A\x1A\x0A\x00\x00\x00\x0D\x49\x48\x44\x52\x00\x00\x00\x0C\x00\x00\x00\x0C\x08"
  "\x06\x00\x00\x00\x56\x75\x5C\xE7\x00\x00\x00\x01\x73\x52\x47\x42\x00\xAE\xCE\x1C\xE9\x00\x00\x00\x04"
  "\x67\x41\x4D\x41\x00\x00\xB1\x8F\x0B\xFC\x61\x05\x00\x00\x00\x09\x70\x48\x59\x73\x00\x00\x16\x25\x00"
  "\x00\x16\x25\x01\x49\x52\x24\xF0\x00\x00\x00\x78\x49\x44\x41\x54\x28\x53\x63\x00\x82\xFF\x20\xFC\xDF"
  "\xE7\x3F\x98\xB6\xF7\x38\x03\xA6\x91\xD9\x30\x39\x10\x66\x02\x12\x24\x01\x26\xA0\x6E\x06\x10\x26\x04"
  "\x60\xEA\x18\x41\x6C\x88\x10\x71\x80\x11\xEA\x4E\x30\x38\xB8\xC3\x84\xE1\xFF\x27\x28\x07\x0A\x18\xF9"
  "\x18\x18\x80\x6A\xA0\x3C\x20\x1F\x88\x51\x6C\x00\x6B\xE0\xDA\x0F\xE1\x7C\x73\x04\x6B\x40\x06\x24\x7B"
  "\x9A\x72\x27\x11\x02\xF0\x60\x85\x05\x2D\xB2\x69\x30\x36\xB2\x1A\x26\xC6\x2D\x8C\x0C\x20\x4C\x08\xC0"
  "\xD4\x91\xE8\x69\x06\x06\x00\xB1\x10\x45\x43\xB8\xFF\xBE\x72\x00\x00\x00\x00\x49\x45\x4E\x44\xAE\x42"
  "\x60\x82";
const char logging[] =
  "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"/><meta name=\"viewport\" content=\"width=dev"
  "ice-width, initial-scale=1.0\"/><title>Logging Command Terminal</title><link rel=\"icon\" href=\"icon.pn"
  "g\"><link rel=\"stylesheet\" href=\"styles.css\"/></head><body><header class=\"topbar hdr\"><a class=\"butto"
  "n\" href=\"homepage.html\">Master Panel</a><h1 class=\"hdrl\" id=\"logging\">KAF Drone Ground Station Termi"
  "nal</h1><a class=\"button\" href=\"https://github.com/Washington-Aerial-Robotics/waar-iarc-m10/tree/Fir"
  "mware-Kent/ESP32/KAF_Drone\">Source Code</a></header><main class=\"logging-layout\"><section class=\"lc1"
  "\"><div class=\"pnl\"><div class=\"pnl-title\">Firmware</div><div class=\"pnl-body\"><div class=\"lc1\"><labe"
  "l for=\"periphselect\">Peripheral:</label><select id=\"periphselect\" class=\"field\" oninput=\"log_selectd"
  "own()\"></select><label for=\"periphup\" class=\"button button-thin\">Upload</label><input type=\"file\" id"
  "=\"periphup\" class=\"fileselect\" oninput=\"log_upload(event)\"/><a id=\"periphdown\" class=\"button button-"
  "thin\" href=\"cmd?DPRP=0$\" download=\"periph.bin\">Download</a></div></div></div><div class=\"pnl\"><div c"
  "lass=\"pnl-title\">Quick Commands</div><div class=\"pnl-body\"><div class=\"lc3\"><button class=\"button\" o"
  "nclick=\"log_quick('474A0001')\">Arm</button><button class=\"button\" onclick=\"log_quick('474A0000')\">Di"
  "sarm</button><button class=\"button\" onclick=\"log_quick('476400')\">Kill</button><button class=\"button"
  "\" onclick=\"log_quick('475B0001')\">Launch</button><button class=\"button\" onclick=\"log_quick('475B0002"
  "')\">Land</button><button class=\"button\" onclick=\"log_quick('475B0003')\">Pos Lock</button><button cla"
  "ss=\"button\" onclick=\"log_quick('474000')\">Ping</button><button class=\"button\" onclick=\"log_quick('47"
  "4100')\">Devices</button><button class=\"button\" onclick=\"log_quick('474200')\">Get State</button><butt"
  "on class=\"button\" onclick=\"log_quick('475A0057')\">Save</button><button class=\"button\" onclick=\"log_q"
  "uick('475A0044')\">Reset</button><button class=\"button\" onclick=\"log_quick('475A0052')\">Load</button>"
  "</div></div></div></section><section class=\"lc1\"><div class=\"lc1\"><pre id=\"commandlog\", class=\"cmdpr"
  "ompt\">KAF Drone Serial Command Terminal<br></pre><input id=\"commandinput\" type=\"text\" class=\"field c"
  "mdfield\", onkeyup=\"log_comcmd(event)\" placeholder=\"Send Quadcopter Command\"/></div></section></main>"
  "<script src=\"app.js\"></script></body></html>";
const char manual[] =
  "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"/><meta name=\"viewport\" content=\"width=dev"
  "ice-width, initial-scale=1.0\"/><title>Manual Control Panel</title><link rel=\"icon\" href=\"icon.png\"><"
  "link rel=\"stylesheet\" href=\"styles.css\"/></head><body><header class=\"topbar hdr\"><a class=\"button\" h"
  "ref=\"homepage.html\">Master Panel</a><h1 class=\"hdrl\" id=\"manual\">KAF Drone Ground Station Terminal</"
  "h1><a class=\"button\" href=\"https://github.com/Washington-Aerial-Robotics/waar-iarc-m10/tree/Firmware"
  "-Kent/ESP32/KAF_Drone\">Source Code</a></header><main class=\"manual-layout\"><section class=\"lc5\"><but"
  "ton class=\"button\" id=\"arming\" onclick=\"manual_arming(event.target)\">Disarm Drone</button><button cl"
  "ass=\"button\" onclick=\"root_kill()\">Kill</button><button class=\"button\" onclick=\"root_manual()\">Manua"
  "l</button><button class=\"button\" onclick=\"root_hover()\">Hover</button><button class=\"button\" onclick"
  "=\"root_dosafety(event.target)\">Enable Safety</button></section><section class=\"lc3\"><div class=\"grap"
  "h control-circle\"><canvas id=\"cn0\" onpointermove=\"load_joystick(0,event)\" onpointerdown=\"load_joysti"
  "ck(0,event)\" onpointerup=\"load_joystick(0,null)\" onpointerout=\"load_joystick(0,null)\"></canvas></div"
  "><div class=\"graph graph-manual\"><canvas id=\"motorstat\"></canvas></div><div class=\"graph control-cir"
  "cle\"><canvas id=\"cn1\" onpointermove=\"load_joystick(1,event)\" onpointerdown=\"load_joystick(1,event)\">"
  "</canvas></div></section></main><script src=\"app.js\"></script></body></html>";
const char root[] =
  "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"0; url=homepage.html\"/></head><body><"
  "a href=\"homepage.html\">Redirecting...</a></body></html>";
const char styles[] =
  "*{box-sizing:border-box;}body{margin:0;font-family:Arial,Helvetica,sans-serif;background:var(--b3);c"
  "olor:var(--fc);}:root{--fc:#ff4cff;--bc:2px solid var(--fc);--b0:#2a0d43;--b1:#360b36;--b2:#0e0041;-"
  "-b3:#000000;--b4:#450e45;--b5:#240824;--b6:#1a1a1a;}.hdr{display:grid;grid-template-columns:60px 1fr"
  " 60px;align-items:center;min-height:70px;padding:5px;color:var(--fc);background:var(--b0);border-bot"
  "tom:var(--bc);}.hdrl{display:flex;align-items:center;justify-content:center;height:100%;margin:0;fon"
  "t-size:2rem;text-align:center;}.pnl{border-radius:5px;margin-bottom:4px;background:var(--b3);border:"
  "var(--bc);}.pnl-title{border-top-left-radius:5px;border-top-right-radius:5px;text-align:center;font-"
  "size:1rem;padding:6px 8px;color:var(--fc);background:var(--b0);border-bottom:var(--bc);}.pnl-body{bo"
  "rder-bottom-left-radius:5px;border-bottom-right-radius:5px;margin:5px;}.lc1,.lc3,.lc5,.lcfield,.lcun"
  "its,.lccoord{display:grid;gap:5px;align-items:start;}.lc5{grid-template-columns:1fr 1fr 1fr 1fr 1fr;"
  "}.lc3{grid-template-columns:1fr 1fr 1fr;}.lc1{grid-template-columns:1fr;}.lcfield{align-items:center"
  ";justify-content:center;grid-template-columns:25% 1fr;}.lcunits{align-items:center;justify-content:c"
  "enter;text-align:center;grid-template-columns:25% 1fr 25%;}.lccoord{align-items:center;justify-conte"
  "nt:center;text-align:center;grid-template-columns:25% 1fr 1fr 1fr;}.lcgraph{display:grid;grid-templa"
  "te-columns:1fr 1fr;gap:8px;}.fileselect{opacity:0;position:absolute;width:1px;height:1px;z-index:-1;"
  "}.field{width:100%;text-align:left;font-size:1rem;padding:6px 8px;border-radius:5px;color:var(--fc);"
  "background:var(--b2);border:var(--bc);}.button{min-width:60px;min-height:60px;border-radius:5px;widt"
  "h:100%;font-size:1rem;line-height:1.25;cursor:pointer;text-decoration:none;display:flex;align-items:"
  "center;justify-content:center;text-align:center;color:var(--fc);background:var(--b1);border:var(--bc"
  ");}.button:hover{border:var(--bc);color:var(--fc);background:var(--b4);}.button:active{border:var(--"
  "bc);color:var(--fc);background:var(--b5);}.button-thin{min-width:0px;min-height:0px;padding:6px 8px;"
  "}.graph{border:var(--bc);color:var(--fc);background:var(--b6);position:relative;overflow:hidden;heig"
  "ht:400px;border-radius:5px;}.graph-manual{height:100%;}.graph-wide{height:110px;}.graph-resp{height:"
  "1220px;}.control-circle{background:var(--b1);width:100%;aspect-ratio:1;border-radius:50%;}.graph can"
  "vas{width:100%;height:100%;display:block;}.cmdfield{font-family:monospace;}.cmdprompt{border:var(--b"
  "c);color:var(--fc);background:var(--b6);overflow:scroll;height:455px;border-radius:5px;font-family:m"
  "onospace;font-size:1rem;margin-top:0px;margin-bottom:4px;}.home-layout{display:grid;grid-template-co"
  "lumns:300px 1fr 300px;align-items:start;gap:8px;padding:8px;}.calib-layout{display:grid;grid-templat"
  "e-columns:300px 300px 1fr;align-items:start;gap:8px;padding:8px;}.logging-layout{display:grid;grid-t"
  "emplate-columns:205px 1fr;align-items:start;gap:8px;padding:8px;}.manual-layout{display:grid;grid-te"
  "mplate-columns:1fr;align-items:start;gap:8px;padding:8px;}@media(max-width:1200px){.home-layout{grid"
  "-template-columns:1fr;}.calib-layout{grid-template-columns:1fr;}.logging-layout{grid-template-column"
  "s:1fr;}}";

struct {
  bool serverInitialized;
  unsigned short comsCmdLen;
  unsigned short comsRespLen;
  char scratch[511];
  radio coms;
  WebServer server = WebServer( WEBSERVER_PORT );
} webserver;

static constexpr unsigned int hashcode4( const char str[5] ) {
  return ( ( ( unsigned int )str[0] ) << 24 ) | ( ( ( unsigned int )str[1] ) << 16 ) | 
         ( ( ( unsigned int )str[2] ) <<  8 ) | (   ( unsigned int )str[3]         );
}

static unsigned short snfloats( const char* name, const float* array, const unsigned short idx, const unsigned char len ) {
  unsigned short i = idx + SNFORMAT( idx, "\"%s\":[", name );
  if( len > 0 ) {
    i += SNFORMAT( i, "%f", array[0] );
  }
  for( unsigned char j = 1; j < len; j++ ) {
    i += SNFORMAT( i, ",%f", array[j] );
  }
  return i + SNFORMAT( i, "]" );
}

static const char* inputnext( char* dest, const char* src, const unsigned char maxlen ) {
  for( unsigned short i = 0; i < maxlen; i++ ) {
    const bool delimNull = src[i] == 0;
    if( delimNull || src[i] == '$' ) {
      memcpy( dest, src, i );
      dest[i] = 0;
      return src + i + ( delimNull ? 0 : 1 );
    }
  }
  dest[ maxlen - 1 ] = 0;
  return src + maxlen;
}

static void registerGS() {
  entity* gs = com_registerEntity( 'G' );
  gs->nodeOrder = 0;
  gs->liason = WEBSERVER_COM_METHOD;
  gs->lastSeen = millis();
}

static void handleUploads() {
  HTTPUpload& upload = webserver.server.upload();
  if( upload.status == UPLOAD_FILE_WRITE ) {
    DPRINTF( "[P] Web Server File Upload Packet Received\n" );
    const unsigned short len = ( unsigned short )upload.currentSize;
    const unsigned short idx = ( unsigned short )upload.totalSize;
    if( idx < sizeof( COMS_BUFFER ) && idx + len < sizeof( COMS_BUFFER ) ) {
      DPRINTF( "[P] Web Server File Write: Index=%u, Length=%u, Total=%u\n", idx, len, idx + len );
      memcpy( &COMS_BUFFER[idx], upload.buf, len );
    }
  }
}

static void handleUploadsEnd() {
  HTTPUpload& upload = webserver.server.upload();
  bool success = upload.status != UPLOAD_FILE_ABORTED && upload.totalSize > 0 && webserver.server.args() == 1;
  if( success ) {
    char file[5] = { 0, 0, 0, 0, 0 };
    strncpy( file, webserver.server.argName( 0 ).c_str(), sizeof( file ) - 1 );
    DPRINTF( "[P] Web Server File Packet Received: File=%s\n", file );
    switch( hashcode4( file ) ) {
      case hashcode4( "INFO" ) : {
        DPRINTF( "[P] Web Server File Upload: Type=ALL PERSISTENT DATA\n" );
        if( firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, 'L' ) == 0 ) {
          success = false;
        }
        break;
      }
      case hashcode4( "CALB" ) : {
        DPRINTF( "[P] Web Server File Upload: Type=CALIBRATION\n" );
        memcpy( &kafenv.cal, COMS_BUFFER, sizeof( kafenv.cal ) );
        break;
      }
      case hashcode4( "PERI" ) : {
        char arg[5];
        strncpy( arg, webserver.server.arg( 0 ).c_str(), sizeof( arg ) - 1 );
        unsigned char idx = ( unsigned char )strtol( arg, NULLPTR, 10 );
        DPRINTF( "[P] Web Server File Upload: Type=PERIPHERAL, Index=%u\n", idx );
        const peripheral* periph = firmware_getPeripheral( idx );
        if( periph != NULLPTR && periph->memory != NULLPTR && periph->length > 0 
            && periph->length < sizeof( COMS_BUFFER ) && periph->memory != COMS_BUFFER ) {
          memcpy( periph->memory, COMS_BUFFER, periph->length );
        }
        break;
      }
      case hashcode4( "SIMS" ) : {
        DPRINTF( "[P] Web Server File Upload: Type=SIMULATION SYSTEM ID\n" );
        break;
      }
      case hashcode4( "RESP" ) : {
        DPRINTF( "[P] Web Server File Upload: Type=PID TIME RESPONSE DATA\n" );
        break;
      }
      default : {
        success = false;
      }
    }
  }
  if( success ) {
    DPRINTF( "[P] Web Server File Status: Response=Success\n" );
    webserver.server.send( 200, "text/plain", "ACK" );
  } else {
    DPRINTF( "[P] Web Server File Status: Response=Failure\n" );
    webserver.server.send( 403, "text/plain", "NACK" );
  }
}

static void handleCommand() {
  if( webserver.server.args() == 1 ) {
    char command[5] = { 0, 0, 0, 0, 0 };
    strncpy( command, webserver.server.argName( 0 ).c_str(), sizeof( command ) - 1 );
    strncpy( webserver.scratch, webserver.server.arg( 0 ).c_str(), sizeof( webserver.scratch ) - 1 );
    const bool isgetcmd = command[0] == 'G';
    const char* data = webserver.scratch;
    DPRINTF( "[P] Web Server Command Packet Received: Type=%s, Data=\"%s\"\n", command, data );
    switch( hashcode4( command ) ) {
      case hashcode4( "KILL" ) : {
        DPRINTF( "[P] Web Server Command: Type=KILL\n" );
        peripheral_esp32KillCommand();
        break;
      }
      case hashcode4( "MANL" ) : {
        DPRINTF( "[P] Web Server Command: Type=MANUAL OVERRIDE\n" );
        kafenv.info.triggerLock = 1;
        FLTSYNC;
        kafenv.info.actuation = true;
        kafenv.info.flightMode = ACTUATION_MODE;
        FPFILL0( i, kafenv.cmd.motors );
        kafenv.info.triggerLock = 0;
        break;
      }
      case hashcode4( "GNET" ) : {
        DPRINTF( "[P] Web Server Command: Type=GET WIFI CONFIG\n" );
        char ip[26], name[26], password[26]; bool isap;
        peripheral_wifiNetwork( false, name, password, ip, &isap );
        SNFORMAT( 0, "{\"ip\":\"%s\",\"name\":\"%s\",\"password\":\"%s\",\"ap\":%u,\"id\":%u}", 
            ip, name, password, isap, kafenv.info.deviceID );
        break;
      }
      case hashcode4( "GSPT" ) : {
        DPRINTF( "[P] Web Server Command: Type=GET COMMAND SETPOINTS\n" );
        unsigned short idx = SNFORMAT( 0, "{" );
        idx = snfloats( "sp", kafenv.cmd.setpoints, idx, FPARLEN( kafenv.cmd.setpoints ) );
        SNFORMAT( idx, "}" );
        break;
      }
      case hashcode4( "GFLT" ) : {
        DPRINTF( "[P] Web Server Command: Type=GET FLIGHT DATA\n" );
        unsigned short idx = SNFORMAT( 0, "{\"armed\":%u,\"mode\":%u,\"step\":%u,", 
            kafenv.info.actuation ? 1 : 0, kafenv.info.flightMode, ( unsigned int )( FLIGHT_BUFFER.timeStep * 1000 ) );
        idx = snfloats( "x", kafenv.state.x.f, idx, FPARLEN( kafenv.state.x.f ) );
        idx += SNFORMAT( idx, "," );
        float rotMat[9];
        flight_rotationMatrix( rotMat );
        idx = snfloats( "r", rotMat, idx, FPARLEN( rotMat ) );
        idx += SNFORMAT( idx, "," );
        idx = snfloats( "t", kafenv.cmd.motors, idx, FPARLEN( kafenv.cmd.motors ) );
        idx += SNFORMAT( idx, "," );
        float vec3[3];
        ITRVEC3( i ) vec3[i] = kafenv.cal.accelfilt[i].gain * ( FLIGHT_BUFFER.accelInput.f[i] - kafenv.cal.accelfilt[i].ofst );
        idx = snfloats( "a", vec3, idx, 3 );
        idx += SNFORMAT( idx, "," );
        ITRVEC3( i ) vec3[i] = kafenv.cal.gyrofilt[i].gain * ( FLIGHT_BUFFER.gyroInput.f[i] - kafenv.cal.gyrofilt[i].ofst );
        idx = snfloats( "w", vec3, idx, 3 );
        SNFORMAT( idx, "}" );
        registerGS();
        break;
      }
      case hashcode4( "GLOG" ) : {
        DPRINTF( "[P] Web Server Command: Type=GET LOGGING DATA\n" );
        unsigned short idx = SNFORMAT( 0, "{\"id\":%u,\"periphs\":[", kafenv.info.deviceID );
        for( unsigned char i = 0; i < 30; i++ ){
          const peripheral* periph = firmware_getPeripheral( i );
          if( periph->name[0] == 0 ) {
            break;
          }
          if( i == 0 ) {
            idx += SNFORMAT( idx, "\"%s\"", periph->name );
          } else {
            idx += SNFORMAT( idx, ",\"%s\"", periph->name );
          }
        }
        SNFORMAT( idx, "]}" );
        break;
      }
      case hashcode4( "GMAN" ) : {
        DPRINTF( "[P] Web Server Command: Type=GET MANUAL DATA, SET MOTOR CONTROL\n" );
        float sp[4];
        char chform[20];
        for( unsigned char i = 0; i < 4; i++ ) {
          data = inputnext( chform, data, sizeof( chform ) );
          sp[i] = 0.5F * strtof( chform, NULLPTR );
        }
        DPRINTF( "[P] Web Server Command Data: SP=[ %.3f, %.3f, %.3f, %.3f ]\n", sp[0], sp[1], sp[2], sp[3] );
        if( kafenv.info.actuation && ( kafenv.info.flightMode & ACTUATION_MODE ) == ACTUATION_MODE ) {
          kafenv.cmd.motors[0] = 0.5F - sp[3] + sp[0] + sp[1] - sp[2];
          kafenv.cmd.motors[1] = 0.5F - sp[3] - sp[0] + sp[1] + sp[2];
          kafenv.cmd.motors[2] = 0.5F - sp[3] + sp[0] - sp[1] + sp[2];
          kafenv.cmd.motors[3] = 0.5F - sp[3] - sp[0] - sp[1] - sp[2];
          for( unsigned char i = 0; i < 4; i++ ) {
            kafenv.cmd.motors[i] = kafenv.cmd.motors[i] > 0 ? ( kafenv.cmd.motors[i] < 1 ? kafenv.cmd.motors[i] : 1 ) : 0;
          }
        }
        unsigned short idx = SNFORMAT( 0, "{\"armed\":%u,\"manual\":%u,\"battery\":%f,", 
            kafenv.info.actuation ? 1 : 0, ( kafenv.info.flightMode & ACTUATION_MODE ) == ACTUATION_MODE, kafenv.info.battery );
        idx = snfloats( "motors", kafenv.cmd.motors, idx, FPARLEN( kafenv.cmd.motors ) );
        SNFORMAT( idx, "}" );
        registerGS();
        break;
      }
      case hashcode4( "GCAL" ) : {
        DPRINTF( "[P] Web Server Command: Type=GET CALIBRATION DATA\n" );
        unsigned short idx = SNFORMAT( 0, "{" );
        idx = snfloats( "x", kafenv.state.x.f, idx, FPARLEN( coordinate ) );
        idx += SNFORMAT( idx, "," );
        idx = snfloats( "v", kafenv.state.v.f, idx, FPARLEN( coordinate ) );
        SNFORMAT( idx, ",\"time\":%f,\"var\":%f}", 0.0F, 0.0F );
        break;
      }
      case hashcode4( "DALL" ) : {
        DPRINTF( "[P] Web Server Command: Type=DOWNLOAD PERSISTENT\n" );
        unsigned short len = firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, 'S' );
        if( len > 0 && len < sizeof( COMS_BUFFER ) ) {
          webserver.server.send_P( 200, "application/octet-stream", COMS_BUFFER, len );
        } else {
          const char byte = 0;
          webserver.server.send_P( 200, "application/octet-stream", &byte, 1 );
        }
        return;
      }
      case hashcode4( "DCAL" ) : {
        DPRINTF( "[P] Web Server Command: Type=DOWNLOAD CALIBRATION\n" );
        webserver.server.send_P( 200, "application/octet-stream", ( char* )&kafenv.cal, sizeof( kafenv.cal ) );
        return;
      }
      case hashcode4( "DPRP" ) : {
        DPRINTF( "[P] Web Server Command: Type=DOWNLOAD PERIPHERAL\n" );
        char index[3];
        inputnext( index, data, sizeof( index ) );
        unsigned char idx = ( unsigned char )strtol( index, NULLPTR, 10 );
        DPRINTF( "[P] Web Server Command Data: Index=%u\n", idx );
        const peripheral* periph = firmware_getPeripheral( idx );
        if( periph == NULLPTR || periph->memory == NULLPTR || periph->length == 0 ) {
          const char byte = 0;
          webserver.server.send_P( 200, "application/octet-stream", &byte, 1 );
        } else {
          webserver.server.send_P( 200, "application/octet-stream", ( char* )periph->memory, periph->length );
        }
        return;
      }
      case hashcode4( "CHVR" ) : {
        DPRINTF( "[P] Web Server Command: Type=SET HOVER MODE\n" );
        kafenv.info.triggerLock = 1;
        FLTSYNC;
        kafenv.info.flightMode = ( kafenv.info.flightMode & CMD_MODE_MASK ) | ACCEL_SETPOINT_MODE;
        kafenv.info.actuation = true;
        FPFILL0( i, kafenv.cmd.motors );
        FPFILL0( i, kafenv.cmd.setpoints );
        kafenv.info.triggerLock = 0;
        break;
      }
      case hashcode4( "CSAV" ) : {
        DPRINTF( "[P] Web Server Command: Type=SAVE STATE TO DISK\n" );
        firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, 'W' );
        break;
      }
      case hashcode4( "CSX0" ) : {
        DPRINTF( "[P] Web Server Command: Type=ZERO OUT STATE ESTIMATION\n" );
        FPFILL0( i, kafenv.state.x.f );
        FPFILL0( i, kafenv.state.v.f );
        FPFILL0( i, kafenv.state.q.f );
        FPFILL0( i, kafenv.state.w.f );
        break;
      }
      case hashcode4( "CRST" ) : {
        DPRINTF( "[P] Web Server Command: Type=RESET QUADCOPTER\n" );
        firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, 'D' );
        peripheral_esp32KillCommand();
        break;
      }
      case hashcode4( "SCNM" ) : {
        DPRINTF( "[P] Web Server Command: Type=COMMANDER SET NOMINAL MODE, Value=%c\n", data[0] );
        kafenv.info.flightMode = ( kafenv.info.flightMode & DEFAULT_MODES_MASK ) | ( data[0] == '1' ? CMD_NOMINAL_MODE : CMD_IDLE_MODE );
        break;
      }
      case hashcode4( "SFLT" ) : {
        DPRINTF( "[P] Web Server Command: Type=SET FLIGHT MODE, Mode=%c\n", data[0] );
        kafenv.info.flightMode = ( kafenv.info.flightMode & CMD_MODE_MASK ) | ( ( data[0] - '0' ) & DEFAULT_MODES_MASK );
        break;
      }
      case hashcode4( "SARM" ) : {
        DPRINTF( "[P] Web Server Command: Type=ARMING, Value=%c\n", data[0] );
        kafenv.info.actuation = data[0] == '1';
        break;
      }
      case hashcode4( "SNET" ) : {
        DPRINTF( "[P] Web Server Command: Type=SET WIFI CONFIG\n" );
        char ip[26], name[26], password[26], id[3], isap[2];
        data = inputnext( ip, data, sizeof( ip ) );
        data = inputnext( name, data, sizeof( name ) );
        data = inputnext( password, data, sizeof( password ) );
        data = inputnext( id, data, sizeof( id ) );
        data = inputnext( isap, data, sizeof( isap ) );
        bool isapbool = isap[0] == '1';
        kafenv.info.deviceID = ( unsigned char )strtol( id, NULLPTR, 16 );
        DPRINTF( "[P] Web Server Command Data: Ip=\"%s\", Name=\"%s\", Password=\"%s\", AP=%u, ID=%02x\n", 
            ip, name, password, isapbool, kafenv.info.deviceID );
        peripheral_wifiNetwork( true, name, password, ip, &isapbool );
        break;
      }
      case hashcode4( "SSPT" ) : {
        DPRINTF( "[P] Web Server Command: Type=SET COMMAND SETPOINTS\n" );
        char flstr[20];
        for( unsigned char i = 0; i < FPARLEN( kafenv.cmd.setpoints ); i++ ) {
          data = inputnext( flstr, data, sizeof( flstr ) );
          kafenv.cmd.setpoints[i] = strtof( flstr, NULLPTR );
        }
        break;
      }
      case hashcode4( "STRJ" ) : {
        DPRINTF( "[P] Web Server Command: Type=SET TRAJECTORY\n" );
        char trajtype[2], value[15];
        data = inputnext( trajtype, data, sizeof( trajtype ) );
        float cmdargs[4];
        for( unsigned char i = 0; i < 4; i++ ) {
          data = inputnext( value, data, sizeof( value ) );
          cmdargs[i] = strtof( value, NULLPTR );
        }
        DPRINTF( "[P] Web Server Command Data: Type=%c, Arg0=%.3f, Arg1=%.3f, Arg2=%.3f, Arg3=%.3f\n", 
            trajtype[0], cmdargs[0], cmdargs[1], cmdargs[2], cmdargs[3] );
        commander_setTrajectories( trajtype[0] - '0', cmdargs );
        break;
      }
      case hashcode4( "SCAL" ) : {
        DPRINTF( "[P] Web Server Command: Type=SET Calibration\n" );
        char type[4], valuestr[15];
        data = inputnext( type, data, sizeof( type ) );
        inputnext( valuestr, data, sizeof( valuestr ) );
        if( type[0] != 0 && type[1] != 0 ) {
          const unsigned char id = strtol( &type[1], NULLPTR, 10 );
          const float value = strtof( valuestr, NULLPTR );
          DPRINTF( "[P] Web Server Command Data: Type=%c, ID=%u, Arg3=%.3f\n", type[0], id, value );
          switch( type[0] ) {
            case 'p' : {
              if( id < FPARLEN( coordinate ) ) {
                kafenv.state.x.f[id] = value;
              }
              break;
            }
            case 'v' : {
              if( id < FPARLEN( coordinate ) ) {
                kafenv.state.v.f[id] = value;
              }
              break;
            }
            case 'c' : {
              if( id < FPARLEN( kafenv.cal ) ) {
                ( ( float* )( ( void* )&kafenv.cal ) )[id] = value;
              }
              break;
            }
          }
        }
        break;
      }
      default : {
        DPRINTF( "[P] Web Server Command: Type=INVALID, Response=NAC\n" );
        webserver.server.send( 403, "text/plain", "NACK" );
      }
    }
    if( isgetcmd ) {
      DPRINTF( "[P] Web Server Command Reply: Response=%s\n", webserver.scratch );
      webserver.server.send( 200, "application/json", webserver.scratch );
    } else {
      DPRINTF( "[P] Web Server Command Success: Response=ACK\n" );
      webserver.server.send( 200, "text/plain", "ACK" );
    }
  } else {
    DPRINTF( "[P] Web Server Command Failure: Response=NACK\n" );
    webserver.server.send( 403, "text/plain", "NACK" );
  }
}

static void handleComs() {
  if( webserver.server.args() == 1 && webserver.server.hasArg( "DATA" ) ) {
    DPRINTF( "[P] Web Server Binary Command Packet Received\n" );
    String data = webserver.server.arg( "DATA" );
    const unsigned int len = data.length();
    unsigned char upper = 0;
    for( unsigned short i = 0; i < len && i / 2 < sizeof( COMS_BUFFER ); i++ ) {
      unsigned char lower;
      unsigned char bn;
      if( ( bn = data[i] - 'A' ) <= ( 'F' - 'A' ) && bn >= 0 ) {
        lower = (unsigned char)( bn + 0xA );
      } else if( ( bn = data[i] - 'a' ) <= ( 'f' - 'a' ) && bn >= 0  ) {
        lower = (unsigned char)( bn + 0xA );
      } else if( ( bn = data[i] - '0' ) <= ( '9' - '0' ) && bn >= 0  ) {
        lower = (unsigned char)bn;
      } else {
        lower = 0;
      }
      if( i % 2 == 0 ) {
        upper = lower << 4;
      } else {
        COMS_BUFFER[ i / 2 ] = upper | lower;
      }
    }
    webserver.comsCmdLen = len / 2 < sizeof( COMS_BUFFER ) ? len / 2 : sizeof( COMS_BUFFER );
    webserver.comsRespLen = 0;
    webserver.coms.currentTime = millis();
    com_step( &webserver.coms );
    if( webserver.comsRespLen > 1 ) {
      webserver.server.send( 200, "text/plain", webserver.scratch );
    } else {
      webserver.server.send( 200, "text/plain", "" );
    }
  } else {
    webserver.server.send( 403, "text/plain", "NACK" );
  }
}

static void handleComsReply( void* ptr, unsigned short len ) {
  unsigned char* buffer = ( unsigned char* )ptr;
  webserver.comsRespLen = len * 2;
  webserver.comsRespLen = webserver.comsRespLen > sizeof( webserver.scratch ) - 1 ? 
      sizeof( webserver.scratch ) - 1 : webserver.comsRespLen;
  for( unsigned short i = 0; i < webserver.comsRespLen; i++ ) {
    unsigned char bn = buffer[ i / 2 ];
    bn = i % 2 == 0 ? ( bn >> 4 ) : ( bn & 0xF );
    webserver.scratch[i] = bn + ( bn < 10 ? '0' : ( 'A' - 10 ) );
  }
  webserver.scratch[ webserver.comsRespLen++ ] = 0;
}

void peripheral_webserverLoop() {
  if( webserver.serverInitialized ) {
    //DPRINTF( "[P] WebServer Handling Client\n" );
    webserver.server.handleClient();
  } else {
    webserver.server.begin();
    webserver.serverInitialized = true;
    DPRINTF( "[P] WebServer Started Successfully\n" );
  }
}

void peripheral_webserverInit() {
  firmware_registerPeripheral( { "webserver", 0, sizeof( webserver ), &webserver, &peripheral_webserverInit, &peripheral_webserverLoop } );
  DPRINTF( "[P] Initializing Web Server\n" );
  webserver.serverInitialized = false;
  webserver.comsCmdLen = 0;
  webserver.comsRespLen = 0;
  memset( webserver.scratch, 0, sizeof( webserver.scratch ) );
  webserver.coms = { 0, WEBSERVER_COM_METHOD, false, true, COMS_BUFFER, 
      [](){ return webserver.comsCmdLen; }, handleComsReply, []( void* ptr, unsigned short len ) { } };
  DPRINTF( "[P] Webpage Size: root=%u, styles=%u, app=%u, icon=%u\n", sizeof( root ), sizeof( styles ), sizeof( app ), sizeof( icon ) );
  DPRINTF( "[P] Webpage Size: homepage=%u, manual=%u, logging=%u, calibration=%u\n", 
      sizeof( homepage ), sizeof( manual ), sizeof( logging ), sizeof( calibration) );
  webserver.server.on( "/", HTTP_GET, []() { webserver.server.send_P( 200, "text/html", root ); } );
  webserver.server.on( "/styles.css", HTTP_GET, []() { webserver.server.send_P( 200, "text/css", styles ); } );
  webserver.server.on( "/app.js", HTTP_GET, []() { webserver.server.send_P( 200, "text/javascript", app ); } );
  webserver.server.on( "/icon.png", HTTP_GET, []() { webserver.server.send_P( 200, "image/png", icon, sizeof( icon ) ); } );
  webserver.server.on( "/homepage.html", HTTP_GET, []() { webserver.server.send_P( 200, "text/html", homepage ); } );
  webserver.server.on( "/manual.html", HTTP_GET, []() { webserver.server.send_P( 200, "text/html", manual ); } );
  webserver.server.on( "/logging.html", HTTP_GET, []() { webserver.server.send_P( 200, "text/html", logging ); } );
  webserver.server.on( "/calibration.html", HTTP_GET, []() { webserver.server.send_P( 200, "text/html", calibration ); } );
  webserver.server.on( "/upload", HTTP_POST, handleUploadsEnd, handleUploads );
  webserver.server.on( "/cmd", HTTP_GET, handleCommand );
  webserver.server.on( "/com", HTTP_GET, handleComs );
}