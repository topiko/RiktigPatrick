use <servo_mount.scad>;
use <servos.scad>;
use <utils.scad>;
use <electronics.scad>;
include <standards.scad>;
include <dims.scad>;

use <gears/gears.scad>;
/*
angle = 110;
servoNTooth = 15;
axleNTooth = round(180/angle*servoNTooth);
*/
//18.2 + 1 + 2 + 2 + 22+ 2
module battrack(key, wallT=bodywallT){
	batD=18.2;
	batL=66;
	bottomT=.5;
	T = 5;
	W = bodyW - wallT*2;	
	H = 3*batD + 2;
	module batt(){
		for (k=[-1,0,1]){
			translate([0, k*batD, batD/2 + bottomT]) rotate([0,90,0]) cylinder(h=batL + 2, r=batD/2, center=true);
		}
	}
	module groowplate(key, W=W, H=H,  boltD=BOLT3TIGHT){
		if (key=="plate"){
			difference(){
				translate([0, 0, T/2]) cube([W, H, T], center=true);
				batt();					
			}
		}
		else if (key=="bolts"){
			for (i=[-1,1]){
				translate([bodyW/2*i, 0, T/2])
				for (j=[-batD, 0, batD]){
					translate([0, j, 0]) rotate([0, -i*90, 0]) bolt(7, boltD, .5);
				}
			}
		}
	}
		
	module top(key, boltD=BOLT3TIGHT){
		Z = key == "mocktop" || key == "bolts" ? bottomT*2 + batD : 0;
		H = H - 2;
		translate([0,0,Z])
		mirror([0,0,1])
		if (key=="top"){
			difference(){
			groowplate("plate", boltD=boltD, W=W-2*TIGHTSP, H=H);
			groowplate("bolts", boltD=boltD, H=H);
			}
		}
		else if (key=="bolts"){
			groowplate("bolts", boltD=boltD, H=H);
		}
	}

	if (key=="bottom"){groowplate("plate");}
	else if (key=="top"){top("top");}
	else if (key=="mocktop"){top("mocktop");}
	else if (key=="cut"){top("bolts", boltD=BOLT3LOOSE); batt();}
}	

module sideelmount(W, wT, H, T, pcbT){
	for (i=[-1,1]) {
		for (j=[-1,1]){
			translate([i*(-wT/2 + W/2), j*(wT/2 + pcbT/2), H/2]) 
			cube([wT, wT, H], center=true);
		}
		translate([i*(W/2 + T/2), 0, H/2]) cube([T, wT*2 + pcbT, H], center=true);
	}
}


module boardmounts(key="mounts", Hmax=0, sp=0){

	pcbT = 1.9; //1.7 too tight
	elWs = [15,15,10,10];
	Hmax = key=="cut" ? headW : 
	       Hmax == 0 ? 12 : 
	       Hmax;

	wT = .64;
	addx = .4; // spacing along the pch dir
	
	W1 = 20.5 + addx;
	H1 = 11;
	T = 1.5;
	W = 2*W1 + 3*T  - .8;
	shiftX = W1/2 + T/2;
	W2 = 21.4 + addx;
	H2 = 6.5;
	W3 = 15.3 + addx;

	module mounts(){
		translate([0, wT + pcbT/2, 0]){
		translate([shiftX, 0, 0]) sideelmount(W1, wT, Hmax, T, pcbT);
		mirror([1,0,0]) translate([shiftX, 0, 0]) sideelmount(W1, wT, Hmax, T, pcbT);
		translate([0, H1, 0]) sideelmount(W2, wT, Hmax, (W - W2)/2, pcbT);
		translate([0, H2+H1, 0]) sideelmount(W3, wT, Hmax, (W - W3)/2, pcbT);
		}
		plates();
	}
	
	module plates(botT=1){
		Ly = H1 + H2 + pcbT + 2*wT + 2*sp;
		translate([0, Ly/2 - sp, botT/2])
		cube([W, Ly, botT], center=true);
		translate([W/2-botT, -sp, 0]) cube([botT+sp, Ly, Hmax]);
		translate([-W/2-sp, -sp, 0]) cube([botT+sp, Ly, Hmax]);
	}

	if (key=="mounts"){mounts();}
	else if (key=="cut"){hull() mounts();}
}


module spacer(dout, din, h){
	
	difference(){
		cylinder(h=h, r=dout/2);
		cylinder(h=h, r=din/2);
	}
}


module body(L, W, T, wallT=3, key="none"){
	
	
	attachH = 6;
	bottomT=2.0;
	topT = attachH+bottomT; 
	botT = T-topT;
	subst = 2*(wallT + TIGHTSP*2);
	wheelbearsp = .05;
	Tup = AXLEBEARINGDIMS[1] + 1 + attachH + bottomT + 2;
	bodyfacetheta = atan((T- Tup)/L);

	module servo(key="cut"){
		neckL = 0; //wallT + 0;
		mountT=AXLEBEARINGDIMS[1] + 1; 
		module servo_(){
			translate([neckX,-botT, L/2 - wallT])
			mirror([1,0,0])
			servo_mount_aligned(key=key, servoNtooth=SERVOGEARNTOOTH, 
					    turnAngle=130, axleL=12, 
					    armAngleMiddle=270,
					    axlehornDin=AXLEHORNDIN, type=3, 
					    hornarmL=neckL, T=mountT, addXL=12, threadhandness="left");

		}

		module centered_mount(){
			difference(){
				servo_();
				bodyshell(wallT=wallT+.1, key="bottom");
				bodyshell(wallT=wallT+.1, key="top");
			}
		}

		if (key=="top"){			
			intersection(){
				servo_();
				bodyshell(key="bulk", L=L-subst, W=W-subst, T=T-subst, R=cornerR - subst/2);
			}
		}
		else {servo_();}
	}
	
	function get_y(z, DY) = -(L/2)*tan(bodyfacetheta) - z*tan(bodyfacetheta)  + topT + DY; 	

	module closebolts(key="bolts", boltD=BOLT25TIGHT){
		NH = 3;
		NW = 3;
		edgeD = 10;
		
		dH = (L/cos(bodyfacetheta)-2*edgeD)/(NH-1);
		dW = (W-2*edgeD)/(NW-1);
			
		sink = .3;
		mnthH = 2;
		mnthR = 3.;
		sz=-bottomT - attachH/2; // 0; // -topT/cos(bodyfacetheta); 	

		module boltrow(){
			//translate([-(N-1)*dx/2, 0, 0])
			
			for (k=[0:1:NH-1]){
				zmin = -L/2 + edgeD;
				z = zmin + k*dH;
				translate([W/2, get_y(z, sz), z]) 
				rotate([0, -90,0])
				if (key=="bolts"){bolt(mnthH + wallT, boltD, sink);}
				else if (key=="mnts"){	
					translate([0,0,wallT]) cylinder(h=mnthH, r1=mnthR, r2=mnthR-mnthH);
				}
			}

		}
		module sidebolts(side){
			rotY = side =="left" ? 90 : -90;
			transX = side == "left" ? -W/2 : W/2;
			//translate([0,0,-dH])
			//rotate([bodyfacetheta, 0, 0])
			//translate([transX, attachH/2, dH]) //-topT + attachH/2, 0])
			rotate([0, rotY, 0]) 
			boltrow(NH, dH);
		}

		boltrow();
		mirror([1, 0, 0]) boltrow();
		//sidebolts("left");
		//sidebolts("right");
		
		
	}

	//echo(get_y(-51, 0) - 2 - 2 - 20, botT);
	module bodyshell(wallT=wallT, key="bottom", bottomT=bottomT, L=L, W=W, T=T, R=cornerR){

		sp = TIGHTSP;
		
		module bulk(L, W, T, R_){
			rotate([-90,0,0])
			difference(){
				linear_extrude(height=T) offset(R_) square([W-2*R, L-2*R], center=true);
				translate([0, -L/2, T-L*tan(bodyfacetheta)]) rotate([bodyfacetheta, 0, 0]) translate([-500,0,0])cube(1000);
			}
		}
		
		module shell(L, W, T, R, wallT=wallT, bottomT=bottomT, x=2){	
			bulkL = 60;
			translate([0,-T,0])
			difference(){
				bulk(L, W, T, R);
				translate([0, bottomT, 0]) bulk(L, W, x*T, R - wallT);	
			}
		}
		
		module attachring(L, W, T, wallT){
			//translate([0,T, 0])
			difference(){
				translate([0,T, 0]) shell(L, W, L*tan(bodyfacetheta) + T, R, wallT=wallT, bottomT=-.0);
				translate([0, 0, -L/2]) rotate([bodyfacetheta, 0, 0]) translate([-W, -100, 0]) cube([2*W, 100, 2*L]);
			}
		}	

		if (key=="top"){
			translate([0,topT, 0])
			difference(){
				//mirror([0,1,0]) 
				shell(L, W, L*tan(bodyfacetheta) + topT, R, wallT = wallT, bottomT=-bottomT, x=1);
				translate([0, -1000 - topT + TIGHTSP*2, 0]) bulk(L, W, 1000, R);		
				translate([0, -topT, 0]) attachring(L, W, attachH, wallT/2 + sp/2);
			}
		}
		else if (key=="bottom"){
			shell(L, W, botT, R, wallT=wallT);
			attachring(L, W, attachH, wallT/2 - sp/2);
		}
		else if (key=="bulk"){
			translate([0, topT, 0])
			shell(L, W, T, R, wallT=L);
		}
		else if (key=="cutring"){
			attachring(L, W, attachH, wallT/2 + sp/2);
		}
	

	}
	
	//bodyshell(wallT=wallT, key="top"); //, bottomT=bottomT, L=L, W=W, T=T, R=cornerR, bodyfacetheta=0){
	shiftZ = -2.3;
	module shiftedbattrack(key){
		posX = 0;
		posY = -botT;
		posZ = -L/2 + wallT + 60.0 + 18*3/2 + shiftZ; //-17;
		if (key!="top"){
			translate([posX, posY, posZ]) rotate([-90,0,0]) 
			battrack(key);
		}
		else if (key=="top"){
			battrack(key);
		}

	}	

	module shiftedservoctrl(key){
		dy = .5;
		posY = -botT + dy;
		posX = 0;
		posZ = 20.5;
		translate([posX, posY, posZ]){
		rotate([-90,0,0]) servoctrl(H=bottomT-dy + 2, key=key); //, boltH=15);
		translate([0,0,25]) 
		rotate([-90,0,0]) protoboard(H=bottomT-dy + 3, key=key); // , boltH=15);
		}
	}

	module shiftedspeaker(key){
		//TODO:
		speakerH = 3;
		speakerR = 20.9/2;
		mountT = 2;

		module speaker(){cylinder(h=speakerH, r=speakerR);}
		module speakermount(){
			difference(){
				cylinder(h=speakerH, r=speakerR+mountT);
				speaker();
			}
		}	
		
		posX = (-W/2 + wallT + speakerR + mountT/2);
		posY = -botT;
		posZ = speakerR + 25.5 + shiftZ;
		translate([posX, posY, posZ]) rotate([-90,0,0]) speakermount();
	}

	module shiftedswitch(key){
		T = 2;
		wT = 1;
		swichframeW = 8.2 + 2*TIGHTSP;
		swichframeH = 7.3 + 2*TIGHTSP;
		swichframeT = 5.4;
			
		module swichmount(){
			translate([-swichframeH/2 - wT, 0, 0])
			rotate([-90,0,0]) rotate([0,0,90]) sideelmount(swichframeW, wT, swichframeT/3*2, T, swichframeH);
		}
		module cut(){
			armT = 3;
			alpha = 40;
			translate([+wT/2, -armT/2 + swichframeT/2, 0])
			rotate([-90,0,0])
			rotate([0,0,-alpha/2])
			rotate_extrude(angle=alpha) square([20, armT]);
		}
		
		posX = W/2 - wallT - swichframeW/2 - TIGHTSP;
		posY = -botT + bottomT;
		posZ = L/2 - wallT - 8; //  24.5 + swichframeW/2 + T + shiftZ;
		
		mirror([1,0,0])
		translate([posX, posY, posZ])
		rotate([0, -90, 0])
		if (key=="cut"){cut();}
		else if (key=="mount"){swichmount();}
	}
	
	module tests(){
		module tmpwall(){translate([W/2 - wallT, -botT + bottomT, 20]) cube([wallT, 7, 30]);}
		
		shiftedswitch("mount");
		difference(){
			tmpwall();	
			shiftedswitch("cut");
			shiftedbattcable();
		}
		translate([0, -bodyT + 9, 0]) cube([bodyW, .5, bodyH], center=true);
		shiftedelmounts("mounts");
		translate([0,10,0]) motormounts("top");
		
		// Wheel bearing test:
		translate([35/2, 0, 0])
		rotate([0,90,0]) 
		difference(){
			cube([35,35,5], center=true);
			cylinder(h=4, r=WHEELBEARINGDIMS[1]/2 + wheelbearsp);
			cylinder(h=9, r=WHEELBEARINGDIMS[1]/2 - 4, center=true);
		}
		
		// Hand servo:
		translate([0,80,0])
		difference(){
			translate([-12, 0, -8])cube([40, wallT, 16]);
			dymond_servo("mockup");
			dymond_servo("bolts");
		}
	}

	module shiftedbattcable(){
		connectorT=2.4 + .2;
		connectorW=8.1 + 2*TIGHTSP;

		translate([-W/2+wallT+13,-botT+bottomT, L/2-wallT*2]) cube([connectorW, connectorT, 20]);
	}
	module shiftedmicarray(key="mockup"){	
		posZ = 65/2 - 51; //-L/2 + 65/2 + wallT + 5; // L/2 - 35;
		posY = get_y(posZ, - bottomT); // topT - bottomT;
		poleH = 1;
	
		translate([0, posY, posZ]) rotate([90 + bodyfacetheta, 0, 0]) micarray(H=poleH, key=key, boltH=poleH + bottomT - .5);	
	}
	
	//echo("TT");
	//echo(T - tan(bodyfacetheta)*(L/2-51) - 22);
	module motormounts(key="cut"){

		R = WHEELBEARINGDIMS[1];
		T = WHEELBEARINGDIMS[2];
		axlehornDin = WHEELBEARINGDIMS[0] - 4; 
		mountT= R+2;
		axleL = 10;
		wheelbearingXout = T - 3; //- wallT/2;
		addXR = 8;
		hornarmL=2;

		module unit(key, shift=true, side="left", threadhandness="none"){
			module mount(){
				servo_mount_aligned(key=key, servoNtooth=20, axleNtooth=26, 
					    //turnAngle=120, 
					    axleL=axleL, bearingdims=WHEELBEARINGDIMS,
					    axlehornDin=axlehornDin, type=5, addXR=addXR, 
					    threadhandness=threadhandness, T=mountT, hornarmL=hornarmL, boltkey="flat");
				if (key=="cut"){translate([0,mountT/2,-T]) cylinder(h=T, r=R/2 + wheelbearsp);}

			}

			mirax = side=="left" ? 0 : 1;
			threadhandness = threadhandness=="right" || threadhandness=="left" ? threadhandness :
					 side =="right" ? "right" : "right"; // WAS: "left" : "right"

			mirror([mirax,0,0])
			if (shift){
				translate([W/2 + wheelbearingXout, -botT, -L/2 + wallT + R/2 + addXR])
				rotate([0,90,0]) mount();
			}
			else {translate([0, -mountT/2, 0]) mount();}
		}

		module both(key){	
			unit(key, side="left");
			unit(key, side="right");
		}
				
		if (key=="bottom"){
			intersection(){
				both(key);
				bodyshell(key="bulk");
			}
		}
		else if (key=="top"){
			echo(cornerR, subst/2);
			difference(){
			intersection(){
				both(key);
				bodyshell(key="bulk", L=L-subst, W=W-subst, T=60, R=cornerR - subst/2);
			}
			shiftedelmounts("cut");
			}
				
		}
		else if (key=="cut"){
			both(key);
		}
		else if (key=="axleparts"){
			both(key);
		}
		else if (key=="axleNOTgearRight" || key=="axleNOTgearLeft"){
			side = key == "axleNOTgearRight" ? "right" : "left";
			key = "axleNOTgear";
			unit(key, false, side=side, threadhandness="right");
			
		}
		else if (key=="gear"){
			unit(key, false);
		}
		else {unit(key);}

	
	}

	module arm(key){
		
		module servo_(key, boltH=2){	
		posX = -(W/2-wallT);
		posY = -botT + 10.2/2 + bottomT + 6; //12;
		posZ = L/2 - 30;
		translate([posX, posY, posZ])
		rotate([90,0,0])
		rotate([0,0,90])
		dymond_servo(key, boltH=boltH, boltD=BOLT3LOOSE);
		}
		if (key=="mockup"){servo_("mockup");}
		else if (key=="cut"){
			servo_(key="bolts", boltH=wallT);
			servo_("mockup");
		} //wallT);
	}	
	

	module cablecutter2(){
		corner = 5;
		cableW = 14;
		posX = neckX - (15 + cornerR); //-(W/2 - cableW/2 - R - wallT);
		posY = cableW/2 -botT + bottomT; // + corner + .1;
		posZ = L/2;
		cutH = 10;
		translate([posX, posY, posZ])
		linear_extrude(cutH*2, center=true) rotate([0,0,90]) offset(corner) square([cableW - 2*corner, .0001], center=true);
	}
	
	
	module shiftedelmounts(key, sp=0){
		H = (WHEELBEARINGDIMS[1] + 1)/2 - bottomT;
		translate([0, -botT + bottomT, -L/2+wallT])
		mirror([0,1,0])
		rotate([90,0,0])
		boardmounts(key, Hmax=H, sp=sp);
	}

	module heart(key){
		posX = -W/6;
		posZ = L/4;
		posY = get_y(posZ, -.5);
		hw = 30;
		wt = 1.2; // wallT
		hframe = 6;
		ledD = 5;

		if (key=="cut"){
			translate([posX, posY, posZ])
			rotate([bodyfacetheta,0,0])
			translate([0,-hw/2, 0]) 
			cube(hw, center=true);
		}
		else if (key=="ledframe"){
			difference(){
				cube([hw-3*TIGHTSP, hw-3*TIGHTSP, hframe] ,center=true);
				translate([0,0,-wt]) cube([hw - 2*wt, hw-2*wt, hframe], center=true);
				cylinder(h=4*hframe, r=ledD/2 + TIGHTSP);
			}	
		} 
	
	}



	module bottom(){
		module wadds(){
			bodyshell(key="bottom", wallT=wallT);
			intersection(){
				servo("bottom");
				bodyshell(key="bulk", wallT=wallT);
			}	
			intersection(){
				motormounts("bottom");
				bodyshell(key="bulk", wallT=wallT);
			}
			shiftedbattrack("bottom");
			shiftedservoctrl("poles");
		}

		module wcuts(){
			difference(){
			wadds();
			servo("cut");
			//shiftedservocontroller(key="bolts");
			motormounts("cut");
			arm("cut");
			closebolts(key="bolts");
			cablecutter2();
			shiftedbattrack("cut");
			shiftedelmounts("cut", sp=2*TIGHTSP);
			shiftedbattcable();
			shiftedswitch("cut");
			shiftedservoctrl("bolts");
			}
			
		}
		//shiftedelmounts("cut");
		/*difference(){
			shiftedelmounts("mounts");
			motormounts("cut");
		}*/
		//shiftedspeaker();
		shiftedswitch("mount");
		wcuts();
	}

	//motormounts("cut");
	module top(){
		module wadds(){
			bodyshell(key="top", wallT=wallT);
			shiftedmicarray(key="poles");
			closebolts(key="mnts");
			//shiftedservocontroller(key="poles");
		}

		module wcuts(){
			difference(){
				wadds();
				//servo("cut");
				shiftedmicarray(key="bolts");
				shiftedmicarray(key="micsleds");
				//motormounts("cut");
				closebolts(key="bolts");
				bodyshell(key="cutring", wallT=wallT);
				heart("cut");
			}
			
		}
		wcuts();
	}
	

	if (key=="mockup"){
		translate([0,0, L/2]){
		bodyshell(key="bulk");
		//bodyshell(key="bottom");
		}
	}
	else if (key=="bottom"){
		bottom();
	}
	else if (key=="top"){
		top();
	}
	else if (key=="servotop"){
		servo(key="top");
	}
	else if (key=="bearingaxles" || key=="servogear" || key=="axle"){
		servo(key=key);
	}
	else if (key=="battracktop"){
		shiftedbattrack("top");
	}
	else if (key=="tests"){tests();}
	else if (key=="axleparts"){
		motormounts(key);
		servo(key);
	}
	else if (key=="wheelaxleparts"){
		motormounts("axleparts");
	}
	else if (key=="axlepartsneck"){
		servo("axleparts");
	}
	else if (key=="spacers"){
		boltD = BOLT25LOOSE;
		N = 8;
		for (x=[0:3*boltD:3*boltD*N]){
			translate([x, 0, 0]) spacer(boltD*2, boltD, 1.5);
		}
	}
	else if (key=="elmounts"){
		shiftedelmounts("mounts");
	}
	else if (key=="wheelaxleLeft"){
		motormounts("axleNOTgearLeft");
	}
	else if (key=="wheelaxleRight"){
		motormounts("axleNOTgearRight");
	}
	else if (key=="cover"){
		axle("cover");
	}
	else if (key=="motorservotop"){
		motormounts("top");
	}
	else if (key=="speakermount"){	
		shiftedspeaker();
	}
	else if (key=="heartframe"){	
		heart("ledframe");
	}
	
}

key = "wheelaxle"; // "axleparts"; //"bottom"; //"tests"; //"bottom"; //"tests"; //"battracktop"; //"tests"; // "xbottom"; //"tests"; //"bottom"; //"axleparts";// "bottom"; //"tests"; // "top"; // "bottom"; //"mockup"; //"bottom"; //, "servobottom", "axle", "bearingaxles", "servogear", "top" 

//body(bodyH, bodyW, bodyT, key=key);
//leg();
//body(bodyH, bodyW, bodyT, key="heartframe");
//body(bodyH, bodyW, bodyT, key="battracktop");
//body(bodyH, bodyW, bodyT, key="top");
//body(bodyH, bodyW, bodyT, key="bottom");
//body(bodyH, bodyW, bodyT, key="elmounts");
//body(bodyH, bodyW, bodyT, key="servotop");
//body(bodyH, bodyW, bodyT, key="motorservotop");
//body(bodyH, bodyW, bodyT, key="axleparts");
//body(bodyH, bodyW, bodyT, key="axlepartsneck");
//boardmounts("cut");
