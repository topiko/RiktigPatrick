use <gears/gears.scad>;
use <servos.scad>;
use <utils.scad>;
include <dims.scad>;

module wedge(H, R, phi_, R2=100, roundedge=false, T=-1, cornerR=-1){

        module cutter(i, phi, addAngle){
		if (roundedge){
			rotate([0,0,i*(phi-addAngle) ])	
			translate([-R,i*R,H/2])
			rotate([0,90,0])
			difference(){
				translate([0,0,R2])cube([H,H,2*R2], center=true);
				translate([0, -i*H/2,0])cylinder(h=2*R2, r=H/2);
			}
		}
	}
	
	module side(i, phi, addAngle){
		H = H-.001;
    		translate([0,0,.0005])
		hull(){
			cylinder(h=H, r=R);
			rotate_extrude(angle=i*phi) square([R2, H]);
		}
	}

        if (T>0){
	  //H = ;
          hull(){
 	    wedge(cornerR*2,R,phi_, R2=R2, roundedge=true, T=-1);  
 	    translate([0,0,T-2*cornerR]) wedge(cornerR*2,R,phi_, R2=R2, roundedge=true, T=-1);  
          }
        }
 	else {	
	addAngle = asin(R/R2);
	phi = phi_/2 + 2*addAngle;

		
	if (roundedge){	
		difference(){	
			for (i=[-1,1]){rotate([0,0,-i*addAngle]) side(i, phi, addAngle);}
			for (i=[-1,1]){rotate([0,0,-i*addAngle]) cutter(i, phi, addAngle);}
		}
	}
	else {for (i=[-1,1]){rotate([0,0,-i*addAngle]) side(i, phi, addAngle);}}
	}
}

//wedge(10, 10, 180, roundedge=true);

module servo_mount_aligned(key="bottom", servoNtooth=SERVOGEARNTOOTH, turnAngle=180, armAngleMiddle=90, axleNtooth=0, H=0, T=0, W=0, axleL=20, type=3, axlehornDin=7, bearingdims=AXLEBEARINGDIMS, axlecoverT=AXLECOVERT, hornarmL=20, addXR=6, addXL=6, boltkey="csunk", threadhandness="right", hornW=0, usebolts=true, modbolts=false){
	
	axleNtooth = axleNtooth == 0 ? getNTeeth(servoNtooth, turnAngle, SERVOTURNANGLE) : axleNtooth;
	pitchD1 = pitchD(servoNtooth, GEARMODUL); 
	pitchD2 = pitchD(axleNtooth, GEARMODUL); 

	gearSp = .0;
	axleX = (pitchD1 + pitchD2)/2 + gearSp; // .1	


	addX = addXL + addXR;
	addY = 1;
	addZ = 1;
	servodims = mg215dims;

	axleD = type==1 ? servodims[0] : 
		(type==2 || type==3 || type==4) ? min(servodims[2], bearingdims[0]) : 
		type==5 ? bearingdims[0] : 
		100;

	axlehornDout = axlehornDout(axleD, axlecoverT);

	axleL = type==1 || type==6 ? axleL : //servodims[1] - SERVOHORNSP - SERVOHORNBEARINGT : 
		type==2 ? SERVOHORNSP + bearingdims[2]*2 + axlehornDout + 4*SERVOHORNSP : 
		type==3 || type==4 || type==5 ? axleL : 
		0;
	
	mountWx = W!=0 ? W :
		  type==1 || type == 6 ? servodims[0] + (pitchD1/2-servodims[3]) + pitchD2/2 + axleD/2 + addX: 
		  type==2 || type==3 || type==5 ? servodims[3] + addXL + pitchD1/2 + pitchD2/2 + bearingdims[1]/2 + addXR: 
		  type==4 ? servodims[0] - servodims[3] + pitchD1/2 + pitchD2/2 + bearingdims[1]/2 + addX:
		  W;
	echo("Servo mount Wx = ", mountWx, type);

	mountTy = T==0 ? bearingdims[1] + addY: T;
	mountHz = H!=0 ? H : 
		  type==1 || type==6? servodims[1] + SERVOHORNT + SERVOHORNBEARINGT + SERVOHORNSP*3 + addZ :
    		  type==2 || type==3 || type==4 ? servodims[1]/3*2 + axleL + SERVOHORNT + 2*SERVOHORNSP : 
		  type==5 ? servodims[1]/3*2 + axleL + SERVOHORNT + SERVOHORNSP : 
		  H;

	echo("Servo mount Hz = ", mountHz + servodims[1]/3);
	shiftX = type==1 || type == 6? -mountWx - servodims[3] + servodims[0] + addXR : 
		 type==2 || type==3 || type==5 ? - servodims[3] - addXL :
       		 type==4 ?  -(pitchD1/2+pitchD2/2 + bearingdims[1]/2 + addXL): 0; //-(pitchD1-6) - pitchD2;
	shiftZ = type==1 || type==6? -servodims[1] - 25:
		 type==2 || type==3  || type==4 || type==5 ? -servodims[1]/3*2 : 
		 0; 

	axlehornZ = SERVOHORNT + axleL/2 + SERVOHORNSP/2;

	module servo(key){
		kst_mg215_servo(key, hornT=SERVOHORNT, ntooth=servoNtooth, hornSp=SERVOHORNSP, topBearing=true, cableT=2);
	}

	module mount_bulk(key){
			
		module side(){
			translate([shiftX, 0, shiftZ])
			cube([mountWx, mountTy/2, mountHz]);
		}

		module cablecutter(){
			R = axleD/4; //axleD/2;
			translate([axleX + axleD/2 - R, 0, 0])
			rotate([-90,0,0])
			union(){
				cylinder(h=mountTy, r=R);
				sphere(r=R);
			}
		}

		if (key=="bottom"){
			difference(){
				mirror([0,1,0]) side();
				cut();
				//if (type==5){cablecutter();}
			}
			
		}
		else if (key=="top"){
			difference(){
				side();
				cut();
				//if (type==5){cablecutter();}
			}

		}

	}
	
	module closebolts(boltD=BOLT25LOOSE){
		//boltD = BOLT25LOOSE;

		boltD = key == "top" ? BOLT25LOOSE :
			key == "bottom" || key=="cut" ? BOLT25TIGHT :
			boltD;
		boltdist = 2.5;
		boltdist2 = 2.0;
		boltXL =  -(pitchD1/2 + pitchD2/2 + bearingdims[1]/2 + boltdist);
		boltXR = servodims[0] - servodims[3] + boltdist;
		boltX1 =  -(servodims[3] + (pitchD1 + pitchD2)/2 - bearingdims[0]/2)/2;
		boltX2 = bearingdims[1]/2 + boltdist;
		boltX3 = -axleX/2;
		boltX4 = -(servodims[3] + boltdist);
		boltX5 = (SERVOBEARINGDIMS[1]/2 + (axleX-bearingdims[1]/2))/2; 
		boltX6 = (axleX + bearingdims[1]/2 + boltdist2); 
		boltX7 = 0; //(axleX + bearingdims[1]/2 + boltdist2); 
		boltX8 = -axleX; // + bearingdims[1]/2 + boltdist2); 

		topbolthz1 = SERVOHORNT + 1*SERVOHORNSP + bearingdims[2]/2 + addZ;	
		topbolthz3 = SERVOHORNT + 2*SERVOHORNSP + bearingdims[2]/2*3 + addZ;	
		topbolthz2= SERVOHORNT + axleL - bearingdims[2]/2;	
		botbolthz1 = type==1 || type==6 ? -axleL - bearingdims[2]/2 + SERVOHORNSP: -servodims[1]/2;
		botbolthz2 = type==1 || type==6 ? -axleL - bearingdims[2]/2*3: -servodims[1]/2;

		boltposbot = (type==1) || type==6 ? modbolts ? [boltXR, boltX8] : [boltXR, boltX1, boltXL] :  [boltXR, boltX4];
		boltposservo = (type==1) || type==6 ? modbolts ?  [[boltXR, mountTy/2, 0], [boltX1, mountTy/2, -3] ] : [[boltXR, mountTy/2, 0]] : [[boltXR, mountTy/2, -3]];

		boltpostop = (type==1) || type==6 ? modbolts ?  [boltX7, boltX8] : [boltX2, boltX3, boltXL] : [boltX5, boltX6, boltX4];
		boltpostop2 = [boltX5, boltX6];
		R1z = 0;
  
		topbolthz = modbolts ? topbolthz3 : topbolthz1;
		botbolthz = modbolts ? botbolthz2 : botbolthz1;

		//sink = 0;
		//sink = type!= 5 ? max(mountTy/2 - 7, .4) : mountTy/2 - 12;
		sink = .5; //max(mountTy/2 - 7, .4);
			
		boltL = mountTy-boltD/2 - sink - 1;
	        
                if (usebolts){	
		for (x=boltpostop){translate([x, mountTy/2, topbolthz]) rotate([90,0,0]) bolt(boltL, boltD, sink, key=boltkey);}
	
		for (x=boltposbot){translate([x, mountTy/2, botbolthz]) rotate([90,0,0]) bolt(boltL, boltD, sink, key=boltkey);}
		
		if (type==2 || type==3 || type==5){
			for (x=boltpostop2){
				translate([x, mountTy/2, topbolthz2]) rotate([90,0,0])  bolt(boltL, boltD, sink, key=boltkey);
				}
		}	
		for (p=boltposservo) {translate(p) rotate([90,0,0]) bolt(boltL, boltD, sink, key=boltkey);}
                }
	}
	
	module axle(key){
		//D = (pitchD1 + pitchD2)/2;
		module axle_(){
			turn_axle(key, axleNtooth, turnAngle, armAngleMiddle, axleD=axleD, axleL=axleL, axlehornZ=axlehornZ, axlehornDin=axlehornDin, type=type, coverT=axlecoverT, hornarmL=hornarmL, bearingdims=bearingdims, threadhandness=threadhandness, hornW=hornW);}
		if (type==1 || type==6){
			translate([-axleX, 0, SERVOHORNT + SERVOHORNSP])
			rotate([180,0,0]) axle_();
		}
		else if (type==2 || type==3 || type==5){
			rotate([0,0,180])
			translate([-axleX, 0, SERVOHORNSP])
			axle_();
		}
		else if (type==4){
			rotate([0,0,180])
			translate([axleX, 0, SERVOHORNSP])
			rotate([0,0,180])
			axle_();
		}
	}
	

	module cut(){
		axle("cut");
		servo("cut");
		closebolts();	
	}
	//shiftX = 0;	
	echo(-bearingdims[1]/2 - addXR- axleX);
	//L = -bearingdims[1]/2 - addXR- axleX;
	move =  type==1 ? [axleX, 0, +axlehornZ - SERVOHORNT- SERVOHORNSP/2] :
	        type==2 ? [-shiftX - mountWx, mountTy/2, -axlehornZ] :
		type==3 ? [-axleX, 0, (-shiftZ - mountHz)] : 
		type==5 ? [-axleX, 0, (-shiftZ - mountHz)]: 
		type==6 ? [axleX, 0, - SERVOHORNT- 2*SERVOHORNSP - bearingdims[2]]  :
		[0,0,0] ;

	echo(move);
	translate(move)
	if (key=="bottom" || key=="top"){
		mount_bulk(key);
	}
	else if (key=="servogear"){servo("servogear");}
	else if (key=="mockup"){
		mount_bulk("bottom");
		axle("mockup");
		servo("mockup");
		closebolts();	
	}	
	else if (key=="cut"){cut();}
	else if (key=="axleparts"){
		axle("axle");
		if (type==1 || type==6) axle("cover");

		translate([0,40,0]) servo("servogear");
		if (type==1 || type == 6)translate([20,0,0])axle("bearingaxle");
		if (type==2){translate([0,70,0]) axle("cover");}
		if (type==1 || type==2 || type==6){translate([0,20,0]) axle("hornarm");}
		if (type==3){translate([0,20,0]) axle("thread");}
	}
	else if (key=="gear"){
		axle("gear");
	}
	else if (key=="axle"){
		axle("axle");
	}
	else if (key=="bearingaxle"){
		axle("bearingaxle");
	}
	else if (key=="cover"){
		axle("cover");
	}
	else if (key=="axleNOTgear"){
		axle("axleNOTgear");
	}

	//axle("cut");
}

module turn_axle_wheel(key, ntooth, turnAngle, turnAngleMiddle, axleD=12, axleL=20, axlehornDin=8, axlehornZ=15, bearingdims=AXLEBEARINGDIMS, type=1, hornarmL=15, coverT=.64){
	
	turn_axle(key, ntooth, turnAngle, turnAngleMiddle, axleD=12, axleL=20, axlehornDin=8, axlehornZ=15, bearingdims=AXLEBEARINGDIMS, type=5, coverT=.64);
}

module turn_axle(key, ntooth, turnAngle, turnAngleMiddle, axleD=12, axleL=20, axlehornDin=8, axlehornZ=15, bearingdims=AXLEBEARINGDIMS, type=1, hornarmL=15, coverT=.93, lthread=LHORNTHREAD, threadhandness="right", hornW=0){
	
	axleD = (type==1 || type==2 || type==3 || type==4 || type==5 || type==6) ? bearingdims[0] : axleD; //NOTE: removed the spacing between bearing and axle.
	//axlehornZ = type==2 ? axlehornZ - 2*TIGHTSP : axlehornZ;
	braxleL = SERVOHORNT/2;
	hornarmR = axlehornDout(axleD, coverT)/2;
	showthreadaligner = key=="mockup" ? true : SHOWALIGNERS; //showthreadaligner;
	horncornerR = 2;

	module axlehorn(key, boltD=0, boltH=0){
		R = axlehornDin/2; // - TIGHTSP;
		translate([0,0,axlehornZ])
		rotate([0,0,-turnAngleMiddle])
		if (key=="axle"){
			rotate([0,90,0])
			cylinder(h=2*axleD, r=R, center=true);}
		else if (key=="bolt"){
			rotate([0,90,0])
			translate([0,0,-axleD/2]) bolt(boltH, boltD, sink=0); //boltD/2);
		}
		else if (key=="cut"){
			addR = coverT + SERVOHORNSP;
			R = axleD/2 + addR;
			H = hornW>0 ? hornW + 2*SERVOHORNSP : 
			    (type==1 || type==2) ? hornarmR*2 + 2*SERVOHORNSP : 0;
			translate([0,0, -H/2]) wedge(H, R, turnAngle, roundedge=true, T=H, cornerR=horncornerR);
			echo(axleD, H, axleL, bearingdims[2], axlehornZ);
		}
		else if (key=="mockup"){
			/*translate([0,0,axlehornZ])
			rotate([0,0,turnAngleMiddle])*/ 
			rotate([0,0,90]) 
			rotate([-90,0,0]) 
			rotate([0,0,90]) 
			translate([0,0,-hornarmL]) hornarm();
		}
	}
	module hornarm(key="arm"){
		R = type==1 ? axleD/2 + coverT: 
		    type==2 ?  axleD/2 + coverT : 0;	
		
		threadL = lthread==0 ? hornarmL : lthread; // hornarmL;
		hornarmL = hornarmL + R;
		module axle_(addR){
			difference(){
				difference(){
					// TODO: mod this to incorporate wide arms hornW:
					if (hornW>0){
						//cube([hornW, hornW, 200], center=true);
						hull(){
						for (i=[-1,1]){
							for (j=[-1,1]){
								translate([i*(hornW/2 - horncornerR), j*(hornarmR - horncornerR), 0]) cylinder(h=hornarmL, r=horncornerR);
							}
						}
						}
					}
 					else {cylinder(h=hornarmL, r=hornarmR);}
					translate([0,0,hornarmL]) 
  					rotate([0,90,0]) cylinder(h=4*axleL, r=R + addR, center=true);
				}
			if (hornW<=0) thread_(RADSP, "cut");
			cylinder(h=hornarmL, d=BOLT3TIGHT); //TIGHTSP);
			}
		}
		
		module thread_(addR, key){
			rotate([0,0,90])
			if (key=="thread"){
				echo(addR);
				difference(){
					thread(threadL, axlehornDin + 2*addR, dtip1=BOLT3TIGHT, dtip2=0, aligner="dontshow", handness=threadhandness);
					cylinder(h=threadL, d=BOLT3TIGHT, center=true);
				}
			}
			else if (key=="cut"){
				echo(addR);
				thread(threadL, axlehornDin + 2*addR, dtip1=BOLT3TIGHT, dtip2=0, aligner=showthreadaligner, handness=threadhandness);
			}
		}
		
		if (key=="arm"){
		echo("hornarmL: ", hornarmL, hornW);

		translate([0,0,-R]) axle_(RADSP);
                /*
		difference(){
			difference(){
				cylinder(h=hornarmL, r=hornarmR);
				axle_(RADSP);
			}
			thread_(RADSP, "cut");
			cylinder(h=hornarmL, d=BOLT3TIGHT); //TIGHTSP);
		}*/
		}
		else if (key=="thread"){thread_(0, "thread");}
	}

	module axlecyl(key){
		
		echo(axleD);
		axleD = key != "cut" ? axleD : 
			type ==1 || type==6 ? axleD + 2*(2*TIGHTSP+coverT) : 
			axleD + 2*(SERVOHORNSP + coverT);

		echo(axleD);
		axleL = type==4 && key =="cut" ? axleL+hornarmL: 
			type==1  || type==6? key=="cut" ? axleL + SERVOHORNSP + bearingdims[2]: axleL + bearingdims[2] :
			(type==1 ||type==2) && key=="cut" ? axleL+SERVOHORNSP : 
			type==3 || type==5 || type==6? axleL+hornarmL :
			axleL;
                

		lthread = lthread == 0 ? hornarmL : lthread + .3 ;

		if (key=="cut"){
			cylinder(h=axleL + SERVOHORNT, r=axleD/2);
		}
		else {
			if (type==1 || type==2){
				difference(){
					cylinder(h=axleL + SERVOHORNT, r=axleD/2);
					cylinder(h=SERVOHORNT + 5, d=BOLT3TIGHT);
					axlehorn("bolt", boltH=20, boltD=BOLT3LOOSE);
				}
			}
			else if (type==6){
				difference(){
					cylinder(h=axleL + SERVOHORNT, r=axleD/2);
					cylinder(h=SERVOHORNT + 5, d=BOLT3TIGHT);
					//axlehorn("bolt", boltH=20, boltD=BOLT3LOOSE);
				}
			}
			else if (type==3){
				if (key!="thread"){
					D = axlehornDin + 2*RADSP;
					difference(){
						cylinder(h=axleL+SERVOHORNT, r=axleD/2);
						//TODO: make into bolt? 
						translate([0,0,SERVOHORNT + axleL]) thread(lthread + 1, D, phi=turnAngleMiddle, aligner="show", handness=threadhandness, dtip1=D, dtip2=D);
					}
				}
				else{
					translate([0,0,SERVOHORNT + axleL + lthread*2])thread(lthread, axlehornDin, phi=turnAngleMiddle, aligner="dontshow", handness=threadhandness);
				} //  cylinder(h=axleL, r=axlehornDin/2+RADSP);
			}
			else if (type==4){cylinder(h=axleL + SERVOHORNT, r=axleD/2);}
			else if (type==5){
				difference(){
					cylinder(h=axleL+SERVOHORNT, r=axleD/2);
					cylinder(h=2*axleL+SERVOHORNT, r=axlehornDin/2);
				}
			}
			if ((type==3 && key!="thread") || type==5){
				// Addiotnal thickening opposite to gear end:
				translate([0,0,SERVOHORNT + axleL - hornarmL]) // xxaxleL-hornarmL])
				difference(){
					cylinder(h=hornarmL, r=axleD/2 + coverT);
					cylinder(h=2*hornarmL, r=axleD/2);
				}
			}
		}
		//gearkey = key == "axle" ? "gear" : 
		//	  key == "cut" ? "cut" : 
		//	  key=="mockup" ? "gear" : 
		//	  "none";
		//
		//gear(gearkey);
	}

	module axle(key){
		
		module modaxle(key, gearkey, choose){

			module joiner(H, sp, phi=0, aligner=showthreadaligner){
				//linear_extrude(height=H, twist=H/3*360, convexity = 10, slices=round(H/.15))
				//translate([hthread, 0, 0]) 
				//circle(rscrew + sp);
				phi = phi+180;
				module th_(){
					translate([0,0,H])
					thread(2*H, D, handness=threadhandness, pitch=H/3, dtip2=D, dtip1=D, aligner=aligner, phi=phi);
				}
				D = 2*(rscrew + sp);
				if (aligner!="onlyaligner"){
					difference(){
					th_();
					translate([0,0,H])cylinder(2*H, r=D);
					}
				}
				else{th_();}
				//circle(axleD/2 - hthread - wJ - sp);
				//cylinder(h=H, r=axleD/2 - wJ + sp, $fn = 8);
			}
			
			H = SERVOHORNT; // + TIGHTSP;
			rscrew = min((axlehornDin + axleD)/4, axleD/2 - .9) + THREADH/2; 
			minout = axleD/2 - rscrew;
			minin = (rscrew-THREADH) - axlehornDin/2;
			echo("Thinnest wall in servo arm axles: ", min(minout, minin)); //axleD/2 - rscrew));
			echo("minout", minout, "minin", minin, "threadH", THREADH);
			echo("Dscrew", rscrew*2, "axleDin", axlehornDin);

			if (choose=="axle"){
				difference(){
					axlecyl(key);
					cylinder(h=H, r=100000);
					//if (showthreadaligner=="show"){
					if (CUTALIGNERS) joiner(H, 0, aligner="onlyaligner");
					//}
				}
   			
				intersection(){
					axlecyl(key);
					joiner(H, 0, aligner="dontshow");
				}	
			}
			else if (choose=="gear"){
				difference(){
					gear(gearkey);
					translate([0,0, H]) cylinder(h=10000, r=20000);
					joiner(H, RADSP, phi=-THREADSPACEPHI, aligner="dontshow");
					if (CUTALIGNERS){
					joiner(H, RADSP, phi=-THREADSPACEPHI, aligner="onlyaligner");
					joiner(H, RADSP, phi=0, aligner="onlyaligner");}
				}
				if (SHOWALIGNERS){
					joiner(H, RADSP, phi=-THREADSPACEPHI, aligner="onlyaligner"); //, aligner="onlyaligner");
					joiner(H, RADSP, phi=0, aligner="onlyaligner");
				}
			}


			//joiner(H, RADSP, phi=20);
		}
		module axle_(key){
			gearkey = key == "axle" ? "gear" : 
			  	  key == "cut" ? "cut" : 
			   	  key=="mockup" ? "gear" : 
			  	  "none";
		

	  		color(GEARCOLOR)		
  			/*
			if (type==0){
				if (key=="cut"){
					axlecyl(key);
					translate([0,0, braxleL]) mirror([0,0,1]) bearingaxle(key, braxleL);
					translate([0,0, axleL + SERVOHORNT - braxleL]) bearingaxle(key, braxleL);}
				
				else if (key=="axle" || key=="mockup"){
					difference(){	
						axlecyl(key);
						//axlehorn("axle");
						translate([0,0, braxleL]) mirror([0,0,1]) bearingaxle("cut", braxleL);
					}
				if (key=="mockup"){
					translate([0,0, braxleL]) mirror([0,0,1]) bearingaxle(key, braxleL);
				}
				
				// Create gear:
				gear(gearkey);
				translate([0,0, axleL + SERVOHORNT - braxleL]) bearingaxle(key, braxleL);
				}
			}
			*/
			if (type==1 || type==2 || type==6){
				if (key=="cut" || key=="axle"|| key=="mockup"){
					axlecyl(key);
					gear(gearkey);
				}
			}
			else if (type==3 || type==4 || type==5){
				if (key=="cut" || key == "mockup"){
					axlecyl(key);
					gear(gearkey);
				}
				else if (key=="axle"){
					modaxle(key, gearkey, "axle");
					translate([40,0,0]) modaxle(key, gearkey, "gear");
					translate([40,0,0]) cover();
				}
				else if (key=="axleNOTgear"){

					modaxle(key, gearkey, "axle");
				}
					//axlecyl(key);

				}
		//	}
		}
		

		module cover(){
			difference(){
				translate([0,0,SERVOHORNT]) cylinder(h=axleL, r=axleD/2 + coverT);
				if (type==1) cylinder(h=5*axleL, d=axleD/2-TIGHTSP);
				else cylinder(h=50*axleL, r=axleD/2 + TIGHTSP); // was 2*TIGHTSP
				if (type==1 || type==2){axlehorn("bolt", boltH=20, boltD=BOLT3LOOSE);}
				if (type!=1) bearings("mockup");
			}
		}

		axle_(key);

		// TYPE 1:
	  	color(GEARCOLOR)		
		if (type==1){
			if (key=="cut"){
				//translate([0,0,-SERVOHORNSP]) 
				mirror([0,0,1]) translate([0,0,SERVOHORNSP]) bearing("cut");
				translate([0,0, axleL + SERVOHORNT]) bearing("cut");
				axlehorn("cut");
				bearingaxle("cut");
			}
			else if (key=="bearingaxle"){
				bearingaxle("axle", braxleL-TIGHTSP);
			}
			else if (key=="mockup"){
				//axle_("axle");	
				axlehorn(key);
				translate([0,0,-SERVOHORNSP]) 
				mirror([0,0,1]) bearing("mockup");
				translate([0,0, axleL + SERVOHORNT]) bearing("mockup");
				bearingaxle("mockup");
			}
			else if (key=="cover"){cover();}
		}
		// TYPE 2 and 3:
		else if (type==2 || type==3 || type==4 || type==5){
			if (key=="cut" || key=="mockup"){
				bearings(key);
				cover();
				if (type==2){axlehorn(key);}
			}
			else if (key=="cover"){cover();}
		}		
		else if (type==6){
			if (key=="cut"){
				translate([0,0,-SERVOHORNSP]) 
				mirror([0,0,1]) translate([0,0,0*SERVOHORNSP]) bearing("cut");
				translate([0,0, axleL + SERVOHORNT]) bearing("cut");
				bearingaxle("cut");
			}
			else if (key=="bearingaxle"){
				bearingaxle("axle", braxleL-TIGHTSP);
			}
			else if (key=="mockup"){
				//axle_("axle");	
				translate([0,0,-SERVOHORNSP]) 
				mirror([0,0,1]) bearing("mockup");
				translate([0,0, axleL + SERVOHORNT]) bearing("mockup");
				bearingaxle("mockup");
			}
			else if (key=="cover"){cover();}
		}
		


	}

	module gear(key){
			
		H = SERVOHORNT; 
		R = axleD/2 + coverT;
		gearBore = type==5 ? axlehornDin :
			   type==1 ? BOLT3TIGHT : 0;	

		color(GEARCOLOR)
		if (key=="gear"){
			k = type==1 || type==6 ? 1 : -1;
			intersection(){
				wedge(H, R, turnAngle);
				herringbone_gear(GEARMODUL, ntooth, SERVOHORNT, 
					   	gearBore, 
					   	pressure_angle=PRESSUREANGLE, 
					   	helix_angle=k*HELIXANGLE, 
					   	optimized=false);

			}
		}
		else if (key=="cut"){
			H = H + SERVOHORNSP*2;
			addR = type==1 ? SERVOHORNSP : 2*SERVOHORNSP;
			R = R + addR;
			R2 = gearD(ntooth, GEARMODUL)/2 + SERVOHORNSP;
			translate([0,0, -SERVOHORNSP]) wedge(H, R, turnAngle*2, R2=R2);
		}
	}
	
	module bearing(key){
		H = bearingdims[2]; //+ TIGHTSP;
		addH = key=="cut" ? type==1 || type==2 || type==3 || type==4 || type==5 || type==6? 2*TIGHTSP: 10: 0; //SERVOHORNSP : 

		R = bearingdims[1]/2 + TIGHTSP/2;
		color(BEARINGCOLOR)
		cylinder(h=H+addH, r=R);
		//if ((key=="cut" || key=="mockup")){cylinder(h=H+addH+1, r=bearingdims[0]/2 + 1);}
	} 
	
	module bearings(key){	
		translate([0,0, SERVOHORNT + SERVOHORNSP]) bearing(key);
		translate([0,0, axleL + SERVOHORNT - bearingdims[2]]) bearing(key);
	}
	module bearingaxle(key){
		
		addH =  type==1 ? key=="cut" ? 1 : 0 : 
			type==6 ? hornarmL : 0;
		addR =  key=="cut" ? 1 : 0;
		
		H = bearingdims[2] + SERVOHORNSP + addH;
		//H = type == 6 ? H + hornarmL : 0;

 		translate([0,0,-H])
		difference(){
	 		union(){
				translate([0,0,H-SERVOHORNSP])cylinder(h=SERVOHORNSP, d=bearingdims[0] +2);
	 			cylinder(h=H, d=bearingdims[0] - TIGHTSP + addR);
			}
			if (key!="cut")bolt(100, BOLT3LOOSE, 1);
		}

		//sp = key=="cut" ? TIGHTSP : 0;
		//H = bearingdims[2] + L + SERVOHORNSP;
		//r2 = bearingdims[0]/2 + .7 - sp;
		//cylinder(h=H, r=bearingdims[0]/2 - TIGHTSP);
		//cylinder(h=H - bearingdims[2], r=r2);
		//if ((key=="cut" || key=="mockup") && type==1){
		//	translate([0,0,H]) cylinder(h=2*SERVOHORNSP, r=r2);
		//}
	}

	if (key=="mockup"){	
		//gear("gear");
		axle("mockup");
	}
	else if (key=="axle"){
		//gear("gear");
		axle("axle");
	}
	else if (key=="axleNOTgear"){
		//gear("gear");
		axle("axleNOTgear");
	}
	else if (key=="cut"){
		//gear("cut");
		axle("cut");
	}
	else if (key=="cover"){
		axle("cover");
	}
	else if (key=="gear"){
		gear("gear");
	}

	else if (key=="bearingaxle"){
		bearingaxle("bearingaxle"); //1); //braxleL);
	}
	else if (key=="hornarm"){
		hornarm();
		if (type!=1) translate([0,0,-hornarmL - 10]) hornarm("thread");
	}
	else if (key=="thread"){
		//gear("gear");
		axlecyl("thread");
	}


}

function axlehornDout(axleD, coverT) = axleD + coverT*2;
//$fn = 30;
//$fa = 10;
//$fs = .10;

//servo_mount_w_axle(false, servoNTooth=14, axleNTooth=26, key="bottom");
//servo_mount_w_axle(false, servoNTooth=14, axleNTooth=26, key="top");
//servo_mount_w_axle(false, servoNTooth=14, axleNTooth=26, key="cut");
//servo_mount_w_axle(true, true);
//axle_w_gear(GEARMODUL, 26, 90, key="buildall");
//axle_w_gear(GEARMODUL, 26, 90, key="cutall");
//axle_w_gear(GEARMODUL, 26, 90, key="buildaxle");
//axle_w_gear(GEARMODUL, 26, 90, key="bearingaxles");
//servo_gear(GEARMODUL, 14, GEART);


//axle_w_gear(GEARMODUL, 26, 60, cut="gear");
//kst_servo(cut=true, hornR=8, hornT=8);
//kst_servo(cut=true, hornR=8, hornT=8);
//kst_servo(cut=true, hornR=8, hornT=8);
//kst_servo(cut=true, hornR=8, hornT=8);
//kst_servo(cut=true, hornR=8, hornT=8);
//kst_servo(cut=true, hornR=8, hornT=8);

//translate([0,0,1])axle_w_gear(GEARMODUL, 26, 60, cut="gear2");
//kst_servo(cut=true, hornR=8, hornT=8);
type=1;
axleL = 20;
axlehornDin=6;
hornarmL=2;
key="mockup"; //"axleparts"; //"cut"; //"axleparts"; //"cut"; //axleparts"; //"mockup";//"mockup"; //"cut"; //"mockup"; // "mockup";// "cut";

servo_mount_aligned(key=key, servoNtooth=SERVOGEARNTOOTH, turnAngle=110, armAngleMiddle=270, axleL=axleL, axlehornDin=axlehornDin, type=type, hornarmL=hornarmL);

//translate([0,10,0])servo_mount_aligned(key="top", servoNtooth=SERVOGEARNTOOTH, turnAngle=110, armAngleMiddle=270, axleL=axleL, axlehornDin=axlehornDin, type=type, hornarmL=hornarmL);

//translate([0,20,50])servo_mount_aligned(key="axleparts", servoNtooth=SERVOGEARNTOOTH, turnAngle=110, armAngleMiddle=270, axleL=axleL, axlehornDin=axlehornDin, type=type, hornarmL=hornarmL);

