include <dims.scad>;
use <utils.scad>;

servoctrldims = [30,20,10];
servoctrlmntpos = [[0,1], [20, 10]];

module generalel(dims, mountpos, boltD=BOLT3TIGHT, H=2, key="mockup", boltH=0, poleD=0){
	
	poleD = poleD==0 ? boltD*2 : poleD;
	module poles(){
		for (p=mountpos){
			translate(p) cylinder(h=H, d=poleD);
		}
	}

	module bolts(){
		boltH = boltH == 0 ? H - .5 : boltH;
		
		color("Gray")	
		for (p=mountpos){
			translate(p) translate([0,0,H]) mirror([0,0, 1]) bolt(boltH, boltD, -boltD/2);
		}
	}
	
	translate([-dims[0]/2, -dims[1]/2, 0])
	if (key=="mockup"){
		echo("mckup");
		poles();
		color("Navy")
		translate([0,0,H]) cube(dims);
	}
	else if (key=="bolts"){bolts();}
	else if (key=="poles"){poles();}
	else if (key=="board"){
    		color("Navy")
		translate([0,0,H]) cube(dims);
	}
}


module micarray(H=5, key="mockup", boltH=0){
	
	dims = [65, 65, 2];
	mountpos = [[3.5, 8], [61.5, 8], [3.5, 57], [61.5, 57]];
	boltD = BOLT25TIGHT;
	boltH= boltH==0 ? H + 1: boltH; //  + 1.5;
	H = H;

	generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);
	
	module ledring(){
		ringh = boltH;
		translate([0,0,H-ringh])
		difference(){
			cylinder(h=ringh, r=62/2);
			cylinder(h=ringh, r=62/2-5);
		}
	}
	
	module micring(){
		mich = H + 15;
		
		for (phi=[90:90:360]){
			rotate([0,0,phi])
			translate([dims[0]/2 - 2 - 1., dims[1]/2 - 3, -mich/2 + H]) 
			cube([4+1, 3+1, mich], center=true);
		}
	}
	
	if (key=="micsleds"){
		ledring();
		micring();
	}
	
}
module huzzah32(H=5, key="mockup", boltH=0, boltD=BOLT25TIGHT){
	dims = [2*25.4, 0.9*25.4, 2];
        mounty = 0.15*25.4/2;
	mountpos = [[2.5, mounty], [dims[0] - 2.5, mounty], [2.5, dims[1] - 2.5], [dims[0] - 2.5, dims[1] - mounty]];
	//boltH= boltH==0 ? H : boltH; //  + 1.5;
	color("DarkSlateGray")	
	if (key!="usb") generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);
	else {
		usbr = 4;
		w = 11;

		hull() for (y=[-1,1]) translate([-dims[0]/2, (w/2-usbr)*y, H + 2.5]) rotate([00,90,0]) cylinder(h=15, r=usbr, center=true);
	}
	

}
module imu(H=5, key="mockup", boltH=0, boltD=BOLT25TIGHT){
	dims = [25.4, 0.9*25.4, 2];
	mountpos = [[2.54, 2.54], [dims[0] - 2.54, 2.54], [2.5, dims[1] - 2.54], [dims[0] - 2.54, dims[1] - 2.54]];

	color("DarkSlateGray")	
	if (key!="usb") generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);
	else {
		usbr = 4;
		w = 11;

		hull() for (y=[-1,1]) translate([-dims[0]/2, (w/2-usbr)*y, H + 2]) rotate([00,90,0]) cylinder(h=15, r=usbr, center=true);
	}
	

}

module buckconverter(H=5, key="mockup", boltH=0, boltD=BOLT25TIGHT){
	dims = [43, 21, 2];
	mountpos = [[36.5, 2.5], [6.5, 21-2.5]];
	
	//boltH = boltH==0 ? H: boltH; //  + 1.5;
	
	color("DarkSlateGray")	
	generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);

}

module servoctrl(H=5, key="mockup", boltH=0, boltD=BOLT25TIGHT){
	dims = [63, 25.5, 2];
	mountpos = [[3.5, 3], [59.5, 3], [3.5, 22], [59.5, 22]];
	//boltH= boltH==0 ? H : boltH; //  + 1.5;
	color("DarkSlateGray")	
	generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);

}

module nanoble(H=5, key="mockup", boltH=0, boltD=BOLT15TIGHT){
        dwall=1.5;
	dims = [43.4, 18, 2];
	mountpos = [[dwall, dwall], [dims[0]-dwall, dwall], [dwall, dims[1]-dwall], [dims[0]-dwall, dims[1]-dwall]];
	//boltH= boltH==0 ? H : boltH; //  + 1.5;
	color("DarkSlateGray")	
	generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH, poleD=4);
        if (key=="bolts"){
          translate([dims[0]/2,0,H+4])
          rotate([0,90,0])
          rounded_cutter(20, 8, 12, 1);
        }

}


module protoboard(H=5, key="mockup", boltH=0, boltD=BOLT25TIGHT){

	dims = [80, 20.5, 2];
	mountpos = [[2.5, 2.5], [77.5, 2.5], [2.5, 18], [77.5, 18]];
	
	//boltH = boltH==0 ? H: boltH; //  + 1.5;
	
	color("DarkSlateGray")	
	generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);
}

module protoboard2(H=5, key="mockup", boltH=0, boltD=BOLT25TIGHT){

	dwall=2.5;
	dims = [71, 30.5, 2];
	mountpos = [[dwall, dwall], [dims[0]-dwall, dwall], [dwall, dims[1]-dwall], [dims[0]-dwall, dims[1]-dwall]];
	
	//boltH = boltH==0 ? H: boltH; //  + 1.5;
	
	color("DarkSlateGray")	
	generalel(dims, mountpos, boltD=boltD, H=H, key=key, boltH=boltH);
}



module camera(cut, H=30, boltD=BOLT25TIGHT){
	holeD = 30;
	cameraW = 38;
	cameraR = 32/2;
	mountH = H;
	mountR = 3.0;

	mountHoles= [for (i=[-1, 1]) for (j=[-1,1]) [i*holeD/2, j*holeD/2, 0]];
	
	module mount(){
		for (p=mountHoles){
			translate(p) cylinder(h=mountH, r=mountR);
		}
	}

	module camandbolts(){

		// Camera	
		color("Black") cylinder(h=35, r=cameraR);
		for (p=mountHoles){
			translate(p) bolt(6, boltD, -boltD/2);
		}
	}

	if (cut){ camandbolts();}
	else {
		difference(){
		mount();
		camandbolts();}
	}
}


module rpizero(key, H=5, T=2, pijuice=false, boltH=0, boltD=BOLT3TIGHT){
	
        if (pijuice) assert(H>7);

	dims = [65,30,T];
	mount_holes = [[3.5, 3.5], [dims[0]-3.5, 3.5],
		       [3.5, dims[1]-3.5], [dims[0]-3.5, dims[1]-3.5]];
	module board(){
		translate([0,0,dims[2]/2]) cube(dims, center=true);
	}
	
	module mountpoles(H=H, key="poles"){
		translate([-dims[0]/2, -dims[1]/2, 0])
		for (p=mount_holes){
			translate(p)
			if (key=="poles"){
				translate([0,0, -H]) {
				cylinder(h=H, r=BOLT3LOOSE);
				//cylinder(h=H+dims[2]+1, r1=2.75/2, r2=2.6/2);
}
			}
			else if (key=="bolts"){
				boltH = boltH==0 ? H-.2 : boltH;
				mirror([0,0,1]) bolt(boltH, boltD, -boltD/2);
			}
		}
	}
	
	module usbcharge(){
		dy = 2.;
		translate([dims[0]/2, -dims[1]/2+11, -1.5])
		hull(){
		for (j=[-1,1]) translate([0, j*dy, 0]) rotate([0,90,0]) cylinder(h=20, r=3.5);
		}
	}
	module sdcard(){
		dy = 2.;
 		wsd = 15;
		tsd=3; 
		zsd=7.0; //TODO: measure this from bottom to sd center!
		translate([dims[0]/2, dims[1]/2-13, zsd])
		rotate([0,90,0]) rounded_cutter(20, tsd, wsd, tsd/4);
	}
	

	module pwrswitch(){
		translate([dims[0]/2 - 20.5, -dims[1]/2 - 2, -H-5]) cube([5, 2, 30]);
	}
	
	translate([0,0,H])
	if (key=="boardpoles"){
		board();
		mountpoles();
	}
	else if (key=="poles"){
		mountpoles();
	}
	else if (key=="bolts"){
		mountpoles(key="bolts");
	}
	else if (key=="cut"){
		translate([0,0,7.5 + 2])  
		rotate([180,0,0]){
		  usbcharge();
		  pwrswitch();
		  sdcard();
		}
		mountpoles(key="bolts");
	}
}

module batt1500(){
  L = 52;
  W = 34;
  H = 8;
  color("Blue")
  translate([0,0,H/2])
  cube([L, W, H], center=true);
}

module battery(key="3S"){
	if (key=="3S"){cube([30,27, 55], center=true);}
	else if (key=="2S"){cube([35,18, 67], center=true);}
}
module switches(key){
    T = 2;
    wT = 1;
    swichframeW = 8.2 + 2*TIGHTSP;
    swichframeH = 7.3 + 2*TIGHTSP;
    swichframeT = 5.4;
    threadL=8;
    threadD=5;

    module swich_(){
        
      translate([-3,0,0]){
        rotate([0,90,0]) cylinder(h=threadL, d=threadD);
        translate(-[swichframeH, swichframeW/2, swichframeT/2]) {
          cube([swichframeH, swichframeW, swichframeT]);
          translate([-swichframeH,.5,0]) cube([swichframeH, swichframeW-1, swichframeT]);
        }
      }
    }
    module cut(){
    	armT = 3;
    	alpha = 40;
    	translate([+wT/2, 0, -armT/2])
    	rotate([0,0,-alpha/2])
    	rotate_extrude(angle=alpha) square([20, armT]);
    } 

    cut();
    swich_();
  }

$fa=1;
$fs=.1;
rpizero("cut");
//nanoble(3, key="poles");
//protoboard2(3, key="poles");
//translate([0,20,0]) buckconverter(3, key="poles");
