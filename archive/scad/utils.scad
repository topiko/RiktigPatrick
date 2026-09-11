include <dims.scad>;
include <usedims.scad>;

function pitchD(ntooth, modul=GEARMODUL) = ntooth*modul;
function getNTeeth(servoNtooth, targetTurnAngle, servoTrunAngle) = ceil(servoNtooth/targetTurnAngle*servoTrunAngle);
function gearD(ntooth, modul=GEARMODUL) = ntooth*modul + 2*modul;
module bolt(h, d, sink, baseL=5, key="csunk"){
   	
	// Expand the bolt sink hole by this amount: 
	sinkExpFac = 1.1;
	
	// Gray bolts
    	color("Gray")
    	translate([0,0,sink])
    	union(){
		// bolt thread
		translate([0,0,d/2]) cylinder(h=h, r=d/2);
		// bolt base
		if (key=="csunk"){cylinder(h=d/2, r1=d*sinkExpFac, r2=d/2*sinkExpFac);}
		else if (key=="flat"){cylinder(h=d/2 - .2, r=d*sinkExpFac);}
		
		translate([0,0,-baseL - sink]) cylinder(h=baseL + sink, r=d*sinkExpFac);
    	}
}


module thread(h, d, phi=0, threadH=THREADH, pitch=THREADPITCH, aligner="show", dtip1=0, dtip2=0, handness="right"){

	handmltp = handness == "right" ? -1 : 1;
	phi = -phi*handmltp;
	twistphi = handmltp*h/pitch*360;
	dtip1 = dtip1==0 ? d*4/5 : dtip1; // top
	dtip2 = dtip2==0 ? d*4/5 : dtip2; // bottom

	module thread_base(){
		difference(){
		rotate([0,0,twistphi/2])
		linear_extrude(height=h, twist=twistphi, center=true, slices=round(h/.15)) 
		translate([threadH/2,0]) 
		circle(d/2-threadH/2);
		if (htip(dtip1)>0){translate([0,0,h/2]) tipcutter(dtip1);}
		if (htip(dtip2)>0){translate([0,0,-h/2]) mirror([0,0,1]) tipcutter(dtip2);}
		}

	}
	
	tiptheta = 60;
	function htip(dtip) = (d - dtip)/2*tan(tiptheta);

	module tipcutter(dtip){
		htip_ = htip(dtip);
		translate([0,0,-htip_])
		difference(){
			cylinder(h=2*htip_, d=2*d);
			cylinder(h=htip_, d1=d, d2=dtip);
		}
	}
	
	rotate([0,0,phi]){
		if (aligner=="dontshow"){
			thread_base();
		}
		else if (aligner=="show"){	
			thread_base();
			translate([50,0,0])cube([100, 1, 1], center=true);
		}
		else if (aligner=="onlyaligner"){
			translate([50,0,0])cube([100, 1, 1], center=true);
		}
	
	}

}
module closebolts(key, positions, L, forceboltD=-1){
 
 boltD_ = key=="boltsup" ? BOLT3LOOSE :
         key == "boltsbot" ? BOLT3TIGHT : 0;
 
 boltD = forceboltD > 0 ? forceboltD : boltD_;

 key_ = key=="boltsup" || key=="boltsbot" ? "bolt" : key;
 for (pos=positions){
   x = pos[0];
   z = pos[1];
   translate([x,0,z]) 
   rotate([90,0,0])
   closebar(L=L, boltD=boltD, key=key_);
 }
}

module closebar(L, boltD=BOLT3LOOSE, key="mockup"){
  Ltop = L/2;
  Lbot = L/2 - .5;
  D = CLOSEBARD; 
  if (key=="barup"){
    cylinder(d=D, h = Ltop);
  }
  else if (key=="barbot"){
    translate([0,0,-L/2])
    cylinder(d=D, h = Lbot);
  }
  else if (key=="bolt"){
    mirror([0,0,1])
    translate([0,0,-L/2])
    bolt(L-3, d=boltD,sink=.5);
    mirror([0,0,1]) cylinder(h=.5, r=boltD);
  }
  
  //bolt(L-3, d=boltD,sink=.5);
}

module rounded_tilted_cylinder(L, R, r, alpha){
  
  assert(R>=r); 
  module end(){
    translate([0,0,L/2])
    rotate([0,alpha, 0])  
    translate([0,0,-r*cos(alpha)])
    scale([1/cos(alpha), 1, 1])
  
    rotate_extrude()
    union(){
      translate([0,-r]) square([R-r, 2*r]);  
      translate([R-r, 0]) 
      intersection(){
        circle(r);
        translate([0,-r]) square([r, 2*r]);
      }
      }

    }

    hull(){
    end();
    mirror([0,0,1]) end();
    }
}

module selector(ycut, side=-1){

  shifty = side==-1 ? 5000 : -5000; 
  
  difference(){
    children();
    translate([0,shifty + ycut, 0]) cube(10000, center=true);
  }
  
  
}

module mount_poles_bolts(positions, D=mountD, negy=-500, holeD=0){
  
  for (p=positions){
    translate([p[0], negy, p[1]])
    rotate([-90,0,0]) 
    if (holeD>0){
      difference(){
        bolt(1000, d=D, sink=.2);
        cylinder(h=2000, d=holeD);
      }
    }
    else {bolt(1000, d=D, sink=.2);}
  }
}



module roundedcube(size = [1, 1, 1], center = false, radius = 0.5, apply_to = "all") {
	// If single value, convert to [x, y, z] vector
	size = (size[0] == undef) ? [size, size, size] : size;

	translate_min = radius;
	translate_xmax = size[0] - radius;
	translate_ymax = size[1] - radius;
	translate_zmax = size[2] - radius;

	diameter = radius * 2;

	module build_point(type = "sphere", rotate = [0, 0, 0]) {
		if (type == "sphere") {
			sphere(r = radius);
		} else if (type == "cylinder") {
			rotate(a = rotate)
			cylinder(h = diameter, r = radius, center = true);
		}
	}

	obj_translate = (center == false) ?
		[0, 0, 0] : [
			-(size[0] / 2),
			-(size[1] / 2),
			-(size[2] / 2)
		];

	translate(v = obj_translate) {
		hull() {
			for (translate_x = [translate_min, translate_xmax]) {
				x_at = (translate_x == translate_min) ? "min" : "max";
				for (translate_y = [translate_min, translate_ymax]) {
					y_at = (translate_y == translate_min) ? "min" : "max";
					for (translate_z = [translate_min, translate_zmax]) {
						z_at = (translate_z == translate_min) ? "min" : "max";

						translate(v = [translate_x, translate_y, translate_z])
						if (
							(apply_to == "all") ||
							(apply_to == "xmin" && x_at == "min") || (apply_to == "xmax" && x_at == "max") ||
							(apply_to == "ymin" && y_at == "min") || (apply_to == "ymax" && y_at == "max") ||
							(apply_to == "zmin" && z_at == "min") || (apply_to == "zmax" && z_at == "max")
						) {
							build_point("sphere");
						} else {
							rotate = 
								(apply_to == "xmin" || apply_to == "xmax" || apply_to == "x") ? [0, 90, 0] : (
								(apply_to == "ymin" || apply_to == "ymax" || apply_to == "y") ? [90, 90, 0] :
								[0, 0, 0]
							);
							build_point("cylinder", rotate);
						}
					}
				}
			}
		}
	}
}


function slice(a, i1=0, i2=-1) =i2==-1 ? slice(a, i1, len(a)) : [for (i = [ 0 :1:len(a)-1]) if (i>=i1 && i<(i2+1)) a[i]];


function sum_slice(a, i1=0, i2=-1) = sum(slice(a, i1, i2));

function sum(a, i=0, r=0) = i < len(a) ? sum(a, i+1, r+a[i]) : r;

function cumsum(a) = [for (i=[0:1:len(a)-1]) sum_slice(a, 0, i)];


module rounded_cutter(L, W, T, r){
  assert(min(W,T)>2*r);

  linear_extrude(height=L, center=true)
  offset(r)
  square([W-2*r, T-2*r],center=true);

}
