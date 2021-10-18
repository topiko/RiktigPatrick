use <servo_mount.scad>;
use <servos.scad>;
use <utils.scad>;
use <electronics.scad>;
//include <standards.scad>;
include <usedims.scad>;



module head_bulk_straight(modRs=0){
  module rotmove(H,x){
    translate([x,Lhead/2 + modRs,H])
    rotate([90,0,0]) children(); 
  }
  

  Rhead=Rhead-modRs;
  rcorner=rcorner-modRs;
  Whead=Whead-2*modRs;
  Hhead=Hhead-modRs;
  Lhead=Lhead-2*modRs;

  hull(){
    rotmove(Rhead+modRs,Whead/2 - Rhead) rounded_tilted_cylinder(Lhead, Rhead, rcorner, 0);
    rotmove(Rhead+modRs,-Whead/2 + Rhead) rounded_tilted_cylinder(Lhead, Rhead, rcorner, 0);
    rotmove(Hhead-Rhead,Whead/2 - Rhead) rounded_tilted_cylinder(Lhead, Rhead, rcorner, 0);
    rotmove(Hhead-Rhead,-Whead/2 + Rhead) rounded_tilted_cylinder(Lhead, Rhead, rcorner, 0);
  }

}
module head_bulk(modRs=0){
  
  module rotmove(H, y){
    translate([0, Lhead/2 + y + modRs,H])
    rotate([0,-90,0]) 
    children(); 
  }
  
  Rhead=Rhead-modRs;
  rcorner=rcorner-modRs;
  Wheadbottom=Wheadbottom-2*modRs; // -2*modRs;
  Wheadtop=Wheadtop-2*modRs; // -2*modRs;
  Hhead=Hhead-modRs;
  Lhead=Lhead-2*modRs;
  addbackh = 9;


  hull(){
    rotmove(Rhead+modRs,Lhead/2 - Rhead) rounded_tilted_cylinder(Wheadbottom, Rhead, rcorner, alpha);
    rotmove(Rhead+modRs,-Lhead/2 + Rhead) rounded_tilted_cylinder(Wheadbottom, Rhead, rcorner, alpha);
    rotmove(Hhead-Rhead,Lhead/2 - Rhead - Hhead*tan(headtheta)) rounded_tilted_cylinder(Wheadtop, Rhead, rcorner, alpha);
    rotmove(Hhead-Rhead +addbackh,-Lhead/2 + Rhead) rounded_tilted_cylinder(Wheadtop-2*tan(alpha)*addbackh, Rhead, rcorner, alpha);
  }

}


module head_shell(wallT, key="bottom"){
  
  module shell_(){
    difference(){
      head_bulk();
      head_bulk(wallT);
    }
  }


  if (key=="bottom"){selector(headBottomT, -1) shell_();}
  if (key=="top"){selector(headBottomT, 1) shell_();}
  
}






module head(key){

  sy = 2.5;
  yback = -Lhead+mountT/2 + sy;
  module moveshell_(){translate([0,yback,0]) children();}
  
  module mountpoles_(key){
    sel = key=="bottom" ? -1 : 1;
    holeD = key=="bottom" ? BOLT25LOOSE : BOLT25TIGHT;
    modT = key=="bottom" ? -.5 : .5;
    // Mounting posts:
    moveshell_()
    intersection(){
      selector(headBottomT + modT, sel) head_bulk();
      mount_poles_bolts(headmountpolepos, mountD, holeD=holeD);
    }
  }

  module mouth_(key="mockup"){ 
    seed=22;
    random_vect=rands(5,15,4,seed);
    nteeth=15;
    sp = 0;


    Hmin = 15;
    Hmax = 15;
    Wmin=7;
    Wmax=9;
    Tmin=1;
    Tmax=3;
    rmin=.5;
    rmax=1.0;
    zmin=0;
    zmax=2;
    gapmin=-.3;
    gapmax=.3;
 
    dims = [[Hmin, Hmax], [Wmin, Wmax], [Tmin, Tmax], [rmin, rmax], [gapmin, gapmax]];

    function teethdims(i) = [for (dimpair=dims) rands(dimpair[0], dimpair[1], 1, seed*i)[0]];  
    teethzs_ = rands(zmin, zmax, nteeth, seed);  

    function substract_mean(a) = [for (z=a) z - sum(a)/len(a)];

    module tooth(W, H, T, r){
      tw=.82;
      ttop = .2*3;
      difference(){
        roundedcube([H,W,2*T], radius=r, center=true);
        roundedcube([H-2*tw,W-2*tw,2*(T-tw)], radius=r-tw, center=true);
      }
    }
    
    function pickcolumn(matrix, i) = [for (p=matrix) p[i]];

    function posfromdims(dims) = [for (i=[0:1:nteeth-1]) sum_slice(pickcolumn(dims, 1), 0, i) + sum_slice(pickcolumn(dims, 4), 0, i) - pickcolumn(dims, 1)[i]/2];

    module teeth_(){
      upteethdims = [for (i=[0:1:nteeth-1]) teethdims(i)];
      lowteethdims = [for (i=[0:1:nteeth-1]) teethdims(-i)];
      upteethPos = substract_mean(posfromdims(upteethdims)); 
      lowteethPos = substract_mean(posfromdims(lowteethdims));

      teethzs = substract_mean(teethzs_);
      
      
      for (i=[0:1:nteeth-1]){
        H_up=upteethdims[i][0];
        W_up=upteethdims[i][1];
        T_up=upteethdims[i][2];
        r_up=upteethdims[i][3];
          
        H_low=lowteethdims[i][0];
        W_low=lowteethdims[i][1];
        T_low=lowteethdims[i][2];
        r_low=lowteethdims[i][3];

        color("Beige")translate([upteethPos[i], H_up/2 + teethzs[i], 0]) tooth(H_up, W_up, T_up, r_up); 
        color("Beige")translate([lowteethPos[i], -H_low/2 + teethzs[i], 0]) tooth(H_low, W_low, T_low, r_low); 
        
      }
    }
     
    module lips_(H, Wmouth=Whead/3*2, Hmouth=.9*Hmax){
      sp = key == "cut" ? TIGHTSP*2 : 0;
      R = 5;
      lipw = 5;
      color("Red")
      linear_extrude(height=H)
      difference(){
        offset(R+sp) square([Wmouth, Hmouth - R], center=true);
        offset(R-lipw) square([Wmouth, Hmouth - R], center=true);
      }
    }
    module mount(key){
      dw = 10;
      sp = key=="cut" ? TIGHTSP : 0;
      h = key=="cut" ? 2. : 1.5;

      for (i=[-1,1]) for (j=[-1,1]) translate([i*(Wmouth/2 - dw), j*(Hmouth/2 - dw/2), -1]) cylinder(h=h, r=1.5+sp);
    }
     
    module build(){
      if (key=="mockup"){
        difference(){
        union(){
          lips_(Tmax, Wmouth, Hmouth);
          hull() lips_(1, Wmouth, Hmouth);
          intersection(){
            color("Red") hull() lips_(Tmax, Wmouth, Hmouth);
            teeth_();
          }
        }
        mount("cut");
        }
      }
 
      else if (key=="cut" || key=="mount"){
        mount(key);
      }
    }

    Wmouth=Wheadbottom/7*7;
    Hmouth=30;
    
    rotate([headtheta,0,0]) 

    translate([0, mountT/2 + sy, 33])
    rotate([0,-0,0])
    rotate([-90,0,0])
    build();


  }


  module bottom_shell_(){
    difference(){
      union(){
        moveshell_() head_shell(wallT, "bottom");
        mountpoles_("bottom");
        rpi_(key="poles");
        camera_(side="back");
      }
      mount_poles_bolts(headmountpolepos, BOLT25LOOSE, negy=yback, holeD=0);
      rpi_("cut");
      camera_("cut", side="back");
    }
    

  }
  module top_shell_(key){
    module shell_(){
    difference(){
      moveshell_() head_shell(wallT, "top");
      servo_("cut");
      camera_("cut");
    }
    
    intersection(){
      moveshell_() head_bulk();
      camera_();
    }
	
    mountpoles_("top");
       
    // Servo mount bottom
    intersection(){
      moveshell_() head_bulk(0);
      servo_("bottom");
    }
     
    // servo mount top
    difference(){
      intersection(){
        moveshell_() selector(headBottomT, 1) head_bulk(wallT + MEDIUMSP);
        translate([0,-MEDIUMSP,0]) servo_("top");
      }
      mount_poles_bolts(headmountpolepos, mountD+MEDIUMSP, negy=yback, holeD=0);
    }
    }
   
    difference(){
      shell_();
      mouth_("cut");
    }
  }

  module servo_(key){
    axleD=AXLEBEARINGDIMS[0] + 2*AXLECOVERT; // + SERVOHORNSP + .1;
    difference(){
      translate([0,0, wallT])
      rotate([180,0,0])
      servo_mount_aligned(key=key, servoNtooth=headServoNteeth, 
                          axleNtooth=headServoNteeth+2, axleL=12, 
                          armAngleMiddle=270,
                          axlehornDin=BOLT3TIGHT, type=5, 
                          hornarmL=wallT + Hneck + Rtop, T=mountT, addXL=12);
      translate([0,0,-Hneck-Rtop]) rotate([0,90,0]) cylinder(d=axleD, h=50, center=true);
    }
  }

  module camera_(key="poles", side="front"){
    camH = Hhead - 25;
    camplateT = 2;
    cut = key=="cut" ? 1 : 0;
    sepy_ = 4;
    sepy = side == "back" ? 0 : sepy_ + camplateT;
    poleh = side == "back" ? sepy_ : 40;
    boltD = side =="back" ? BOLT25LOOSE : BOLT25TIGHT;
    sink = 6;

    //translate([Whead/2-22,-mountT/2 + sink, camH])
    //translate([0,-mountT/2 + sink, camH])
    translate([0,yback + wallT + sepy, camH])
    rotate([-90,0,0]) 
    camera(cut, poleh, boltD=boltD);
  }
  
  module rpi_(key="boardpoles"){ 
    hm = 1.5;
    translate([0, yback, Hhead*0 + wallT + 1 + 65/2 ])
    rotate([0,180,0])
    rotate([0,0,180])
    rotate([90,0,0])
    rotate([0,0,90])
    rpizero(key=key, H=hm + wallT, T=10, boltH=hm + 2*wallT, boltD=BOLT3LOOSE );
    
    if (key=="cut"){
      // CABLE HOLE
      rpit = 7;
      translate([17,rpit + 2.+yback+wallT,0]) rounded_cutter(20, 14, 4, .5);

      // COOLING
      hc = 2.9; 
      for (i=[-1,1]) 
        for (k=[0:1:11]) 
          translate([i*7,yback, k*1.5*hc + 11]) rotate([90,0,0]) rounded_cutter(20, 12, 3, .5);      
    
   }
    
  }
  
  if (key=="top"){top_shell_();}
  if (key=="bottom"){bottom_shell_();}
  if (key=="servoparts"){servo_("axleparts");}
  if (key=="servo"){servo_("cut");}
  if (key=="camera"){camera_("cut"); camera_("poles");}
  if (key=="mockup"){moveshell_() head_bulk(); camera_("cut"); mouth_("mockup");}
  if (key=="rpi"){rpi_(); rpi_("cut");}
  if (key=="mouth"){
    mouth_("mockup");
    translate([0,-10,0]) mouth_("mount");
  }
}


module moverothead(phi, theta){
  rotate([theta, 0, 0])
  rotate([0, 0, phi])
  translate([0, 0, Rtop + Hneck]) 
  children();
}

//moverothead(0, 0) 
//translate([0, mountT/2, 0]) 

echo(key);
key="bottom"; //"top"; //
head(key);
//head("top");
//head_shell(wallT);
head("mouth");
//head("rpi");
//head("camera");
//head("rpi");
//head("servo");
//head("servoparts");
//head_mockup3("servo");


