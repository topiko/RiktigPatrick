use <servo_mount.scad>;
use <servos.scad>;
use <utils.scad>;
use <electronics.scad>;
//include <standards.scad>;
include <usedims.scad>;



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


    Hmin = 25;
    Hmax = 25;
    Wmin=7;
    Wmax=8;
    Tbot = 1;
    Tmin=rcorner + Tbot + 2;
    Tmax=Tmin + 3;
    rmin=.5;
    rmax=1.0;
    zmin=0;
    zmax=1;
    gapmin=-.5;
    gapmax=1.;
    gap = 5;
    Rlip =12;
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

        color("Beige")translate([upteethPos[i], H_up/2 + gap/2 + teethzs[i], 0]) tooth(H_up, W_up, T_up, r_up); 
        color("Beige")translate([lowteethPos[i], -H_low/2 - gap/2  + teethzs[i], 0]) tooth(H_low, W_low, T_low, r_low); 
        
      }
    }
     
    module lips_(H, Wmouth=Wheadbottom/3*2, Hmouth=.9*Hmax){
      sp = key == "cut" ? TIGHTSP*2 : 0;
      R = Rlip;
      lipw = 5;
      color("Red")
      linear_extrude(height=H)
      difference(){
        offset(R+sp) square([Wmouth - 2*R, Hmouth - 2*R], center=true);
        offset(R-lipw) square([Wmouth - 2*R, Hmouth - 2*R], center=true);
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
          color("Black") hull() lips_(rcorner + Tbot, Wmouth, Hmouth);
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

    Wmouth=Wheadbottom*1.1; ///8*8;
    Hmouth=35;
    Zmouth = 40; 
    difference(){
      rotate([headtheta,0,0]) 
      translate([0, mountT/2 + sy - rcorner, Zmouth])
      rotate([0,-0,0])
      rotate([-90,0,0])
      build();
      moveshell_() head_bulk(-MEDIUMSP);
    }


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
      antenna_("cut");
    }
    

  }
  module top_shell_(key){
    module shell_(){
    difference(){
      moveshell_() head_shell(wallT, "top");
      servo_("cut");
      camera_("cut");
      antenna_("cut");
      cutaxlepath();
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
    sy = acc=="print" ? -30 : 0;
    translate([0,sy,0])
    difference(){
      intersection(){
        union(){ 
          moveshell_() selector(headBottomT, 1) head_bulk(wallT + MEDIUMSP);
          cutaxlepath();
        }
        translate([0,-MEDIUMSP,0]) servo_("top");
      }
      mount_poles_bolts(headmountpolepos, mountD+MEDIUMSP, negy=yback, holeD=0);
    }
    }
   
    shell_();
  }

  module cutaxlepath(SP=0){
    axleD = AXLEBEARINGDIMS[0] + (AXLECOVERT + SERVOHORNSP)*2;
    Dy = mountT/2 - sy - SP;
    H = wallT + 2*MEDIUMSP + TIGHTSP;
    translate([0,-Dy/2 - SP,H/2 - rcorner * tan(alpha)]) cube([axleD + 2 - 2*SP, Dy, H], center=true);
  }

  module servo_(key){
    axleD=AXLEBEARINGDIMS[0] + 2*AXLECOVERT; // + SERVOHORNSP + .1;

    mountT = key=="top" || key=="cuttop" ? mountT - 2*sy : mountT;
    key = key=="cuttop" ? "cut" : key;
    extraNeck = 0; //.2 + rcorner*tan(alpha);
    hornL = wallT + Hneck + Rtop + extraNeck;
    difference(){
      translate([0,0, wallT])
      rotate([180,0,0])
      servo_mount_aligned(key=key, servoNtooth=headServoNteeth, 
                          axleNtooth=headServoNteeth+2, axleL=12, 
                          armAngleMiddle=270,
                          axlehornDin=BOLT3TIGHT, type=5, 
                          hornarmL=hornL, T=mountT, addXL=12);
       
      translate([0,0,-Hneck-Rtop-extraNeck]) rotate([0,90,0]) cylinder(d=axleD, h=50, center=true);
    }
    
    if (key=="top") difference(){cutaxlepath(MEDIUMSP); servo_("cuttop");}
  }

  module camera_(key="poles", side="front"){
    camH = Hhead - 25;
    camplateT = 1.4;
    cut = key=="cut" ? 1 : 0;
    sepy_ = 4;
    sepy = side == "back" ? 0 : sepy_ + camplateT;
    poleh = side == "back" ? sepy_ : 40;
    boltD = side =="back" ? BOLT25LOOSE : BOLT25TIGHT;
    sink = 6;

    translate([0,yback + wallT + sepy, camH])
    rotate([-90,0,0]){ 
      camera(key, poleh, boltD=boltD);
      if (side=="front" && key!="cut") translate([0,0,0*12]) camera("ring", poleh, boltD=boltD);
    }
  }
  
  module rpi_(key="boardpoles"){ 
    hm = 1.5;
    translate([0, yback, Hhead*0 + wallT + 1 + 65/2 ])
    rotate([0,180,0])
    rotate([0,0,180])
    rotate([90,0,0])
    rotate([0,0,90])
    rpizero(key=key, H=hm + wallT, T=10, boltH=hm + 2*wallT, boltD=BOLT3LOOSE );
    

    holex = 21.5;


    if (key=="cut"){
      // CABLE HOLE
      translate([holex,yback,7+Rhead]) rotate([90,0,0]) rounded_cutter(20, 6, 14, .5);

      // COOLING
      hc = 2.9; 
      for (i=[-1,1]) 
        for (k=[0:1:11]) 
          translate([i*7,yback, k*1.5*hc + 11]) rotate([90,0,0]) rounded_cutter(20, 12, 3, .5);      
    
   }
   else {

     // cable zip tie mount:
     translate([holex,yback,14+wallT]) rotate([-90,0,0]) {hull() {cylinder(h=wallT+ 5, r=5); translate([0,5,0])cylinder(h=wallT+ 5, r=5);}}
   }
    
  }

  module antenna_(key="mockup"){
    L = 30;
    Rbot = 6;
    Rtop = 8/2;
    Tled = 1;
    Dled = 5;
    coneT = .8;
    Hled = L - 10;
    theta = headtheta;

    module bulk(wallT=0){

      Rtop = Rtop - wallT;
      Rbot = Rbot - wallT;

      hull(){
        translate([0,0,L-Rtop - wallT]) sphere(Rtop);
        cylinder(h=.0001, r=Rbot);
      }
    }

    module ledmount(){

      intersection(){
        bulk();
        translate([0,0,Hled])
        difference(){
          cylinder(h=Tled, d=2*Rbot);
          cylinder(h=Tled, d=Dled);
        }
      }
    }
   
    module ring(){
      ringT = 1.2;
      ringH = 2;

      intersection(){
        difference(){
          shift(){
            difference(){
              bulk(-ringT);
              bulk(-TIGHTSP);
            }
          }
          moveshell_() head_bulk();
        }
        moveshell_() head_bulk(-ringH);
      }
    }

    module build(){
      module cone(){
        difference(){
          bulk();
          bulk(coneT);
        }
        ledmount();
      }

      difference(){
        shift() cone();
        moveshell_() head_bulk(wallT);
      }
      ring();

    }

    module shift(){
      //for (i=[-1,1]){
      i = 0;
      translate([i*12, yback + Rbot + rcorner + 5, Hhead-Rbot*sin(theta)])
      rotate([theta,0, 0]) children();
      //}
    }

    if (key=="cut") shift() bulk(-TIGHTSP*2);
    else build();


  }
  

  if (key=="top"){top_shell_();}
  if (key=="bottom"){bottom_shell_();}
  if (key=="servoparts"){servo_("axleparts");}
  if (key=="servo"){servo_("cut");}
  if (key=="camera"){camera_("cut"); camera_("poles");}
  if (key=="mockup"){moveshell_() head_bulk(); camera_("cut"); mouth_("mockup"); antenna_();}
  if (key=="rpi"){rpi_(); rpi_("cut");}
  if (key=="antenna"){antenna_();}
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
//key="bottom"; //"top"; //
head(key);
//head("top");
//head_shell(wallT);
//head("mouth");
//head("antenna");
//head("rpi");
//head("camera");
//head("rpi");
//head("servo");
//head("servoparts");
//head_mockup3("servo");

