use <utils.scad>;
use <electronics.scad>;
use <servo_mount.scad>;
use <nema14.scad>;
include <usedims.scad>;


module frame_bulk(modRs=0){
  
  //alpha = atan(((Wbottom-Wtop)/2)/Hframe);
  module rotmove(H, y){
    translate([0,y+modRs,H])
    rotate([0,-90,0]) children(); 
  }
  
  Rtop = Rtop - modRs;
  Rbottom = Rbottom - modRs;
  rcorner=rcorner - modRs;
  Wbottom = Wbottom - 2*modRs;
  Wtop= Wtop- 2*modRs;

  phiback = 20;
  addHback =  tan(phiback)*Rtop;
  addHfront = 5;
  hull(){
    rotmove(Hframe,Rtop) rounded_tilted_cylinder(Wtop, Rtop, rcorner, alpha);
    rotmove(Hframe + addHback,Rbottom) rounded_tilted_cylinder(Wtop-2*addHback*tan(alpha), Rbottom, rcorner, alpha);
    //rotmove(Hframe+addHfront,Rtop + sqrt(pow((Rtop - Rbottom), 2) - pow(addHfront, 2))) rounded_tilted_cylinder(Wtop-2*addHfront*tan(alpha), Rbottom, rcorner, alpha);
    rotmove(0,Rbottom) rounded_tilted_cylinder(Wbottom, Rbottom, rcorner, alpha);
    rotmove(0,Rbottom + 32) rounded_tilted_cylinder(Wbottom, Rbottom, rcorner, alpha);
    rotmove(Hthick, Rbottom + Tframe) rounded_tilted_cylinder(Wbottom - 2*Hthick*tan(alpha), Rbottom, rcorner, alpha);
  }

}

module frame_shell(wallT, key="top"){

  module shell(){
    difference(){
      frame_bulk();
      frame_bulk(wallT);
    }
  }

  maxW = 2*max(Wtop, Wbottom);

  module choose(moveY=0){
    module cutter(){translate([-maxW/2, 0, -Hframe]) cube([maxW, Rtop, 3*Hframe]);}
    
    if (key=="bottom"){
      intersection(){
        children();
        cutter();
      }
    }
    else if (key=="top"){
      difference(){
        children();
        cutter();
      }
    }
   
    else if (key=="bulk"){
      children();
    }
  }
 
  choose() shell();


}


module frame(key){  
  yback = -Rtop;
  module moveshell_(){translate([0,yback,0]) children();}
  
  module mountpoles_(key){
    sel = key=="bottom" ? -1 : 1;
    holeD = key=="bottom" ? BOLT25LOOSE : BOLT25TIGHT;
    modT = key=="bottom" ? -.5 : .5;
    // Mounting posts:
    //moveshell_()
    intersection(){
      selector(modT, sel) moveshell_() frame_bulk();
      mount_poles_bolts(framemountpolepos, mountD, holeD=holeD);
    }
  }
  module cable_hole(){
    translate([24, 0, Hframe-28]){
      rounded_cutter(40, 10 ,12, 1);
      translate([0,-10,-7.5+20]) rotate([90,0,0])rounded_cutter(20, ,10 ,15, 1);
    }
  }

  module bottom(){
    module bottom_(){
      moveshell_() frame_shell(wallT,"bottom");

      // Neck servo
      intersection(){
        selector(0, -1) moveshell_() frame_bulk();
        neckservo("top");
      }

      // leg motor mounts
      intersection(){
        selector(0, -1) moveshell_() frame_bulk();
        motors("mount");
      }

      mountpoles_("bottom");
      arduino("poles");
      protoplate("poles");
    }
    
    difference(){
      bottom_();
      mount_poles_bolts(framemountpolepos, BOLT25LOOSE, holeD=0, negy=yback);
      neckservo("cut");
      motors("cut");
      cable_hole();
      arduino("bolts");
      protoplate("bolts");
      switches();
      batt_plate("chargecable");
    }
  }
  module top(){
    //selector(0, 0)
    module top_(){
      moveshell_() frame_shell(wallT, "top");
      // Neck servo mount
      intersection(){
        selector(0, 1) moveshell_() frame_bulk();
        neckservo("bottom", 4*Rtop);
      }
      // leg motor mounts
      intersection(){
        selector(0, 1) moveshell_() frame_bulk();
        motors("mount");
      }
      // Batt plate
      intersection(){
        selector(0, 1) moveshell_() frame_bulk();
        batt_plate();
      }
      // Converter poles:
      intersection(){
        selector(0, 1) moveshell_() frame_bulk();
        converter("poles");
      }





      mountpoles_("top");
    }
    difference(){
      top_();
      intersection(){
        mount_poles_bolts(framemountpolepos, BOLT25TIGHT, holeD=0, negy=yback);
        moveshell_() frame_bulk(wallT);
      }
      neckservo("cut");
      motors("cut");
      cable_hole();
      converter("bolts");
      arduino("bolts");
     batt_plate("batt");
    }

  }

  module neckservo(key="cut", T=2*Rtop){
    translate([0,0, Hframe])
    rotate([0,0,180])
    rotate([0,90,0])
    servo_mount_aligned(key=key, servoNtooth=headServoNteeth+2, 
                        //axleNtooth=headServoNteeth+8, 
			axleL=18, W=56, T=T, H = 90, 
			turnAngle=90,  
                        armAngleMiddle=200,
                        axlehornDin=BOLT3TIGHT, type=1, 
                        hornarmL=20, addXL=12, modbolts=true);

  }

  module batt_plate(key="plate"){
    translate([0,0,(Hframe - 75)]){
    if (key=="batt"){
      rotate([0,0,0]){
        translate([-15.5,5,0])battery();
        translate([15.5,5,0])battery();
      }
    }
    else if (key=="chargecable"){
      W=18;
      T=2.45;
      translate([0,yback,+30.7]) cube([W,20,T], center=true);
    }
    else {
      L1 = 20;
      L2 = 10;
      translate([0,0,-(wallT + 55)/2]) 
      difference(){
        cube([Hframe*2, Hframe, wallT], center=true);
        for (i=[-1,0,1]) for (j=[0,1]) translate([i*L1, j*L2 + 23.5, 0]) rounded_cutter(20, 10, 3, 1);
        rounded_cutter(20, 55, 28, 5);
      }
    }
    }
  }

  //batt_plate("chargecable");
  module switches(key){
    dswitch=5 + .5;
    sp=1; 
    module sw_(i,h){
      translate([i*edgex(h), yback + wallT + dswitch/2 + sp, h])
      rotate([0,i*(90-alpha),0]) cylinder(h=20, d=dswitch, center=true);
    }
    h = 105;
    D=10;
    sw_(-1, h);
    sw_(1, h);
  }  
//  switches();

  module motors(key){
    module left(){
      nemah = 21;
      nemamountT=2.2;
      nemay =yback + 20; // 10;
      translate([edgex(nemah), nemay, nemah])
      rotate([0,90-alpha,0]) 
      if (key=="nema"){nema14_z0(nemamountT);}
      else if (key=="mount"){nema14_mountz0(nemamountT);}
      else if (key=="bolts"){nema14_boltsz0(10, T=nemamountT);}
      else if (key=="cut"){nema14_z0(nemamountT);nema14_boltsz0(10, T=nemamountT);}
    }
    left();
    mirror([1,0,0]) left();
  }

  
  module arduino(key="mockup"){
    translate([0,yback,43.5/2 - Rbottom + wallT + 1. ]) //+wallT+1])
    rotate([-90,0,0])
    rotate([0,0,90])
    nanoble(H=3+wallT, key=key);
  }

  module protoplate(key="cut"){
    translate([0,yback,63])
    rotate([-90,0,0]) protoboard2(key=key, H=3+wallT);
  }
  
  module converter(key="cut"){
    theta=12.5;
    addH = 25;
    Lleg = 6;
    translate([0,Tframe + Rbottom + Lleg + yback - addH*tan(theta) - 4.0,Hthick + addH])
    rotate([90+theta,0,0]) buckconverter(H=Lleg, key=key);
  }
  //converter();
  //arduino();
  //motors("mount");
  //motors("nema");
  if (key=="bottom") bottom();
  if (key=="top") top();
  if (key=="batt") batt_plate("batt");
  if (key=="motors") motors("nema");
  if (key=="servoparts") necservo("axleparts");
  if (key=="servo") neckservo("mockup");
  if (key=="arduino") {arduino();arduino("bolts");}
  if (key=="converter") converter("mockup");
  //batt_plate("batt");
  //switches();
  //top();
  //neckservo();
  //neckservo("servo");
}
key= "bottom"; //"arduino"; //"top";//
//frame(key);
frame("top");
//frame("servo");
//frame("converter");
//frame("arduino");
frame("batt");


