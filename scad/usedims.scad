
acc =  "show"; //"print"; 

$fa = acc=="print" ? 2 : 7; //1;
$fs = acc=="print" ? .3 : 1; //.2; //.02


mountT=22;
rcorner=2;

//BODY:
Rtop=11; //mountT;
Hframe=193;
Wbottom=100; //95; //  124;
alpha = 4.5; //4.5; // 7.5
Wtop = edgex(Hframe)*2; // Wbottom - 2*Hframe*tan(alpha); //Whead-2; // + 2*Rtop*tan(alpha);
//alpha = atan(((Wbottom-Wtop)/2)/(Hframe + Rtop));
Rbottom = rcorner;
Tframe=55;
Hthick = 30;

//HEAD:
//Whead=59; 
Hhead=115;
Lhead=42;
Hneck = .4;
Rhead=rcorner;
headaddW=0;
Zheadbottom=Hframe+Rtop+Hneck+Rhead; 
Wheadbottom=edgex(Zheadbottom)*2 + headaddW; 
Wheadtop=edgex(Zheadbottom+Hhead)*2 + headaddW; 
wallT=1.60;
headtheta=7;
headBottomT = Lhead - mountT;
headServoNteeth=15;
mountD=6;
dwall = 4;
headmountpolepos1=[for (z=[dwall]) [edgex(z, Wheadbottom) - dwall,z]];
sf = 6;
headmountpolepos2= concat(headmountpolepos1, [for (p=headmountpolepos1) [-p[0], p[1]]]); // [[Whead/2 - dwall, Hhead/2 - sf], [-Whead/2 + dwall, Hhead/2 + sf]]);
headmountpolepos= concat(headmountpolepos2, [for (z=[[-1, Hhead/2-4], [1, Hhead/2-4]]) [z[0]*edgex(z[1], Wheadbottom)-z[0]*dwall, z[1]]]); // [[Whead/2 - dwall, Hhead/2 - sf], [-Whead/2 + dwall, Hhead/2 + sf]]);



//
function edgex(h, W=Wbottom) = W/2 - h*tan(alpha);

framemountpolepos_l=[for (h=[-Rbottom+dwall,Hframe/2-12, Hframe-40, Hframe]) [edgex(h) - dwall, h]];
framemountpolepos_r=[for (p=framemountpolepos_l) [-p[0], p[1]]];
framemountpolepos_b=[for (i=[-1,1]) [i*18, -Rbottom + dwall]];
framemountpolepos = concat(framemountpolepos_l, framemountpolepos_r, framemountpolepos_b);

echo(framemountpolepos);

TIGHTSP = .05;
MEDIUMSP = .1;
//
// BOLTS:
BOLT5TIGHT=4.8;
BOLT5LOOSE=5.05;
BOLT3TIGHT=2.8;
BOLT3LOOSE=3.05;
BOLT25TIGHT = 2.30; // 2.35
BOLT25LOOSE = 2.60;
BOLT15TIGHT = 1.30; // 2.35
BOLT15LOOSE = 1.60;

AXLEBEARINGDIMS = [10, 15, 4];
AXLECOVERT = 1.2;
SERVOHORNSP = .96;



