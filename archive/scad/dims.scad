
Llow = 96;
Lup_low = 119;
Lup_top = 65;
Wbot=80; //72;
servoaxleL = Wbot/5*2;
hornW = .85*servoaxleL;
Dbot=30;
Lservo = 57;
wallT = 2;
rbot=wallT/2;
rcablebundle=6;
xcablebundle = Wbot/2 - rcablebundle - rbot - 5.5;
xconnection=-3; //-6.1;
mouthR = 11/2+wallT; //Dbot/4;
dedge = Wbot/2 - rbot - 3.5; //BOLT3LOOSE;

BEAMMAXANGLE=120;
mouthcablex = 13;
CLOSEBARD=3+.42*8;
CHINDIST=.2;
SERVOTURNANGLE = 120;

beamServoNtooth= 20;
mg215dims = [23, 26.5, 12, 6];	
LHORNTHREAD = 12;
hornarmL=Dbot/2 - 5.3;
xleg=10;

Wlegs = Wbot + 2;
Tlegs = Dbot/3*2;
Hlegs = 18;

// BOLTS:
BOLT5TIGHT=4.8;
BOLT5LOOSE=5.05;
BOLT3TIGHT=2.8;
BOLT3LOOSE=3.05;
BOLT25TIGHT = 2.30; // 2.35
BOLT25LOOSE = 2.60;
BOLT15TIGHT = 1.30; // 2.35
BOLT15LOOSE = 1.60;

// GEARS:
//GEART = 12; //6
GEARMODUL = 1.0; //1.0;
HELIXANGLE=20; // 30
PRESSUREANGLE=10; //25;

SERVOGEARNTOOTH = 15; // 15
SERVOHORNT = 7.0; //6
SERVOHORNBEARINGT = 4;

// Bore, Diam, Thickness
SERVOBEARINGDIMS = [5, 11, 4];
AXLEBEARINGDIMS = [10, 15, 4];
WHEELBEARINGDIMS = [20, 27, 4];

AXLEHORNDIN = AXLEBEARINGDIMS[0]/sqrt(2);

SERVOHORNSP = .96;
TIGHTSP = .05;
RADSP = .15; // .25
AXLECOVERT = 1.2;

THREADH=.6;
THREADPITCH = 2;



GEARCOLOR="Ivory"; //WhiteSmoke"; //SlateGray";
BEARINGCOLOR="Gray";
$fn = 0; //50000;


THREADSPACEPHI = 30; // Positive means you have to twist further to tighten by phi than theory.
SHOWALIGNERS = false; //true; 
CUTALIGNERS = false; // Cut wedges to gears to check match.
