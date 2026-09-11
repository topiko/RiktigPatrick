use <head.scad>;
use <frame.scad>;
include <usedims.scad>;

phi=10;
theta=0; //-90 +30;


translate([0,0,Hframe])
//moverothead(phi, theta) head_mockup();
//moverothead(phi, theta) head_mockup2();
moverothead(phi, theta) head("mockup");
translate([0,-Rtop,0]) frame_bulk(0);
