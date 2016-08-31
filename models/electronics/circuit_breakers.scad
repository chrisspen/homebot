
//http://www.digikey.com/product-detail/en/te-connectivity-potter-brumfield-relays/W58-XC4C12A-7/PB246-ND/254464
module make_w58_series(){
    cube([34.93, 34.93, 16.76], center=true);
    translate([34.93/2,0,0])
    cylinder(d=10, h=16.76, center=true);
    translate([-34.93/2,0,0])
    rotate([0,90,0])
    cylinder(d=10, h=16.76, center=true);
}

module make_ts_701(){
    cube([35.5, 31, 13], center=true);
    translate([35.5/2,0,0])
    cylinder(d=10, h=13, center=true);
    translate([-35.5/2,0,0])
    rotate([0,90,0])
    cylinder(d=10, h=16.76, center=true);
}

module make_104_pr(){
    cube([27.6, 21, 11], center=true);
    translate([-27.6/2,0,0])
    rotate([0,90,0])
    cylinder(d=10, h=5, center=true);
}

//http://www.e-t-a.com/fileadmin/user_upload/Ordnerstruktur/pdf-Data/Products/Elektromechanik/1_pdf_thermisch/1_pdf_englisch/D_1410-L1_L4_ENG.pdf
module make_1410(){
    cube([25, 12.5, 15], center=true);
    translate([0,0,15/2])
    rotate([0,0,0])
    cylinder(d=10, h=5, center=true);
}

//make_w58_series();
//make_ts_701();
//make_104_pr();
make_1410();
