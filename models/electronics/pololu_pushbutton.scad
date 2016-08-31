/*
Pololu 2808 mini pushbutton power switch

https://www.pololu.com/product/2808
*/

module make_pololu_2808_pushbutton(smd_clearway=1){
    
    angle_header_h = 5+3;//TODO:confirm
    board_w = 15.4;
    board_l = 18;
    
    //smd bb
    color("orange")
    translate([0,0,3.048/2])
    cube([
        board_w,//-2.54*2,
        board_l-1,
        3.048], center=true);
    
    if(smd_clearway)
    color("orange")
    translate([0,0,3.048])
    cube([
        board_w-2.54*2,
        board_l-1,
        3.048*2], center=true);
    
    //board
    color("brown")
    translate([0,0,1/2])
    cube([board_w,board_l,1], center=true);
    
    //angle headers
    translate([board_w/2-2.54/2,0,-(angle_header_h)/2+.01])
    cube([2.54,board_l,angle_header_h], center=true);
    translate([-(board_w/2-2.54/2),0,-(angle_header_h)/2+.01])
    cube([2.54,board_l,angle_header_h], center=true);
    
}
    
make_pololu_2808_pushbutton();
