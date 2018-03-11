include <../settings.scad>;

module make_battery_patriot(){
    cube([battery_length, battery_width, battery_height], center=true);
}

module make_battery_tenergy_4400(){
    cube([67,37,19], center=true);
}

module make_battery_adafruit_6600(){
    //http://www.adafruit.com/product/353
    cube([69,54,18], center=true);
}

module make_powerboost_1000(){
    //https://www.adafruit.com/product/2465
    cube([23,45,10], center=true);
}

module make_battery_18650(tab=0){
    cylinder(d=18.5, h=65+(tab?tab:0), center=true, $fn=100);
}

module make_battery_tray_1(){
    color("red"){
    
    // across
    /*translate([12,0,0])
    rotate([0,0,90])
    make_battery_adafruit_6600();*/
    
    //straight
    translate([5,0,-4])
    rotate([0,0,00])
    make_battery_adafruit_6600();
    
    //make_battery_tenergy_4400();
    }
    
    color("purple")
    /*
    translate([-27,0,0])
    make_powerboost_1000();*/
    
    translate([-45,0,-5])
    rotate([0,0,0])
    make_powerboost_1000();
}

module make_battery_dc12300(){
    //3000mAh
    color("red")
    cube([90,45,18],center=true);
}

module make_battery_dc1298A(){
    //9800mAh
    color("red")
    cube([141,66,23],center=true);
}

//make_battery_patriot();
//make_battery_tenergy_4400();
//make_powerboost_1000();
make_battery_18650();
