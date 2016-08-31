// library for parametric involute gears 
// Author: Rudolf Huttary, Berlin 
// Jan 2015

$fn = 500; 
iterations = 10; // increase for enhanced resolution beware: large numbers will take lots of time!

// default values
// z = 10; // teeth - beware: large numbers may take lots of time!
// m = 1;  // modulus
// x = 0;  // profile shift
// h = 4;  // face_width	respectivly axial height
// w = 20; // profile angle
// clearance  // assymmetry of tool to clear tooth head and foot
//    = 0.1; // for external splines
//    = -.1  // for internal splines 
// w_bevel = 45; // axial pitch angle
// w_helix = 45; // radial pitch angle 

// use this prototype:
// gear(m = modulus, z = Z, x = profile_shift, w = alpha, h = face_width);

//====  external splines with default values ===
// gear();  
// gear_helix(); 
// gear_herringbone(); 
// gear_bevel(); 

//====  internal splines with default values ===
// Gear();  
// Gear_helix(); 
// Gear_herringbone(); 
// Gear_bevel(); 
// 

//====  internal splines - usage and more examples ===
// gear(z = 25, m = 1, x = -.5, h = 4); 
// gear_bevel(z = 26, m = 1, x = .5, h = 3, w_bevel = 45, w_helix = -45); 
// gear_helix(z = 16, m = 0.5, h = 4, w_helix = -20, clearance = 0.1); 
// gear_herringbone(z = 16, m = 0.5, h = 4, w_helix = -20, clearance = 0.1); 


//====  external splines - usage and more examples ===
// Gear(z = 10, m = 1.1, x = 0, h = 2, w = 20, D = 14, clearance = -0.2); 
// gear(z = 10, m = 1, x = .5, h = 4, w = 20, D = 13, clearance = 0.2); 

//Gear_herringbone(z = 40, m = 1, h = 4, w_helix = 45, clearance = -0.2, D = 49); 
// gear_herringbone(z = 40, m = 1, h = 4, w_helix = 45, clearance = 0.2); 

// Gear_helix(z = 20, m=1.3, h = 15, D = 30, w_helix = 45, clearance = -0.2); 
// gear_helix(z = 20, m=1.3, h = 15, w_helix = 45, clearance = 0.2); 


// ====  grouped examples ===
//	di = 18; //axial displacement
//	translate([di, di])   Gear(z = 25, D=32); 
//	translate([-di, di])  Gear_helix(z = 25, D=32); 
//	translate([-di, -di]) Gear_herringbone(z = 25, D=32); 
//	translate([di, -di])  Gear_bevel(z = 25, D=32); 
//	
//	di = 8; //axial displacement
//	translate([di, di])   gear(); 
//	translate([-di, di])  gear_helix(); 
//	translate([-di, -di]) gear_herringbone(); 
//	translate([di, -di])  gear_bevel(); 

// profile shift examples
//	di = 9.5; //axial displacement
//	gear(z = 20, x = -.5); 
//	translate([0, di+3]) rotate([0, 0, 0]) gear(z = 7, x = 0, clearance = .2); 
//	translate([di+3.4, 0]) rotate([0, 0, 0]) gear(z = 6, x = .25); 
//	translate([0, -di-3]) rotate([0, 0, 36])   gear(z = 5, x = .5); 
//	translate([-di-3.675, 0]) rotate([0, 0, 22.5]) gear(z = 8, x = -.25); 
 

// tooth cutting principle - dumping frames for video
//	du = PI/36; 
//	i = -($t-.5)*72; 
//	rotate([0, 0, i])
//	translate([i*du, 0, 0])
//	Tool(); 
//	Tooth_half(); 



// === modules for internal splines
module Gear(m = 1, z = 10, x = 0, h = 4, w = 20, D = 13, clearance = -.1, center = true) 
{
   difference()
   {
		cylinder(r = D/2, h = h, center = center);
		gear(m, z, x, h+1, w, center = center, clearance = clearance); 
	}
}

module Gear_herringbone(m = 1, z = 10, x = 0, h = 4, w = 20, w_helix = 45, D = 13, clearance = -.1) 
{
   difference()
   {
		cylinder(r = D/2, h = h-.01, center = true); // CSG!
		gear_herringbone(m, z, x, h, w, w_helix, clearance = clearance); 
	}
}

module Gear_helix(m = 1, z = 10, x = 0, h = 4, w = 20, w_helix = 45, D = 13, clearance = -.1) 
{
   difference()
   {
		cylinder(r = D/2, h = h-.01, center = true); // CSG!
		gear_helix(m, z, x, h, w, w_helix, clearance = clearance); 
	}
}

module Gear_bevel(m = 1, z = 10, x = 0, h = 4, w = 20, w_bevel = 45, w_helix = 0, D = 13, clearance = -0.1, center = true)
{
   rotate([0, 180, 0])
   difference()
   {
		cylinder(r = D/2, h = h-.01, center = true); // CSG!
		gear_bevel(m, z, x, h, w, w_bevel, w_helix, clearance = clearance, center = center); 
	}
}


// === modules for external splines
module gear_herringbone(m = 1, z = 10, x = 0, h = 4, w = 20, w_helix = 45, clearance = 0.1)
{
   for(i=[0, 1])
   mirror([0,0,i])
   translate([0, 0, -0.001]) // CSG!
   gear_helix(m, z, x, h/2, w, w_helix, center = false, clearance = clearance); 
}

module gear_helix(m = 1, z = 10, x = 0, h = 4, w = 20, w_helix = 45, center = true, clearance = 0.1)
{
   gear_info(m, z, x, h, w, w_helix, 0, clearance); 
	r_wk = m*z/2 + x; 
   tw = (h * w_helix)/ r_wk  ; 
	linear_extrude(height = h, center = center, twist = tw, slices = ceil(10*h), convexity = z)
   gear_(m, z, x, w, clearance); 
}

module gear_bevel(m = 1, z = 10, x = 0, h = 4, w = 20, w_bevel = 45, w_helix = 0, clearance = 0.1, center = true)
{
   gear_info(m, z, x, h, w, 0, w_bevel, clearance); 
	r_wk = m*z/2 + x; 
   sc = (r_wk-tan(w_bevel)*h)/r_wk; 
   tw = (h * w_helix)/ r_wk  ; 
	linear_extrude(height = h, center = center, twist = tw, scale = [sc, sc], convexity = z)
   gear_(m, z, x, w, clearance); 
}

module gear(m = 1, z = 10, x = 0, h = 4, w = 20, clearance = 0.1, center = true)
{
   gear_info(m, z, x, h, w, clearance = clearance); 
	linear_extrude(height = h, center = center, convexity = z)
   gear_(m, z, x, w, clearance); 
}

module gear_info(m = 1, z = 10, x = 0, h = 0, w = 20, helix_angle = 0, bevel_angle = 0, clearance = 0.1)
{
  	r_wk = m*z/2 + x; 
   	dy = m;  
  	r_kk = r_wk + dy;   
  	r_fk = r_wk - dy;  
  	r_kkc = r_wk + dy *(1-clearance/2);   
  	r_fkc = r_wk - dy *(1+clearance/2);  
   echo(str ("modulus = ", m)); 
   echo(str ("Z = ", z)); 
   echo(str ("profile angle = ", w, "°")); 
   echo(str ("d = ", 2*r_wk)); 
   echo(str ("d_outer = ", 2*r_kk, "mm")); 
   echo(str ("d_inner = ", 2*r_fk, "mm")); 
   echo(str ("height = ", h, "mm")); 
   echo(str ("clearance factor = ", clearance)); 
   echo(str ("d_outer_cleared = ", 2*r_kkc, "mm")); 
   echo(str ("d_inner_cleared = ", 2*r_fkc, "mm")); 
   echo(str ("helix angle = ", helix_angle, "°")); 
   echo(str ("bevel angle = ", bevel_angle, "°")); 
   echo("================"); 
}

// === from here 2D stuff == 
module gear_(m = 1, z = 10, x = 0, w = 20, clearance = 0.1)
{
  	r_wk = m*z/2 + x; 
   	dy = m;  
  	r_fkc = r_wk - dy *(1+clearance/2)+.01;  //.01 has numerical reason!
   union()
	{
		for(i = [0:z-1]) 
			rotate([0, 0, i*360/z])
			Tooth(m, z, x, w, clearance);
     circle(r_fkc); 
	}
}

module Tooth(m = 1, z = 10, x = 0, w = 20, clearance = 0)
{
	union()
	{
		Tooth_half(m, z, x, w, clearance); 
		mirror([1, 0, 0])
		Tooth_half(m, z, x, w, clearance); 
	}
}

module Tooth_half(m = 1, z = 10, x = 0, w = 20, clearance = 0)
{
  	r_wk = m*z/2 + x; 
  	r_fkc = r_wk - m *(1+clearance/2);   
	du = z*m/360*PI; 
	dw = 360/z; 
 	difference()
	{
		cake_piece(m, z, x, w, clearance); 
      	union()
		{
			for(i = [-dw:dw/iterations:dw])
			  rotate([0, 0, i])
			  translate([i*du, 0, 0])
			Tool(m, z, x, w, clearance); 
		}
	}
}

module Tool(m = 1, z = 10, x = 0, w = 20, clearance = 0)
{
   p = m*PI; 
   dy = 2*m;  
   dx = dy * tan(w);  
   ddx = dx/2 * clearance/2; 
   ddy = dy/2 * clearance/2; 
	r_wk = m*z/2 + x; 
   polygon(points = [
		[0, r_wk+dy], 
		[0, r_wk+dy/2-ddy], 
		[p/4-dx/2 + ddx, r_wk+dy/2 - ddy], 
		[p/4+dx/2 + ddx, r_wk-dy/2 - ddy], 
		[p/2,            r_wk -dy/2 - ddy],
		[p/2,            r_wk +dy/2 - ddy],
		[p/2,            r_wk+dy], 
		]);  
}

module cake_piece(m = 1, z = 10, x = 0, w = 20, clearance = 0)
{
	$fn = 100; 
	r_wk = m*z/2 + x; 
   p = m*PI; 
   dy = p/2*cos(w);  
   r_kk = r_wk + dy*(1+clearance);   
   intersection()
   {
     half_circle(r_kk); 
     rotate([0, 0, 180])
     rotate([0, 0, -181/z]) // 181 has numerical reason!
     half_circle(r_kk); 
	}
}

module half_circle(r)
{
	intersection()
  	{
		circle(r = r); 
     translate([0, -r, 0])
     square(2*r); 
	}
}

