use <MCAD/shapes/polyhole.scad>
include <MCAD/units/metric.scad>

pin_d = 6.7;
pin_depth = 23;

shaft_d = 6;
shaft_wall_thickness = 5;

wheel_d = 50;
wheel_clearance = 6;
wheel_cover_thickness = -epsilon;
wheel_cover_d = wheel_d + wheel_clearance + wheel_cover_thickness;
center_thickness = 14;

hub_height = wheel_cover_d / 2;

caster_width = 38;
hub_d = caster_width;
shaft_offset = hub_d / 2;

$fs = 0.4;
$fa = 1;

module ball_end_cylinder (d, h)
{
    intersection () {
        union () {
            cylinder (d = d, h = h);
            sphere (d = d);
        }

        translate ([0, 0, -d / 2])
        cylinder (d = d, h = h + d / 2);
    }
}

module caster ()
{
    elevation = shaft_wall_thickness + shaft_d / 2;

    translate ([0, 0, elevation])
    difference () {
        hull () {
            rotate (90, X)
            cylinder (d = wheel_cover_d, h = caster_width, center = true);

            translate ([shaft_offset, 0, 0])
            cylinder (d = hub_d, h = wheel_cover_d / 2);
        }

        // cut off bottom
        translate ([0, 0, -wheel_cover_d - elevation])
        linear_extrude (height = wheel_cover_d)
        square ([1, 1] * wheel_cover_d * 2.5, center = true);

        // cut out wheels
        for (y = [1, -1] * (center_thickness / 2 + caster_width / 2))
        translate ([0, y])
        rotate (90, X)
        translate ([0, 0, -caster_width / 2])
        mcad_polyhole (
            d = wheel_d + wheel_clearance,
            h = caster_width
        );

        // pin
        translate ([shaft_offset, 0, wheel_cover_d / 2 - pin_depth])
        mcad_polyhole (d = pin_d + 0.3, h = wheel_d);

        // shaft
        rotate (90, X)
        translate ([0, 0, -caster_width / 2])
        mcad_polyhole (d = shaft_d + 0.3, h = caster_width);
    }
}

!caster ();
