use <../electronics/audio.scad>;

translate([-5,0,0])
rotate([0,-90,0])
make_speaker_mount(side=3);

rotate([0,90,0])
make_speaker_mount(side=4);

translate([16,0,7.725])
rotate([0,50+180,0])
make_speaker_mount_bridge();