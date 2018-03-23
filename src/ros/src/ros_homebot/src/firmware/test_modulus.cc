//gcc -Wall test_modulus.cc -o test_modulus
#include <stdio.h>

int truemod(int v, int r){
    return ((v % r) + r) % r;
}

int calc_angle(int z0, int z1, bool is_cw){
    if(is_cw){
        return truemod(z1 - z0, 360);
    }else{
        return -truemod(z0 - z1, 360);
    }
}

int main (void)
{
    int z0;int z1;int expected_angle;bool is_cw;
    
    z0=354;z1=10;expected_angle=16;is_cw=true;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(354, 10, -344, False),
    z0=354;z1=10;expected_angle=-344;is_cw=false;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(10, 354, 344, True),
    z0=10;z1=354;expected_angle=344;is_cw=true;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(10, 354, -16, False),
    z0=10;z1=354;expected_angle=-16;is_cw=false;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(0, 179, 179, True),
    z0=0;z1=179;expected_angle=179;is_cw=true;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(0, 179, -181, False),
    z0=0;z1=179;expected_angle=-181;is_cw=false;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(0, 181, 181, True),
    z0=0;z1=181;expected_angle=181;is_cw=true;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    //(0, 181, -179, False),
    z0=0;z1=181;expected_angle=-179;is_cw=false;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
    z0=0;z1=400;expected_angle=40;is_cw=true;
    printf ("mod: %i==%i => %i\n", calc_angle(z0, z1, is_cw), expected_angle, calc_angle(z0, z1, is_cw) == expected_angle);
    
  return 0;
}
