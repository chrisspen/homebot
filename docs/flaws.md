Flaws
=====

This is a list of known design flaws.

1. battery charger overcharges

    If the battery pack is left connected while the robot is connected to external power, the LinkMan 2S-3S Lipo Balance Charger trickle charges the lipo until dangerously overcharged. Testing found that the 7.4V battery was overcharged to 8.9V, causing the battery to swell.
    
    Future designs should modify the recharge board to include a transistor that allows disconnecting the charger from the battery once the voltage reaches a safe storage voltage of 7.4V.
    
    This is really caused by the larger problem of our use of LiPo batteries. We're essentially trying to use the LiPo like an uninterruptible power supply, which means we try to keep the battery fully charged at all times. However, unlike Lead Acid batteries, used in most UPSes, which like to be kept fully charge, LiPo batteries tend to deterioate if kept at their full voltage for a few days.
    
    Workaround: When the robot is connected to external power, the battery should be charged to a safe storage charge of 7.4V and then removed.
    
    Effect: serious
    
2. limited expansion

    The overall footprint was kept small (15cm^2) to minimize cost and 3d printing time.
    However, even with use of small electronics, like the Arduino and Raspberry Pi, this leaves very little room for expansion and wire routing.
    
    Future designs should double this footprint.
    
    The motors and battery currently selected should still be powerful enough to actuate that size.
    
    Effect: average

3. front-panels are difficult to access

    The front panels, such as the bumper/edge sensor array, recharging dock, ultrasonic sensor array, and powerbutton, were designed as mainly a single piece, with electronics mounted behind, and then the entire assembly screwed onto the frame. This is difficult to access for maintainence because the panels can't easily be removed without also removing the wire connections to the Arduino. And in order to remove those, the head must be removed.
    
    Future designs should make enough room so wire slack can be added so the panels can be removed, or a socket used so the panel's electrical connections don't need direct wiring.
    
    Effect: average

4. battery charger discharges battery when load is off and external power is absent

    This is caused by the charger drawing current through the 3-pin balanced charging cable

    Future designs should add a diodes to ensure current only goes to the battery, back into the charger.

    Workaround: When the robot is turned off and disconnected from external power, the battery should be removed.
    
    Effect: average

5. treads lack suspension

    The treads ends are too blunt to overcome some sharp obstacles like door thresholds, without subjecting the robot to extensive shock.
    
    Future designs should raise the main idlers up, and include smaller more inset idlers, to allow suspension to dampen shocks.

    Workaround: place ballast in the bottom panels
    
    Effect: average

6. jittery head tilt

    The servo used to tilt the head up and down has considerable jittery, due to either a poor servo, or inadequate shielding along the servo control cable.
    
    Effect: average

7. inconsistent head pan reference sensor

    The sensor used to mark a centered head position is an IR sensor detecting a white reflective strip. This leads to false positives if there's considerable ambient light.
    
    Future designs should either use IR interruptor switch or hall effect sensor.
    
    Effect: minor

8. inadequate restraint in head side panels

    The two head neck struts are mechanically attached to the center head box via the servo spline on one side and a small clamp on the other. This allows for considerable play between the side head panels and the center head panels, creatign several mm of gap.
    
    Future designs should add small sliding retaining clasps, that attach to the corners of the head box and loop over the edge of the neck struts.

    Effect: minor
