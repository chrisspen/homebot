Flaws
=====

This is a list of known design flaws.

1. battery charger overcharges

    If the battery pack is left connected while the robot is connected to external power, the LinkMan 2S-3S Lipo Balance Charger trickle charges the lipo until dangerously overcharged. Testing found that the 7.4V battery was overcharged to 8.9V, causing the battery to swell.
    
    Future designs should modify the recharge board to include a transistor that allows disconnecting the charger from the battery once the voltage reaches a safe storage voltage of 7.4V. 
    
    Workaround: When the robot is connected to external power, the battery should be charged to a safe storage charge of 7.4V and then removed.
    
2. limited expansion

    The overall footprint was kept small (15cm^2) to minimize cost and 3d printing time.
    However, even with use of small electronics, like the Arduino and Raspberry Pi, this leaves very little room for expansion and wire routing.
    
    Future designs should double this footprint.
    
    The motors and battery currently selected should still be powerful enough to actuate that size.

3. front-panels are difficult to access

    The front panels, such as the bumper/edge sensor array, recharging dock, ultrasonic sensor array, and powerbutton, were designed as mainly a single piece, with electronics mounted behind, and then the entire assembly screwed onto the frame. This is difficult to access for maintainence because the panels can't easily be removed without also removing the wire connections to the Arduino. And in order to remove those, the head must be removed.
    
    Future designs should make enough room so wire slack can be added so the panels can be removed, or a socket used so the panel's electrical connections don't need direct wiring.

4. battery charger discharges battery when load is off and external power is absent

    This is caused by the charger drawing current through the 3-pin balanced charging cable

    Future designs should add a diodes to ensure current only goes to the battery, back into the charger.

    Workaround: When the robot is turned off and disconnected from external power, the battery should be removed.

5. treads lack suspension

    The treads ends are too blunt to overcome some sharp obstacles like door thresholds, without subjecting the robot to extensive shock.
    
    Future designs should raise the main idlers up, and include smaller more inset idlers, to allow suspension to dampen shocks.

    Workaround: place ballast in the bottom panels
