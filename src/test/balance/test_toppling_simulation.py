#!/usr/bin/env python
"""
https://github.com/pybox2d/pybox2d/wiki/manual
"""
from Box2D import *

#https://github.com/pybox2d/pybox2d/blob/master/examples/backends/pygame_framework.py
from examples.framework import main, Keys
from examples.backends.pygame_framework import PygameFramework

#DIVISOR = 1000.#too high, shapes don't move
DIVISOR = 10.

LEDGE_HEIGHT_METERS = 10/DIVISOR

def calculate_density(volume, mass):
    # density = mass/volume
    ret = mass/float(volume)
    #print 'density:', ret
    #return 10000
    return ret

class CounterbalanceTest(PygameFramework):
    name = "Counterbalance Test"
    #description = 'A test for very fast moving objects (bullets)'

    def __init__(self):
        super(CounterbalanceTest, self).__init__()

        ground = self.world.CreateStaticBody(
            position=(0, 0),
            shapes=[
                b2EdgeShape(vertices=[(-100, 0), (0, 0)]),
                b2EdgeShape(vertices=[(0, 0), (0, -LEDGE_HEIGHT_METERS)]),
                b2EdgeShape(vertices=[(0, -LEDGE_HEIGHT_METERS), (100, -LEDGE_HEIGHT_METERS)]),
            #    b2PolygonShape(box=(0.2, 1, (0.5, 1), 0))
            ]
        )

#         self._x = 0.20352793
        #1 unit = meter or kg
        
        self.speed = 1
        use_counterweight = 1
        wheel_friction = 100
        wheel_torque = 2000
        wheel_radius = 25/DIVISOR
        wheel_hz = 4.0
        wheel_zeta = 1.0#0#0.7 # spring dampening
        #wheel_axis = (0.0, -1.)#ok
        wheel_axis = (0.0, -1.25)
        #wheel_axis = (0.0, 0.0)
        
        body_offset_x = -(65+100)/DIVISOR
        body_offset_y = wheel_radius*2
        
        body_box = (65/DIVISOR, 200/DIVISOR)
        body_volume = body_box[0] * body_box[1]
        body_mass = 1300/DIVISOR # kg
        self.body = self.world.CreateDynamicBody(
            position=(body_offset_x, body_box[1] + body_offset_y),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=body_box),
                density=calculate_density(volume=body_volume, mass=body_mass)),
        )
        #print [_ for _ in dir(self.body) if 'mass' in _.lower()]
            
        #Set dampingRatio to 1.0 for zero oscillations (i.e. inflexible distance).
        #You may need to set frequencyHz to 0 as well.
        
        #mass_data = self.body.GetMassData()
        mass_data = self.body.massData
        
        # Make COM a little forward and top-heavy. => unstable
        mass_data.center[0] = body_box[0]*.5
        mass_data.center[1] = body_box[1]*.5
        
        # Make COM horizontally centered but top heavy. => a little unstable
#         mass_data.center[0] = body_box[0]*.0
#         mass_data.center[1] = body_box[1]*.5
         
        # Make COM perfectly centered. => stable
#         mass_data.center[0] = body_box[0]*.0
#         mass_data.center[1] = body_box[1]*.0
        
        if use_counterweight:
            
            counterweight_mass_grams = 1000000
            counterweight_mass = counterweight_mass_grams/1000./DIVISOR 
            counterweight_box = (20/DIVISOR, 20/DIVISOR)
            counterweight_volume = counterweight_box[0]*counterweight_box[1]
            counterweight = self.world.CreateDynamicBody(
                position=(body_offset_x-20/DIVISOR+1/DIVISOR, wheel_radius*2 - counterweight_box[1]/2),
                fixtures=b2FixtureDef(
                    shape=b2PolygonShape(box=counterweight_box),
                    density=calculate_density(volume=counterweight_volume, mass=counterweight_mass)),
            )
            self.world.CreateWeldJoint(
                bodyA=self.body,
                bodyB=counterweight,
                anchor=counterweight.position,
            )
        
        self.body.massData = mass_data
        print 'mass_data:', mass_data
        
        #-x = left, +x=right
        left_wheel = self.world.CreateDynamicBody(
            position=(body_offset_x-65/DIVISOR, wheel_radius),
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=wheel_radius),
                friction=wheel_friction,
                density=1.0,
            )
        )
        right_wheel = self.world.CreateDynamicBody(
            position=(body_offset_x+65/DIVISOR, wheel_radius),
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=wheel_radius),
                friction=wheel_friction,
                density=1.0,
            )
        )
        self.wheels = [left_wheel, right_wheel]
        
        springs = self.springs = []
        
        left_spring = self.world.CreateWheelJoint(
            bodyA=self.body,
            bodyB=left_wheel,
            anchor=left_wheel.position,
            axis=wheel_axis,
            motorSpeed=0.0,
            maxMotorTorque=wheel_torque,
            enableMotor=True,
            frequencyHz=wheel_hz,
            dampingRatio=wheel_zeta
        )
        springs.append(left_spring)
        
        right_spring = self.world.CreateWheelJoint(
            bodyA=self.body,
            bodyB=right_wheel,
            anchor=right_wheel.position,
            axis=wheel_axis,
            motorSpeed=0.0,
            maxMotorTorque=wheel_torque,
            enableMotor=True,
            frequencyHz=wheel_hz,
            dampingRatio=wheel_zeta
        )
        springs.append(right_spring)
        
    def Keyboard(self, key):
        if key == Keys.K_a: # forward
            for spring in self.springs:
                spring.motorSpeed = -self.speed
        elif key == Keys.K_s: # stop
            for spring in self.springs:
                spring.motorSpeed = 0
        elif key == Keys.K_d: # backward
            for spring in self.springs:
                spring.motorSpeed = self.speed
        elif key in (Keys.K_q, Keys.K_e):
            if key == Keys.K_q:
                self.hz = max(0, self.hz - 1.0)
            else:
                self.hz += 1.0

            for spring in self.springs:
                spring.springFrequencyHz = self.hz
                
#     def Launch(self):
#         self.body.transform = [(0, 4), 0]
#         self.body.linearVelocity = (0, 0)
#         self.body.angularVelocity = 0
# 
#         self.x = b2Random()
#         self.bullet.transform = [(self.x, 10), 0]
#         self.bullet.linearVelocity = (0, -50)
#         self.bullet.angularVelocity = 0

    def Step(self, settings):
        super(CounterbalanceTest, self).Step(settings)
#         if (self.stepCount % 60) == 0:
#             self.Launch()

if __name__ == "__main__":
    main(CounterbalanceTest)
    