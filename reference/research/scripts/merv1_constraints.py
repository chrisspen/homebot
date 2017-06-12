
from constraint_calculator import *

class PhysicalComponent(Component):

    gram = Resource(units='gm', type=float)

    length = Resource(units='mm', type=float)

    width = Resource(units='mm', type=float)

    height = Resource(units='mm', type=float)

    price = Resource(units='dollars', type=float)

    quantity = Resource(type=int, default=1)

class ElectricalComponent(PhysicalComponent):

    min_volt_input = Resource(units='V', type=float)

    max_volt_input = Resource(units='V', type=float)

    min_amp_hour_input = Resource(units='Ah', type=float)

    max_amp_hour_input = Resource(units='Ah', type=float)

    min_volt_output = Resource(units='V', type=float)

    max_volt_output = Resource(units='V', type=float)

    min_amp_hour_output = Resource(units='Ah', type=float)

    max_amp_hour_output = Resource(units='Ah', type=float)

class RawDigitalPeripheral(ElectricalComponent):

    gpio_pin = Resource(type=int)

class Microphone(RawDigitalPeripheral):

    datasource = 'microphones.ods'

class Speaker(RawDigitalPeripheral):

    datasource = 'speakers.ods'

class USBPeripheral(ElectricalComponent):

    usb_host = Resource(type=int)

    usb_client = Resource(type=int)

class WebCamera(USBPeripheral):

    datasource = 'cameras.ods'

    visual_dimension = Resource(type=int)

class SpatialDepthSensor(GroupComponent):
    """
    Collects visual data necessary for directly or indirectly providing
    3-dimensional spatial measurements.
    """

    # Camera must supply two distinct dimensions and be of all the same type.
    # e.g. one stereo camera containing two lens, or two single-lens cameras
    cameras = Resource(WebCamera.constrain(visual_dimension__sum=2).constrain(type__set__len=1))

class MainComputer(RawDigitalPeripheral, USBPeripheral):
    """
    Performs all high-level processing tasks.
    """

    datasource = 'computers.ods'

    ethernet_port = Resource(type=int)

    wifi_transeiver = Resource(type=int)

class SecondaryComputer(RawDigitalPeripheral, USBPeripheral):
    """
    Performs low-level processing tasks and extends the number of IO pins.
    """

    datasource = 'computers.ods'

class Actuator(RawDigitalPeripheral):

    stall_torque = Resource(units='kg*cm', type=float)

    stall_current = Resource(units='A', type=float)

    position_feedback = Resource(type=bool)

    continuous = Resource(type=bool)

class DriveMotor(Actuator):
    """
    Responsible for physically moving the entire device.
    """

    datasource = 'drive-motors.ods'

class SecondaryMotor(Actuator):
    """
    Responsible for moving a sensor or physical component
    for non-locomotion purposes.
    """

class MotorDriver(RawDigitalPeripheral):
    """
    Responsible for controlling the drive motors and potentially any secondary motors.
    """

    datasource = 'motor-drivers.ods'

class Head(GroupComponent):

    camera = Resource(SpatialDepthSensor)

    visor_actuator = Resource(SecondaryMotor.constrain(quantity=1, continuous=False))

    horizontal_rotator = Resource(SecondaryMotor.constrain(quantity=1, continuous=True))

    vertical_rotator = Resource(SecondaryMotor.constrain(quantity=1, continuous=True))

    microphone = Resource(Microphone.constrain(quantity=3))

    speaker = Resource(Speaker)#2?

class Battery(ElectricalComponent):

    datasource = 'batteries.ods'

class Merv1(System):

    main_computer = Resource(MainComputer)

    secondary_computer = Resource(SecondaryComputer)

    head = Resource(Head)

    drive_motors = Resource(DriveMotor.constrain(quantity=2))

    motor_driver = Resource(MotorDriver.constrain(quantity__gte=1, quantity__lte=2))

    batteries = Resource(Battery.constrain(quantity__gte=1, quantity__lte=2))

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Configure MERV1.')
    subparsers = parser.add_subparsers()

    list_p = subparsers.add_parser('list')
    list_p.add_argument(
        "name",
        nargs='?',
        default=None,
        help='Name of the component.')
    list_p.add_argument(
        "attribute",
        nargs='?',
        default=None,
        help='Name of the component attribute.')
    list_p.set_defaults(command='list')

    m = Merv1()

    args = parser.parse_args()
    command = args.command
    if command == 'list':
        m.print_list(name=args.name, attribute=args.attribute)
    else:
        parser.print_help()
        sys.exit(1)

#merv1 = System()
#merv1.configure(
#    minimize=['price', 'gram'],
#    maximize=['amp_hour__sum'],
#)

