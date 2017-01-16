cmake_minimum_required(VERSION 2.8.3)
project(ros_homebot_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES
    ArduinoTemperatureChange.msg
    BatteryChargeRatioChange.msg
    BatteryTemperatureChange.msg
    BatteryVoltageChange.msg
    BumperChange.msg
    CPUUsage.msg
    DiskUsage.msg
    EdgeChange.msg
    ExternalPowerChange.msg
    ImuAccelerometerChange.msg
    ImuCalibrationChange.msg
    ImuEulerChange.msg
    ImuGyroscopeChange.msg
    ImuMagnetometerChange.msg
    LaserLineColumns.msg
    MemoryUsage.msg
    MicrophoneChange.msg
    MotorCalibrationChange.msg
    MotorEncoderChange.msg
    MotorErrorChange.msg
    PacketRead.msg
    PanAngleChange.msg
    PanCentermarkChange.msg
    PanFullRevCountChange.msg
    Pong.msg
    RemainingTime.msg
    StatusButtonChange.msg
    TiltAngleChange.msg
    TwistDoneChange.msg
    UltrasonicChange.msg
)

add_service_files(FILES
    AllStop.srv
    Calibrate.srv
    ForceSensors.srv
    GetValue.srv
    GoToCenter.srv
    Led.srv
    LedAuto.srv
    MicrophoneEnable.srv
    MotorAcceleration.srv
    MotorSpeed.srv
    PacketWrite.srv
    PanAngle.srv
    PanPower.srv
    PanSpeed.srv
    RechargePowerdown.srv
    Shutdown.srv
    Sleep.srv
    SonarPower.srv
    TTS.srv
    TiltAngle.srv
    TiltPower.srv
    Tone.srv
    Twist.srv
)

generate_messages(   
	DEPENDENCIES  
	std_msgs  
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
