package com.team3646.frc2019;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

     /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 27.16;
    public static final double kDriveWheelDiameterInches = 4;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 68;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.09;  // V
    public static final double kDriveKv = 0.133;  // V per rad/s
    public static final double kDriveKa = 0.038;  // V per rad/s^2

    // Geometry
    public static final double kCenterToFrontBumperDistance = 37.20 / 2.0;
    public static final double kCenterToRearBumperDistance = 37.20 / 2.0;
    public static final double kCenterToSideBumperDistance = 35.43 / 2.0;

    /* CONTROL LOOP GAINS */ 

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final int kAnalogPressureSensorPort = 0;

    /* SUBSYSTEMS */

    // Drive
    public static final int kLeftDriveMasterId = 0;
    public static final int kLeftDriveSlaveAId = 1;
    public static final int kLeftDriveSlaveBId = 2;

    public static final int kRightDriveMasterId = 3;
    public static final int kRightDriveSlaveAId = 4;
    public static final int kRightDriveSlaveBId = 5;

    // Cargo Intake
    public static final int kWristSlaveId = 6;
    public static final int kWristMasterId = 7;
    public static final int kIntakeRollerId = 13;

    // Elevator
    public static final int kElevatorLeftSlaveAId = 8;
    public static final int kElevatorLeftSlaveBId = 9;
    public static final int kElevatorMasterId = 10;
    public static final int kElevatorRightSlaveId = 11;

    // Hatch

    // Climb
    public static final int kClimbWheelId = 12;

    //Control Board
    public static final int kFirstDriveGamepadPort = 0;
    public static final int kSecondDriverGamepadPort = 1;
    

    /* ANSI COLORS FOR CONSOLE LOGS & WARNINGS */

    public class Colors {
        public static final String RESET = "\u001B[0m";
        public static final String BLACK = "\u001B[30m";
        public static final String RED = "\u001B[91m";
        public static final String GREEN = "\u001B[32m";
        public static final String YELLOW = "\u001B[33m";
        public static final String BLUE = "\u001B[34m";
        public static final String PURPLE = "\u001B[35m";
        public static final String CYAN = "\u001B[36m";
        public static final String WHITE = "\u001B[37m";
    }

}