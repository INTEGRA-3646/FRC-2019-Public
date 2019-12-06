package com.team3646.frc2019;

import com.team3646.frc2019.auto.AutoModeBase;
import com.team3646.frc2019.auto.AutoModeExecutor;
import com.team3646.frc2019.paths.TrajectoryGenerator;
import com.team3646.frc2019.auto.modes.*;
import com.team3646.frc2019.loops.Looper;
import com.team3646.frc2019.subsystems.*;
import com.team3646.lib.drivers.AnalogPressureSensor;
import com.team3646.lib.geometry.Pose2d;
import com.team3646.lib.util.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.team3646.frc2019.ControlBoard;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends IterativeRobot {

    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private DriveHelper mDriveHelper = new DriveHelper();
    private ControlBoard mControlBoard = ControlBoard.getInstance();

    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(
            RobotStateEstimator.getInstance(), Drive.getInstance(), Elevator.getInstance(), Intake.getInstance(),
            Hatch.getInstance(), Superstructure.getInstance(), Climb.getInstance(), IntegratedVision.getInstance()));

    private static final Drive mDrive = Drive.getInstance();
    private static final Elevator mElevator = Elevator.getInstance();
    private static final Intake mIntake = Intake.getInstance();
    private static final Hatch mHatch = Hatch.getInstance();
    private static final Climb mClimb = Climb.getInstance();
    private static final IntegratedVision mIntegratedVision = IntegratedVision.getInstance();

    private Superstructure mSuperstructure = Superstructure.getInstance();
    private AutoModeExecutor mAutoModeExecutor;
    private SendableChooser autoChooser;

    private boolean autoOn = false;
    private boolean stalled = false;
    private boolean climbMode = false;
    private double climbTimeout;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {

        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            CameraServer.getInstance().startAutomaticCapture();

            autoChooser = new SendableChooser();
            autoChooser.addDefault("Rocket MF Vision Right", 1);
            autoChooser.addOption("Rocket MF Vision Left", 2);
            autoChooser.addDefault("Rocket Pure Motion Right", 3);
            autoChooser.addOption("Rocket Pure Motion Left", 4);
            autoChooser.addOption("Middle Cargo Right", 5);
            autoChooser.addOption("Middle Cargo Left", 6);

            SmartDashboard.putData("Autonomous Chooser", autoChooser);

            mIntegratedVision.turnLedOff();
            mIntegratedVision.visionMode();

            mTrajectoryGenerator.generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            
            mClimb.collectSubsystem();
            mDrive.shiftLow();

            mIntegratedVision.turnLedOff();

            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mDrive.zeroSensors(true);

            mAutoModeExecutor = new AutoModeExecutor();
            mDrive.setBrakeMode(false);

            // mAutoModeExecutor.setAutoMode(new NearRocketHatchesMode(false));   
            
            mDisabledLooper.start();
            outputToSmartDashboard();
            // mSubsystemManager.stop(); // WTF
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mDrive.zeroSensors(true);

            mAutoModeExecutor.start();
            mEnabledLooper.start();

            mSuperstructure.setIntakeAngle(-1);
            mSuperstructure.setElevatorPosition(9);

            outputToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            mEnabledLooper.start();
            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
            mDrive.setBrakeMode(true);

            climbMode = false;

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            mDrive.setBrakeMode(false);
            mIntegratedVision.turnLedOff();

            switch ((int) autoChooser.getSelected()) {
                case 1:
                    mAutoModeExecutor.setAutoMode(new NearRocketMFMode(false)); // False - right     true - left
                    break;
                case 2:
                    mAutoModeExecutor.setAutoMode(new NearRocketMFMode(true)); // False - right     true - left
                    break;
                case 3:
                    mAutoModeExecutor.setAutoMode(new NearRocketHatchesMode(false)); // False - right     true - left
                    break;
                case 4:
                    mAutoModeExecutor.setAutoMode(new NearRocketHatchesMode(true)); // False - right     true - left
                    break;
                case 5:
                    mAutoModeExecutor.setAutoMode(new MiddleCargo(false)); // False - right     true - left
                    break;
                case 6:
                    mAutoModeExecutor.setAutoMode(new MiddleCargo(true)); // False - right     true - left
                    break;
                default:
                    break;
            } 
            outputToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            // if (driveStick.getRawAxis(2) > 0.5 && driveStick.getRawButton(3) && !autoOn)
            // {
            // endAutoMode();
            // autoOn = true;
            // initAutoMode(new NearHatchAuto(false));
            // } else if (driveStick.getRawAxis(3) > 0.5 && driveStick.getRawButton(3) &&
            // !autoOn) {
            // endAutoMode();
            // autoOn = true;
            // initAutoMode(new FarHatchAuto(false));
            // } else if (!autoOn) {

            // /*if(driveStick.getRawAxis(1) > 0 ) {
            // mDrive.setOpenLoop(mDriveHelper.curvatureDrive(-driveStick.getRawAxis(1) *
            // heightConstant,
            // (driveStick.getRawButton(5) ? -driveStick.getRawAxis(4) / 2 :
            // -driveStick.getRawAxis(4)) * heightConstant,
            // driveStick.getRawButton(5), false));
            // } else {
            // mDrive.setOpenLoop(mDriveHelper.curvatureDrive(-driveStick.getRawAxis(1) *
            // heightConstant,
            // (driveStick.getRawButton(5) ? driveStick.getRawAxis(4) / 2 :
            // driveStick.getRawAxis(4)) * heightConstant,
            // driveStick.getRawButton(5), false));
            // }*/

            // mDrive.setOpenLoop(mDriveHelper.curvatureDrive(mControlBoard.getThrottle(),
            // (driveStick.getRawButton(5) ? driveStick.getRawAxis(4) / 2 :
            // driveStick.getRawAxis(4)),
            // driveStick.getRawButton(5), false));

            // if (secondDriveStick.getRawButton(7)) {
            // mSuperstructure.setIntakeManualControl(secondDriveStick.getRawAxis(5));
            // if (secondDriveStick.getRawButton(8)) mIntake.zeroSensors();
            // }

            // if (driveStick.getRawButton(4))
            // mDrive.shiftLow();
            // if (driveStick.getRawButton(1))
            // mDrive.shiftHigh();

            // Second driver placements and positioning
            // if (secondDriveStick.getRawButton(5)) { // Hatch commands
            // if (secondDriveStick.getRawButton(1)) {
            // mSuperstructure.goHatchPlacementPosition(1);
            // } else if (secondDriveStick.getRawButton(2)) {
            // mSuperstructure.goHatchPlacementPosition(2);
            // } else if (secondDriveStick.getRawButton(4)) {
            // mSuperstructure.goHatchPlacementPosition(3);
            // } else if (secondDriveStick.getRawButton(3)) {
            // mSuperstructure.goHatchLoadingPosition();
            // }
            // } else if (secondDriveStick.getRawButton(6)) { // Cargo commands
            // if (secondDriveStick.getRawButton(1)) {
            // mSuperstructure.goCargoPlacementPosition(1);
            // } else if (secondDriveStick.getRawButton(2)) {
            // mSuperstructure.goCargoPlacementPosition(2);
            // } else if (secondDriveStick.getRawButton(4)) {
            // mSuperstructure.goCargoPlacementPosition(3);
            // } else if (secondDriveStick.getRawButton(3)) {
            // mSuperstructure.goCollectCargo();
            // }
            // } else if (secondDriveStick.getRawButton(9)) {
            // mSuperstructure.goBasePosition();
            // } else if (secondDriveStick.getRawButton(10)) {
            // mSuperstructure.goCargoShipPosition();
            // }

            // Hatch Grip/Release
            // if (secondDriveStick.getRawAxis(2) > 0.5) {
            // mSuperstructure.gripHatch();
            // } else if (driveStick.getRawAxis(2) > 0.5) {
            // mSuperstructure.releaseHatch();
            // }

            // ------------------ Intake Roller ------------------
            // if (driveStick.getRawAxis(3) > 0.5) {
            // mSuperstructure.setRollerRelease();
            // stalled = false;
            // } else if (secondDriveStick.getRawAxis(3) > 0.5) {
            // mSuperstructure.setRollerSuck();
            // stalled = true;
            // } else if(stalled){
            // mSuperstructure.stallRoller();
            // } else {
            // mSuperstructure.stopRoller();
            // }
            // ------------------ End Intake Roller ------------------

            // ------------------ Climb ------------------
            // if (driveStick.getRawButton(8)) {
            // mSuperstructure.preClimb();
            // climbMode = true;
            // climbTimeout = Timer.getFPGATimestamp();
            // }

            // if (climbMode && Timer.getFPGATimestamp() - climbTimeout >= 0.5) {
            // mSuperstructure.onClimb(climbTimeout);
            // mClimb.extendSubsystem();
            // mClimb.setWheelsForward(false);
            // }

            // if (climbMode && Timer.getFPGATimestamp() - climbTimeout >= 5.5) {
            // mClimb.setWheelsForward(true);
            // }

            // if (driveStick.getRawButton(7)) {
            // mSuperstructure.postClimb();
            // mClimb.collectSubsystem();
            // climbMode = false;
            // }
            // ------------------ End Climb ------------------

            // }

            // if (driveStick.getRawButton(4)) {
            // endAutoMode();
            // }

            outputToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    @Override
    public void teleopPeriodic() {

        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            // if (mControlBoard.getNearHatch() > 0.5 && mControlBoard.getAutoMode() &&
            // !autoOn) {
            // endAutoMode();
            // autoOn = true;
            // initAutoMode(new NearHatchAuto(false));
            // } else if (mControlBoard.getFarHatch() > 0.5 && mControlBoard.getAutoMode()
            // && !autoOn) {
            // endAutoMode();
            // autoOn = true;
            // initAutoMode(new FarHatchAuto(false));
            /* } else */ if (!autoOn) {

                /*
                 * if(driveStick.getRawAxis(1) > 0 ) {
                 * mDrive.setOpenLoop(mDriveHelper.curvatureDrive(-mControlBoard.getThrottle(),
                 * (mControlBoard.getQuickTurn() ? -mControlBoard.getTurn() / 2 :
                 * -mControlBoard.getTurn()), mControlBoard.getQuickTurn(), false)); } else {
                 * mDrive.setOpenLoop(mDriveHelper.curvatureDrive(-mControlBoard.getThrottle(),
                 * (mControlBoard.getQuickTurn() ? mControlBoard.getTurn() / 2 :
                 * mControlBoard.getTurn()), mControlBoard.getQuickTurn(), false)); }
                 */

                double heightConstant = Math.min(1, (85.0 - mElevator.getHeightInches()) / 50.0);

                mIntegratedVision.updateLimelightData();

                if (mControlBoard.limeTurn()) {
                    mIntegratedVision.turnLedOn();
                    if (/*Math.abs(mIntegratedVision.getTx()) > 1 && */mIntegratedVision.getTa() > 0) {
                        mDrive.setOpenLoop(mDriveHelper.curvatureDrive(Math.abs(mControlBoard.getThrottle()) > 0.1 ? mControlBoard.getThrottle() : 0,
                            mIntegratedVision.turnToTarget() * heightConstant, false, false));
                    } else {
                        mDrive.setOpenLoop(mDriveHelper.curvatureDrive(mControlBoard.getThrottle(),
                        0.0, false, false));
                    }
                } else if (mControlBoard.getAutoAlign()) {
                    //mDrive.autoAlign(heightConstant);
                }
                else {
                    mIntegratedVision.turnLedOff();
                    mIntegratedVision.resetIntegral();
                    mDrive.setOpenLoop(mDriveHelper.curvatureDrive(mControlBoard.getThrottle() * heightConstant,
                        (mControlBoard.getQuickTurn() ? mControlBoard.getTurn() / 2 : mControlBoard.getTurn()) 
                            * heightConstant, mControlBoard.getQuickTurn(), false));
                }

                if (mControlBoard.getManualControls()) {
                    mSuperstructure.setIntakeManualControl(mControlBoard.getIntake());
                    if (mControlBoard.getZeroSensors())
                        mIntake.zeroSensors();
                } else if (mControlBoard.getManualControlsReleased()) {
                    mSuperstructure.setIntakeAngle(mIntake.getAngle());
                }

                if (mControlBoard.getShiftLow())
                    mDrive.shiftLow();
                if (mControlBoard.getShiftHigh())
                    mDrive.shiftHigh();

                // Second driver placements and positioning
                if (mControlBoard.getEmergencyHomePos()) {
                    mSuperstructure.goEmergencyHomePos();
                } else if (mControlBoard.isHatch()) { // Hatch commands
                    if (mControlBoard.goFirstHatchPlacementPosition()) {
                        mSuperstructure.goHatchPlacementPosition(1);
                    } else if (mControlBoard.goSecondHatchPlacementPosition()) {
                        mSuperstructure.goHatchPlacementPosition(2);
                    } else if (mControlBoard.goThirdHatchPlacementPosition()) {
                        mSuperstructure.goHatchPlacementPosition(3);
                    } else if (mControlBoard.goHatchLoadingPosition()) {
                        mSuperstructure.goHatchLoadingPosition();
                    }
                } else if (mControlBoard.isCargo()) { // Cargo commands
                    if (mControlBoard.goFirstCargoPlacementPosition()) {
                        mSuperstructure.goCargoPlacementPosition(1);
                    } else if (mControlBoard.goSecondCargoPlacementPosition()) {
                        mSuperstructure.goCargoPlacementPosition(2);
                    } else if (mControlBoard.goThirdCargoPlacementPosition()) {
                        mSuperstructure.goCargoPlacementPosition(3);
                    } else if (mControlBoard.goCollectCargo()) {
                        mSuperstructure.goCollectCargo();
                    }
                } else if (mControlBoard.goBasePosition()) {
                    mSuperstructure.goBasePosition();
                } else if (mControlBoard.goCargoShipPosition()) {
                    mSuperstructure.goCargoShipPosition();
                } else if (mControlBoard.getCargoLoading()) {
                    mSuperstructure.goCargoLoadingPosition();
                }

                // Hatch Grip/Release
                if (mControlBoard.gripHatch() > 0.5) {
                    mSuperstructure.gripHatch();
                } else if (mControlBoard.getReleaseHatch() > 0.5) {
                    mSuperstructure.releaseHatch();
                }

                // ------------------ Intake Roller ------------------
                if (mControlBoard.getShootBall() > 0.5) {
                    mSuperstructure.setRollerRelease();
                    stalled = false;
                } else if (mControlBoard.setRollerSuck() > 0.5) {
                    mSuperstructure.setRollerSuck();
                    stalled = true;
                } else if (stalled) {
                    mSuperstructure.stallRoller();
                } else {
                    mSuperstructure.stopRoller();
                }

                if (mControlBoard.stopRollers()) {
                    stalled = false;
                    mSuperstructure.stopRoller();
                }

                // ------------------ End Intake Roller ------------------

                // ------------------ Climb ------------------
                if (mControlBoard.preClimb()) {
                    mSuperstructure.preClimb();
                    climbMode = true;
                    climbTimeout = Timer.getFPGATimestamp();
                }

                if (climbMode && Timer.getFPGATimestamp() - climbTimeout >= 0.5) {
                    mSuperstructure.onClimb(climbTimeout);
                    mClimb.extendSubsystem();
                    mClimb.setWheelsForward(false);
                }

                if (climbMode && Timer.getFPGATimestamp() - climbTimeout >= 5.5) {
                    mClimb.setWheelsForward(true);
                }

                if (mControlBoard.postClimb()) {
                    mSuperstructure.postClimb();
                    mClimb.collectSubsystem();
                    mClimb.stopWheels();
                    climbMode = false;
                }
                // ------------------ End Climb ------------------

            }

            if (mControlBoard.endAutoMode()) {
                endAutoMode();
            }

            outputToSmartDashboard();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
    }

    public void outputToSmartDashboard() {

        RobotState.getInstance().outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();

        // SmartDashboard.putNumber("UltBack", leftRear.getDistance());
        // SmartDashboard.updateValues();
    }

    private void initAutoMode(AutoModeBase auto) {
        mAutoModeExecutor.setAutoMode(auto);
        mAutoModeExecutor.start();
    }

    private void endAutoMode() {
        autoOn = false;
        mAutoModeExecutor.stop();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
        mDrive.zeroSensors();
        mDrive.setBrakeMode(true);
    }
}
