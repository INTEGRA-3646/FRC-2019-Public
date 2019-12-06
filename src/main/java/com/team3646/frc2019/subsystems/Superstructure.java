
package com.team3646.frc2019.subsystems;

import com.team3646.frc2019.subsystems.Hatch.ExtenderState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    private static Superstructure mInstance = null;
    private static Elevator mElevator = Elevator.getInstance();
    private static Intake mIntake = Intake.getInstance();
    private static Hatch mHatch = Hatch.getInstance();
    private SuperstructureState mSuperstructureState = SuperstructureState.IDLE, mPreviousSuperstructureState = SuperstructureState.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private boolean isSafetyAcquired = false;


    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            return mInstance = new Superstructure();
        }
        return mInstance;
    }

    protected enum SuperstructureState {
        BASE_POSITION,
        HATCH_LOADING,
        HATCH_PLACING,
        CARGO_PLACING,
        CARGO_COLLECTING,
        CARGO_LOADING,
        IDLE,
    }

    private Superstructure() {
    }

    public void extendHatchSystem() {
        mHatch.extendSubsystem();
    }

    public void collectHatchSystem() {
        mHatch.collectSubsystem();
    }

    public void gripHatch() {
        mHatch.grip();
    }

    public void releaseHatch() {
        mHatch.release();
    }

    public void setIntakeAngle(double angle) {
        if (mHatch.getExtenderState() != ExtenderState.EXTEND) {
            mIntake.setAngle(angle);
        }
    }

    public void goHatchLoadingPosition() {
        isSafetyAcquired = false;
        mSuperstructureState = SuperstructureState.HATCH_LOADING;
    }

    public void goCargoLoadingPosition() {
        isSafetyAcquired = false;
        mSuperstructureState = SuperstructureState.CARGO_LOADING;
    }

    public void goCollectCargo() {
        isSafetyAcquired = false;
        mSuperstructureState = SuperstructureState.CARGO_COLLECTING;
    }

    public void goHatchPlacementPosition(int level) {
        isSafetyAcquired = false;
        mSuperstructureState = SuperstructureState.HATCH_PLACING;
        mPeriodicIO.active_hatch_level = level;
    }

    public void goCargoPlacementPosition(int level) {
        isSafetyAcquired = false;
        mSuperstructureState = SuperstructureState.CARGO_PLACING;
        mPeriodicIO.active_cargo_level = level;
    }

    public void goCargoShipPosition() {
        isSafetyAcquired = false;
        mSuperstructureState = SuperstructureState.CARGO_PLACING;
        mPeriodicIO.active_cargo_level = -1;
    }

    public void setRollerSuck() {
        mIntake.setRollerReverse(-0.95);
    }

    public void setRollerRelease() {
        mIntake.setRollerForward(1.0);
    }

    public void stopRoller() {
        mIntake.setRollerIdle();
    }

    public void stallRoller() {
        mIntake.setRollerStall();
    }

    public void safeCargoPosition() {
        if (!isSafeCargoPosition()) {
            mHatch.collectSubsystem();
            mHatch.grip();
            mIntake.setAngle(15);
        }
    }

    public void climbMode() {
        mElevator.goToPosition(30);
        mIntake.setAngle(115);
    }

    public void safeHatchPosition() {
        if (!isSafeHatchPosition()) {
            mIntake.setAngle(8);
            mHatch.extendSubsystem();
            mHatch.grip();
        }
    }

    public boolean isSafeCargoPosition() {
        if (mIntake.getAngle() > 12.5 && mIntake.getAngle() < 30 && mHatch.getExtenderState() == Hatch.ExtenderState.COLLECT
            && mHatch.getGripperState() == Hatch.GripperState.GRIP) {
                isSafetyAcquired = true;
                return true;
        } 
        return false;
    }

    public boolean isSafeHatchPosition() {
        if (mIntake.getAngle() > 6 && mIntake.getAngle() < 25 && mHatch.getExtenderState() == Hatch.ExtenderState.EXTEND) {
            if (mSuperstructureState == SuperstructureState.HATCH_LOADING) {
                isSafetyAcquired = true;
                return true;
            } else if (mHatch.getGripperState() == Hatch.GripperState.GRIP) {
                isSafetyAcquired = true;
                return true;
            }
        }
        return false;
    }

    public void goBasePosition() {
        mSuperstructureState = SuperstructureState.BASE_POSITION;
    }

    public void setIdle() {
        mSuperstructureState = SuperstructureState.IDLE;
    }

    public void setElevatorPosition(double position) {
        mElevator.goToPosition(position);
    }

    public void preClimb() {
        mElevator.goToPosition(25);
        mIntake.setAngle(150);
    }

    public void onClimb(double timeout) {
        mElevator.goToPosition(Timer.getFPGATimestamp() - timeout < 6.12 ? 30 - (Timer.getFPGATimestamp() - timeout) * 4.9 : 0);
    }

    public void postClimb() {
        mIntake.setAngle(10);
    }

    public void setIntakeManualControl(double power) {
        mIntake.set(power);
    }

    public void goEmergencyHomePos() {
        mSuperstructureState = SuperstructureState.BASE_POSITION;
        isSafetyAcquired = true;
    }
    
    @Override
    public void writePeriodicOutputs() {

        if (!isSafetyAcquired) {
            // Hatch safety check
            if (mSuperstructureState == SuperstructureState.HATCH_LOADING || mSuperstructureState == SuperstructureState.HATCH_PLACING) {
                if (!isSafeHatchPosition()) {
                    safeHatchPosition();
                    return;
                }
            }

            // Cargo safety check
            else if (mSuperstructureState == SuperstructureState.CARGO_COLLECTING || mSuperstructureState == SuperstructureState.CARGO_PLACING || mSuperstructureState == SuperstructureState.CARGO_LOADING || mSuperstructureState == SuperstructureState.BASE_POSITION) {
                if (!isSafeCargoPosition()) {
                    safeCargoPosition();
                    return;
                }
            }
        }
        
        switch (mSuperstructureState) {
            case BASE_POSITION:
                mElevator.goHome();
                mIntake.setAngle(3.0);
                mHatch.collectSubsystem();
                mHatch.grip();
                break;
            case HATCH_LOADING:
                mIntake.setAngle(5.0);
                extendHatchSystem();
                releaseHatch();
                mElevator.goHatchLevel(1);
                break;
            case HATCH_PLACING:
                mElevator.goHatchLevel(mPeriodicIO.active_hatch_level);
                mIntake.setAngle(5.0);
                mHatch.extendSubsystem();
                break;
            case CARGO_COLLECTING:
                mElevator.goHome();
                mIntake.setAngle(79.0);
                break;
            case CARGO_LOADING:
                mElevator.goToPosition(23.7);
                mIntake.setAngle(2);
                break;
            case CARGO_PLACING:
                mElevator.goCargoLevel(mPeriodicIO.active_cargo_level);
                mIntake.setAngle(7.0);
                break;
            case IDLE:
                break;
            default:
                break;
        }
        mPreviousSuperstructureState = mSuperstructureState;
        mSuperstructureState = SuperstructureState.IDLE;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Superstructure State", mSuperstructureState.toString());

    }

    @Override
    public void stop() {
        mElevator.stop();
        mIntake.stop();
        mHatch.stop();
    }

    public class PeriodicIO {

        // Outputs
        public int active_hatch_level;
        public int active_cargo_level;
        public int current_speed;
    }
}