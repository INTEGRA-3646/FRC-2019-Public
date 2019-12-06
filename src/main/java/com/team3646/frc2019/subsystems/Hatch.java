package com.team3646.frc2019.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hatch extends Subsystem {

    private final Solenoid mGripper, mExtender;
    
    private static Hatch mInstance = null;
    private GripperState mGripperState = GripperState.GRIP;
    private ExtenderState mExtenderState = ExtenderState.COLLECT;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public enum GripperState {
        GRIP,
        RELEASE
    }

    public enum ExtenderState {
        EXTEND,
        COLLECT
    }

    public GripperState getGripperState() {
        return mGripperState;
    }

    public ExtenderState getExtenderState() {
        return mExtenderState;
    }

    public synchronized static Hatch getInstance() {
        if (mInstance == null) {
            mInstance = new Hatch();
        }
        return mInstance;
    }

    private Hatch() {
        mGripper = new Solenoid(1);
        mExtender = new Solenoid(0);

        mGripper.set(false);
        mExtender.set(true);
    }

    public void grip() {
        if (mGripperState != GripperState.GRIP)
            mGripperState = GripperState.GRIP;
        mPeriodicIO.gripperState = false;
    }

    public void release() {
        if (mGripperState != GripperState.RELEASE)
            mGripperState = GripperState.RELEASE;
        mPeriodicIO.gripperState = true;
    }

    protected void extendSubsystem() {
        if (mExtenderState != ExtenderState.EXTEND)
            mExtenderState = ExtenderState.EXTEND;
        mPeriodicIO.extenderState = true;
    }

    protected void collectSubsystem() {
        if (mExtenderState != ExtenderState.COLLECT)
            mExtenderState = ExtenderState.COLLECT;
        mPeriodicIO.extenderState = false;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Hatch Extender State", mExtenderState.toString());
        SmartDashboard.putString("Hatch Gripper State", mGripperState.toString());
    }

    @Override
    public void stop() {

    }

    public synchronized void readPeriodicInputs() {}

    public synchronized void writePeriodicOutputs() {
        switch (mGripperState) {
            case GRIP:
                mGripper.set(mPeriodicIO.gripperState);
                break;
            case RELEASE:
                mGripper.set(mPeriodicIO.gripperState);
                break;
            default:
                break;
        }
        switch (mExtenderState) {
            case EXTEND:
                mExtender.set(mPeriodicIO.extenderState);
                break;
            case COLLECT:
                mExtender.set(mPeriodicIO.extenderState);
                break;
            default:
                break;
        }
    }
    
    public static class PeriodicIO {

        // Outputs
        public boolean extenderState;
        public boolean gripperState;
    }

}