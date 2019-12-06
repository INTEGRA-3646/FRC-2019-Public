package com.team3646.frc2019.auto.actions;

import com.team3646.frc2019.subsystems.Intake;
import com.team3646.frc2019.subsystems.Hatch;
import com.team3646.frc2019.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class ReleaseHatch implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final Hatch mHatch = Hatch.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private static final double kTimeout = 4.0;
    private double mStartTime;

    public ReleaseHatch() {
    }
    
    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.releaseHatch();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - mStartTime > kTimeout) {
            System.out.println("Set Superstructure Position timed out!!!");
            return true;
        }
        if (mHatch.getGripperState() == Hatch.GripperState.RELEASE) {
            return true;       
        } else {
            return false;
        }
    }

    @Override
    public void done() {
    }
}