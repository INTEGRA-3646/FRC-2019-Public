package com.team3646.frc2019.auto.actions;

import com.team3646.frc2019.subsystems.Intake;
import com.team3646.frc2019.subsystems.Elevator;
import com.team3646.frc2019.subsystems.Hatch;
import com.team3646.frc2019.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class ExtendAction implements Action {
    private static final Hatch mHatch = Hatch.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private static final double kTimeout = 4.0;
    private double mStartTime;

    public ExtendAction() {
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.extendHatchSystem();
        mSuperstructure.setIntakeAngle(5.0);
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
        if (mHatch.getExtenderState() == Hatch.ExtenderState.EXTEND) {
            return true;       
        } else {
            return false;
        }
    }

    @Override
    public void done() {
    }
}