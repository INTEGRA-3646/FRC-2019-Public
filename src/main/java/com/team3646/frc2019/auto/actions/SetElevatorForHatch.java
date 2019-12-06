package com.team3646.frc2019.auto.actions;

import com.team3646.frc2019.subsystems.Intake;
import com.team3646.frc2019.subsystems.Elevator;
import com.team3646.frc2019.subsystems.Hatch;
import com.team3646.frc2019.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import com.team3646.lib.util.Util;

public class SetElevatorForHatch implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final Hatch mHatch = Hatch.getInstance();
    private static final Elevator mElevator = Elevator.getInstance();
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private static final double kTimeout = 1;
    private static final double kHeightEpsilon = 2.0;
    private double mStartTime;
    private final int mStage;
    private final boolean mWaitForCompletion;
    private double height = 0.0;

    public SetElevatorForHatch(int stage, boolean waitForCompletion) {
        mStage = stage;
        mWaitForCompletion = waitForCompletion;
    }

    @Override   
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        if(mStage == 0) {
            mSuperstructure.goHatchLoadingPosition();
        } else {
            mSuperstructure.goHatchPlacementPosition(mStage); 
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - mStartTime > kTimeout) {
            System.out.println("Set Elevator timed out!!!");
            return true;
        }
        if (mWaitForCompletion) {
            double mHeight = mElevator.getHeightInches();
            return Util.epsilonEquals(mHeight, getHeight(mStage), kHeightEpsilon);
        } else {
            return true;
        }
    }

    @Override
    public void done() {
    }

    private double getHeight(int level) {
        switch (level) {
            case 1:
                height = 9.5;
                break;
            case 2:
                height = 41.0;
                break;
            case 3:
                height = 70.0;
                break;
            default:
                break;
        }
        return height;
    }

}