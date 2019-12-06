package com.team3646.frc2019.auto.actions;

import com.team3646.frc2019.subsystems.Intake;
import com.team3646.frc2019.subsystems.Elevator;
import com.team3646.frc2019.subsystems.Hatch;
import com.team3646.frc2019.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class CollectCargo implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();

    private static final double kTimeout = 2;
    private double mStartTime;

    public CollectCargo() {
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.goCollectCargo();
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
        return false;
    }

    @Override
    public void done() {
    }
}