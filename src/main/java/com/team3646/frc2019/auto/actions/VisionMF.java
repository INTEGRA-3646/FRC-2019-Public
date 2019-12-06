package com.team3646.frc2019.auto.actions;

import com.team3646.frc2019.RobotState;
import com.team3646.frc2019.subsystems.Drive;
import com.team3646.frc2019.subsystems.IntegratedVision;
import com.team3646.lib.util.DriveHelper;
import com.team3646.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class VisionMF implements Action {

    private DriveHelper mDriveHelper = new DriveHelper();

    private static final Drive mDrive = Drive.getInstance();
    private static final IntegratedVision mIntegratedVision = IntegratedVision.getInstance();

    private double mXBoundary = 0;
    private double mStartTime;
    private final double mDuration, mSlowBoundary;
    private final boolean mIsQuickie;
    private double mSpeed, mFirstSpeed, mTemp;

    public VisionMF(double endPoint, double duration, boolean isQuickie, double speed, double firstSpeed, double slowBoundary) {
        mXBoundary = endPoint;
        mDuration = duration;
        mIsQuickie = isQuickie;
        mSpeed = speed;
        mFirstSpeed = firstSpeed;
        mSlowBoundary = slowBoundary;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x()) > Math.abs(mXBoundary) 
                || Timer.getFPGATimestamp() - mStartTime >= mDuration;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntegratedVision.turnLedOn();
        
        if(mFirstSpeed != 0) {
            mTemp = mSpeed;
            mSpeed = mFirstSpeed;
        }
    }

    @Override
    public void update() {
        mIntegratedVision.updateLimelightData();
        if(mFirstSpeed != 0 && RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x() > mSlowBoundary) {
            mSpeed = mTemp;
        } 

        if (mIntegratedVision.getTa() > 0) {
            if(mIsQuickie) {
                mDrive.setOpenLoop(new DriveSignal(mSpeed + mIntegratedVision.turnToTarget() * 0.25, mSpeed - mIntegratedVision.turnToTarget() * 0.25));
            } else {
                mDrive.setOpenLoop(mDriveHelper.curvatureDrive(mSpeed, mIntegratedVision.turnToTarget(), false, false));
            }
        } else {
            mDrive.setOpenLoop(new DriveSignal(mSpeed, mSpeed));
        }

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(mDriveHelper.curvatureDrive(0.0, 0.0, false, false));
        mIntegratedVision.turnLedOff();
    }
}
