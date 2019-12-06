package com.team3646.frc2019.auto.modes;

import com.team3646.frc2019.auto.AutoConstants;
import com.team3646.frc2019.auto.AutoModeBase;
import com.team3646.frc2019.auto.AutoModeEndedException;
import com.team3646.frc2019.auto.actions.*;
import com.team3646.frc2019.paths.TrajectoryGenerator;
import com.team3646.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearRocketMFMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;

    private DriveTrajectory mSideStartToNearRocket;
    private DriveTrajectory mMFNearRocketToPreLoadingStation;
    private DriveTrajectory mMFLoadingStationToPreNearRocket;
    private DriveTrajectory mMFPreNearRocketToNearRocket;
    private DriveTrajectory mNearRocketToReturn;

    public NearRocketMFMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToNearRocket = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearRocket.get(mStartedLeft), true);
        mMFNearRocketToPreLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().mFNearRocketToPreLoadingStation.get(mStartedLeft));
        mMFLoadingStationToPreNearRocket = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().mFLoadingStationToPreNearRocket.get(mStartedLeft));
        mNearRocketToReturn = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearRocketToReturn.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        
        // Score hatch
        runAction(new ParallelAction(
                Arrays.asList(
                        mSideStartToNearRocket,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilCrossXBoundaryCommand(40.0),
                                        //new WaitAction(1.5),
                                        new SetElevatorForHatch(1, true),
                                        new ExtendAction()
                                )
                        )
                )
        ));
        
        runAction(new ReleaseHatch());

        runAction(mMFNearRocketToPreLoadingStation);

        runAction(new WaitAction(0.1));

        runAction(new VisionMF(-46.0, 2.6, true, 0.3, 0.0, 0.0));

        runAction(new CollectHatch());
        
        runAction(new ParallelAction(
                Arrays.asList(
                        mMFLoadingStationToPreNearRocket,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.5),
                                        new SetElevatorForHatch(2, true),
                                        new ExtendAction()
                                )
                        )
                )
        ));

        //runAction(new VisionMF(130.0, 2.75, false, 0.27, 0.57, 90.0));
        //runAction(new VisionMF(138.0, 2.75, false, 0.27, 0.57, 90.0));
        runAction(new VisionMF(136.0, 2.6, false, 0.25, 0.6, 92.5));

        runAction(new ReleaseHatch());

        runAction(new WaitAction(0.1));

        runAction(new OpenLoopDrive(-0.35, -0.35, 0.5));
        runAction(new OpenLoopDrive(-0.2, -0.8, 0.5));

        runAction(new SetElevatorForHatch(0, true)); // Hatch loading pos.
    }
}