package com.team3646.frc2019.auto.modes;

import com.team3646.frc2019.auto.AutoConstants;
import com.team3646.frc2019.auto.AutoModeBase;
import com.team3646.frc2019.auto.AutoModeEndedException;
import com.team3646.frc2019.auto.actions.*;
import com.team3646.frc2019.paths.TrajectoryGenerator;
import com.team3646.lib.geometry.Translation2d;

import java.util.Arrays;

public class NearRocketHatchesMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;

    private DriveTrajectory mSideStartToNearRocket;
    private DriveTrajectory mNearRocketToPreLoadingStation;
    private DriveTrajectory mPreToLoadingStation;
    private DriveTrajectory mLoadingStationToPreNearRocket;
    private DriveTrajectory mPreNearRocketToNearRocket;
    private DriveTrajectory mNearRocketToReturn;

    public NearRocketHatchesMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToNearRocket = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearRocket.get(mStartedLeft), true);
        mNearRocketToPreLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearRocketToPreLoadingStation.get(mStartedLeft));
        mPreToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().preToLoadingStation.get(mStartedLeft));
        mLoadingStationToPreNearRocket = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToPreNearRocket.get(mStartedLeft));
        mPreNearRocketToNearRocket = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().preNearRocketToNearRocket.get(mStartedLeft));
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

        runAction(mNearRocketToPreLoadingStation);

        runAction(mPreToLoadingStation);

        runAction(new CollectHatch());

        runAction(mLoadingStationToPreNearRocket);

        runAction(new ParallelAction(
                Arrays.asList(
                        mPreNearRocketToNearRocket,
                        new SeriesAction(
                                Arrays.asList(
                                        new SetElevatorForHatch(2, true),
                                        new ExtendAction()
                                )
                        )
                )
        ));

        runAction(new ReleaseHatch());

        runAction(mNearRocketToReturn);
    }
}