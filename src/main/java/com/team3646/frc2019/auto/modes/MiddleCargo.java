package com.team3646.frc2019.auto.modes;

import com.team3646.frc2019.auto.AutoConstants;
import com.team3646.frc2019.auto.AutoModeBase;
import com.team3646.frc2019.auto.AutoModeEndedException;
import com.team3646.frc2019.auto.actions.*;
import com.team3646.frc2019.paths.TrajectoryGenerator;
import com.team3646.lib.geometry.Translation2d;

import java.util.Arrays;

public class MiddleCargo extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;

    private DriveTrajectory mSideStartToMiddleCargo;
    private DriveTrajectory mMiddleCargoToPreLoad;
    private DriveTrajectory mPreLoadToLoading;
    private DriveTrajectory mMiddleCargoLoadingStationToPreNearRocket;


    public MiddleCargo(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        mSideStartToMiddleCargo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToMiddleCargo.get(mStartedLeft), true);
        mMiddleCargoToPreLoad = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().middleCargoToPreLoad.get(mStartedLeft));
        mPreLoadToLoading = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().preLoadToLoading.get(mStartedLeft));
        mMiddleCargoLoadingStationToPreNearRocket  = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().middleCargoLoadingStationToPreNearRocket.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
          
        // Score hatch
        runAction(new ParallelAction(
                Arrays.asList(
                        mSideStartToMiddleCargo,
                        new SeriesAction(
                                Arrays.asList(
                                        //new WaitUntilCrossXBoundaryCommand(40.0),
                                        new WaitAction(1.0),
                                        new SetElevatorForHatch(1, true),
                                        new ExtendAction()
                                )
                        )
                )
        ));

        runAction(new ReleaseHatch());

        runAction(new WaitAction(0.05));
        
        runAction(mMiddleCargoToPreLoad);

        runAction(mPreLoadToLoading);

        runAction(new WaitAction(0.05));

        //runAction(new VisionMF(-42.0, 3.5, true, 0.31, 0.0, 0.0));
        //runAction(new VisionMF(-47.0, 3.5, true, 0.31, 0.0, 0.0));
        //runAction(new VisionMF(-48.0, 3.5, true, 0.4, 0.0, 0.0));
        runAction(new VisionMF(-46.0, 3.0, true, 0.33, 0.0, 0.0));

        runAction(new CollectHatch());

        runAction(new ParallelAction(
                Arrays.asList(
                        mMiddleCargoLoadingStationToPreNearRocket,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.5),
                                        new SetElevatorForHatch(2, true),
                                        new ExtendAction()
                                )
                        )
                )
        ));

        //runAction(new VisionMF(130.0, 2.75, false, 0.25, 0.65, 85.0));
        //runAction(new VisionMF(135.0, 2.75, false, 0.25, 0.65, 85.0));
        runAction(new VisionMF(134.0, 2.75, false, 0.25, 0.65, 85.0));


        runAction(new ReleaseHatch());
        
        runAction(new OpenLoopDrive(-0.35, -0.35, 0.5));

    }
}