package com.team3646.frc2019.paths;

import com.team3646.frc2019.planners.DriveMotionPlanner;
import com.team3646.lib.geometry.Pose2d;
import com.team3646.lib.geometry.Pose2dWithCurvature;
import com.team3646.lib.geometry.Rotation2d;
import com.team3646.lib.geometry.Translation2d;
import com.team3646.lib.trajectory.Trajectory;
import com.team3646.lib.trajectory.TrajectoryUtil;
import com.team3646.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team3646.lib.trajectory.timing.TimedState;
import com.team3646.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 140.0;
    private static final double kMaxAccel = 110.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 110.0;
    private static final double kFirstPathMaxVel = 130.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    public static final Pose2d kSideStartPose = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d
            .fromDegrees(0.0 + 0.0));

    public static final Pose2d kRampPose = new Pose2d(new Translation2d(45.0, 0.0), Rotation2d
            .fromDegrees(0.0 + 0.0));

    //public static final Pose2d kNearRocketPose1 = new Pose2d(new Translation2d(137.0, -92.0), Rotation2d.fromDegrees(-30.0 + 0.0));
    //public static final Pose2d kNearRocketPose2 = new Pose2d(new Translation2d(137.0, -92.0 - 7.0), Rotation2d.fromDegrees(-30.0 - 5.0 + 0.0));
    public static final Pose2d kNearRocketPose1 = new Pose2d(new Translation2d(146.0 + 0.5, -92.0 + -4.0), Rotation2d.fromDegrees(-30.0 + 0.0));
    public static final Pose2d kNearRocketPose2 = new Pose2d(new Translation2d(147.0, -92.0 - 7.0), Rotation2d.fromDegrees(-30.0 - 5.0 + 0.0));

    public static final Pose2d kPreLoadingStationPose1 = new Pose2d(new Translation2d(35.0, -80.0), Rotation2d.fromDegrees(-30.0 + 0.0));
    public static final Pose2d kPreLoadingStationPose2 = new Pose2d(new Translation2d(35.0 + 35.0, -80.0), Rotation2d.fromDegrees(-30.0 + 0.0));

    public static final Pose2d kRevPreLoadingStationPose1 = new Pose2d(new Translation2d(25.0, -80.0), Rotation2d.fromDegrees(180.0 + 10.0 + 0.0));
    public static final Pose2d kRevPreLoadingStationPose2 = new Pose2d(new Translation2d(25.0 + 35.0, -80.0), Rotation2d.fromDegrees(180.0 - 10.0 + 0.0));

    //public static final Pose2d kPreLoadingStationPose = new Pose2d(new Translation2d(-11.0, -100.0), Rotation2d.fromDegrees(180.0 + 0.0));
    public static final Pose2d kLoadingStationPose = new Pose2d(new Translation2d(-44.0, -98.0), Rotation2d.fromDegrees(180.0 - 4.0 + 0.0));
    
    public static final Pose2d kReturnPose = new Pose2d(new Translation2d(106.0, -70.0), Rotation2d.fromDegrees(-30.0 - 5.0 + 0.0));


    // VisionMF Poses
    public static final Pose2d kMFPreLoadingStationPose1 = new Pose2d(new Translation2d(40.0, -90.0), Rotation2d.fromDegrees(-30.0 + 0.0));
    public static final Pose2d kMFPreLoadingStationPose2 = new Pose2d(new Translation2d(35.0 + 35.0, -80.0), Rotation2d.fromDegrees(-30.0 + 0.0));

    public static final Pose2d kMFRevPreLoadingStationPose1 = new Pose2d(new Translation2d(30.0, -90.0), Rotation2d.fromDegrees(180.0 + 6.0 + 0.0));
    public static final Pose2d kMFRevPreLoadingStationPose2 = new Pose2d(new Translation2d(25.0 + 35.0, -80.0), Rotation2d.fromDegrees(180.0 - 10.0 + 0.0));

    public static final Pose2d kMFLoadToPrePose1 = new Pose2d(new Translation2d(-25.0, -97.0), Rotation2d.fromDegrees(180.0 - 4.0 + 0.0));
    public static final Pose2d kMFLoadToPrePose2 = new Pose2d(new Translation2d(60.0, -47.0), Rotation2d.fromDegrees(-35.0));

    // Middle Hatch Poses
    public static final Pose2d kMiddleHatchPose = new Pose2d(new Translation2d(140.0, 30.0), Rotation2d.fromDegrees(0.0 + 0.0));
    public static final Pose2d kPreLoadHatchPose1 = new Pose2d(new Translation2d(115.0, -47.0), Rotation2d.fromDegrees(180.0 + 0.0));
    public static final Pose2d kPreLoadHatchPose2 = new Pose2d(new Translation2d(30.0, -78.0), Rotation2d.fromDegrees(180.0 + 3.0));
   
    public static final Pose2d kMiddleCargoLoadToPrePose1 = new Pose2d(new Translation2d(-25.0, -98.0), Rotation2d.fromDegrees(180.0 + 3.0));
    public static final Pose2d kMiddleCargoLoadToPrePose2 = new Pose2d(new Translation2d(55.0, -45.0), Rotation2d.fromDegrees(-32.0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirroredTrajectory sideStartToNearRocket;
        public final MirroredTrajectory nearRocketToPreLoadingStation;
        public final MirroredTrajectory preToLoadingStation;
        public final MirroredTrajectory loadingStationToPreNearRocket;
        public final MirroredTrajectory preNearRocketToNearRocket;
        public final MirroredTrajectory mFNearRocketToPreLoadingStation;
        public final MirroredTrajectory mFLoadingStationToPreNearRocket;
        public final MirroredTrajectory nearRocketToReturn;
        public final MirroredTrajectory sideStartToMiddleCargo;
        public final MirroredTrajectory middleCargoToPreLoad;
        public final MirroredTrajectory preLoadToLoading;
        public final MirroredTrajectory middleCargoLoadingStationToPreNearRocket;


        private TrajectorySet() {
            sideStartToNearRocket = new MirroredTrajectory(getSideStartToNearRocket());
            nearRocketToPreLoadingStation = new MirroredTrajectory(getNearRocketToPreLoadingStation());
            preToLoadingStation = new MirroredTrajectory(getPreToLoadingStation());
            loadingStationToPreNearRocket = new MirroredTrajectory(getLoadingStationToPreNearRocket());
            preNearRocketToNearRocket = new MirroredTrajectory(getPreNearRocketToNearRocket());
            nearRocketToReturn = new MirroredTrajectory(getNearRocketToReturn());

            mFNearRocketToPreLoadingStation = new MirroredTrajectory(getMFNearRocketToPreLoadingStation());
            mFLoadingStationToPreNearRocket = new MirroredTrajectory(getMFLoadingStationToPreNearRocket());

            sideStartToMiddleCargo = new MirroredTrajectory(getSideStartToMiddleCargo());
            middleCargoToPreLoad = new MirroredTrajectory(getMiddleCargoToPreLoad());
            preLoadToLoading = new MirroredTrajectory(getPreToLoad());
            middleCargoLoadingStationToPreNearRocket = new MirroredTrajectory(getMiddleCargoLoadingStationToPreNearRocket());

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kRampPose);
            waypoints.add(kNearRocketPose1);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kFirstPathMaxVel, 60.0, kFirstPathMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearRocketToPreLoadingStation() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kNearRocketPose1);
                waypoints.add(kPreLoadingStationPose1);
                waypoints.add(kRevPreLoadingStationPose1);
                return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                        kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getPreToLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRevPreLoadingStationPose1);
            //waypoints.add(kPreLoadingStationPose);
            waypoints.add(kLoadingStationPose);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    55.0, 55.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToPreNearRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(kRevPreLoadingStationPose2);
            waypoints.add(kPreLoadingStationPose2);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, 70.0, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getPreNearRocketToNearRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kPreLoadingStationPose2);
            waypoints.add(kNearRocketPose2);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearRocketToReturn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearRocketPose2);
            waypoints.add(kReturnPose);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)),
                    kMaxVelocity, 40.0, kMaxVoltage);
        }

        // Vision MotherFucker

        private Trajectory<TimedState<Pose2dWithCurvature>> getMFNearRocketToPreLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kNearRocketPose1);
            waypoints.add(kMFPreLoadingStationPose1);
            waypoints.add(kMFRevPreLoadingStationPose1);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMFLoadingStationToPreNearRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(kMFLoadToPrePose1);
            waypoints.add(kMFLoadToPrePose2);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(90.0)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // Middle Cargo
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToMiddleCargo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSideStartPose);
            waypoints.add(kRampPose);
            waypoints.add(kMiddleHatchPose);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                110.0, 60.0, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleCargoToPreLoad() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kMiddleHatchPose);
            waypoints.add(kPreLoadHatchPose1);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPreToLoad() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kPreLoadHatchPose1);
            waypoints.add(kPreLoadHatchPose2);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleCargoLoadingStationToPreNearRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(kMiddleCargoLoadToPrePose1);
            waypoints.add(kMiddleCargoLoadToPrePose2);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(90.0)),
                kMaxVelocity, 90.0, kMaxVoltage);
        }
    }
}
