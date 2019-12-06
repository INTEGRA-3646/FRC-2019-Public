package com.team3646.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team3646.frc2019.Constants;
import com.team3646.frc2019.loops.Loop;
import com.team3646.lib.drivers.TalonSRXFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    private final double kEncoderTicksPerRotation = 4096;
    private final TalonSRX mWristSlave, mRoller;
    protected final TalonSRX mWristMaster;

    private static Intake mInstance = null;
    private static boolean mHasBeenZeroed = false;
    private IntakeControlState mIntakeControlState = IntakeControlState.OPEN_LOOP;
    private RollerState mRollerState = RollerState.IDLE;
    private double feedForward = 0.0;


    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private enum IntakeControlState {
        MOTION_MAGIC,
        OPEN_LOOP,
        POSITION_PID
    }

    private enum RollerState {
        FORWARD,
        REVERSE,
        STALL,
        IDLE
    }

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    private Intake() {

        mWristMaster = TalonSRXFactory.createDefaultTalon(Constants.kWristMasterId);
        mWristSlave = TalonSRXFactory.createDefaultTalon(Constants.kWristSlaveId);
        mRoller = TalonSRXFactory.createDefaultTalon(Constants.kIntakeRollerId);

        mWristMaster.setInverted(true);
        mWristMaster.setSensorPhase(false);
        zeroSensors();

        mWristSlave.follow(mWristMaster);
        mWristSlave.setInverted(InvertType.OpposeMaster);

        mWristMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder , 0, 20);

        mWristMaster.setNeutralMode(NeutralMode.Brake);
        mWristSlave.setNeutralMode(NeutralMode.Brake);
        mRoller.setNeutralMode(NeutralMode.Brake);


        mWristMaster.configMotionAcceleration(750);  //1000
        mWristMaster.configMotionCruiseVelocity(2000);  //2500
        mWristMaster.config_kP(0, 3.5);
        mWristMaster.config_kI(0, 0);
        mWristMaster.config_kD(0, 45);
        mWristMaster.config_kF(0, 1.05);
    }

    private void configUpConstants() {
        mWristMaster.configMotionAcceleration(900);  //650
        mWristMaster.configMotionCruiseVelocity(1500);  //2000
        mWristMaster.config_kP(0, 3.5);
        mWristMaster.config_kI(0, 0);
        mWristMaster.config_kD(0, 45);
        mWristMaster.config_kF(0, 1.05);
    }

    private void configDownConstants() {
        mWristMaster.configMotionAcceleration(450);
        mWristMaster.configMotionCruiseVelocity(300);
        mWristMaster.config_kP(0, 2.5);
        mWristMaster.config_kI(0, 0);
        mWristMaster.config_kD(0, 20);
        mWristMaster.config_kF(0, 0);
    }

    protected void setAngle(double angle) {
        double desired_position = convertAngleToTicks(angle);
        if (mIntakeControlState != IntakeControlState.MOTION_MAGIC) mIntakeControlState = IntakeControlState.MOTION_MAGIC;
        
        if (desired_position > mWristMaster.getSelectedSensorPosition()) 
        configDownConstants();
        else configUpConstants();

        mPeriodicIO.demand = desired_position;
    }

    protected void stayAtAngle() {
        if (mIntakeControlState != IntakeControlState.POSITION_PID) mIntakeControlState = IntakeControlState.POSITION_PID;
        mPeriodicIO.demand = mPeriodicIO.wrist_angle_ticks;
    }


    protected void set(double percentage) {
        if (mIntakeControlState != IntakeControlState.OPEN_LOOP) mIntakeControlState = IntakeControlState.OPEN_LOOP;
        mPeriodicIO.demand = percentage;
    }

    protected void setRollerForward(double percentage) {
        if (mRollerState != RollerState.FORWARD) mRollerState = RollerState.FORWARD;
        mPeriodicIO.rollerDemand = percentage;
    }

    protected void setRollerReverse(double percentage) {
        if (mRollerState != RollerState.REVERSE) mRollerState = RollerState.REVERSE;
        mPeriodicIO.rollerDemand = percentage;
    }

    protected void setRollerStall() {
        mPeriodicIO.rollerDemand = -0.18;
    }

    protected void setRollerIdle() {
        if (mRollerState != RollerState.IDLE) mRollerState = RollerState.IDLE;
        mPeriodicIO.rollerDemand = 0;
    }

    public double getAngle() {
        return (mWristMaster.getSelectedSensorPosition() * 360.0) / kEncoderTicksPerRotation;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake Angle", getAngle());
        //SmartDashboard.putNumber("intake ticks", mWristMaster.getSelectedSensorPosition());
        //SmartDashboard.putNumber("intake_target_pos", mWristMaster.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Intake Power", mWristMaster.getMotorOutputPercent());
    }

    @Override
    public void stop() {
        mIntakeControlState = IntakeControlState.OPEN_LOOP;
        mPeriodicIO.demand = 0.0;
    }

    @Override
    public synchronized void zeroSensors() {
        mWristMaster.setSelectedSensorPosition(0, 0, 10);
        mHasBeenZeroed = true;
    }

    public synchronized void readPeriodicInputs() {}

    public synchronized void writePeriodicOutputs() {
        switch (mIntakeControlState) {
            case MOTION_MAGIC:
                mWristMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, feedForward); // SET FF
                break;
            case POSITION_PID:
                mWristMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.0); // SET FF
                break;
            case OPEN_LOOP:
                mWristMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.0); // SET FF
                break;
            default:
                break;
        }
    
        mRoller.set(ControlMode.PercentOutput, mPeriodicIO.rollerDemand);
    }

    private double convertAngleToTicks(double angle) {
        return (angle * kEncoderTicksPerRotation) / 360.0;
    }

    public static class PeriodicIO {
        // Inputs
        public double wrist_angle_ticks;
        
        // Outputs
        public double demand;
        public double rollerDemand;
    }

}