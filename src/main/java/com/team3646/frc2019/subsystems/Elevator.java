package com.team3646.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team3646.lib.drivers.TalonSRXFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team3646.frc2019.Constants;

public class Elevator extends Subsystem {

    private final double kHighLimit = 30.0;
    private final double kLowLimit = 1.0;
    private final double kEncoderTicksPerInch = 600.0;
    private final TalonSRX mMaster, mRightSlave, mLeftSlaveA, mLeftSlaveB;

    private static Elevator mInstance = null; 
    private static boolean isGoingHome = false;
    private ElevatorControlState mElevatorControlState = ElevatorControlState.OPEN_LOOP;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private enum ElevatorControlState {
        MOTION_MAGIC,
        OPEN_LOOP,
        POSITION_PID,
        CLIMBING
    }

    private enum ElevatorState {
        GO_TO_POSITION,
        HOLD_POSITION,
        IDLE
    }

    public static Elevator getInstance() {
        if (mInstance == null)
            return mInstance = new Elevator();
        return mInstance;
    }

    private Elevator() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);
        mRightSlave = TalonSRXFactory.createDefaultTalon(Constants.kElevatorRightSlaveId);
        mLeftSlaveA = TalonSRXFactory.createDefaultTalon(Constants.kElevatorLeftSlaveAId);
        mLeftSlaveB = TalonSRXFactory.createDefaultTalon(Constants.kElevatorLeftSlaveBId);

        mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        zeroSensors();

        mMaster.config_kP(0, 0.15);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 4);
        mMaster.config_kF(0, 0.06);

        mMaster.configMotionAcceleration(7000);
        mMaster.configMotionCruiseVelocity(20000);

        mRightSlave.follow(mMaster);
        mLeftSlaveA.follow(mMaster);
        mLeftSlaveB.follow(mMaster);

        mMaster.setNeutralMode(NeutralMode.Brake);
        mRightSlave.setNeutralMode(NeutralMode.Brake);
        mLeftSlaveA.setNeutralMode(NeutralMode.Brake);
        mLeftSlaveB.setNeutralMode(NeutralMode.Brake);
        
        mMaster.setInverted(InvertType.None);
        mRightSlave.setInverted(InvertType.FollowMaster);
        mLeftSlaveA.setInverted(InvertType.OpposeMaster);
        mLeftSlaveB.setInverted(InvertType.OpposeMaster);
    }

    protected void goHatchLevel(int level) {
        switch (level) {
            case 1:
                goToPosition(9.5);
                break;
            case 2:
                goToPosition(41.0);
                break;
            case 3:
                goToPosition(70.0);
                break;
            default:
                break;
        }
    }

    protected void goCargoLevel(int level) {
        switch (level) {
            case 1:
                goToPosition(6.8);       
                break;
            case 2:
                goToPosition(38.0);
                break;
            case 3:
                goToPosition(67.0);
                break;
            case -1:
                goCargoShip();
                break;
            default:
                break;
        }
    }

    protected void goCargoShip() {
        goToPosition(23.0);
    }

    private void configUpConstants() {
        mMaster.config_kP(0, 0.2);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 4);
        mMaster.config_kF(0, 0.07);

        mMaster.configMotionAcceleration(7000);
        mMaster.configMotionCruiseVelocity(20000);
    }

    private void configDownConstants() {
        mMaster.config_kP(0, 0.15);
        mMaster.config_kI(0, 0);
        mMaster.config_kD(0, 4);
        mMaster.config_kF(0, 0.06);

        mMaster.configMotionAcceleration(5000);
        mMaster.configMotionCruiseVelocity(8000);
    }

    protected void goToPosition(double distanceInInches) {
        double desiredPosition = kEncoderTicksPerInch * distanceInInches;
        if (mElevatorControlState != ElevatorControlState.MOTION_MAGIC) {
            mElevatorControlState = ElevatorControlState.MOTION_MAGIC;
        }

        if (desiredPosition < mMaster.getSelectedSensorPosition() && mMaster.getSelectedSensorPosition() > 22000) {
            configDownConstants();
        } else {
            configUpConstants();
        }

        mPeriodicIO.demand = desiredPosition;
    }

    public double getHeightInches() {
         return mMaster.getSelectedSensorPosition() / (double) kEncoderTicksPerInch;
    }

    protected void stayAtPosition() {
        if (mElevatorControlState != ElevatorControlState.POSITION_PID) {
            mElevatorControlState = ElevatorControlState.POSITION_PID;
        }
        mPeriodicIO.demand = mPeriodicIO.current_position_ticks;
    }

    public void goHome() {
        isGoingHome = true;
        goToPosition(1);
    }

    public double getSensorPosition() {
        return mMaster.getSelectedSensorPosition();
    }

    protected void set(double percentage) {
        //mMaster.set(ControlMode.PercentOutput, percentage);
        if (mElevatorControlState != ElevatorControlState.OPEN_LOOP) {
            mElevatorControlState = ElevatorControlState.OPEN_LOOP;
        }
        mPeriodicIO.demand = percentage;
    }

    public void setBrakeMode(boolean brake) {
        if (brake) {
            mMaster.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Power", mMaster.getMotorOutputPercent());
        //SmartDashboard.putNumber("Elevator demand", mPeriodicIO.demand);
    }

    @Override
    public void stop() {
        set(0);
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
    }

    public synchronized void resetOnLimits() {
        if (mPeriodicIO.limit_switch)
            zeroSensors();
    }

    public synchronized void readPeriodicInputs() {
    }

    public synchronized void writePeriodicOutputs() {
        switch (mElevatorControlState) {
            case MOTION_MAGIC:
                if (isGoingHome && mMaster.getSelectedSensorPosition() < 1500) {
                    mElevatorControlState = ElevatorControlState.OPEN_LOOP;
                    mPeriodicIO.demand = 0;
                    stop();
                    isGoingHome = false;
                }
                mMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.06); // SET FF   
                break;
            case POSITION_PID:
                mMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.0); // SET FF
                break;
            case OPEN_LOOP:
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, 0.0); // SET FF
                break;
            case CLIMBING:
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
                break;
            default:
                break;
        }
    }

    public static class PeriodicIO {
        // Inputs
        public int current_position_ticks;
        public boolean limit_switch;
        public double climbing_height;

        // Outputs
        public double demand;
    }
}