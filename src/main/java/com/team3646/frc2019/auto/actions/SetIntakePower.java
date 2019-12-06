package com.team3646.frc2019.auto.actions;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team3646.lib.drivers.TalonSRXChecker;
import com.team3646.lib.drivers.TalonSRXFactory;
import com.team3646.frc2019.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;;

public class SetIntakePower implements Action {

    private double mStartTime;
    private double mDuration, mMotorSignal;
    private TalonSRX mIntake;
    
    public SetIntakePower(double motorSignal, double duration) {
        mMotorSignal = motorSignal;
        mDuration = duration;

        mIntake = TalonSRXFactory.createDefaultTalon(4);
        configureMaster(mIntake, true);
    }

    private void configureMaster(TalonSRX talon, boolean left) {

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .QuadEncoder, 0, 100); //primary closed-loop, 100 ms timeout
        
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }

        talon.setInverted(!left);
        talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration;
    }

    @Override
    public void update() {
        System.out.println((Timer.getFPGATimestamp() - mStartTime) + " > " + mDuration);

    }

    @Override
    public void done() {
        mIntake.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void start() {
        mIntake.set(ControlMode.PercentOutput, mMotorSignal);
        mStartTime = Timer.getFPGATimestamp();
    }
}