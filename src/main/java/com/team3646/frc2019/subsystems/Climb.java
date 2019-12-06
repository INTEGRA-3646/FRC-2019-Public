package com.team3646.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team3646.frc2019.Constants;
import com.team3646.lib.drivers.TalonSRXFactory;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends Subsystem {

    private final DoubleSolenoid mClimbCylinders;
    private final TalonSRX mCylinderWheels;
    public boolean isClimbing = false;
    private Intake mIntake = Intake.getInstance();
    
    private static Climb mInstance = null;
    
    public synchronized static Climb getInstance() {
        if (mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
    }

    private Climb() {
        mCylinderWheels = TalonSRXFactory.createDefaultTalon(Constants.kClimbWheelId);
        mCylinderWheels.setNeutralMode(NeutralMode.Brake);
        mClimbCylinders = new DoubleSolenoid(2, 3);
        
        mClimbCylinders.set(DoubleSolenoid.Value.kReverse); // Closed state
        mCylinderWheels.setNeutralMode(NeutralMode.Brake);
    }

    public void collectSubsystem() {
        mClimbCylinders.set(Value.kReverse);
        isClimbing = false;
    }
       
    public void extendSubsystem() {
        mClimbCylinders.set(Value.kForward);
        isClimbing = true;
        mIntake.mWristMaster.configClosedLoopPeakOutput(0, 0.85);
    }

    public void setOff() {
        mClimbCylinders.set(Value.kOff);
    }

    public void setWheelsForward(boolean postRaising) {
        if (postRaising)
            mCylinderWheels.set(ControlMode.PercentOutput, -1.0);
        else
            mCylinderWheels.set(ControlMode.PercentOutput, -0.13);
    }
    
    public void stopWheels() {
        mCylinderWheels.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Climbing Status", isClimbing);
    }

    @Override
    public void stop() {

    }

    public synchronized void readPeriodicInputs() {}

    public synchronized void writePeriodicOutputs() {}

}