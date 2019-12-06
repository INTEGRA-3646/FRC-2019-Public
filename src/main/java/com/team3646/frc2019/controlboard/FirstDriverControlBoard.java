package com.team3646.frc2019.controlboard;

import com.team3646.frc2019.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class FirstDriverControlBoard implements IFirstDriverControlBoard {
    private static FirstDriverControlBoard mInstance = null;

    public static FirstDriverControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new FirstDriverControlBoard();
        }

        return mInstance;
    }

    private Joystick mJoystick;

    private FirstDriverControlBoard() {
        mJoystick = new Joystick(Constants.kFirstDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return -mJoystick.getRawAxis(1);
    }

    @Override
    public double getTurn() {
        return mJoystick.getRawAxis(4);
    }

    @Override
    public boolean getQuickTurn() {
        return mJoystick.getRawButton(5);
    }

    @Override 
    public double getReleaseHatch() {
        return mJoystick.getRawAxis(2);
    }

    @Override 
    public double getShootBall() {
        return mJoystick.getRawAxis(3);
    }

    @Override
    public boolean getAutoMode() {
        return mJoystick.getRawButton(3);
    }

    @Override 
    public boolean endAutoMode() {
        return mJoystick.getRawButton(4);
    }

    @Override 
    public double getNearHatch() {
        return mJoystick.getRawAxis(2);
    }

    @Override 
    public double getFarHatch() {
        return mJoystick.getRawAxis(3);
    }

    @Override 
    public boolean getShiftLow() {
        return mJoystick.getRawButton(4);
    }

    @Override 
    public boolean getShiftHigh() {
        return mJoystick.getRawButton(1);
    }

    @Override 
    public boolean preClimb() {
        return mJoystick.getRawButton(8);
    }

    @Override 
    public boolean postClimb() {
        return mJoystick.getRawButton(7);
    }

    @Override
    public boolean limeTurn() {
        return mJoystick.getRawButton(6);
    }

    @Override
    public boolean getAutoAlign() {
        return mJoystick.getPOV() == 0;
    }
}