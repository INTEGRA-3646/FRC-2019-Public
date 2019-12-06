package com.team3646.frc2019.controlboard;

import com.team3646.frc2019.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class SecondDriverControlBoard implements ISecondDriverControlBoard {
    private static SecondDriverControlBoard mInstance = null;

    public static SecondDriverControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new SecondDriverControlBoard();
        }

        return mInstance;
    }

    private Joystick mSecondJoystick;

    private SecondDriverControlBoard() {
        mSecondJoystick = new Joystick(Constants.kSecondDriverGamepadPort);
    }

    @Override
    public boolean goCargoShipPosition() {
        return mSecondJoystick.getRawButton(10);
    }

    @Override 
    public boolean goBasePosition() {
        return mSecondJoystick.getRawButton(9);
    }

    @Override
    public boolean goHatchLoadingPosition() {
        return mSecondJoystick.getRawButton(3);
    }

    @Override 
    public boolean goCollectCargo() {
        return mSecondJoystick.getRawButton(3);
    }

    @Override 
    public boolean isHatch() {
        return mSecondJoystick.getRawButton(5);
    }

    @Override 
    public boolean isCargo() {
        return mSecondJoystick.getRawButton(6);
    }
    
    @Override 
    public boolean goFirstHatchPlacementPosition() {
        return mSecondJoystick.getRawButton(1);
    }

    @Override 
    public boolean goSecondHatchPlacementPosition() {
        return mSecondJoystick.getRawButton(2);
    }

    @Override 
    public boolean goThirdHatchPlacementPosition() {
        return mSecondJoystick.getRawButton(4);
    }

    @Override 
    public boolean goFirstCargoPlacementPosition() {
        return mSecondJoystick.getRawButton(1);
    }

    @Override 
    public boolean goSecondCargoPlacementPosition() {
        return mSecondJoystick.getRawButton(2);
    }

    @Override 
    public boolean goThirdCargoPlacementPosition() {
        return mSecondJoystick.getRawButton(4);
    }

    @Override 
    public double gripHatch() {
        return mSecondJoystick.getRawAxis(2);
    }

    @Override 
    public double setRollerSuck() {
        return mSecondJoystick.getRawAxis(3);
    }

    @Override
    public boolean getManualControls() {
        return mSecondJoystick.getRawButton(7);
    }

    @Override
    public boolean getManualControlsReleased() {
        return mSecondJoystick.getRawButtonReleased(7);
    }

    @Override 
    public boolean getZeroSensors() {
        return mSecondJoystick.getRawButton(8);
    }

    @Override 
    public double getIntake() {
        return mSecondJoystick.getRawAxis(5) / 2;
    }

    @Override
    public boolean stopRollers() {
        return mSecondJoystick.getPOV() == 180;
    }

    @Override
    public boolean getCargoLoading() {
        return mSecondJoystick.getPOV() == 90;
    }

    @Override
    public boolean getEmergencyHomePos() {
        return mSecondJoystick.getPOV() == 270;
    }
}