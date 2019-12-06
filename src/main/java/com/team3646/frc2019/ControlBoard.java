package com.team3646.frc2019;

import com.team3646.frc2019.controlboard.*;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private IFirstDriverControlBoard mFirstDriverControlBoard = FirstDriverControlBoard.getInstance();
    private ISecondDriverControlBoard mSecondDriverControlBoard = SecondDriverControlBoard.getInstance();

    private ControlBoard() {
    }

    // First Driver

    @Override
    public double getThrottle() {
        return mFirstDriverControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mFirstDriverControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mFirstDriverControlBoard.getQuickTurn();
    }

    @Override
    public double getReleaseHatch() {
        return mFirstDriverControlBoard.getReleaseHatch();
    }

    @Override
    public double getShootBall() {
        return mFirstDriverControlBoard.getShootBall();
    }

    public boolean limeTurn() {
        return mFirstDriverControlBoard.limeTurn();
    }

    // Auto Modes

    @Override
    public boolean getAutoMode() {
        return mFirstDriverControlBoard.getAutoMode();
    }

    @Override 
    public boolean endAutoMode() {
        return mFirstDriverControlBoard.endAutoMode();
    }

    @Override 
    public double getNearHatch() {
        return mFirstDriverControlBoard.getNearHatch();
    }

    @Override 
    public double getFarHatch() {
        return mFirstDriverControlBoard.getFarHatch();
    }

    // Shifter

    @Override
    public boolean getShiftLow() {
        return mFirstDriverControlBoard.getShiftLow();
    }

    @Override
    public boolean getShiftHigh() {
        return mFirstDriverControlBoard.getShiftHigh();
    }

    @Override
    public boolean preClimb() {
        return mFirstDriverControlBoard.preClimb();
    }

    @Override
    public boolean postClimb() {
        return mFirstDriverControlBoard.postClimb();
    }

    // Second Driver 

    @Override
    public boolean goCargoShipPosition() {
        return mSecondDriverControlBoard.goCargoShipPosition();
    }

    @Override
    public boolean goBasePosition() {
        return mSecondDriverControlBoard.goBasePosition();
    }

    @Override
    public boolean goHatchLoadingPosition() {
        return mSecondDriverControlBoard.goHatchLoadingPosition();
    }

    @Override
    public boolean goCollectCargo() {
        return mSecondDriverControlBoard.goCollectCargo();
    }

    @Override
    public boolean isHatch() {
        return mSecondDriverControlBoard.isHatch();
    }

    @Override
    public boolean isCargo() {
        return mSecondDriverControlBoard.isCargo();
    }

    @Override
    public boolean goFirstHatchPlacementPosition() {
        return mSecondDriverControlBoard.goFirstHatchPlacementPosition();
    }

    @Override
    public boolean goSecondHatchPlacementPosition() {
        return mSecondDriverControlBoard.goSecondHatchPlacementPosition();
    }

    @Override
    public boolean goThirdHatchPlacementPosition() {
        return mSecondDriverControlBoard.goThirdHatchPlacementPosition();
    }

    @Override
    public boolean goFirstCargoPlacementPosition() {
        return mSecondDriverControlBoard.goFirstCargoPlacementPosition();
    }

    @Override
    public boolean goSecondCargoPlacementPosition() {
        return mSecondDriverControlBoard.goSecondCargoPlacementPosition();
    }

    @Override
    public boolean goThirdCargoPlacementPosition() {
        return mSecondDriverControlBoard.goThirdCargoPlacementPosition();
    }

    @Override
    public double gripHatch() {
        return mSecondDriverControlBoard.gripHatch();
    }

    @Override
    public double setRollerSuck() {
        return mSecondDriverControlBoard.setRollerSuck();
    }

    @Override
    public boolean getManualControls() {
        return mSecondDriverControlBoard.getManualControls();
    }

    @Override
    public boolean getZeroSensors() {
        return mSecondDriverControlBoard.getZeroSensors();
    }

    @Override
    public double getIntake() {
        return mSecondDriverControlBoard.getIntake();
    }

    @Override
    public boolean stopRollers() {
        return mSecondDriverControlBoard.stopRollers();
    }

    @Override
    public boolean getManualControlsReleased() {
        return mSecondDriverControlBoard.getManualControlsReleased();
    }

    @Override
    public boolean getAutoAlign() {
        return mFirstDriverControlBoard.getAutoAlign();
    }

    @Override
    public boolean getCargoLoading() {
        return mSecondDriverControlBoard.getCargoLoading();
    }

    @Override
    public boolean getEmergencyHomePos() {
        return mSecondDriverControlBoard.getEmergencyHomePos();
    }
}