package com.team3646.frc2019;

import com.team3646.frc2019.controlboard.IFirstDriverControlBoard;
import com.team3646.frc2019.controlboard.ISecondDriverControlBoard;

public interface IControlBoard extends IFirstDriverControlBoard, ISecondDriverControlBoard {
    
    // // First Driver
    
    // double getThrottle();

    // double getTurn();

    // boolean getQuickTurn();

    // double getReleaseHatch();

    // double getShootBall();

    // // Auto Modes

    // boolean getAutoMode();

    // boolean endAutoMode();
    
    // double getNearHatch();

    // double getFarHatch();

    // // Shifter

    // boolean getShiftLow();

    // boolean getShiftHigh();

    // // Climb 

    // boolean preClimb();

    // boolean postClimb();

    // // Second Driver

    // boolean goCargoShipPosition();

    // boolean goBasePosition();

    // boolean goHatchLoadingPosition();

    // boolean goCollectCargo();

    // boolean isHatch();

    // boolean isCargo();

    // // Hatch

    // boolean goFirstHatchPlacementPosition();

    // boolean goSecondHatchPlacementPosition();

    // boolean goThirdHatchPlacementPosition();

    // // Cargo

    // boolean goFirstCargoPlacementPosition();
    
    // boolean goSecondCargoPlacementPosition();

    // boolean goThirdCargoPlacementPosition();

    // // Release Hatch

    // double gripHatch();

    // // Shoot Ball

    // double setRollerSuck();

    // // Manual Controls 

    // boolean getManualControls();

    // boolean getZeroSensors();

    // double getIntake();

}