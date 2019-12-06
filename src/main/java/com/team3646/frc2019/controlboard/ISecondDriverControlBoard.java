package com.team3646.frc2019.controlboard;

public interface ISecondDriverControlBoard {
    
    boolean goCargoShipPosition();

    boolean goBasePosition();

    boolean goHatchLoadingPosition();

    boolean goCollectCargo();

    boolean isHatch();

    boolean isCargo();

    boolean getCargoLoading();

    // Positions for placing hatch.

    boolean goFirstHatchPlacementPosition();

    boolean goSecondHatchPlacementPosition();

    boolean goThirdHatchPlacementPosition();

    // Positions for shooting ball.

    boolean goFirstCargoPlacementPosition();

    boolean goSecondCargoPlacementPosition();

    boolean goThirdCargoPlacementPosition();

    // Release Hatch

    double gripHatch();

    // Shoot Ball

    double setRollerSuck();

    boolean stopRollers();

    // Manual Controls

    boolean getManualControls();

    boolean getManualControlsReleased();

    boolean getZeroSensors();

    double getIntake();

    boolean getEmergencyHomePos();
}