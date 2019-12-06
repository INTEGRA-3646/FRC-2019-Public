package com.team3646.frc2019.controlboard;

public interface IFirstDriverControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getAutoAlign();

    double getReleaseHatch();

    double getShootBall();

    // Auto Modes
    
    boolean getAutoMode();

    boolean endAutoMode();

    double getNearHatch();

    double getFarHatch();

    // Shifter 

    boolean getShiftLow();

    boolean getShiftHigh();

    // Climb

    boolean preClimb();

    boolean postClimb();

    //Vision

    boolean limeTurn();

}