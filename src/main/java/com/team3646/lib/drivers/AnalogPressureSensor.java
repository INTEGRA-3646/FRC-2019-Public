package com.team3646.lib.drivers;

import com.team3646.frc2019.Constants;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogPressureSensor {

    private static AnalogPressureSensor mInstance = null;
    private AnalogInput pressureSensor = new AnalogInput(Constants.kAnalogPressureSensorPort);

    private AnalogPressureSensor() {
    }

    public static AnalogPressureSensor getInstance() {
        if (mInstance == null) {
            return mInstance = new AnalogPressureSensor();
        }
        return mInstance;
    }

    public double getPressure() {
        return pressureSensor.getVoltage();
    }
}