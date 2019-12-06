package com.team3646.lib.drivers;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class HCSR04 {   

    // Poor implementation :/ -- gonna improve

    private DigitalInput echoChannel;
    private DigitalOutput triggerChannel;
    private final double kPingDuration = 10 * 1e-6;
    private Counter m_counter;
    private double highPeriod = 0, lastTimestamp;
    private boolean hasPinged = false;

    public HCSR04(int trigger, int echo) {
        triggerChannel = new DigitalOutput(trigger);
        echoChannel = new DigitalInput(echo);
        m_counter = new Counter(echoChannel);
        m_counter.setSemiPeriodMode(true);
    }

    public double getDistance() {
        if (!hasPinged) {
            triggerChannel.pulse(kPingDuration);
            lastTimestamp = Timer.getFPGATimestamp();
            hasPinged = true;
        }

        if (Timer.getFPGATimestamp() - lastTimestamp > 0.1 && hasPinged) {
            m_counter.reset();
            //System.out.println("Ultrasonic debug  " + highPeriod);
            highPeriod = m_counter.getPeriod();
            hasPinged = false;
        }
        return 100 * highPeriod * 340 / 2;
    }
}