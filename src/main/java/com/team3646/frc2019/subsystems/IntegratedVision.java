package com.team3646.frc2019.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntegratedVision extends Subsystem {

    public static IntegratedVision sInstance = null;

    private static double ty, tx, ta;
    private static boolean td;

    private double ballDistance, hatchDistance;
    //private double ballXAxisDist, ballYAxisDist, hatchXAxisDistance, hatchYAxisDistance;

    private static double sumOfErr = 0, oldTx = 0;
    
    private static final double kP = 0.055;
    private static final double kD = 0.55;
    private static final double kI = 0.0; //Dont do this pls
    
    private NetworkTable table = null;

    public static IntegratedVision getInstance() {
        if (sInstance == null) {
            return sInstance = new IntegratedVision();
        }
        return sInstance;
    }
   
    private IntegratedVision() {
    }
    
    public void updateLimelightData() {
        oldTx = td ? tx : oldTx;
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        td = table.getEntry("td").getBoolean(false);
        sumOfErr += tx;
        
        /*
        hatchDistance = 44.5 / Math.tan(Math.toRadians(33.7 + ty));
        hatchXAxisDistance = hatchDistance * Math.abs(Math.sin(Math.toRadians(tx + mNavXBoard.getRawYawDegrees() - 90.0)));
        hatchYAxisDistance = hatchDistance * Math.abs(Math.cos(Math.toRadians(tx + mNavXBoard.getRawYawDegrees() - 90.0)));

        ballDistance = 58.0 / Math.tan(Math.toRadians(32.8 + ty));
        ballXAxisDist = ballDistance * Math.abs(Math.cos(Math.toRadians(mNavXBoard.getRawYawDegrees())));
        ballYAxisDist = ballDistance * Math.abs(Math.sin(Math.toRadians(mNavXBoard.getRawYawDegrees())));
        */
    }

    /*public double alignCurve() {
        double lBDist = leftBack.getDistance();
        double lFDist = leftFront.getDistance();

        double curve = (lBDist-lFDist) * 0.01 * (1 / hatchDistance) * 200; //Vision correction

        if(Math.abs(lBDist-lFDist) < 3){
            curve += ((lBDist + lFDist) / 2 - 55) * 0.01;
        } else if(((lBDist + lFDist) / 2 > 55 && lBDist <= lFDist) || (lBDist + lFDist) / 2 <= 55 && lBDist >= lFDist){
            curve *= 2;    
        }

        return curve;
    }

    public double alignFeed() {
        return hatchDistance * 0.0025;
    }*/

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    public boolean getTd() {
        return td;
    }

    public double getBallDistance() {
        return ballDistance;
    }

    public double turnToTarget() {
        double motorPwr = tx * kP;
        if(tx != 0 && oldTx != 0) {
            motorPwr += (tx - oldTx) * kD;
        }
        return motorPwr + 0.11; // Alignment correction
        //return (tx * kP) + (tx - oldTx) * kD + (sumOfErr * kI);
    }

    public void resetIntegral() {
        sumOfErr = 0;
    }

    public void turnLedOff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void blinkLed() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }

    public void turnLedOn() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void visionMode() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Vision processor
    }

    public void writePeriodicOutputs() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Vision tx", tx);
        SmartDashboard.putNumber("Vision ty", ty);
        SmartDashboard.putNumber("Vision ta", ta);
    }

    @Override
    public void stop() {

    }

}