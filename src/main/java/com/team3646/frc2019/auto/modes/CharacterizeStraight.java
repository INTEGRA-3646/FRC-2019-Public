package com.team3646.frc2019.auto.modes;

import com.team3646.frc2019.auto.AutoModeBase;
import com.team3646.frc2019.auto.AutoModeEndedException;
import com.team3646.frc2019.auto.actions.CollectAccelerationData;
import com.team3646.frc2019.auto.actions.CollectVelocityData;
import com.team3646.frc2019.auto.actions.WaitAction;
import com.team3646.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class CharacterizeStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        // runAction(new ShiftHighGearAction(false));
        // runAction(new WaitAction(10));

        runAction(new CollectVelocityData(velocityData, false, true));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, false, true));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        try (PrintWriter constFile = new PrintWriter(new FileWriter("/home/lvuser/constants.txt", true))) {
            constFile.println("ks: " + constants.ks);
            constFile.println("kv: " + constants.kv);
            constFile.println("ka: " + constants.ka);
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}
