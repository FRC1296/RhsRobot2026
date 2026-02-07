// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LedSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */

    private AddressableLED ledStrip;
    private AddressableLEDBuffer buffer;

    private final int ledCount = 14;

    public LedSubsystem() {
        buffer = new AddressableLEDBuffer(ledCount);
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(ledCount);

        // Set all leds to red for startup
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kOrange);
        }

        ledStrip.setData(buffer);
        ledStrip.start();
    }

    private void visionStatus() {
       
        Color red = Color.kRed;
        Color green = Color.kGreen;
        PoseEstimate LLA = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-a");
        PoseEstimate LLB = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-b");

        if (LLA.tagCount > 0 || LLB.tagCount > 0) {

            for (int i = 0; i < ledCount; i++) {
                buffer.setLED(i, green);
            }
            ledStrip.setData(buffer);
        } else {
            for (int i = 0; i < ledCount; i++) {
                buffer.setLED(i, red);
            }
            ledStrip.setData(buffer);
        }
        
    }

    @Override
    public void periodic() {
        visionStatus();
    }
}
