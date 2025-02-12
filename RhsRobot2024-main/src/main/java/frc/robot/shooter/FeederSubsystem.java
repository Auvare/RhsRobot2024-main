// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    /** Creates a new FeederSubsystem. */

    private TalonSRX feeder;
    private final double FEED_SPEED = -1.0;

    public FeederSubsystem() {
        feeder = new TalonSRX(Constants.CAN_FEEDER);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runFeeder() {
        feeder.set(ControlMode.PercentOutput, FEED_SPEED);
        
    }

    public void reverseFeeder() {
        feeder.set(ControlMode.PercentOutput, -FEED_SPEED);
    }

    public void runIntake() {
        feeder.set(ControlMode.PercentOutput, -0.40);
    }

    public void stopFeeder() {
        feeder.set(ControlMode.PercentOutput, 0.0);
    }
}
