// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class FullMetalControlPanel extends GenericHID {

    private final int SPEEDROTARYDIALAXIS = 0;
    private final int CLIMBER_SWITCH = 4;

    public FullMetalControlPanel(int port) {
        super(port);
    }

    public double getRawSpeedControlValue() {
        return this.getRawAxis(SPEEDROTARYDIALAXIS);
    }

    public Trigger getClimberSwitch() {
        return this.climberSwitch(CommandScheduler.getInstance().getDefaultButtonLoop()).castTo(Trigger::new);
    }

    protected BooleanEvent climberSwitch(EventLoop loop) {
        return new BooleanEvent(loop, this::getClimberControlSwitch);
    }

    protected boolean getClimberControlSwitch() {
        return this.getRawButton(CLIMBER_SWITCH);
    }

    public double getSpeedAdjustment() {
        double value = 1.0;

        if (this.isConnected()) {
            // the constants
            double rawValue = this.getRawSpeedControlValue();
            int spinMin = -75;
            int spinMax = 87;

            // normalize speed
            double spinDialOut = rawValue * 100;

            // turn the range from (spin min *** spin max) to (0 *** spinMin&spinmax) or
            // when this code was written -75 to 87 to 0 to 162
            spinDialOut += Math.abs(spinMin);

            // make the spin dial range from 0 to 1
            value = spinDialOut / (Math.abs(spinMin) + spinMax);
        }

        return value;
    }
}
