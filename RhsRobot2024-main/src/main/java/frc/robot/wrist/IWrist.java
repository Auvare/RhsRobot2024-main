// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

/** Add your docs here. */
public interface IWrist {
    public void holdWristDownForIntake();
    public void stopWrist();

    public void wristUp();

    public void wristDown();

    public void moveToAmpHeight();

    public void moveToZero();

    public void moveToPosition(double position);

    public void enable();
}