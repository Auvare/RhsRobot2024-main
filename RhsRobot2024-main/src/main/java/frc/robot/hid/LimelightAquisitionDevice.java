// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/** Add your docs here. */
public class LimelightAquisitionDevice {
    protected String limelightName;
    protected PIDController pidController;
    protected double default_kD = 0.0;
    protected double default_kI = 0.0;

    // Range based on TA (Target Area Percentage) ~1.5% of screen taken up by target
    protected double minimumRange = 0.5;

    // Mount angle of the limelight
    protected double mountAngle = 0.0; // The angle of the limelight camera (0 is horizontal, 90 is fully virtical)
    
    // The height in inches of the camera from ground
    protected double mountHeight = 0.0;

    // The height of target from ground in inches
    protected double expectedTargetHeight = 0.0;

    protected int ampTagNumber;
    protected int speakerTagNumber;
    
    /**
     * Constructor
     * @param name The network table name for the device
     * @param kP The kP value to use for PID control
     * @param minimumRange The minimum range to consider a target viable
     * @param angle Mount angle(targeted upward) of the limelight (0 to 90 degrees)
     * @param height The height in inches of the camera from ground (inches)
     * @param targetHeight The height of target from ground in inches
     */
    public LimelightAquisitionDevice(String name, double kP, double minimumRange, double angle, double height, double targetHeight) {
        limelightName = name;
        pidController = new PIDController(kP, default_kI, default_kD);
        this.minimumRange = minimumRange;
        this.mountAngle = angle;
        this.mountHeight = height;
        this.expectedTargetHeight = targetHeight;
    }

    /**
     * Set the minimum Target Area value that triggers automated systems
     * @param range
     */
    public void setMinimumRange(double range) {
        this.minimumRange = range;
    }

    public void reset() {
        pidController.reset();
    }

    /**
     * Used PIDController to calculates the controller value 
     * based on the current Limelight tx (horizontal offset)
     * This will only return a non-zero value if there is a target
     * 
     * @return value to be used as controller input
     */
    public double getLimelightX() {
        double output = 0.0;

        if (LimelightHelpers.getTV(limelightName)) {
            output = pidController.calculate(LimelightHelpers.getTX(limelightName), 0);
            //MathUtil.clamp(output, -1.0, 1.0);
            if (output > 1) {
                output = 1;
            }
            if (output < -1) {
                output = -1;
            }
        }

        return output;
    }

    /**
     * Is the Robot within the minimum range of the note to turn on automated systems
     * 
     * @return true if within minimum range
     */
    public boolean isWithinRange() {
        boolean result = false;

        // Do we have a target(tv)
        if (LimelightHelpers.getTV(limelightName)) {
            // Target Area(ta) (0% of image to 100% of image)
            if (LimelightHelpers.getTA(limelightName) >= minimumRange) {
                result = true;
            }
        }

        return result;
    }

    public double calculateDistanceToTarget() {
        // get Vertical offset from crosshair to target. this will be more accurate than poseDistance in
        // most instances, unknown unit of measure
        double ty = LimelightHelpers.getTY(limelightName);
        double angleToGoalDegree = this.mountAngle + ty;
        double angleToGoalRadian = angleToGoalDegree * (3.1415926 / 180.0);
        double llDistance = (this.expectedTargetHeight - this.mountHeight) / Math.tan(angleToGoalRadian);
        return llDistance;
    }

    public int getAmpTagNumber() {
        return ampTagNumber;
    }

    public void setAmpTagNumber(int ampTagNumber) {
        this.ampTagNumber = ampTagNumber;
    }

    public int getSpeakerTagNumber() {
        return speakerTagNumber;
    }

    public void setSpeakerTagNumber(int speakerTagNumber) {
        this.speakerTagNumber = speakerTagNumber;
    }
}
