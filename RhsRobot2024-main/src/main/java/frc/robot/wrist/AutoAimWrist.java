// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.FMJRobot;
import frc.robot.LimelightHelpers;

public class AutoAimWrist extends Command {

    private BooleanSubscriber intakeBeambreak;
    private DoubleSubscriber aprilTagDistance;

    private IWrist wristSubsystem;

    private double ampTagNumber = -1;
    private double speakerTagNumber = -1;

    // private static InterpolatingTreeMap<InterpolatingDouble, Vector2> TUNED_MAP = new InterpolatingTreeMap<>();
    // static {
    //     TUNED_MAP.put(new InterpolatingDouble(65.0), new Vector2(180.0, 8500.0)); // 65" to 90" // minus 25
    //     TUNED_MAP.put(new InterpolatingDouble(90.0), new Vector2(80.0, 9000.0));  // +90" to 125" // minus 35
    //     TUNED_MAP.put(new InterpolatingDouble(125.0), new Vector2(40.0, 10000.0));  // +90" to 125" // minus 30
    //     TUNED_MAP.put(new InterpolatingDouble(155.0), new Vector2(20.0, 11000.0));  // +125.0" to 155" // minus 30
    //     TUNED_MAP.put(new InterpolatingDouble(185.0), new Vector2(0.0, 12000.0));  // >185" // Measuring tape 180
    //     TUNED_MAP.put(new InterpolatingDouble(227.0), new Vector2(0.0, 15000.0));
    // }

    private static InterpolatingDoubleTreeMap wristMap = new InterpolatingDoubleTreeMap();
    static {
        /*0ft, dist 30.278, wrist 0.655
         * 3ft, dist 53.25, wrist 0.733
         * 7ft, dist 81.85, wrist 0.76
         * 13ft, dist 96.67, wrist 0.765
         */
        wristMap.put(30.278, 0.655);
        wristMap.put(53.25, 0.733);
        wristMap.put(81.85, 0.76);
        wristMap.put(96.67, 0.765);  
    }

    /** Creates a new AutoAimWrist. */
    public AutoAimWrist(FMJRobot robot) {
        wristSubsystem = robot.getWristSubsystem();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements((Subsystem)wristSubsystem);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);

        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        intakeBeambreak = beamTopic.subscribe(false);

        DoubleTopic aprilTagDistanceTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_DISTANCE);
        aprilTagDistance = aprilTagDistanceTopic.subscribe(0.0);

        ampTagNumber = (double) robot.getAmpTagNumber();
        speakerTagNumber = (double) robot.getSpeakerTagNumber();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currAprilTagId = LimelightHelpers.getFiducialID(Constants.NT_LIMELIGHT_TAG_TABLE);
        double currDistance = aprilTagDistance.get();

        // If aiming at speaker then adjust systems based on current conditions
        //if (currAprilTagId == speakerTagNumber) {
            // Calculate the value needed for wrist & shooter
            double setWrist = wristMap.get(currDistance); //TODO: test this value is correct

            wristSubsystem.moveToPosition(setWrist);
        //} else {
            // Do Nothing, we need to find a target
        //}
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.moveToZero();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean result = false;

        if (intakeBeambreak.getAsBoolean() == false) {
            result = true;
        }
        
        return result;
    }

        @Override
    public String getName() {
        return "Auto Aim Wrist";
    }
}
