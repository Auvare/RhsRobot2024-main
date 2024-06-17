// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

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
import frc.robot.climber.ClimberSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.IWrist;
import frc.robot.wrist.ProfiledWristSubsystem;

public class AutoShoot extends Command {
    private BooleanSubscriber intakeBeambreak;
    private DoubleSubscriber aprilTagDistance;
    private BooleanSubscriber autoAimEnabled;
    private ClimberSubsystem climberSS;
    private IWrist wristSS;
    private ShooterSubsystem shooterSS;
    private FMJRobot fmjRobot;

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
    private static InterpolatingDoubleTreeMap shootMap = new InterpolatingDoubleTreeMap();
    static {
        wristMap.put(0.0, ProfiledWristSubsystem.MIN_ABSOLUTE_POSITION);  // Distance of 0, wrist needs to be at position 0
        shootMap.put(0.0, ShooterSubsystem.SHOOT_SPEED); // Distance of 0, shooter needs to be at a speed of ? rpm
        wristMap.put(240.0, ProfiledWristSubsystem.PODIUM_POSITION);  // Distance of 240inches, wrist needs to be at position -10
        shootMap.put(240.0, ShooterSubsystem.SHOOT_SPEED); // Distance of 240inches, shooter needs to be at a speed of ? rpm
    }

    /** Creates a new AutoShoot. */
    public AutoShoot(FMJRobot robot) {

        fmjRobot = robot;
        climberSS = fmjRobot.getClimberSubsystem();
        wristSS = fmjRobot.getWristSubsystem();
        shooterSS = fmjRobot.getShooterSubsystem();

        addRequirements(climberSS, shooterSS, (Subsystem)wristSS);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);

        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        intakeBeambreak = beamTopic.subscribe(false);

        BooleanTopic autoAimTopic = fmjTable.getBooleanTopic(Constants.NT_AUTO_AIM);
        autoAimEnabled = autoAimTopic.subscribe(true);

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

        climberSS.moveToAmpHeight();

        // If aiming at speaker then adjust systems based on current conditions
        if (currAprilTagId == speakerTagNumber) {
            // Calculate the value needed for wrist & shooter
            double setWrist = wristMap.get(currDistance); //TODO: test this value is correct
            double setShooter = shootMap.get(currDistance); //TODO: test this value is correct
            wristSS.moveToZero();
            shooterSS.runShooter();
        } else {
            // Do Nothing, we need to find a target
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSS.stopShooter();

        // If Auto Aiming is still enabled and beam break is false 
        // then we have made the shot and climber should be reset
        if (autoAimEnabled.get() && intakeBeambreak.getAsBoolean() == false) {
            wristSS.moveToZero();
            climberSS.moveToZero();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean result = false;

        if (autoAimEnabled.get() == false || intakeBeambreak.getAsBoolean() == false) {
            result = true;
        }
        
        return result;
    }

}
