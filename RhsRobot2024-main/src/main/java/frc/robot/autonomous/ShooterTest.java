// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FMJRobot;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.drivesystem.CommandSwerveDrivetrain;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.FeederSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.IWrist;

/** Add your docs here. */
public class ShooterTest extends AutonomousRoutine {

    public ShooterTest(FMJRobot robot, double velocity, double acceleration, boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDriveSubsystem();
        ShooterSubsystem shooter = robot.getShooterSubsystem();
        FeederSubsystem feeder = robot.getFeederSubsystem();
        IntakeSubsystem intake = robot.getIntakeSubsystem();
        ClimberSubsystem climber = robot.getClimberSubsystem();
        IWrist wrist = robot.getWristSubsystem();


        PathPlannerPath firstPath = PathPlannerPath.fromPathFile("Blue A 2N");
        PathPlannerPath secondPath = PathPlannerPath.fromPathFile("Blue A 3N");
        if (isRedAlliance) {
            firstPath = firstPath.flipPath();
            secondPath = secondPath.flipPath();
        }
        this.initialPose = firstPath.getPreviewStartingHolonomicPose();

        addCommands(
            Commands.runOnce(shooter::runShooter, shooter),
            Commands.waitSeconds(0.1),
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.3),
            Commands.runOnce(feeder::stopFeeder, feeder),
            Commands.runOnce(shooter::stopShooter, shooter)
            );
    }
    
}
