// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.FMJRobot;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.drivesystem.CommandSwerveDrivetrain;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.FeederSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.AutoAimWrist;
import frc.robot.wrist.IWrist;

/** Add your docs here. */
public class FourNoteLeave extends AutonomousRoutine {

    private final SwerveRequest.FieldCentric trackDrive =
            new SwerveRequest.FieldCentric().withDeadband(Constants.CTRESwerve.MAXSPEED * 0.1)
                    .withRotationalDeadband(Constants.CTRESwerve.MAXANGULARRATE * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private DoubleSubscriber tagControllerSub;

    public FourNoteLeave(FMJRobot robot, double velocity, double acceleration, boolean isRedAlliance) {
        super(robot, velocity, acceleration, isRedAlliance);

        CommandSwerveDrivetrain drivetrain = robot.getDriveSubsystem();
        ShooterSubsystem shooter = robot.getShooterSubsystem();
        FeederSubsystem feeder = robot.getFeederSubsystem();
        IntakeSubsystem intake = robot.getIntakeSubsystem();
        ClimberSubsystem climber = robot.getClimberSubsystem();
        IWrist wrist = robot.getWristSubsystem();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);

        DoubleTopic tagTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_PID_VALUE);
        tagControllerSub = tagTopic.subscribe(0);

        PathPlannerPath firstPath = PathPlannerPath.fromPathFile("Red A 2N FAST");
        PathPlannerPath secondPath = PathPlannerPath.fromPathFile("A4A");
        PathPlannerPath thirdPath = PathPlannerPath.fromPathFile("A5A");
        //PathPlannerPath fourPath = PathPlannerPath.fromPathFile("Blue A 5N");
        if (isRedAlliance) {
            firstPath = firstPath.flipPath();
            secondPath = secondPath.flipPath();
            thirdPath = thirdPath.flipPath();
            //fourPath = fourPath.flipPath();
        }

        this.initialPose = firstPath.getPreviewStartingHolonomicPose();

        addCommands(
            Commands.runOnce(shooter::runShooter, shooter),
            Commands.waitSeconds(0.5),
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.3),
            Commands.runOnce(feeder::stopFeeder, feeder),
            Commands.runOnce(wrist::moveToZero, (Subsystem)wrist),
            new ParallelCommandGroup(
                new IntakeCommand(intake, feeder, wrist, 200),
                drivetrain.getAutoPath(firstPath)
            ),
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.3),
            Commands.runOnce(feeder::stopFeeder, feeder),
            new ParallelCommandGroup(
                new IntakeCommand(intake, feeder, wrist, 200),
                drivetrain.getAutoPath(secondPath)
            ),
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.3),
            Commands.runOnce(feeder::stopFeeder, feeder),
            new ParallelCommandGroup(
                new IntakeCommand(intake, feeder, wrist, 200),
                drivetrain.getAutoPath(thirdPath)
            ),
            /* new ParallelCommandGroup(
                    drivetrain.applyRequest(() -> trackDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(
                        tagControllerSub.get() * Constants.CTRESwerve.MAXANGULARRATE)),
                    new AutoAimWrist(robot)
                ).withTimeout(1.5), */
            Commands.runOnce(feeder::runFeeder, feeder),
            Commands.waitSeconds(0.3),
            Commands.runOnce(feeder::stopFeeder, feeder),
            Commands.runOnce(shooter::stopShooter, shooter)



        
        );
    }
    
}
