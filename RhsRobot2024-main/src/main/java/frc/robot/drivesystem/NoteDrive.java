// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivesystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import frc.robot.climber.ClimberSubsystem;
import frc.robot.wrist.IWrist;

/**
 * Pre-requisites:
 * 1. Reset the Limelight PID prior to calling this command
 * 2. Run the IntakeCommand with this command in a Parrallel Command Group
 */
public class NoteDrive extends Command {
    private FMJRobot robot;
    private CommandSwerveDrivetrain drivetrain;
    private ClimberSubsystem climber;
    private IWrist wrist;

    private BooleanSubscriber intakeBeambreak;
    private DoubleSubscriber noteControllerSub;
    private DoubleSubscriber noteDistanceSub;

    private SwerveRequest.RobotCentric noteDrive = 
        new SwerveRequest.RobotCentric()
            .withDeadband(Constants.CTRESwerve.MAXSPEED * 0.1)
            .withRotationalDeadband(Constants.CTRESwerve.MAXANGULARRATE * 0.3)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private double initialPoseX;
    private double initialDistance;

    /** Creates a new NoteDrive. */
    public NoteDrive(FMJRobot fmjRobot) {
        robot = fmjRobot;
        drivetrain = robot.getDriveSubsystem();
        climber = robot.getClimberSubsystem();
        wrist = robot.getWristSubsystem();

        addRequirements(drivetrain, climber, (Subsystem)wrist);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);
        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        intakeBeambreak = beamTopic.subscribe(false);

        DoubleTopic noteTopic = fmjTable.getDoubleTopic(Constants.NT_NOTE_LL_PID_VALUE);
        noteControllerSub = noteTopic.subscribe(0);

        DoubleTopic noteDistanceTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_NOTE_DISTANCE);
        noteDistanceSub = noteDistanceTopic.subscribe(0.0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // you will need to reset the limelight PID prior to starting this command
        initialPoseX = drivetrain.getState().Pose.getX();
        initialDistance = noteDistanceSub.get() * 0.0254;
        climber.moveToZero();
        wrist.moveToZero();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Only drive to note if the rotation will be small
        double driveValue = 0.0;
        double rotationPIDValue = noteControllerSub.get();
        if (rotationPIDValue < 0.5) {
            // Drive at 1/4 max speed
            driveValue = -0.5;
        }
        
        drivetrain.setControl(
            noteDrive
                .withRotationalRate( rotationPIDValue * Constants.CTRESwerve.MAXANGULARRATE)
                .withVelocityX(driveValue * Constants.CTRESwerve.MAXSPEED)
                .withVelocityY(0.0)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean result = false;

        if (intakeBeambreak.get()) {
            result = true;
        } else {
            // If we have travelled past our initial distance measurment then we are done
            double currX = drivetrain.getState().Pose.getX();
            if (initialDistance < Math.abs(currX - initialPoseX)) {
                result = true;
            } 
        }
        return result;
    }
}
