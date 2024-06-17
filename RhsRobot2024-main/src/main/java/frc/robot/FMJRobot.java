// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Kaden waz here 2024 states comp
//Miles was here 2024 states comp
/*Bing Bong aka johny small wong was here
 Luke was here states 2024
 */
//Elijah was here
//a mexican woman was here



package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.Rotation2dSupplier;
import frc.robot.autonomous.AmpSideAutoAim;
import frc.robot.autonomous.AutoAimTest;
import frc.robot.autonomous.AutoShoot;
import frc.robot.autonomous.FiveNoteCenter;
import frc.robot.autonomous.FourNoteLeave;
import frc.robot.autonomous.IAuto;
import frc.robot.autonomous.MidLineSourceClear;
import frc.robot.autonomous.ShooterTest;
import frc.robot.autonomous.TwoNoteLeave;
import frc.robot.autonomous.TwoNoteMidRedA4A;
import frc.robot.autonomous.TwoNoteMidRedA5A;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.climber.ElevatorSubsystem;
import frc.robot.drivesystem.CommandSwerveDrivetrain;
import frc.robot.hid.FullMetalControlPanel;
import frc.robot.hid.LimelightAquisitionDevice;
import frc.robot.intake.IntakeCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.led.LedSubsystem;
import frc.robot.shooter.FeederSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.AutoAimWrist;
import frc.robot.wrist.IWrist;
import frc.robot.wrist.ProfiledWristSubsystem;

/**
 * This is our main robot class
 */
public class FMJRobot {
    // Boolean that identifies whether this instance is the Competition Robot or the Practice Robot
    private boolean competitionBot;
    private DutyCycleEncoder m_dutyCycleEncoder;

    // The current alliance for our robot
    private boolean currAllianceBlue;

    // Choose auton for robot to run
    private SendableChooser<Command> autonChooser = new SendableChooser<>();
    private Command MidLineSourceClear = null;
    private Command FourNoteLeave = null;
    private Command TwoNoteLeave = null;
    private Command TwoNoteMidRedA4A = null;
    private Command ShooterTest = null;
    private Command TwoNoteMidRedA5A = null;
    private Command FiveNote = null;
    private Command AutoAimTest = null;
    private Command AmpSideAutoAim = null;

    private Command MidLineSourceClearRed = null;
    private Command FourNoteLeaveRed = null;
    private Command TwoNoteLeaveRed = null;
    private Command TwoNoteMidRedA4ARed = null;
    private Command ShooterTestRed = null;
    private Command TwoNoteMidRedA5ARed = null;
    private Command FiveNoteRed = null;
    private Command AutoAimTestRed = null;
    private Command AmpSideAutoAimRed = null;

    // Add Field display so we can see where the robot thinks it is
    private Field2d field2d = new Field2d();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver =
            new CommandXboxController(Constants.DRIVER_CONTROL); // My joystick
    private final CommandXboxController operator =
            new CommandXboxController(Constants.OPERATOR_CONTROL);
    private final CommandJoystick joystickController = 
            new CommandJoystick(3);
    //Joystick joystickController = new Joystick(3);
    private final FullMetalControlPanel customController =
            new FullMetalControlPanel(Constants.CUSTOM_CONTROL);
    private final CommandXboxController characterizer =
            new CommandXboxController(Constants.SYSID_CONTROL);
    private final LimelightAquisitionDevice limelightNote =
            new LimelightAquisitionDevice(Constants.NT_LIMELIGHT_NOTE_TABLE, Constants.LL_NOTE_KP,
                    Constants.LL_MIN_RANGE_NOTE, Constants.LL_REAR_MOUNT_ANGLE,
                    Constants.LL_REAR_MOUNT_HEIGHT, Constants.LL_NOTE_HEIGHT);
    private final LimelightAquisitionDevice limelightTarget =
            new LimelightAquisitionDevice(Constants.NT_LIMELIGHT_TAG_TABLE, Constants.LL_TAG_KP,
                    Constants.LL_MIN_RANGE_TAG, Constants.LL_FRONT_MOUNT_ANGLE,
                    Constants.LL_FRONT_MOUNT_HEIGHT, Constants.LL_APRIL_TAG_HEIGHT);

    public CommandSwerveDrivetrain drivetrain =
            new CommandSwerveDrivetrain(Constants.CTRESwerve.DrivetrainConstants,
                    Constants.CTRESwerve.CompFrontLeftSwerveModule,
                    Constants.CTRESwerve.CompFrontRightSwerveModule,
                    Constants.CTRESwerve.CompBackLeftSwerveModule,
                    Constants.CTRESwerve.CompBackRightSwerveModule);

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric().withDeadband(Constants.CTRESwerve.MAXSPEED * 0.1)
                    .withRotationalDeadband(Constants.CTRESwerve.MAXANGULARRATE * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.FieldCentric trackDrive =
            new SwerveRequest.FieldCentric().withDeadband(Constants.CTRESwerve.MAXSPEED * 0.1)
                    .withRotationalDeadband(Constants.CTRESwerve.MAXANGULARRATE * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private AutoShoot autoAimCommand;

    /* Path follower */
    //private Command runAuto = drivetrain.getAutoPath("Tests");
    //private Command runAuto;
    private final Telemetry logger = new Telemetry(Constants.CTRESwerve.MAXSPEED);

    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private FeederSubsystem feeder = new FeederSubsystem();
    private DigitalInput beamBreak = new DigitalInput(Constants.DP_BEAMBREAK);
    //private DigitalInput climberLimit = new DigitalInput(Constants.DP_CLIMBER_LIMIT);
    //private DigitalInput wristLimit = new DigitalInput(Constants.DP_WRIST_LIMIT);
    //private WristSubsystem wrist = new WristSubsystem();
    private ProfiledWristSubsystem wrist = new ProfiledWristSubsystem();
    private AutoAimWrist autoWrist;
    private ClimberSubsystem climber = new ClimberSubsystem();
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
        private LedSubsystem led = null;

    private BooleanPublisher autoAimPublisher;
    private BooleanPublisher beamBooleanPublisher;
    private BooleanPublisher climberLimitPublisher;
    private BooleanPublisher wristLimitPublisher;
    private BooleanPublisher noteRangePublisher;
    private BooleanPublisher tagRangePublisher;
    private DoublePublisher limelightTagPublisher;
    private DoublePublisher tagDistancePublisher;
    private DoublePublisher limelightNotePublisher;
    private DoublePublisher noteDistancePublisher;
    private BooleanPublisher climberHeightPublisher;

    private BooleanSubscriber autoAimSubscriber;
    private BooleanSubscriber beamBreakSubscriber;
    private BooleanSubscriber climberLimitSubscriber;
    private BooleanSubscriber wristLimitSubscriber;
    private BooleanSubscriber tagRangeSubscriber;
    private BooleanSubscriber noteRangeSubscriber;
    private DoubleSubscriber tagControllerSub;
    private DoubleSubscriber noteControllerSub;

    private int speakerTagNumber;
    private int ampTagNumber;
    private List<Integer> stageTags;

    public static Orchestra FMJOrchestra = new Orchestra();

    public FMJRobot(boolean compBot, boolean isBlue) {
        competitionBot = compBot;
        currAllianceBlue = isBlue;

        if (currAllianceBlue) {
            speakerTagNumber = Constants.BlueAllianceAprilTags.SPEAKER_CENTERED;
            ampTagNumber = Constants.BlueAllianceAprilTags.AMP;
            stageTags = Arrays.asList(Constants.BlueAllianceAprilTags.STAGE_CENTER,
                    Constants.BlueAllianceAprilTags.STAGE_LEFT,
                    Constants.BlueAllianceAprilTags.STAGE_RIGHT);
        } else {
            speakerTagNumber = Constants.RedAllianceAprilTags.SPEAKER_CENTERED;
            ampTagNumber = Constants.RedAllianceAprilTags.AMP;
            stageTags = Arrays.asList(Constants.RedAllianceAprilTags.STAGE_CENTER,
                    Constants.RedAllianceAprilTags.STAGE_LEFT,
                    Constants.RedAllianceAprilTags.STAGE_RIGHT);
        }

        LedSubsystem led = new LedSubsystem();

        configureNetworkTable();

        drivetrain.configurePublishers();

        
        configureDriverController();

        configureOperatorController();

        // If control panel is available then configure
        if (DriverStation.isJoystickConnected(Constants.CUSTOM_CONTROL) == true) {
            configureCustomController();
        }

        // If third joystick is connected  then configure it for system characterization
        if (DriverStation.isJoystickConnected(Constants.SYSID_CONTROL) == true) {
            configureCharacterizationController();
        }

        drivetrain.registerTelemetry(logger::telemeterize);
        //drivetrain.configurePathPlanner(!currAllianceBlue);

        configureAutonomousOptions();

        configureShuffleboard();
    }

    private void configureDriverController() {
        // Configuration for Swerve Drive is handled within the TeleopSwerveCommand Class
        // Left Stick used for Drive
        // Right Stick used for Rotation
        // Right Bumper used for Robot Centric Driving
        // Right Trigger used to disable slew filtering

        Command defaultDrive = drivetrain.applyRequest(() -> drive
                .withVelocityX(-driver.getLeftY() * Constants.CTRESwerve.MAXSPEED)
                .withVelocityY(-driver.getLeftX() * Constants.CTRESwerve.MAXSPEED)
                .withRotationalRate(-driver.getRightX() * Constants.CTRESwerve.MAXANGULARRATE));

        drivetrain.setDefaultCommand(defaultDrive);

        driver.a().whileTrue(Commands.run(climber::climberDown, climber)).whileFalse(Commands.run(climber::stopClimber,climber));
        driver.y().whileTrue(Commands.run(climber::climberUp, climber)).whileFalse(Commands.run(climber::stopClimber,climber));
        driver.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // TODO: Test the note auto drive pickup routine
        /* Command noteAutonDrive = new NoteDrive(this);
        driver.x()
            .onTrue(
                new ParallelCommandGroup(
                    noteAutonDrive.beforeStarting(limelightNote::reset),
                    new IntakeCommand(intake, feeder, wrist)
                )
            ); *//* 
                  .onFalse(
                  new SequentialCommandGroup(
                     defaultDrive,
                     Commands.runOnce(intake::stopIntake, intake),
                     Commands.runOnce(feeder::stopFeeder, feeder)
                  )
                  ); */

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight
                .withVelocityX(0.25 * Constants.CTRESwerve.MAXSPEED).withVelocityY(0)));
        driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight
                .withVelocityX(-0.25 * Constants.CTRESwerve.MAXSPEED).withVelocityY(0)));

        // Create simulated trigger for beambreak
        Trigger haveNoteTrigger = new Trigger(beamBreakSubscriber);
        Trigger noNoteTrigger = haveNoteTrigger.negate();

        // Create simulated note range trigger
        Trigger noteRangeTrigger = new Trigger(noteRangeSubscriber);

        // Create auto aim trigger
        Trigger autoAimTrigger = new Trigger(autoAimSubscriber);

        // Create simulated note range trigger
        Trigger tagRangeTrigger = new Trigger(tagRangeSubscriber);

        // Create climber position reset trigger
//        Trigger climberPositionReset = new Trigger(climberLimitSubscriber);
        // TODO: uncomment this to enable the climber position reset wtih limit switch
//        climberPositionReset.onTrue(Commands.runOnce(climber::stopClimber, climber)
//               .andThen(Commands.runOnce(climber::resetPosition, climber)));

        // Create wrist position reset trigger
//        Trigger wristPositionReset = new Trigger(wristLimitSubscriber);
        // TODO: uncomment this to enable the wrist position reset wtih limit switch
        // wristPositionReset.onTrue(
        //     Commands.runOnce(wrist::stopWrist, wrist)
        //         .andThen(Commands.runOnce(wrist::resetPosition, wrist))
        // );

        Command noteDrive = drivetrain.applyRequest(() -> trackDrive
                .withVelocityX(-driver.getLeftY() * Constants.CTRESwerve.MAXSPEED)
                .withVelocityY(-driver.getLeftX() * Constants.CTRESwerve.MAXSPEED)
                .withRotationalRate(noteControllerSub.get() * Constants.CTRESwerve.MAXANGULARRATE));

        // TODO: we need to investigate creating a single command that
        //  1. Determines whether we have a Note in the intake
        //  2. If we have a note then this must be April Tag tracking
        //      a. Based on Alliance, only target the correct April Tag number
        //  3. Else - this must be Note targeting

        driver.start().and(noNoteTrigger)
                .toggleOnTrue(noteDrive.beforeStarting(limelightNote::reset))
                .toggleOnFalse(defaultDrive);

        /*haveNoteTrigger.and(tagRangeTrigger)
            //.onTrue(Commands.run(climber::moveToAmpHeight, climber))
            //.onFalse(Commands.run(climber::climberDown, climber));
            .onTrue(new ParallelCommandGroup(
                Commands.run(climber::moveToAmpHeight, climber),
                Commands.run(wrist::moveToAmpHeight, wrist),
                Commands.run(shooter::setAmpSpeed, shooter)
        
            ))
            .onFalse(new ParallelCommandGroup(
                Commands.run(climber::climberDown, climber),
                Commands.run(wrist::wristDown, wrist),
                Commands.run(shooter::stopShooter, shooter)
            ));*/
        autoAimCommand = new AutoShoot(this);
        autoAimTrigger.and(haveNoteTrigger).and(tagRangeTrigger).onTrue(autoAimCommand);

        Command tagDrive = drivetrain.applyRequest(() -> trackDrive
                .withVelocityX(-driver.getLeftY() * Constants.CTRESwerve.MAXSPEED)
                .withVelocityY(-driver.getLeftX() * Constants.CTRESwerve.MAXSPEED)
                .withRotationalRate(tagControllerSub.get() * Constants.CTRESwerve.MAXANGULARRATE));

        autoWrist = new AutoAimWrist(this);
        driver.start().and(haveNoteTrigger)
                .toggleOnTrue(new ParallelCommandGroup(
                    tagDrive.beforeStarting(limelightTarget::reset),
                    autoWrist
                ))
                .toggleOnFalse(defaultDrive);

        driver.rightBumper().onTrue(new IntakeCommand(intake, feeder, wrist));

        // Command to run feeder to shoot
        // TODO: add logic to only allow if shooter is at speed
        driver.rightTrigger()
                .whileTrue(Commands.run(feeder::runFeeder, feeder))
                .whileTrue(Commands.runOnce(intake::runIntake, intake))
                .whileFalse(Commands.runOnce(intake::stopIntake, intake))
                .whileFalse(Commands.run(feeder::stopFeeder, feeder));
    }

    private void configureOperatorController() {
        operator.leftTrigger().and(operator.back().negate())
                .whileTrue(Commands.run(intake::runIntake, intake))
                .whileFalse(Commands.run(intake::stopIntake, intake));

        operator.leftTrigger().and(operator.back())
                .whileTrue(Commands.run(intake::reverseIntake, intake))
                .whileFalse(Commands.run(intake::stopIntake, intake));

        //operator.leftTrigger()
        //    .whileTrue(Commands.run(shooter::runShooter, shooter))
        //    .whileFalse(Commands.run(shooter::stopShooter, shooter));
        operator.rightBumper().toggleOnTrue(
                Commands.startEnd(shooter::runShooter, shooter::stopShooter, shooter));
        operator.leftBumper().toggleOnTrue(
                Commands.startEnd(shooter::setFeedSpeed, shooter::stopShooter, shooter));

        operator.rightTrigger().and(operator.back().negate())
                .whileTrue(Commands.run(feeder::runFeeder, feeder))
                .whileFalse(Commands.run(feeder::stopFeeder, feeder));

        operator.rightTrigger().and(operator.back())
                .whileTrue(Commands.run(feeder::reverseFeeder, feeder))
                .whileFalse(Commands.run(feeder::stopFeeder, feeder));

        // Left Stick will run climber up or down
        /* operator.b()
                .whileTrue(Commands.run(climber::climberDown, climber))
                .whileFalse(Commands.run(climber::stopClimber, climber));
        operator.x()
                .whileTrue(Commands.run(climber::climberUp, climber))
                .whileFalse(Commands.run(climber::stopClimber, climber)); */

        operator.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.5)
                .whileTrue(Commands.run(elevator::elevatorDown, elevator))
                .whileFalse(Commands.run(elevator::stopElevator, elevator));
        operator.axisLessThan(XboxController.Axis.kLeftY.value, -0.5)
                .whileTrue(Commands.run(elevator::elevatorUp, elevator))
                .whileFalse(Commands.run(elevator::stopElevator, elevator));

        // POV will move Climber to set positions
        //operator.povUp().onTrue(Commands.run(climber::moveToTrapHeight, climber));
        operator.povUp().onTrue(Commands.run(elevator::moveToAmpHeight, elevator));

        //operator.povUp().onTrue(Commands.run(elevator::moveToTrapHeight, elevator));

        operator.povDown().onTrue(Commands.run(elevator::moveToZero, elevator));

        boolean useWristPIDControl = true;
        if (useWristPIDControl) {
            operator.y().onTrue(Commands.runOnce(wrist::moveToAmpHeight, wrist).beforeStarting(wrist::enable, wrist));
            //operator.x().onTrue(Commands.runOnce(wrist::movePodiumHeight, wrist).beforeStarting(wrist::enable, wrist));
            operator.a().onTrue(Commands.runOnce(wrist::moveToZero, wrist).beforeStarting(wrist::enable, wrist));
            operator.b().onTrue(Commands.runOnce(wrist::moveLevelHeight, wrist).beforeStarting(wrist::enable, wrist));

            /* // Right Stick will run Wrist up or down
            operator.axisGreaterThan(XboxController.Axis.kRightY.value, 0.5)
                    .whileTrue(Commands.run(wrist::wristDown, wrist))
                    .whileFalse(Commands.run(wrist::stopWrist, wrist));
            operator.axisLessThan(XboxController.Axis.kRightY.value, -0.5)
                    .whileTrue(Commands.run(wrist::wristUp, wrist))
                    .whileFalse(Commands.run(wrist::stopWrist, wrist)); */
        } else {
            // Right Stick will run Wrist up or down
            operator.axisGreaterThan(XboxController.Axis.kRightY.value, 0.5)
                    .whileTrue(Commands.run(wrist::wristDown, wrist))
                    .whileFalse(Commands.run(wrist::stopWrist, wrist));
            operator.axisLessThan(XboxController.Axis.kRightY.value, -0.5)
                    .whileTrue(Commands.run(wrist::wristUp, wrist))
                    .whileFalse(Commands.run(wrist::stopWrist, wrist));

            // Bottons to move wrist to set positions
            // operator.y().onTrue(
            //     Commands.run(wrist::moveToAmpHeight, wrist).alongWith(Commands.run(shooter::setAmpSpeed, shooter))
            // );


            operator.x().toggleOnTrue(
                Commands.startEnd(shooter::setAmpSpeed, shooter::stopShooter, shooter));

            operator.a().onTrue(Commands.run(wrist::moveToZero, wrist));
        }

        operator.y().onTrue(
            Commands.sequence(
                Commands.runOnce(wrist::moveToAmpHeight, wrist),
                Commands.runOnce(shooter::setAmpSpeed, shooter)
            )
        );

       /*  operator.x().onTrue(Commands.run(wristP::wristUp, wristP));
        operator.x().onTrue(Commands.run(wristP::movePodiumHeight, wristP)); */

        // Turn off auto aim for targeting
        // TODO: validate that this works properly
        operator.start()
            .toggleOnTrue(Commands.startEnd(
                () -> {
                    autoAimPublisher.set(false);
                }, 
                () -> {
                    autoAimPublisher.set(true);
                }
            ));

        // Turn off Stator Current Limit - WARNING - only to be done during climb or other extreme circumstances
        //operator.x().and(operator.b()).onTrue(Commands.runOnce(climber::setToClimbMode, climber));
    }

    private void configureCustomController() {
        Trigger wristOverride = customController.getClimberSwitch();
        wristOverride.toggleOnTrue(Commands.runOnce(wrist::disablePID, (Subsystem)wrist))
                     .toggleOnFalse(Commands.runOnce(wrist::enablePID, (Subsystem)wrist));
    }

    // TODO: test sysid of swerve
    private void configureCharacterizationController() {
        //SignalLogger.enableAutoLogging(true);
        //SignalLogger.start();
        /* Bindings for drivetrain characterization */
        /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
        /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
        characterizer.back().and(characterizer.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        characterizer.back().and(characterizer.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        characterizer.start().and(characterizer.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        characterizer.start().and(characterizer.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Setup Shooter bindings
        characterizer.start().and(characterizer.a())
                .whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        characterizer.start().and(characterizer.b())
                .whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        characterizer.back().and(characterizer.a())
                .whileTrue(shooter.sysIdDynamic(Direction.kForward));
        characterizer.back().and(characterizer.b())
                .whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    }

    private void configureNetworkTable() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);

        BooleanTopic autoAimTopic = fmjTable.getBooleanTopic(Constants.NT_AUTO_AIM);
        autoAimPublisher = autoAimTopic.publish();
        autoAimPublisher.setDefault(true);
        autoAimSubscriber = autoAimTopic.subscribe(true);

        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        beamBooleanPublisher = beamTopic.publish();
        beamBooleanPublisher.setDefault(false);
        beamBreakSubscriber = beamTopic.subscribe(false);

        BooleanTopic climberTopic = fmjTable.getBooleanTopic(Constants.NT_CLIMBER_LIMIT);
        climberLimitPublisher = climberTopic.publish();
        climberLimitPublisher.setDefault(false);
        climberLimitSubscriber = climberTopic.subscribe(false);

        BooleanTopic wristTopic = fmjTable.getBooleanTopic(Constants.NT_WRIST_LIMIT);
        wristLimitPublisher = wristTopic.publish();
        wristLimitPublisher.setDefault(false);
        wristLimitSubscriber = wristTopic.subscribe(false);

        DoubleTopic tagTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_PID_VALUE);
        limelightTagPublisher = tagTopic.publish();
        limelightTagPublisher.setDefault(0);
        tagControllerSub = tagTopic.subscribe(0);

        BooleanTopic tagRangeTopic = fmjTable.getBooleanTopic(Constants.NT_TAG_LL_TAG_RANGE);
        tagRangePublisher = tagRangeTopic.publish();
        tagRangePublisher.setDefault(false);
        tagRangeSubscriber = tagRangeTopic.subscribe(false);

        DoubleTopic tagDistanceTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_DISTANCE);
        tagDistancePublisher = tagDistanceTopic.publish();
        tagDistancePublisher.setDefault(0);

        DoubleTopic noteTopic = fmjTable.getDoubleTopic(Constants.NT_NOTE_LL_PID_VALUE);
        limelightNotePublisher = noteTopic.publish();
        limelightNotePublisher.setDefault(0);
        noteControllerSub = noteTopic.subscribe(0);

        BooleanTopic noteRangeTopic = fmjTable.getBooleanTopic(Constants.NT_TAG_LL_NOTE_RANGE);
        noteRangePublisher = noteRangeTopic.publish();
        noteRangePublisher.setDefault(false);
        noteRangeSubscriber = noteRangeTopic.subscribe(false);

        DoubleTopic noteDistanceTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_NOTE_DISTANCE);
        noteDistancePublisher = noteDistanceTopic.publish();
        noteDistancePublisher.setDefault(0.0);

        BooleanTopic climberHeightTopic = fmjTable.getBooleanTopic("climberHeight");
        climberHeightPublisher = climberHeightTopic.publish();
        climberHeightPublisher.setDefault(false);
    }

    private void configureAutonomousOptions() {
        // Add Autonomous Options to Dashboard
        ShuffleboardTab robotTab = Shuffleboard.getTab("FMJ Robot");
        SendableRegistry.setName(autonChooser, "Autonomous Options");
        boolean redPath = true;
        boolean bluePath = false;

        // Create Option 1 and add to list of choices
        //     drivetrain.generateAutoPath(2.0, 0.0)
        // );

        // Create Option 2 and add to list of choices
        FourNoteLeave = new FourNoteLeave(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("4 Note Blue", FourNoteLeave);
        FourNoteLeaveRed = new FourNoteLeave(this, 0.0, 0.0, redPath);
        autonChooser.addOption("4 Note Red", FourNoteLeaveRed);

        TwoNoteLeave = new TwoNoteLeave(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Two Note Leave Blue", TwoNoteLeave);
        TwoNoteLeaveRed = new TwoNoteLeave(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Two Note Leave Red", TwoNoteLeaveRed);

        TwoNoteMidRedA4A = new TwoNoteMidRedA4A(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Two Note Mid Blue A4A", TwoNoteMidRedA4A);
        TwoNoteMidRedA4ARed = new TwoNoteMidRedA4A(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Two note Mid Red A4A", TwoNoteMidRedA4ARed);

        TwoNoteMidRedA5A = new TwoNoteMidRedA5A(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Two Note Mid Blue A5A", TwoNoteMidRedA5A);
        TwoNoteMidRedA5ARed = new TwoNoteMidRedA5A(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Two note Mid Red A5A", TwoNoteMidRedA5ARed);

        MidLineSourceClear = new MidLineSourceClear(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Blue I love keller", MidLineSourceClear);
        MidLineSourceClearRed = new MidLineSourceClear(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Red I love keller", MidLineSourceClearRed);

        FiveNote = new FiveNoteCenter(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Five Note", FiveNote);
        FiveNoteRed = new FiveNoteCenter(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Five Note Red", FiveNoteRed);

        ShooterTest = new ShooterTest(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Shooter Test Blue", ShooterTest);
        ShooterTestRed = new ShooterTest(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Shooter Test Red", ShooterTestRed);

        AutoAimTest = new AutoAimTest(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Auto Aim Test Blue", AutoAimTest);
        AutoAimTestRed = new AutoAimTest(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Auto Aim Test Red", AutoAimTestRed);

        AmpSideAutoAim = new AmpSideAutoAim(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Amp Side Auto Aim Blue", AmpSideAutoAim);
        AmpSideAutoAimRed = new AmpSideAutoAim(this, 0.0, 0.0, redPath);
        autonChooser.addOption("Amp Side Auto Aim Red", AmpSideAutoAimRed);


        
        /* KeatonAuton = new KeatonAuton(this, 0.0, 0.0, bluePath);
        autonChooser.addOption("Good Luck by Keaton", KeatonAuton); */


        //     drivetrain.generateAutoPath(2.0, 0.0),
        //     drivetrain.generateAutoPath(0.0, 2.0)
        // );
        // autonChooser.addOption("Drive 2m X & Y directions", autonOption2);

        // Create Option 3 and add to list of choices 

        ShuffleboardLayout autonLayout = robotTab.getLayout("Autonomous", BuiltInLayouts.kList)
                .withSize(2, 1).withPosition(0, 0);
        autonLayout.add(autonChooser).withSize(4, 1).withPosition(0, 0);

        // Add rotational override
        PPHolonomicDriveController.setRotationTargetOverride(null);
    }

    private void configureShuffleboard() {
        ShuffleboardTab robotTab = Shuffleboard.getTab("FMJ Robot");
        robotTab.add(field2d).withWidget(BuiltInWidgets.kField).withSize(8, 5);

        robotTab.addDouble("Shoot Speed", shooter::getShooterSpeed).withWidget(BuiltInWidgets.kDial)
                .withSize(2, 2).withProperties(Map.of("min", 0, "max", 40));

        //robotTab.add(wrist);
    }

    public Command getAutonomousCommand() {
        Command auton = autonChooser.getSelected();

        Pose2d initialPose;
        if (auton instanceof IAuto) {
            initialPose = ((IAuto) auton).getInitialPose();
        } else {
            initialPose = new Pose2d();
        }
        drivetrain.initializePoseForAutonomous(initialPose);

        return auton;
    }

    public void robotPeriodic() {
        // System.out.println("April Tag(tx) - '" + limelightTarget.getLimelightX() + "'");
        beamBooleanPublisher.set(!beamBreak.get());
        //climberLimitPublisher.set(!climberLimit.get());
        //wristLimitPublisher.set(!wristLimit.get());

        // Publish Tag calculated values to Network Table
        limelightTagPublisher.set(limelightTarget.getLimelightX());
        tagRangePublisher.set(limelightTarget.isWithinRange());
        tagDistancePublisher.set(limelightTarget.calculateDistanceToTarget());

        // Publish Note calculated values to Network Table
        limelightNotePublisher.set(limelightNote.getLimelightX());
        noteRangePublisher.set(limelightNote.isWithinRange());
        noteDistancePublisher.set(limelightNote.calculateDistanceToTarget());

        // Publish whether climber down or not
        //climberHeightPublisher.set(!climberLimit.get());

        field2d.setRobotPose(drivetrain.getState().Pose);

    }

    public void autonomousInit() {
        autoAimPublisher.set(false);
        PPHolonomicDriveController.setRotationTargetOverride(new Rotation2dSupplier(false));
    }

    public void autonomousPeriodic() {}

    public void teleopInit() {
        autoAimPublisher.set(false);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance.equals(Alliance.Blue)) {
            drivetrain.seedFieldRelative(new Pose2d(), 0.0);
        } else {
            drivetrain.seedFieldRelative(new Pose2d(), 180.0);
        }
    }

    public void teleopPeriodic() {}

    public ClimberSubsystem getClimberSubsystem() {
        return this.climber;
    }

    public CommandSwerveDrivetrain getDriveSubsystem() {
        return this.drivetrain;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return this.intake;
    }

    public FeederSubsystem getFeederSubsystem() {
        return this.feeder;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return this.shooter;
    }

    public IWrist getWristSubsystem() {
        return wrist;
    }

    public int getAmpTagNumber() {
        return ampTagNumber;
    }

    public int getSpeakerTagNumber() {
        return speakerTagNumber;
    }
}
