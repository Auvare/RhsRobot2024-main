// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class ProfiledWristSubsystem extends ProfiledPIDSubsystem implements IWrist {
    private TalonFX wristMotor = new TalonFX(Constants.CAN_WRIST);

    private DutyCycleEncoder absShaftEncoder;
    private static final double kMaxVelocity = 2.0; //20
    private static final double kMaxAcceleration = 1.0; //10
    private static TrapezoidProfile.Constraints profileConstraints =
            new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private ArmFeedforward feedForword;

    private DutyCycleOut dcOut = new DutyCycleOut(0.0);
    private double WRISTUP_SPEED = -0.20;
    private double WRISTDOWN_SPEED = 0.20;

    public static double MIN_ABSOLUTE_POSITION = 0.655;
    public static double PODIUM_POSITION = 0.765;
    public static double LEVEL_POSITION = 0.81;
    public static double AMP_POSITION = 0.93;
    public static double MAX_ABSOLUTE_POSITION = 0.935;

    private double INCREMENT = 0.05;

    // Feed Forward Constants
    // Calculated using https://www.reca.lc/arm
    //      Motor: 1 Falcon 500 (FOC)
    //      Ratio: 100 [Reduction]
    //      Efficiency(%): 87
    //      Current Limit: 30 (A) [this is the stator current]
    //      CoM Distance: 10 (inches)
    //      Arm Mass: 10 (lbs)
    //      Start Angle: 0 (deg)
    //      End Angle: 90 (deg)
    //      Iteration Limit: 10000
    private final double currentLimit = 30.0;
    private final double kG = 0.40;
    private final double kV = 1.88;
    private final double kA = 0.01;
    private final double kS = 0.0; // Not generated from above website so set to zero

    // PID Controller constants
    // Calculated using sysid tool
    //      Feedforward Gains (Theoretical)
    //          Kv = 1.88
    //          Ka = 0.01 
    //          Response Timescale = 5.3191
    //      Feedback Analysis
    //          Gain Preset: CRE (Pro)
    //          Max Controller Output: 12.0
    //          Velocity Denomintator Units: 0.1
    //          Controller Period: 1
    //          Time-normalized: true
    //          Measurement delay: 20
    //      Loop Type: Position
    //      Kp = 5.9887
    //      Kd = 2.4394
    //      Max Position Error: 1
    //      Max Velocity Error: 1.5
    //      Max Control Effort: 7

    private static final double kP = 56.0; // 5.9887
    private static final double kI = 0.0;
    private static final double kD = 2.0; //2.4394

    private static ProfiledPIDController pidController =
            new ProfiledPIDController(kP, kI, kD, profileConstraints);

    /** Creates a new ProfiledWristSubsystem. */
    public ProfiledWristSubsystem() {
        super(pidController, MIN_ABSOLUTE_POSITION);

        MotorOutputConfigs currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs statorLimitConfig = new CurrentLimitsConfigs();
        statorLimitConfig.StatorCurrentLimitEnable = true;
        statorLimitConfig.StatorCurrentLimit = currentLimit;

        TalonFXConfiguration talonConfig = new TalonFXConfiguration()
                .withMotorOutput(currentConfigs)
                .withCurrentLimits(statorLimitConfig)
                .withAudio(Constants.ORCHESTRA_CONFIGS);
        wristMotor.getConfigurator().apply(talonConfig);

        feedForword = new ArmFeedforward(kS, kG, kV);

        //pidController.setTolerance(1.0);
        configureAbsoluteEncoder();
        setGoal(MIN_ABSOLUTE_POSITION);
        this.enable();
    }

    private void configureAbsoluteEncoder() {
        // Absolution Encoder
        absShaftEncoder = new DutyCycleEncoder(Constants.DP_WRIST_ENCODER);

        // Absolute Pulse Output (Duty Cycle)
        //      Period: 1025us
        //      Freq  : 975.6Hz
        //      Min Pulse(0deg)   : 1us
        //      Max Pulse(360deg) : 1024us
        //      Pulse Resolution  : 10-bit
        // 360deg = 2 * pi
        absShaftEncoder.setDistancePerRotation(2 * Math.PI);

        absShaftEncoder.setDutyCycleRange(0, 1);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putBoolean("Wrist PID Enabled", this.isEnabled());
        SmartDashboard.putNumber("Current Absolute Pos", absShaftEncoder.getAbsolutePosition());
        
        Command currCmd = this.getCurrentCommand();
        String currCmdStr = "-none-";
        if (currCmd != null) {
            currCmdStr = currCmd.getName();
        }
        SmartDashboard.putString("Wrist Command", currCmdStr);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        if (this.isEnabled()) {
            // Calculate the feedforward from the sepoint
            // Add the feedforward to the PID output to get the motor output
            double vValue = -output;
            if (output >= 0) {
                // Calculate the feedforward from the sepoint
                // Add the feedforward to the PID output to get the motor output
                double feedforward = -feedForword.calculate(setpoint.position, setpoint.velocity);
                vValue = -output - feedforward;
            }
            wristMotor.setVoltage(vValue);
            SmartDashboard.putNumber("voltage", vValue);
        }
    }

    @Override
    protected double getMeasurement() {
        return absShaftEncoder.getAbsolutePosition();
    }

    public void moveToZero() {
        setGoal(MIN_ABSOLUTE_POSITION);
    }

    public void movePodiumHeight() {
        setGoal(PODIUM_POSITION);
    }

    public void moveLevelHeight() {
        setGoal(LEVEL_POSITION);
    }

    public void moveToAmpHeight() {
        setGoal(AMP_POSITION);
    }

    public void moveToPosition(double position) {
        if (position <= MIN_ABSOLUTE_POSITION) {
            position = MIN_ABSOLUTE_POSITION;
        } else if (position >= MAX_ABSOLUTE_POSITION) {
            position = MAX_ABSOLUTE_POSITION;
        }
        setGoal(position);
    }

    public void moveUp() {
        double newGoal = this.getController().getGoal().position + INCREMENT;
        setGoal(MathUtil.clamp(newGoal, MIN_ABSOLUTE_POSITION, MAX_ABSOLUTE_POSITION));
    }

    public void moveDown() {
        double newGoal = this.getController().getGoal().position - INCREMENT;
        setGoal(MathUtil.clamp(newGoal, MIN_ABSOLUTE_POSITION, MAX_ABSOLUTE_POSITION));
    }

    public void wristUp() {
        if (this.isEnabled() == false) {
            wristMotor.setControl(dcOut.withOutput(WRISTUP_SPEED));
        }
    }

    public void stopWrist() {
        if (this.isEnabled() == false) {
            wristMotor.setControl(dcOut.withOutput(0.0));
        }
    }

    public void wristDown() {
        if (this.isEnabled() == false) {
            wristMotor.setControl(dcOut.withOutput(WRISTDOWN_SPEED));
        }
    }

    public void holdWristDownForIntake() {
        // do nothing
    }

    public void disablePID() {
        this.disable();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        wristMotor.getConfigurator().apply(motorConfigs);
    }

    public void enablePID() {
        this.enable();
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        wristMotor.getConfigurator().apply(motorConfigs);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Wrist");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
            ".default",
            () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
            null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
            ".command",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
            null);
    }
}
