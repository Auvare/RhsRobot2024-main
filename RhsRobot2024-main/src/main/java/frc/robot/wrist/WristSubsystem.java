// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// WRIST RUNS POSTITIONAL CONTROL

package frc.robot.wrist;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FMJRobot;

public class WristSubsystem extends SubsystemBase implements IWrist {
    /** Creates a new WristSubsystem. */
    private TalonFX wristMotor;

    private double WRISTUP_SPEED = -0.10;
    private double WRISTDOWN_SPEED = 0.10;
    private double STATOR_DOWN = 10;
    private double STATOR_CONTINOUS_DOWN = 10;

    private double AMP_POSITION = -16.0;

    private DutyCycleOut dcOut = new DutyCycleOut(0);
    private MotionMagicDutyCycle mmOut = new MotionMagicDutyCycle(0);

    CurrentLimitsConfigs statorLimitConfigUp;
    CurrentLimitsConfigs statorLimitConfigDown;
    CurrentLimitsConfigs statorLimitConfigHold;
    private DutyCycleEncoder shaftEncoder;
    private double MIN_ABSOLUTE_POSITION = 0.0;
    private double MID_ABSOLUTE_POSTITON = 56.0;
    private double MAX_ABSOLUTE_POSITION = 90.0;

    private DutyCycleEncoder m_dutyCycleEncoder;

    private static final double kMaxVelocity = 1.75;
    private static final double kMaxAcceleration = 0.75;
    private static TrapezoidProfile.Constraints profileConstraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private ProfiledPIDController profileController;
    private ArmFeedforward feedForword;
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
    private final double kG = 0.20;
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

    private static final double kP = 5.9887;
    private static final double kI = 0.0;
    private static final double kD = 2.4394;

    private boolean enabled = false;
    private double goal = -1;
    private boolean connected = false;

    // 56 deg for down
    // -34 deg for Amp Angle
    public WristSubsystem() {
        profileController = new ProfiledPIDController(kP, kI, kD, profileConstraints);
        feedForword = new ArmFeedforward(kS, kG, kV);

        /*
         * Documentation from CTRE
            In a Position closed loop, the gains should be configured as follows:
        
            kS - unused, as there is no target velocity
            kV - unused, as there is no target velocity
            kA - unused, as there is no target acceleration
            kP - output per unit of error in position (output/rotation)
            kI - output per unit of integrated error in position (output/(rotation*s))
            kD - output per unit of error derivative in position (output/rps)
         */
        wristMotor = new TalonFX(Constants.CAN_WRIST);

        MotorOutputConfigs currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Brake;

        /* HardwareLimitSwitchConfigs hwLimitConfig = new HardwareLimitSwitchConfigs();
        hwLimitConfig.ReverseLimitAutosetPositionEnable = true;
        hwLimitConfig.ReverseLimitAutosetPositionValue = 0.0;
        hwLimitConfig.ForwardLimitAutosetPositionEnable = true;
        hwLimitConfig.ForwardLimitAutosetPositionValue = 0.0; */

        //SoftwareLimitSwitchConfigs swLimitConfig = new SoftwareLimitSwitchConfigs();
        //swLimitConfig.ForwardSoftLimitEnable = true;
        //swLimitConfig.ForwardSoftLimitThreshold = 0;
        //swLimitConfig.ReverseSoftLimitEnable = true;
        //swLimitConfig.ReverseSoftLimitThreshold = -21;

        statorLimitConfigUp = new CurrentLimitsConfigs();
        statorLimitConfigUp.StatorCurrentLimitEnable = true;
        statorLimitConfigUp.StatorCurrentLimit = 12;
        statorLimitConfigDown = new CurrentLimitsConfigs();
        statorLimitConfigDown.StatorCurrentLimitEnable = true;
        statorLimitConfigDown.StatorCurrentLimit = STATOR_DOWN;
        statorLimitConfigHold = new CurrentLimitsConfigs();
        statorLimitConfigHold.StatorCurrentLimitEnable = true;
        statorLimitConfigHold.StatorCurrentLimit = STATOR_CONTINOUS_DOWN;


        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.16;
        slot0Configs.kS = 0.1; //TODO: why are we setting this
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
        mmConfigs.MotionMagicAcceleration = 75.0;
        mmConfigs.MotionMagicCruiseVelocity = 150;

        TalonFXConfiguration talonConfig = new TalonFXConfiguration()
                .withMotorOutput(currentConfigs)
                //.withHardwareLimitSwitch(hwLimitConfig)
                .withSlot0(slot0Configs).withMotionMagic(mmConfigs)
                //.withSoftwareLimitSwitch(swLimitConfig)
                .withCurrentLimits(statorLimitConfigUp).withAudio(Constants.ORCHESTRA_CONFIGS);

        wristMotor.getConfigurator().apply(talonConfig);
        wristMotor.setPosition(0.0);

        mmOut.Slot = 0;

        FMJRobot.FMJOrchestra.addInstrument(wristMotor);

        //configureShaftEncoder();

        /* m_dutyCycleEncoder = new DutyCycleEncoder(Constants.DP_WRIST_ENCODER);
        // Set to 0.5 units per rotation
        m_dutyCycleEncoder.setDistancePerRotation(360.0);

        connected = m_dutyCycleEncoder.isConnected();

        if (connected == false) {
            System.out.println(" Not CONNECTED");
        }

        double distance = m_dutyCycleEncoder.getDistance();

        int frequency = m_dutyCycleEncoder.getFrequency();

        double output = m_dutyCycleEncoder.get();

        SmartDashboard.putBoolean("Connected", connected);
        SmartDashboard.putNumber("Frequency", frequency);
        SmartDashboard.putNumber("Output", output);
        SmartDashboard.putNumber("Distance", distance); */
    }
    
    /*  private void configureShaftEncoder() {
        shaftEncoder = new DutyCycleEncoder(Constants.DP);
        if (shaftEncoder.isConnected()) {
            shaftEncoder.reset();
        }
    } */

    @Override
    public void initSendable(SendableBuilder builder) {
        shaftEncoder.initSendable(builder);
    }

    @Override
    public void periodic() {
        /* double currPos = m_dutyCycleEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("Current Absolute Pos", currPos); */

        // if(enabled) {
        // }
        /* if (connected == true) {
            if (distance >= MID_ABSOLUTE_POSTITON) {
                WRISTUP_SPEED = 0.0;
            } else {
                WRISTUP_SPEED = 0.2;
            }

            if (distance <= MIN_ABSOLUTE_POSITION) {
                WRISTDOWN_SPEED = 0.0;
            } else {
                WRISTDOWN_SPEED = -0.2;
            }
        } */
    }

    public void wristUp() {
        wristMotor.getConfigurator().apply(statorLimitConfigUp);
        wristMotor.setControl(dcOut.withOutput(WRISTUP_SPEED));
    }

    public void stopWrist() {
        wristMotor.setControl(dcOut.withOutput(0.0));
    }

    public void wristDown() {
        wristMotor.getConfigurator().apply(statorLimitConfigDown);
        wristMotor.setControl(dcOut.withOutput(WRISTDOWN_SPEED));
    }

    public void holdWristDownForIntake() {
        wristMotor.getConfigurator().apply(statorLimitConfigHold);

        wristMotor.setControl(dcOut.withOutput(0.20));
    }

    public void moveToAmpHeight() {
        wristMotor.getConfigurator().apply(statorLimitConfigUp);
        wristMotor.setControl(mmOut.withPosition(AMP_POSITION));
    }

    public void moveToZero() {
        wristMotor.getConfigurator().apply(statorLimitConfigDown);
        // Moving to Positon 2, this will cause wrist to move past the zero point 
        // thus insuring that we trip the limit switch
        wristMotor.setControl(mmOut.withPosition(0));
    }

    public double getPos() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public void moveToPosNeg5() {
        wristMotor.getConfigurator().apply(statorLimitConfigUp);
        wristMotor.setControl(mmOut.withPosition(-5));
    }

    public void resetPosition() {
        wristMotor.setPosition(0.0);
    }

    public void enable() {
        // do nothing.
    }

    @Override
    public void moveToPosition(double position) {
        // do nothing
        
    }
}
