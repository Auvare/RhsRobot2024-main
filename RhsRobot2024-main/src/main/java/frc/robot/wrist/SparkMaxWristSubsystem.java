// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SparkMaxWristSubsystem extends SubsystemBase {
    public CANSparkMax wristMotor;
    private final AbsoluteEncoder absoluteShaftEncoder;
    //private final RelativeEncoder armEncoder;
    private SparkPIDController wristPIDController;

    private double WRISTUP_SPEED = -0.20;
    private double WRISTDOWN_SPEED = 0.10;

    private double INTAKE_POSITION = 0;
    private double AMP_POSITION = 0;

    // Feed Forward Constants
    // Calculated using https://www.reca.lc/arm
    //      Motor: 1 775Pro
    //      Ratio: 100 [Reduction]
    //      Efficiency(%): 85
    //      Current Limit: 30 (A) [this is the stator current]
    //      CoM Distance: 10 (inches)
    //      Arm Mass: 10 (lbs)
    //      Start Angle: 0 (deg)
    //      End Angle: 90 (deg)
    //      Iteration Limit: 10000
    private final double currentLimit = 30.0;
    private final double kG = 1.62;
    private final double kV = 0.61;
    private final double kA = 0.06;
    private final double kS = 0.0; // Not generated from above website so set to zero

    // PID Controller constants
    // Calculated using sysid tool
    //      Feedforward Gains (Theoretical)
    //          Kv = 0.61
    //          Ka = 0.06 
    //          Response Timescale = 98.361 (this looks wrong but not sure what is up)
    //      Feedback Analysis
    //          Gain Preset: REV Brushed Encoder
    //          Max Controller Output: 1.0
    //          Velocity Denomintator Units: 60.0
    //          Controller Period: 1
    //          Time-normalized: false
    //          Measurement delay: 81.5
    //      Loop Type: Position
    //      Kp = 0.56335
    //      Kd = 0.33533
    //      Max Position Error: 1
    //      Max Velocity Error: 1.5
    //      Max Control Effort: 7

    private static final double kP = 0.56335;
    private static final double kI = 0.0;
    private static final double kD = 0.33533;

    private static final double kIz = 0.0;
    private static final double kFF = 0.0;
    private static final double kMaxOutput = 0.85;
    private static final double kMinOutput = -0.85;

    private static final double maxVelocity = 1000; // rpm
    private static final double maxAcceleration = 500;
    private static final double minVelocity = 0;
    private static final double allowedError = 0;


    /** Creates a new SparkMaxWristSubsystem. */
    public SparkMaxWristSubsystem() {
        wristMotor = new CANSparkMax(Constants.CAN_WRIST, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();
        wristPIDController = wristMotor.getPIDController();

        absoluteShaftEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristPIDController.setFeedbackDevice(absoluteShaftEncoder);

        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(true);
        wristMotor.setSmartCurrentLimit(25); // TODO: need to valide that this works for us
        absoluteShaftEncoder.setZeroOffset(0.2721140); // TODO: we need to get the real value for our robot

        wristPIDController.setP(kP);
        wristPIDController.setI(kI);
        wristPIDController.setD(kD);
        wristPIDController.setIZone(kIz);
        wristPIDController.setFF(kFF);
        wristPIDController.setOutputRange(kMinOutput, kMaxOutput);

        wristPIDController.setPositionPIDWrappingEnabled(false);
        //wristPIDController.setPositionPIDWrappingMinInput(0);
        //wristPIDController.setPositionPIDWrappingMaxInput(1);

        /**
         * Smart Motion coefficients are set on a SparkPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        wristPIDController.setSmartMotionMaxVelocity(maxVelocity, smartMotionSlot);
        wristPIDController.setSmartMotionMinOutputVelocity(minVelocity, smartMotionSlot);
        wristPIDController.setSmartMotionMaxAccel(maxAcceleration, smartMotionSlot);
        wristPIDController.setSmartMotionAllowedClosedLoopError(allowedError, smartMotionSlot);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm", getAbsoluteEncoderPosition());
    }

    public void wristUp() {
        wristMotor.set(WRISTUP_SPEED);   
    }

    public void wristDown() {
        wristMotor.set(WRISTDOWN_SPEED);   
    }

    public void stopWrist() {
        wristMotor.stopMotor();
    }

    public void setPosition(double position) {
        wristPIDController.setReference(position, ControlType.kSmartMotion);
    }

    public double getAbsoluteEncoderPosition() {
        return absoluteShaftEncoder.getPosition();
    }

    public void moveToAmpAngle() {
        setPosition(AMP_POSITION);
    }

    public void moveToIntakeAngle() {
        setPosition(INTAKE_POSITION);
    }
}
