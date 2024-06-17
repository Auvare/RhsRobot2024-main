// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

/** Add your docs here. */
public class Rotation2dSupplier implements Supplier<Optional<Rotation2d>> {
    private BooleanSubscriber beamBreakSubscriber;
    private BooleanSubscriber tagRangeSubscriber;
    private BooleanSubscriber noteRangeSubscriber;
    private DoubleSubscriber tagControllerSub;
    private DoubleSubscriber noteControllerSub;
    private DoubleSubscriber yawSubscriber;
    private boolean outputDebug = false;

    public Rotation2dSupplier() {
        this(false);
    }

    public Rotation2dSupplier(boolean debug) {
        outputDebug = debug;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fmjTable = inst.getTable(Constants.NETWORK_TABLE);

        BooleanTopic beamTopic = fmjTable.getBooleanTopic(Constants.NT_INTAKE_BEAM_BROKEN);
        beamBreakSubscriber = beamTopic.subscribe(false);

        DoubleTopic tagTopic = fmjTable.getDoubleTopic(Constants.NT_TAG_LL_PID_VALUE);
        tagControllerSub = tagTopic.subscribe(0);

        BooleanTopic tagRangeTopic = fmjTable.getBooleanTopic(Constants.NT_TAG_LL_TAG_RANGE);
        tagRangeSubscriber = tagRangeTopic.subscribe(false);

        DoubleTopic noteTopic = fmjTable.getDoubleTopic(Constants.NT_NOTE_LL_PID_VALUE);
        noteControllerSub = noteTopic.subscribe(0);

        BooleanTopic noteRangeTopic = fmjTable.getBooleanTopic(Constants.NT_TAG_LL_NOTE_RANGE);
        noteRangeSubscriber = noteRangeTopic.subscribe(false);

        DoubleTopic yawTopic = fmjTable.getDoubleTopic(Constants.NT_YAW);
        yawSubscriber = yawTopic.subscribe(0);
    }

    @Override
    public Optional<Rotation2d> get() {
        Optional<Rotation2d> result = Optional.ofNullable(null);
        // If we are in range of note
        if (noteRangeSubscriber.get() && !beamBreakSubscriber.get()) {
            if(outputDebug) {System.out.println("------------> Note is within Range <------------");}
            double angleAdjustment = 10.0; // if off target then bias the current heading by 5 degrees

            double controllerInput = noteControllerSub.get();
            double currYaw = yawSubscriber.get();
            // Use PID value from network table to determine if adjustment is necessary
            if (controllerInput > 0.1) {
                if(outputDebug) {System.out.println("------------> Input is positive <------------");}
                // Need to get current heading and adjust to desired heading
                result = Optional.of(Rotation2d.fromDegrees(currYaw + angleAdjustment));
            } else if (controllerInput < -0.1) {
                if(outputDebug) {System.out.println("------------> Input is negative <------------");}
                result = Optional.of(Rotation2d.fromDegrees(currYaw - angleAdjustment));
            }
        }

        return result;
    }
}
