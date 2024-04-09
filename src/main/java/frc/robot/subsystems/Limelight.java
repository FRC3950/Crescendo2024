// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.odometry.LimelightHelpers;
import lib.odometry.LimelightHelpers.Results;

public class Limelight extends SubsystemBase {
    /**
     * Creates a new Limelight.
     */

    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("");

    private static Limelight instance;

    public static Results limelightResults;
    public static Pose2d llPose;

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    private Limelight() {
    }

    public double getTx() {
        // Only pull tx value for tag id #7
        //limelightTable.getValue(getName())
        var results = limelightResults;

        for (LimelightHelpers.LimelightTarget_Fiducial result : results.targets_Fiducials) {
            if (result.fiducialID == 7 || result.fiducialID == 4) {
                return result.tx;
            }
        }

        return 0;
    }

    public double getTag() {
        return limelightTable.getValue("tid").getDouble();
    }

    public void blink() {
        limelightTable.getEntry("ledMode").setNumber(2);
    }

    @Override
    public void periodic() {

        limelightResults = LimelightHelpers.getLatestResults("limelight").targetingResults;
        llPose = limelightResults.getBotPose3d_wpiBlue().toPose2d();

        // This method will be called once per scheduler run
    }
}
