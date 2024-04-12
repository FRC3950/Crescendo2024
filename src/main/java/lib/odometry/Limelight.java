// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.Swerve;
import lib.odometry.LimelightHelpers.Results;

public class Limelight {

    private static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("");

    public static Results limelightResults;
    public static Pose2d llPose;

    static {
        limelightResults = LimelightHelpers.getLatestResults("limelight").targetingResults;
        llPose = limelightResults.getBotPose2d_wpiBlue();

        // int[] validIds = {7, 8};

        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIds);
    }

    private Limelight() {}

    public static void blink() {
        limelightTable.getEntry("ledMode").setNumber(2);
    }

    public static double getTx() {
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

    public static double getTag() {
        return limelightTable.getValue("tid").getDouble();
    }

    public static Results getResults() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults;
    }

    public static double[] getMegaPose2(){
        return limelightTable.getValue("botpose_orb_wpiblue").getDoubleArray();
    }

    public static void updateRotation(Swerve drive) {
        LimelightHelpers.SetRobotOrientation("limelight", drive.getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    public static boolean validRotationRate(Swerve drive) {
        return drive.getPigeon2().getRate() < 720;
    }

    //Likely unnecessary if we are only use mega tag 2
    public static boolean canUpdatePose(Swerve drive) {
        if(validRotationRate(drive)){
            limelightResults = getResults();
            llPose = limelightResults.getBotPose3d_wpiBlue().toPose2d();

            var validResults = llPose != null && limelightResults != null;
            var withinDistance = true;
            
            //drive.getState().Pose.getX() < 4.5 || drive.getState().Pose.getX() > 12.55;

            return validResults && withinDistance;
        }

        return false;
    }

    public static void updatePose(Swerve drive) {
        if(true){//canUpdatePose(drive)
            var mt2Pose = getMt2Pose(drive);

            if(Math.abs(drive.getPigeon2().getRate()) > 720){
                return;
            };

            if(mt2Pose.tagCount < 1){
                return;
            }


            //These  are inhertly the same, but... 
            // drive.getSwerveDrivePoseEstimator().getEstimatedPosition().getRotation().getDegrees()
            // drive.getState().Pose.getRotation().getDegrees();

            // drive.getSwerveDrivePoseEstimator().update(drive.getState().Pose.getRotation(), );
            
            drive.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 99999999));

            if(drive.getState().Pose.getTranslation().getDistance(mt2Pose.pose.getTranslation())<2.0){

                drive.addVisionMeasurement(mt2Pose.pose, mt2Pose.timestampSeconds);
            }
        }
    } 

    public static LimelightHelpers.PoseEstimate getMt2Pose(Swerve drive) {
                    Limelight.updateRotation(drive);

        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    public static void updateHeadingMt1(Swerve drive) {
        var mt1Pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        drive.addVisionMeasurement(mt1Pose, Timer.getFPGATimestamp());
    }   
}
