package lib.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

public final class NoteKinematics {

    public static final Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(0));
    public static final Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));

    public static DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    public static final Pose2d activeSpeaker;

    static {
        activeSpeaker = alliance.equals(DriverStation.Alliance.Red) ? redSpeaker : blueSpeaker;
    }

    public static double getHeadingAdjustment(Pose2d botPose, Pose2d speakerPose) {
        var speakerDistance = botPose.getTranslation().getDistance(speakerPose.getTranslation());

        return Math.acos(botPose.getX() / speakerDistance);
    }

    public static double getLobPivot(DoubleSupplier distance) {
        return (Math.PI/2 - Math.asin(Constants.Physics.yVelocity/getVelocityVector(distance))) * (0.25/(Math.PI/2));
    }

    public static double getAllianceSpeakerDistance(Swerve drive){
        return drive.getState().Pose.getTranslation().getDistance(NoteKinematics.activeSpeaker.getTranslation());
    }

    public static double getVelocityVector(DoubleSupplier distance) {
        var d = distance.getAsDouble();
        var velocityVector = Math.hypot(d/Constants.Physics.airTime, Constants.Physics.yVelocity);

        return velocityVector;
    }

    public static double getLobVelocityRps(DoubleSupplier distance) {

        var rpsMsConversion = (187.978/60);

        return rpsMsConversion * getVelocityVector(distance);
    }

    public static double getHeadingDifference(Pose2d activeSpeaker, Pose2d botPose) {

        var xDistance = botPose.getTranslation().getX() - activeSpeaker.getX();
        var yDistance = botPose.getTranslation().getY() - activeSpeaker.getY();

        var targetAngle = Math.atan2(yDistance, xDistance);
        var currentAngle = botPose.getRotation().getRadians();

        return currentAngle - targetAngle;
    }

    public static double getTargetPivot(DoubleSupplier distance) {
        var gravityComp = distance.getAsDouble() < 3.99 ?
            0.01 * Math.pow(distance.getAsDouble(), 2):
            0.019 * Math.pow(distance.getAsDouble(), 2);


        //90 -arcTan2 (SpeakerHeight - pivotAngleHeight / RobotDistance)
        // SmartDashboard.putNumber("BridgwoodDistance", distance.getAsDouble());
        return 0.25 - ((Math.atan2(Constants.Physics.targetSpeakerHeight + gravityComp, distance.getAsDouble()) * (180 / Math.PI)) / 360);
        // return -((Math.atan2(1.7, distance.getAsDouble()) * (180/Math.PI)) / 360);
    }
}
