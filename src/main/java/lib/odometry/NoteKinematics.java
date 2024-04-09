package lib.odometry;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.DoubleSupplier;

public final class NoteKinematics {

    public static double getHeadingAdjustment(Pose2d botPose, Pose2d speakerPose) {
        var speakerDistance = botPose.getTranslation().getDistance(speakerPose.getTranslation());

        return Math.acos(botPose.getX() / speakerDistance);
    }

    public static double getHeadingDifference(Pose2d activeSpeaker, Pose2d botPose) {

        var xDistance = botPose.getTranslation().getX() - activeSpeaker.getX();
        var yDistance = botPose.getTranslation().getY() - activeSpeaker.getY();

        var targetAngle = Math.atan2(yDistance, xDistance);
        var currentAngle = botPose.getRotation().getRadians();

        return currentAngle - targetAngle;
    }

    public static double getTargetPivot(DoubleSupplier distance) {
        var gravityComp = 0.015 * distance.getAsDouble();
        //90 -arcTan2 (SpeakerHeight - pivotAngleHeight / RobotDistance)
        return 0.25 - ((Math.atan2(1.55 + gravityComp, distance.getAsDouble()) * (180 / Math.PI)) / 360);
        // return -((Math.atan2(1.7, distance.getAsDouble()) * (180/Math.PI)) / 360);
    }
}
