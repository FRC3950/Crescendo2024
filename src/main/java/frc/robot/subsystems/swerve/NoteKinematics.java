package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class NoteKinematics {

    public static double getHeadingAdjustment(Pose2d botPose, Pose2d speakerPose) {
        var speakerDistance = botPose.getTranslation().getDistance(speakerPose.getTranslation());
        var speakerTheta = Math.acos(botPose.getX()/speakerDistance);

        return speakerTheta;
    }

    public static double getTargetPivot(DoubleSupplier distance) {
        var gravityComp = 0.015 * distance.getAsDouble();
        //90 -arcTan2 (SpeakerHeight - pivotAngleHeight / RobotDistance) 
        return 0.25 - ((Math.atan2(1.55 + gravityComp, distance.getAsDouble()) * (180/Math.PI)) / 360);
        // return -((Math.atan2(1.7, distance.getAsDouble()) * (180/Math.PI)) / 360);
    }
}
