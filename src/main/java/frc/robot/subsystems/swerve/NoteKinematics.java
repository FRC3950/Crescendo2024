package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants;

public final class NoteKinematics {


    public static double getHeadingAdjustment(Pose2d botPose, Pose2d speakerPose, double longitudinalVel) {
        var velocityAngle = Math.atan(longitudinalVel/Constants.Physics.noteExitVelocity);
        var speakerDistance = botPose.getTranslation().getDistance(speakerPose.getTranslation());

        var speakerTheta = Math.acos(botPose.getX()/speakerDistance);

        return (velocityAngle + speakerTheta) * (180/Math.PI);
    }

    public static double getTargetPivot() {
        // TODO figure out geometry of field, robot & relate to angle reading from CANcoder soon-to-be installed on the pivot shaft
        // Can hopefully eliminate regression entirely & improve range
        return 0.0;
    }

    private static double getPivotAdjustment() {
        // TODO figure out field geometry for pivot adjustment based on lateral velocity of bot
        return 0.0;
    }
}
