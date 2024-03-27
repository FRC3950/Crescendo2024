package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants;

public final class NoteKinematics {


    public static double getHeadingAdjustment(double longitudinalVel) { // Assumes speaker lock is on 
        var velocityAngle = Math.atan(longitudinalVel/Constants.Physics.noteExitVelocity);

        return velocityAngle;
    }

    public static double getTargetPivot() {
        // TODO figure out geometry of field, robot & relate to angle reading from CANcoder soon-to-be installed on the pivot shaft
        // Can hopefully eliminate regression entirely & improve range
        return 0.0;
    }

    private static double getPivotAdjustment(double lateralVel, Pose2d botPose, Pose2d speakerPose) {
        var heightDifference = Constants.Physics.targetSpeakerHeight - Constants.Physics.shooterHeight;
        var distance = botPose.getX() - speakerPose.getX();

        
        // TODO figure out field geometry for pivot adjustment based on lateral velocity of bot
        return 0.0;
    }
}
