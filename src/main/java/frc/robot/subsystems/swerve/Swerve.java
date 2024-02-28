package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public void applyVisionToPose() {

        var visionResults = LimelightHelpers.getLatestResults("limelight").targetingResults;

        if (visionResults.getBotPose2d_wpiBlue().getX() == 0.0)
            return;

        double poseDifference = this.getState().Pose.getTranslation()
                .getDistance(visionResults.getBotPose2d_wpiBlue().getTranslation());

        if (visionResults.targets_Fiducials.length > 0) {
            double xyStds;
            double degStds;

            // multiple targets detected
            if (visionResults.targets_Fiducials.length >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }

            // 1 target with large area and close to estimated pose
            else if (visionResults.targets_Fiducials[0].ta > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }

            // 1 target farther away and estimated pose is close
            else if (visionResults.targets_Fiducials[0].ta > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }

            // conditions don't match to add a vision measurement
            else
                return;

            this.addVisionMeasurement(visionResults.getBotPose2d_wpiBlue(),
                    Timer.getFPGATimestamp() - (visionResults.botpose[6] / 1000.0),
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

        }
    }
}
