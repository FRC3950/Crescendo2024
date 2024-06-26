package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.TunerConstants;
import lib.odometry.LimelightHelpers;
import lib.odometry.NoteKinematics;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Swerve extends SwerveDrivetrain implements Subsystem {

    // may need to add
    // swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
    // Constants.Swerve.openLoopRamp;
    // swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
    // Constants.Swerve.openLoopRamp;
    // Consider Phoniex Pro - Torque drive controls and 250mhz update rate!!!!!

    Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(180));
    Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double lastSimTime;

    public boolean isLockedRotational = false;

    private final double rotationalKpBlue = 1.25;
    private final double rotationalKiBlue = 0.00001;
    private final double rotationalKdBlue = 0.00001;

    private final double rotationalKpRed = 1.25;
    private final double rotationalKiRed = 0.00001;
    private final double rotationalKdRed = 0.00001;

    private final PIDController rotationalBluePid = new PIDController(rotationalKpBlue, rotationalKiBlue,
            rotationalKdBlue);
    private final PIDController rotationalRedPid = new PIDController(rotationalKpRed, rotationalKiRed, rotationalKdRed);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds(); // .withDriveRequestType(DriveRequestType.Velocity);

    @Override
    public void periodic() {}

    public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
        return m_odometry;
    }

    public double getRotationalSpeed(DoubleSupplier xboxInput) {
        // rotationalRedPid.setTolerance(0.02);
        rotationalRedPid.enableContinuousInput(-Math.PI, Math.PI);

        var activeSpeaker = blueSpeaker;

       // var activeSpeaker = DriverStation.getAlliance().get() == (DriverStation.Alliance.Red) ? redSpeaker : blueSpeaker;

         if (!Utils.isSimulation()) {
            activeSpeaker = DriverStation.getAlliance().get() == (DriverStation.Alliance.Red) ? redSpeaker : blueSpeaker;
        }


        if (isLockedRotational) {
           // System.out.println(activeSpeaker.getX());

            var botPose = this.getState().Pose;
            var xDistance = botPose.getTranslation().getX() - activeSpeaker.getX();
            var yDistance = botPose.getTranslation().getY() - activeSpeaker.getY();

            var targetAngle = Math.atan2(yDistance, xDistance);
            var currentAngle = botPose.getRotation().getRadians();

            var angleDifference = currentAngle - targetAngle;
           // var angleDifference = NoteKinematics.getHeadingDifference(activeSpeaker, getState().Pose);

            if (activeSpeaker == blueSpeaker) {
                return rotationalBluePid.calculate(angleDifference) * 0.85;
            } else if (activeSpeaker == redSpeaker) {
                return rotationalRedPid.calculate(angleDifference) * 0.85;
            }

            return xboxInput.getAsDouble();
        }

        return xboxInput.getAsDouble();
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
                  SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void setAllianceDirection(DriverStation.Alliance alliance) {
        this.m_fieldRelativeOffset = new Rotation2d(alliance.equals(DriverStation.Alliance.Red) ? Math.PI : 0);
    }
//    public void setDirectionForRedAlliance() {
//        this.m_fieldRelativeOffset = new Rotation2d(Math.PI);
//    }
//
//    public void setDirectionForBlueAlliance() {
//        this.m_fieldRelativeOffset = new Rotation2d(0);
//    }

    private void configurePathPlanner() {

        double driveBaseRadius = 0;

        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        NamedCommands.registerCommand("doNothing2", new WaitCommand(2));

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                // robot
                new HolonomicPathFollowerConfig(new PIDConstants(3.05, 0, 0),
                        new PIDConstants(2.5, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),

                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    // May need to erase
    // This is a closed loop implementation of chassis drive for pathplanner.
    // The open/PID version might be good enough, but this could be better if we get
    // our sysID data 'lit'
    // blargh

    /**
     * Accepts a generic ChassisSpeeds to apply to the drivetrain.
     */
    public class ApplyChassisSpeedsClosed implements SwerveRequest {

        /**
         * The chassis speeds to apply to the drivetrain.
         */
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        /**
         * The center of rotation to rotate around.
         */
        public Translation2d CenterOfRotation = new Translation2d(0, 0);
        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the chassis speeds to apply to the drivetrain.
         *
         * @param speeds Chassis speeds to apply to the drivetrain
         * @return this request
         */
        public ApplyChassisSpeedsClosed withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }

        /**
         * Sets the center of rotation to rotate around.
         *
         * @param centerOfRotation Center of rotation to rotate around
         * @return this request
         */
        public ApplyChassisSpeedsClosed withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public ApplyChassisSpeedsClosed withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public ApplyChassisSpeedsClosed withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

    }
}
