package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.misc.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Limelight;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
//blargh
//may need to add
// swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
// swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

    Pose2d redSpeaker = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(180));

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double lastSimTime;

    public boolean isLockedRotational = false;

    private final double rotationalKpBlue = 1.200;
    private final double rotationalKiBlue = 0.00001;
    private final double rotationalKdBlue = 0.00001;

    private final double rotationalKpRed = 1.2;
    private final double rotationalKiRed = 0.00001;
    private final double rotationalKdRed = 0.00001;

    private final PIDController rotationalBluePid = new PIDController(rotationalKpBlue, rotationalKiBlue, rotationalKdBlue);
    private final PIDController rotationalRedPid = new PIDController(rotationalKpRed, rotationalKiRed, rotationalKdRed);
    

    //red and blue speaker pose
    // Pose2d redSpeakerPose = new Pose2d(16.55, 5.55, Rotation2d.fromDegrees(180));
    Pose2d blueSpeaker = new Pose2d(0, 5.55, Rotation2d.fromDegrees(0));

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();   //.withDriveRequestType(DriveRequestType.Velocity);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Speaker distance", this.getState().Pose.getTranslation().getDistance(blueSpeaker.getTranslation()));
    }

    public double getRotationalSpeed(DoubleSupplier xboxInput) {
        rotationalRedPid.setSetpoint(0);
        if (isLockedRotational) {
            var activeSpeaker = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red) ? redSpeaker : blueSpeaker;
            var currentPose = getState().Pose;

            var xDistance = currentPose.getTranslation().getX() - activeSpeaker.getX();
            var yDistance = currentPose.getTranslation().getY() - activeSpeaker.getY();

                var targetAngle = Math.atan(yDistance/
                xDistance);
                

                if (yDistance >0.1){

                     targetAngle =Math.PI/2-Math.asin(xDistance / currentPose.getTranslation().getDistance(redSpeaker.getTranslation()));

                }else if(yDistance <0.1)
                {
                     targetAngle =- Math.acos(-xDistance / -currentPose.getTranslation().getDistance(redSpeaker.getTranslation()));

                }
                else{
                    targetAngle = 0;
                }
                
                var currentAngle = currentPose.getRotation().getRadians();
              

                var  angleDifference = currentAngle - targetAngle;



                                System.out.println("targetAngle: " + targetAngle+ " currentAngle: " + currentAngle + " angleDifference: " + angleDifference);


            if(activeSpeaker == blueSpeaker){

                // var targetAngle = Math.atan2(yDistance, xDistance);
                // var currentAngle = currentPose.getRotation().getRadians();
                // var angleDifference = currentAngle - targetAngle;
                //                 System.out.println("targetAngle: " + targetAngle+ " currentAngle: " + currentAngle + " angleDifference: " + angleDifference);

                return rotationalBluePid.calculate(angleDifference) * 0.85;
            }

            else if(activeSpeaker == redSpeaker){

                if (targetAngle < 0) {
                    targetAngle *= -1;
                }
                
return rotationalRedPid.calculate(angleDifference) *0.85 ;
                
                
                // return xboxInput.getAsDouble() * 1.5 * Math.PI;

                // var targetAngle = Math.atan2(yDistance, 
                // -xDistance);
                
                // var currentAngle = Math.abs(currentPose.getRotation().getRadians());

             
                // var  angleDifference = currentAngle - (Math.PI + targetAngle);

                // if(currentAngle < 0){
                //     angleDifference = Math.PI - Math.abs(currentAngle);
                //     angleDifference += (Math.PI - Math.abs(targetAngle));

                //     return rotationalPid.calculate(angleDifference) * 0.85;
                // }

               



                  


                        
                    
                 
                
            }

        }

        return xboxInput.getAsDouble();
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules){
        super(driveTrainConstants, modules);

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {

        double driveBaseRadius = 0;

        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        NamedCommands.registerCommand("doNothing2", new WaitCommand(2));

        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(3.05, 0, 0),
                                            new PIDConstants(2.5, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),

            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }


    public void applyVisionToPose() {
        //ToDo: Test STD Values

        var visionResults = LimelightHelpers.getLatestResults("limelight").targetingResults;
       // System.out.println("past the var");

        if (visionResults.getBotPose2d_wpiBlue().getX() == 0.0) {
            return;
        }

        double poseDifference = this.getState().Pose.getTranslation()
                .getDistance(visionResults.getBotPose2d_wpiBlue().getTranslation());

        if (visionResults.targets_Fiducials.length > 0) {
            double xyStds;
            double degStds;
            //System.out.println("caught an april");
            //System.out.println(poseDifference);

            // multiple targets detected
            if (visionResults.targets_Fiducials.length >= 2) {
                xyStds = 0.4;
                degStds = 6;
            }

            // 1 target with large area and close to estimated pose
            else if (visionResults.targets_Fiducials[0].ta > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
                System.out.println("caught a single one");
            }

            // 1 target farther away and estimated pose is close
            else if (visionResults.targets_Fiducials[0].ta > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }

            // conditions don't match to add a vision measurement
            else {
                System.out.println("no conditions matched - no vision applied");
                return;
            }

            if (visionResults.valid){
//(visionResults.latency_pipeline / 1000.0)

                addVisionMeasurement(visionResults.getBotPose2d_wpiBlue(),
                        Timer.getFPGATimestamp() ,
                        VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            }


        }

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


//May need to erase
//This is a closed loop implementation of chassis drive for pathplanner.
//The open/PID version might be good enough, but this could be better if we get our sysID data 'lit'
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
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public ApplyChassisSpeedsClosed withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public ApplyChassisSpeedsClosed withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }


    }
}
