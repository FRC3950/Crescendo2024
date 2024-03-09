// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  private static Limelight instance;

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }
    return instance;
  }
  // if two apriltags in view, access tx of only apriltag ID #7

  private Limelight() {
  }

  public double getTx() {
    // Only pull tx value for tag id #7
    //limelightTable.getValue(getName())
      var results = LimelightHelpers.getLatestResults("limelight").targetingResults;
      for (LimelightHelpers.LimelightTarget_Fiducial result : results.targets_Fiducials) {
        if(result.fiducialID == 7 || result.fiducialID == 4){
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
    System.out.println("ll blink");
  }

  

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
