// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.math.Matter;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public final class Constants {

  public static enum AimPoints {
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
    RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
    BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    public static final Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
  }

  public static final double ROBOT_MASS = 90 * 0.453592;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(11.2)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13;
  public static final double MAX_SPEED = Units.feetToMeters(15);


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 2;
    public static final int kOperatorControllerPort = 3;
    public static final double DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT = 6;
  }

  public static class DriveConstants {
    public static final double kInnerDeadband = 0.10;
    public static final double kOuterDeadband = 0.98;

    public static final double kMaxSpeedMetersPerSecond = 3.25; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

    public static Pose3d cameraOffsetFromRobotCenter = new Pose3d(new Translation3d(), new Rotation3d());
    public static Translation3d turretPivotCenterFromCamera = new Translation3d(Inches.of(0), Inches.of(0),Inches.of(0));
  }

  public static final class IntakeConstants {
    public static int kRollerMotorIdLeader = 30;
    public static int kRollerMotorIdFollower = 37;
    public static int kPivotMotorId = 32;
  }

  public static class HopperConstants {
    public static final int kHopperMotorId = 40;
  }

  public static class KickerConstants {
    public static final int kKickerMotorId = 50;
  }

  public static final class TurretConstants {
    //public static final int kIDVortex = 31;
    public static final double kmaxminAngle = 90;
    public static final double kMotorToEncoderRatio = 3.0;
    public static final double kEncoderGearToTurretGearRatio = 6.296; //170.0 / 27.0
    public static final double TotalReduction = kMotorToEncoderRatio * kEncoderGearToTurretGearRatio;
    public static final double kTolerance = 2 * 0.0349; // allowable angle error in radians for the PIDSubsystem to
                                                        // report atSetpoint() to true
    public static final double kLow = 0.383; // Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kHigh = 5.90; // Maximum angle in radians allowed (defines the turret deadzone)
    public static final double kAccel = 15000;
    public static final double kMaxVel = 5000;
    public static final double kMotionTol = 0.2;
    public static final double kRatio = 0.0171875;
    public static final double kStartingPositionDegrees = 180.0;
    public static final float kLowLimit = (float) (kLow / kRatio / 2.0 / Math.PI - 1.0);
    public static final float kHighLimit = (float) (kHigh / kRatio / 2.0 / Math.PI + 1.0);
    public static final double kNearDeadzone = 0.20;
  }

  public static final class ShooterConstants {
    
  }

  public class ClimberConstants {

    public static final int kClimbermotorID = 53;

    public static final double kClimberSpeed = 0.7;
  }

  public static final class climberConfigs {

    public static final SparkBaseConfig climberConfig =
        new SparkMaxConfig().idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);
  }

  // Auto constants OLD APRIL TAG ALIGN REEFSCAPE 2025
  public static final double MAX_ALIGN_TRANSLATION_SPEED = 0.8; // m/s
  public static final double MAX_ALIGN_ROTATION_SPEED = 1.5;    // rad/s

	public static final double X_REEF_ALIGNMENT_P = 2.7;
	public static final double Y_REEF_ALIGNMENT_P = 2.7;
	public static final double ROT_REEF_ALIGNMENT_P = 0.094;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.7;
	public static final double X_SETPOINT_REEF_ALIGNMENT = 1.20;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

  public static final class VisionConstants {
    public static final double kElevationOffset = 38.5; //degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngle = 0.0; //degree Azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLens = 81.0; //Center Height of the target in inches above the lens

    public static final double kTrackTolerance = 0.02000; //Allowable limelight angle error in radians
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
}
