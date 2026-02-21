package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.MathUtils;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.pipeline.NeuralClassifier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.units.measure.Angle;

public class Turret extends SubsystemBase {
  private final SparkFlex m_motor = new SparkFlex(31, MotorType.kBrushless);

  private final double MAX_ONE_DIR_FOV = TurretConstants.kmaxminAngle;

  Limelight                limelight;
  LimelightPoseEstimator   limelightPoseEstimator;

  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(0.2, 0.1, 0.1, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
    .withFeedforward(new SimpleMotorFeedforward(0.3, 2.5, 0.1))
    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 3.58)))
    .withMotorInverted(true)
    .withIdleMode(MotorMode.BRAKE)
    .withSoftLimit(Degrees.of(-90), Degrees.of(90))
    .withStatorCurrentLimit(Amps.of(35))
    .withClosedLoopRampRate(Seconds.of(0.1))
    .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new SparkWrapper(m_motor, DCMotor.getNeoVortex(1), smcConfig);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(0.55))
      .withMaxRobotLength(Meters.of(0.68))
      .withRelativePosition(new Translation3d(Meters.of(-0.205), Meters.of(0.0), Meters.of(0.375)));

  
  private final PivotConfig turretConfig = new PivotConfig(smc)
      //.withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withStartingPosition(Degrees.of(0))
      .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(robotToMechanism)
      /*.withMechanismPositionConfig(
        new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation))*/;

  private Pivot turret = new Pivot(turretConfig);

  public Turret() {
    setupLimelight();
  }

  public void setupLimelight()  {

    limelight = new Limelight("limelight");
    limelight.getSettings()
             .withPipelineIndex(0)
             .withCameraOffset(new Pose3d(Units.inchesToMeters(0),
                                          Units.inchesToMeters(0),
                                          Units.inchesToMeters(0),
                                          new Rotation3d(0, 0, Units.degreesToRadians(0))))
             //.withAprilTagIdFilter(List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11))
             .save();
             limelightPoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }

  

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  public void setAngleSetpoint(Angle angle) {
    smc.setPosition(angle);
    smc.setKp(0.2);
    smc.setKs(0.1);
    smc.setKd(0.1);
    smc.setFeedforward(0.3, 0.5, 0.1, 0);
  }

  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return turret.getAngle().plus(Degrees.of(180));
  }

  public Angle getRawAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command rezero() {
    return Commands.runOnce(() -> m_motor.getEncoder().setPosition(0), this).withName("Turret.Rezero");
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  //mudar dps essa bosta
  public void zeroTurret() {
    turret.setAngle(Degrees.of(0));
  }

  /*public boolean visionAligned() {
    if (Limelight.valid() && Math.abs(Limelight.tx()) < VisionConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }*/

  /* MANUAL CONTROL FOR TESTING *********************
  public void manualTurret(XboxController controller){
    double speed = MathUtil.applyDeadband(controller.getRawAxis(4), 0.15);
    if(turret.getAngle().in(Degrees) >= MAX_ONE_DIR_FOV - 5 && speed > 0)
      stop();

    else if(turret.getAngle().in(Degrees) <= -MAX_ONE_DIR_FOV + 5 && speed < 0)
      stop();
    
    else
      turret.set(speed);
  }*/

  public Angle getAngle() {
    return turret.getAngle();
  }

  public void stop() {
    m_motor.stopMotor();
  }

  private int     outofAreaReading = 0;
  private boolean initialReading   = false;

  @Override
  public void periodic() {
    turret.updateTelemetry();

    turret.updateTelemetry();

    limelight.getSettings()
             .withRobotOrientation(new Orientation3d(new Rotation3d(SwerveSubsystem.swerveDrive.getOdometryHeading()
                                                                               .rotateBy(Rotation2d.kZero)),
                                                     new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0))))
              .withCameraOffset(Constants.DriveConstants.cameraOffsetFromRobotCenter.rotateAround(Constants.DriveConstants.turretPivotCenterFromCamera, new Rotation3d(0, Degrees.of(65).in(Radians), turret.getAngle().in(Radians))))
             .save(); //camera pose is the camera pose from the center of robot
    Optional<PoseEstimate>     poseEstimates = limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results       = limelight.getLatestResults();
    if (results.isPresent()/* && poseEstimates.isPresent()*/)
    {
        LimelightResults result       = results.get();
        PoseEstimate     poseEstimate = poseEstimates.get();
        SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
        SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
        SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
        SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
        SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
        SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
        SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
        SmartDashboard.putNumber("Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
        if (result.valid)
        {
          // Pose2d estimatorPose = poseEstimate.pose.toPose2d();
          Pose2d usefulPose     = result.getBotPose2d(Alliance.Blue);
          double distanceToPose = usefulPose.getTranslation().getDistance(SwerveSubsystem.swerveDrive.getPose().getTranslation());
          if (distanceToPose < 0.5 || (outofAreaReading > 10) || (outofAreaReading > 10 && !initialReading))
          {
            if (!initialReading)
            {
              initialReading = true;
            }
            outofAreaReading = 0;
            
            // System.out.println(usefulPose.toString());
            SwerveSubsystem.swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
            // System.out.println(result.timestamp_LIMELIGHT_publish);
            // System.out.println(result.timestamp_RIOFPGA_capture);
            SwerveSubsystem.swerveDrive.addVisionMeasurement(usefulPose, result.timestamp_RIOFPGA_capture);
          } else
          {
            outofAreaReading += 1;
          }
       }
    }
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}