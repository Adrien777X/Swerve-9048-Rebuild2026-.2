package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Kicker extends SubsystemBase {

  private static final double KICKER_SPEED = 0.62;

  // Nova motor controller with NEO motor
  private SparkMax kicker_motor = new SparkMax(Constants.KickerConstants.kKickerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3))) // 4:1 gear reduction
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController smc = new SparkWrapper(kicker_motor, DCMotor.getNeo550(1), smcConfig);

  private final FlyWheelConfig kickerConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

  private FlyWheel kicker = new FlyWheel(kickerConfig);

  private SparkMax antijam_motor = new SparkMax(51, MotorType.kBrushless);
  private SparkMax antijam_motorhex = new SparkMax(52, MotorType.kBrushed);

  private SmartMotorControllerConfig antijam_smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5))) // 4:1 gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController antijam_smc = new SparkWrapper(antijam_motor, DCMotor.getNeo550(1), antijam_smcConfig);

  private final FlyWheelConfig atijamConfig = new FlyWheelConfig(antijam_smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.2))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

  private FlyWheel antijam = new FlyWheel(atijamConfig);

  private SmartMotorControllerConfig antijam_smcConfighex = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3))) // 4:1 gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController antijam_smchex = new SparkWrapper(antijam_motorhex, DCMotor.getMiniCIM(1), antijam_smcConfighex);

  private final FlyWheelConfig atijamConfighex = new FlyWheelConfig(antijam_smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.2))
      .withUpperSoftLimit(RPM.of(3000))
      .withLowerSoftLimit(RPM.of(-3000))
      .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

  private FlyWheel antijamhex = new FlyWheel(atijamConfighex);

  public Kicker() {
  }

  /**
   * Command to run the kicker forward while held, stops when released.
   */
  public Command feedCommand() {
    return run(() -> {
      kicker.set(-KICKER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Kicker.Feed");
      antijam.set(-KICKER_SPEED).finallyDo(() -> antijam_smc.setDutyCycle(0)).withName("Antijam.Feed");
    });
  }

  /**
   * Command to stop the kicker.
   */
  public Command stopCommand() {
    return kicker.set(0).withName("Kicker.Stop")
      .alongWith(antijam.set(0).withName("Antijam.Stop"));
  }

  @Override
  public void periodic() {
    double currentOutput = antijam_motor.get(); 
    antijam_motorhex.set(currentOutput);
    kicker.updateTelemetry();
    antijam.updateTelemetry();
    //kicker_motor.set(-0.60);
    //antijam_motor.set(0.40);
  }

  @Override
  public void simulationPeriodic() {
    kicker.simIterate();
    antijam.simIterate();
  }
}