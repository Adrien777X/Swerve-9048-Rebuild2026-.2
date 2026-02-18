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

public class Antijam extends SubsystemBase {

  private SparkMax antijamMotor = new SparkMax(51, MotorType.kBrushless);
  private SparkMax antijamMotorHex = new SparkMax(52, MotorType.kBrushed);

  private SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("AntijamMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(20));

  private SmartMotorController smc =
      new SparkWrapper(antijamMotor, DCMotor.getNeo550(1), config);

  private FlyWheelConfig flyConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.2))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Antijam", TelemetryVerbosity.HIGH);

  private FlyWheel antijam = new FlyWheel(flyConfig);

  public Command feedCommand() {
    return startEnd(
        () -> antijam.set(-0.55).schedule(),
        () -> antijam.set(0).schedule()
    ).withName("Antijam.Feed");
  }

  public Command stopCommand() {
    return runOnce(() -> antijam.set(0).schedule());
  }

  @Override
  public void periodic() {
    antijam.updateTelemetry();
    antijamMotorHex.set(antijamMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    antijam.simIterate();
  }
}
