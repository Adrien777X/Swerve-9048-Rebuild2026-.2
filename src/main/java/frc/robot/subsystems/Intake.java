package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static final double INTAKE_SPEED = 0.7;

    private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.IntakeConstants.pivotEncoder);

    private final SparkMax m_rollerLeader  = new SparkMax(Constants.IntakeConstants.kRollerMotorIdLeader, MotorType.kBrushless);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(20))
      .withFollowers(Pair.of(new SparkMax(Constants.IntakeConstants.kRollerMotorIdFollower, MotorType.kBrushless), true));

    private final SmartMotorController smc = new SparkWrapper(m_rollerLeader, DCMotor.getNeo550(2), smcConfig);

    private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(2))
      .withMass(Grams.of(2229))
      .withUpperSoftLimit(RPM.of(30))
      .withLowerSoftLimit(RPM.of(-30))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);

    private FlyWheel intake = new FlyWheel(intakeConfig);

    private SparkFlex pivotMotor = new SparkFlex(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);

    private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withExternalEncoder(pivotEncoder)
      //.withExternalEncoderInverted(false)
      //.withExternalEncoderGearing(1)
      //.withExternalEncoderZeroOffset(Degrees.of(355))
      .withClosedLoopController(1.25, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(180))
      .withFeedforward(new ArmFeedforward(0, 10, 0))
      .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(60.0)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(20))
      .withClosedLoopRampRate(Seconds.of(0.3))
      .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNeoVortex(1), intakePivotSmartMotorConfig);

    private final ArmConfig intakePivotConfig = new ArmConfig(intakePivotController)
      .withSoftLimits(Degree.of(0), Degrees.of(90))
      .withHardLimit(Degrees.of(0), Degrees.of(155))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(4.4))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

    private Arm intakePivot = new Arm(intakePivotConfig);

    public Intake() {
      //pivotMotor.configure(new SparkFlexConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      //m_rollerLeader.configure(new SparkMaxConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public Angle getAngle() {
      double pivotdeg = pivotEncoder.get() - IntakeConstants.pivotEncoderOffsetDeg;
      pivotdeg = MathUtil.inputModulus(pivotdeg, -180, 180);
      return Degrees.of(pivotdeg);
    }

    public Command intakeCommand() {
      return intake.set(INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Run");
    }

    public Command ejectCommand() {
      return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
    }

    public Command setPivotAngle(Angle angle) {
      return intakePivot.setAngle(angle).withName("IntakePivot.SetAngle");
    }

    public Command rezero() {
      return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
    }
 
    public Command deployAndRollCommand() {
      return Commands.run(() -> {
        setIntakeDeployed();
        smc.setDutyCycle(INTAKE_SPEED);
      }, this).finallyDo(() -> {
        smc.setDutyCycle(0);
        setIntakeHold();
      }).withName("Intake.DeployAndRoll");
    }

    public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      smc.setDutyCycle(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.BackFeedAndRoll");
  }

    private void setIntakeStow() {
      intakePivotController.setPosition(Degrees.of(258));
    }

    private void setIntakeFeed() {
      intakePivotController.setPosition(Degrees.of(59));
    }

    private void setIntakeHold() {
      intakePivotController.setPosition(Degrees.of(115));
    }

    private void setIntakeDeployed() {
      intakePivotController.setPosition(Degrees.of(355));
    }

    @Override
    public void periodic() {
      //System.out.println("Ecoder YAMS pivot:" + intakePivot.getAngle());
      //System.out.println("PivotEncoder:"+ pivotEncoder.get());
      //double pivotdeg = pivotEncoder.get();

      //intakePivotController.setEncoderPosition(Degrees.of(pivotdeg));

      intake.updateTelemetry();
      intakePivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
      intake.simIterate();
      intakePivot.simIterate();
    }
}