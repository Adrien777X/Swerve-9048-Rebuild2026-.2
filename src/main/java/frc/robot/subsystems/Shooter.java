package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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

public class Shooter extends SubsystemBase {
    private final SparkMax leaderSpark = new SparkMax(33, MotorType.kBrushless);
    private final SparkMax followerSpark = new SparkMax(34, MotorType.kBrushless);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withFollowers(Pair.of(followerSpark, true))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.00936, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNEO(2), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(0))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private final FlyWheel shooter = new FlyWheel(shooterConfig);

    public Shooter() {
        //m_PID.setTolerance(ShooterConstants.kRPMTolerance);
        //m_PID.setIntegratorRange(0, 0);

        //m_PID.setIntegratorRange(-ShooterConstants.kIntRange, ShooterConstants.kIntRange);

    }

    public Command stopCommand() {
        return runOnce(() -> shooter.setSpeed(RPM.of(0)));
    }

    public Command setSpeed(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
        return shooter.setSpeed(speedSupplier);
    }

    public Command spinUp() {
        return setSpeed(RPM.of(-2500));
    }

    public Command stop() {
        return setSpeed(RPM.of(0));
    }

    public AngularVelocity getSpeed() {
        return shooter.getSpeed();
    }

    public Command sysId() {
        return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
        //leaderSpark.set(-0.40);
        //followerSpark.set(-0.40);
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    private Distance wheelRadius() {
        return Inches.of(4).div(2);
    }

    /*public boolean isAtSetpoint() {
        double speed_m = m_encoder1.getVelocity();
        return Math.abs(m_RPM - speed_m) <= ShooterConstants.kRPMTolerance;
    } */
    
    public Optional<Angle> atSetpoint() {
        return shooter.getMechanismSetpoint();
    }

    public LinearVelocity getTangentialVelocity() {
    // Calculate tangential velocity at the edge of the wheel and convert to
    // LinearVelocity

    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
    }
}
