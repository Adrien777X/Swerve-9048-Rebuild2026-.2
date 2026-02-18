package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.climberConfigs;

  public class ClimberSubsystem extends SubsystemBase {

    SparkMax m_LeftClimber;

    public ClimberSubsystem() {
      m_LeftClimber = new SparkMax(ClimberConstants.kClimbermotorID, MotorType.kBrushed);

      m_LeftClimber.configure(
        climberConfigs.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
   }

  public Command c_climb() {
     return this.startEnd(
         () -> {
           m_LeftClimber.set(ClimberConstants.kClimberSpeed);
         },
         () -> {
           m_LeftClimber.set(0);
         });
  }

  public Command c_climbReverse() {
    return this.startEnd(
        () -> {
          m_LeftClimber.set(-ClimberConstants.kClimberSpeed);
        },
        () -> {
          m_LeftClimber.set(0);
        });
  }
}