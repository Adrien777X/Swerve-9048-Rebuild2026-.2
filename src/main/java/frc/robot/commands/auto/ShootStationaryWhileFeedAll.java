package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.TurretedShooter.RunShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;


public class ShootStationaryWhileFeedAll extends ParallelCommandGroup {
    public ShootStationaryWhileFeedAll(Superstructure superstructure, Turret turret, SwerveSubsystem swerveSubsystem, Shooter shooter) {
        Command runShooterCommand = new RunShooter(shooter, turret, swerveSubsystem, true);

        Command feedWhenReadyCommand = new RunCommand(() ->  {
            if (turret.atDesiredAngle() && shooter.isAtSetpoint()) {
                superstructure.feedAllCommand().schedule();
            } else {
                superstructure.stopAllCommand();
            }
        }, superstructure, turret, shooter);
        addCommands(runShooterCommand, feedWhenReadyCommand);
    }
}