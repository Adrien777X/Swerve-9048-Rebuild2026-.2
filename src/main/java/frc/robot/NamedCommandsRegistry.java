/*package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.AllianceFlipUtil;
//import frc.robot.commands.Auto.AutoScoreCoralAutonomous;
//import frc.robot.commands.Auto.AutoUpdateOdometry;
//import frc.robot.commands.Auto.CollectCoralAutonomous;
//import frc.robot.commands.States.DefaultPosition;
//import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Superstructure;
import frc.robot.commands.auto.ShootOnMoveWhileFeedAll;
import frc.robot.commands.TurretedShooter.SmartShooter;

public class NamedCommandsRegistry {
  Turret turret;
  SwerveSubsystem drivebase;
  Intake intake;
  Hopper hopper;
  Superstructure superstructure;
  SmartShooter smartShooter;
  Kicker kicker;

  public NamedCommandsRegistry(SwerveSubsystem drivebase,  Turret turret, Intake intake, Superstructure superstructure, Hopper hopper, Kicker kicker, SmartShooter smartShooter) {
    this.turret = turret;
    this.drivebase = drivebase;
    this.intake = intake;
    this.hopper = hopper;
    this.superstructure = superstructure;
    this.smartShooter = smartShooter;
    this.kicker = kicker;
  }

  public void registerAllAutoCommands() {
    //this.registerResetOdometryCommands(this.drivetrain);
    //this.registerIntakeCommands(this.intake);
    this.registerShootOnMoveCommand();
  }

  private void registerShootOnMoveCommand() {

    if (superstructure == null)
        throw new RuntimeException("Superstructure is NULL");

    if (turret == null)
        throw new RuntimeException("Turret is NULL");

    if (drivebase == null)
        throw new RuntimeException("SwerveSubsystem is NULL");

    if (smartShooter == null)
        throw new RuntimeException("SmartShooter is NULL");

    ShootOnMoveWhileFeedAll shootCommand =
        new ShootOnMoveWhileFeedAll(
            superstructure,
            smartShooter,
            turret,
            drivebase
        );

    NamedCommands.registerCommand(
        "Shoot On Move Alliance Hub",
        shootCommand
    );
    NamedCommands.registerCommand("Shoot On Move Alliance Hub", shootCommand);
  }
}*/