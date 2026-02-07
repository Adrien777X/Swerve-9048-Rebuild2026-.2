package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Superstructure;
import frc.robot.commands.auto.ShootStationaryWhileFeedAll;

public class NamedCommandsRegistry {
  private final Turret turret;
  private final SwerveSubsystem drivebase;
  private final Superstructure superstructure;
  private final Shooter shooter; // Reference to the shooter subsystem

  // Note: Intake, Hopper, Kicker, and SmartShooter are present in the constructor but unused below.
  public NamedCommandsRegistry(SwerveSubsystem drivebase, Turret turret, 
                               Superstructure superstructure, Shooter shooter) {
    this.turret = turret;
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.shooter = shooter; // Assign the shooter reference
  }

  public void registerAllAutoCommands() {
    this.registerShootingCommands();
  }

  private void registerShootingCommands() {

    if (superstructure == null)
        throw new RuntimeException("Superstructure is NULL");

    if (turret == null)
        throw new RuntimeException("Turret is NULL");

    if (drivebase == null)
        throw new RuntimeException("SwerveSubsystem is NULL");
    
    if (shooter == null)
        throw new RuntimeException("Shooter is NULL");


    // Create an instance of the AutoShooterSequence command group
    ShootStationaryWhileFeedAll shootCommand =
        new ShootStationaryWhileFeedAll(
            superstructure,
            turret,
            drivebase,
            shooter
        );

    // Register the command with a specific name that matches your PathPlanner configuration
    NamedCommands.registerCommand(
        "Shoot Alliance Hub While Stationary", // Use a descriptive and unique name
        shootCommand
    );
  }
}