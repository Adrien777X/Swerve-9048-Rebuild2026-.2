/*package frc.robot.commands.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TurretedShooter.SmartShooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class ShootOnMoveWhileFeedAll extends Command {
    SwerveSubsystem swerveSubsystem;
    Superstructure superstructure;
    SmartShooter smartShooter;
    Turret turret;
    public static final Translation2d m_redHubCenter = new Translation2d(11.908, 4.031);
    public static final Translation2d m_blueHubCenter = new Translation2d(8.2, 4.031); 

    boolean useVision = true;

    private Translation2d targetGoalTranslation;

    public ShootOnMoveWhileFeedAll(Superstructure superstructure, SmartShooter smartShooter, Turret turret, SwerveSubsystem swerveSubsystem) {
        this.superstructure = superstructure;
        this.smartShooter = smartShooter;
        this.turret = turret;
        addRequirements(this.superstructure, this.turret, this.swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.turret.center();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                targetGoalTranslation = m_redHubCenter;
            }
            if (alliance.get() == Alliance.Blue) {
                targetGoalTranslation = m_blueHubCenter;
            }
        } else {
            System.err.println("Alliance information not found. Defaulting to Blue Hub.");
            targetGoalTranslation = m_blueHubCenter;
        }
    }

    @Override
    public void execute() {
        Pose2d currentRobotPose = swerveSubsystem.getPose(); 
        this.turret.aimAtGoal(currentRobotPose, targetGoalTranslation, isFinished());
        if (this.turret.atDesiredAngle()) {
            this.smartShooter.execute();
            this.superstructure.feedAllCommand();
        }
    }

    

    @Override
    public void end(boolean interrupted) {
        this.superstructure.stopShootingCommand();
        this.turret.center();
        this.superstructure.stopFeedingAllCommand();
    }
}
*/