package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnAndShoot extends Command {
    
    boolean hasSent;
     public TurnAndShoot() {
    addRequirements(RobotContainer.intake);
    

  }

  @Override
  public void initialize() {
    hasSent = false;
    RobotContainer.shooterSubsystem.setShooterAngle(-30);
  }

  @Override
  public void execute() {
    
    if(Math.abs(RobotContainer.shooterSubsystem.getAngleError()) < .5 && !hasSent){
         RobotContainer.intake.sendToShooter();
         hasSent = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
         RobotContainer.shooterSubsystem.setShooterAngle(-5);

  }

  @Override
  public boolean isFinished() {
    return false;
  } 
} 
