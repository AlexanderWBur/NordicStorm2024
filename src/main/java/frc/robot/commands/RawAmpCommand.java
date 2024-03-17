package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RawAmpCommand extends Command {
    
    public RawAmpCommand() {
    addRequirements(RobotContainer.shooterSubsystem);
    

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RobotContainer.shooterSubsystem.setAmpRaw(-RobotContainer.leftJoystick.getY());
    RobotContainer.shooterSubsystem.setShooterRaw(RobotContainer.leftJoystick.getY());
    // RobotContainer.shooterSubsystem.setIndexerRaw(-RobotContainer.leftJoystick.getY());


  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.setAmpRaw(0);
    RobotContainer.shooterSubsystem.setShooterRaw(0);
    // RobotContainer.shooterSubsystem.setIndexerRaw(0);


  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
