package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RawIndexerCommand extends Command {
    
    public RawIndexerCommand() {
    addRequirements(RobotContainer.shooterSubsystem);
    

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RobotContainer.shooterSubsystem.setIndexerRaw(RobotContainer.leftJoystick.getY());

  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.setIndexerRaw(0);

    

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
