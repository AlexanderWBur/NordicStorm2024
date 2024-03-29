package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetRumble extends Command {
    
    private double rumble;
    private long timeToEnd = 0;
/**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetRumble(double rumble) {    
    this.rumble = rumble;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeToEnd = System.currentTimeMillis() + 1000;
    RobotContainer.xbox.setRumble(RumbleType.kBothRumble, rumble);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.xbox.setRumble(RumbleType.kBothRumble, 0);

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > timeToEnd;
  }

}
