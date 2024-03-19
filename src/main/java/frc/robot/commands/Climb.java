package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Climb extends Command {
    
private double power;
  private long timeOut = 0;
  private long timeToStop = 0;


  
  public Climb() {
    addRequirements(RobotContainer.intake);
    

  }

  @Override
  public void initialize() {

    RobotContainer.intake.doIntake(5000);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    return true;
  }
    
}
