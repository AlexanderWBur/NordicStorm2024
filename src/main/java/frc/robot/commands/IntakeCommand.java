// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  private double power;
  private long timeOut = 0;
  private long timeToStop = 0;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(double power, long timeOut) {
    addRequirements(RobotContainer.intake);
    this.power = power;
    this.timeOut = timeOut;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.intake.doIntake(5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
