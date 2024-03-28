// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AimShooter extends Command {
  

 

  public AimShooter(ExampleSubsystem subsystem) {
   
    // addRequirements(RobotContainer.ShooterSubsystem);
  }

  @Override
  public void initialize() {

    // LoadShooter();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
