// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LoadShooter extends Command {
  
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;
  private long timeToStop;

  

  public LoadShooter(ShooterSubsystem shooter, IntakeSubsystem  intake) {
   
    // pinion.setIdleMode(IdleMode.kBrake);

    this.shooter = shooter;
    this.intake = intake;


    addRequirements();
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
