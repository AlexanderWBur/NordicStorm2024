// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoWithInit;
import frc.robot.commands.DisableClimberLimits;
import frc.robot.commands.DoAmpDumb;
import frc.robot.commands.DoAmpSequence;
import frc.robot.commands.DriveToPos;
import frc.robot.commands.FollowNote;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OperatorControl;
import frc.robot.commands.RawPinionCommand;
import frc.robot.commands.TurnAndShoot;
import frc.robot.commands.RawIndexerCommand;
import frc.robot.commands.auto.GeneralAuto;
import frc.robot.commands.auto.StraightAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Pixy;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final Joystick leftJoystick = new Joystick(1);
  public static final Joystick rightJoystick = new Joystick(0);
  public static final XboxController xbox = new XboxController(2);

  public static boolean isRed;
  public static double AllianceAngleDeg;

  public static double AllianceAngleRad;

  public static DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  // public static Pixy pixyController = new Pixy();

  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem(); //
  // Added to actually make the shooter shoot
  // or turn or what not
  public static VisionSubsystem visionSubsystem = new VisionSubsystem();

  public static IntakeSubsystem intake = new IntakeSubsystem();

  public static Pose2d targetLocation;
  public static Pose2d aimingLocation;

  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    isRed = DriverStation.getAlliance().get() == Alliance.Red;

    if (isRed) {
      targetLocation = new Pose2d(16.54 + 0.05, 5.5478, new Rotation2d(0));
      aimingLocation = new Pose2d();
    } else {
      targetLocation = new Pose2d(-0.038099999, 5.547867999999999, new Rotation2d(0));
      aimingLocation = new Pose2d(-0.038099999 + Units.inchesToMeters(15), 5.547867999999999, new Rotation2d(0));
    }

    driveTrain.setDefaultCommand(new OperatorControl());

    AllianceAngleDeg = isRed ? 180 : 0;
    AllianceAngleRad = Units.degreesToRadians(AllianceAngleDeg);
    // SmartDashboard.putNumber("Alliance Angle", AllianceAngleDeg);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(rightJoystick, 2).onTrue(new IntakeCommand(1, 0));
    new JoystickButton(leftJoystick, 9).whileTrue(new DisableClimberLimits());
    new JoystickButton(xbox, 5).whileTrue(new DoAmpSequence());
    new JoystickButton(rightJoystick, 11).whileTrue(new DriveToPos());
    new JoystickButton(xbox, XboxController.Button.kY.value).whileTrue(new DoAmpDumb());
    new JoystickButton(xbox, XboxController.Button.kRightBumper.value)
        .whileTrue(new FollowNote(true, false, 1, .75, 0.2, 1000));

    // Reset gyro
    new JoystickButton(xbox, XboxController.Button.kBack.value).onTrue(new InstantCommand() {

      @Override
      public void execute() {
        driveTrain.zeroGyroscope();
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });
    new JoystickButton(leftJoystick, 3).onTrue(new InstantCommand() {
      @Override
      public void execute() {
        shooterSubsystem.setAmpMode();
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });
    new JoystickButton(leftJoystick, 1).onTrue(new InstantCommand() {
      @Override
      public void execute() {
        shooterSubsystem.setShooterMode();
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });
    new JoystickButton(leftJoystick, 2).onTrue(new InstantCommand() {
      @Override
      public void execute() {
        shooterSubsystem.setOffMode();
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });
    new JoystickButton(xbox, XboxController.Button.kStart.value).onTrue(new InstantCommand() {

      @Override
      public void execute() {

        driveTrain.setPose(3.327, 7.766, 0); // 8.21 - 7.766
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });
    new JoystickButton(xbox, XboxController.Button.kX.value).onTrue(new InstantCommand() {

      @Override
      public void execute() {

        driveTrain.setPose(2.0, driveTrain.getPose().getY(), 0); // 8.21 - 7.766
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });

    new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new InstantCommand() {

      @Override
      public void execute() {

        SmartDashboard.putNumber("targetRPM", 0);
        SmartDashboard.putNumber("targetPitch", 2);
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoWithInit auto = new GeneralAuto();
    // var auto = new ExamplePathAuto(driveTrain);
    auto.initializeCommands();

    return auto;
  }
}
