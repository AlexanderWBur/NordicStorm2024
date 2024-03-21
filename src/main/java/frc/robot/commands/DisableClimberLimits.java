package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DisableClimberLimits extends Command {

    @Override
    public void initialize() {
        RobotContainer.climberSubsystem.setLimitsEnabled(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.leftJoystick.getRawButton(8)) {
            RobotContainer.climberSubsystem.resetPosition();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.climberSubsystem.setLimitsEnabled(true);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
