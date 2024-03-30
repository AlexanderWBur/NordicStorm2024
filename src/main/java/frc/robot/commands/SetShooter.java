package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.CommandPathPiece;

public class SetShooter extends CommandPathPiece {
    private int mode;

    public SetShooter(int mode) {
        this.mode = mode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.shooterSubsystem.setMode(mode);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
