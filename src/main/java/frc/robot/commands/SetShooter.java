package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.commands.paths.CommandPathPiece;

public class SetShooter extends CommandPathPiece {
    private ShooterMode mode;

    public SetShooter(ShooterMode mode) {
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
