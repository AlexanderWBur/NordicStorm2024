package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class takes any standard Command and makes into a CommandPathPiece. This
 * class shouldn't normally be used by the user, please make your command
 * implement CommandPathPiece instead.
 */
public class PathPieceWrapper extends CommandPathPiece {

    private double startSpeed;
    private Command command;
    public PathPieceWrapper(Command command, double startSpeed) {
        this.startSpeed = startSpeed;
        this.command = command;
        //addCommands(command);
    }

    
    @Override
    public double getRequestedStartSpeed() {
        return startSpeed;
    }
    @Override
    public void initialize(){
        command.schedule();
    }
    @Override
    public final void execute() {
    }

    @Override
    public final void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public final boolean isFinished() {
        return !command.isScheduled();
    }

}
