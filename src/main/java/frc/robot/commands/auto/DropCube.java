package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DropCube extends CommandBase {


    long timeToStop;
    DropCube(){
        
    }
    
    @Override
    public void initialize() {
        timeToStop = System.currentTimeMillis()+4000;
    }

    @Override
    public void execute() {
        //RobotContainer.intakeSubsystem.setIntakeBeltMotorSpeed(1);

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > timeToStop;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
