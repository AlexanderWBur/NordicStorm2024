package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveForTime extends CommandBase {


    double xSpeed;
    double ySpeed;
    double rotSpeed;
    long time;
    long timeToStop;
    DriveForTime(double xSpeed, double ySpeed, double rotSpeed, long time){
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        this.time = time;
        addRequirements(RobotContainer.driveTrain);
    }
    
    @Override
    public void initialize() {
        timeToStop = System.currentTimeMillis()+time;
    }

    @Override
    public void execute() {
        RobotContainer.driveTrain.drive(xSpeed, ySpeed, rotSpeed);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() > timeToStop;
    }
    @Override
    public void end(boolean interrupted) {

    }

}
