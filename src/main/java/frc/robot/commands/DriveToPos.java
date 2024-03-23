package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.MultiPartPath;

public class DriveToPos extends SequentialCommandGroup {
    
    public DriveToPos(){
        DriveTrainConfig config = RobotContainer.driveTrain.getConfig().makeClone();
        config.maxAcceleration = 1;
        config.maxVelocity = 1;
        MultiPartPath path = new MultiPartPath(RobotContainer.driveTrain, config, null);
        path.addWaypoint(1.589,3.7624);
        path.addStop();
        addRequirements(RobotContainer.driveTrain);
        addCommands(path.finalizePath());
    }
 
}
