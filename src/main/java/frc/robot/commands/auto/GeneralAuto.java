package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoWithInit;
import frc.robot.commands.TurnAndShoot;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.MultiPartPath;

public class GeneralAuto extends AutoWithInit {
    public GeneralAuto() {
    }

    @Override
    public void initializeCommands() {
        // !PATHWEAVER_INFO: {"trackWidth":0.762,"gameName":"Crescendo"}
        boolean doLastPart = SmartDashboard.getBoolean("DoLastPart?", true);
        RobotContainer.driveTrain.resetAngle();

        DriveTrainConfig config = RobotContainer.driveTrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 4;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;

        MultiPartPath pathA = new MultiPartPath(RobotContainer.driveTrain, config, null);
        pathA.resetPosition(1.473, 5.671);
        pathA.addWaypoint(1.891, 5.540);
        pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:2.106,5.409
        // pathA.addSequentialCommand(new GrabNote());// ENDPOS:2.917,5.564
        pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:3.072,5.576
        // pathA.addSequentialCommand(new GrabNote());// ENDPOS:2.833,4.096
        pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:2.440,4.180
        pathA.addWaypoint(2.774, 6.268);
        // pathA.addSequentialCommand(new GrabNote());// ENDPOS:2.857,7.043

        if (doLastPart) {// path off
            pathA.addWaypoint(4.430, 5.364);
            pathA.addWaypoint(3.287, 4.908);
            pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:1.117,2.611
        }
        pathA.addStop();
        addCommands(pathA.finalizePath());
        MultiPartPath pathB = new MultiPartPath(RobotContainer.driveTrain, config, null);
        pathB.resetPosition(1.056, 4.335);
        pathB.addWaypoint(1.831, 6.363);

        MultiPartPath pathC = new MultiPartPath(RobotContainer.driveTrain, config, null);
        pathC.resetPosition(1.056, 4.335);
        pathC.addWaypoint(1.831, 6.363);

        MultiPartPath pathD = new MultiPartPath(RobotContainer.driveTrain, config, null);
        pathD.resetPosition(1.056, 4.335);
        pathD.addWaypoint(1.831, 6.363);
    }
}