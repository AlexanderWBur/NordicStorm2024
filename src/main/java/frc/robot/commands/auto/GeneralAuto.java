package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoWithInit;
import frc.robot.commands.FollowNote;
import frc.robot.commands.SetShooter;
import frc.robot.commands.TurnAndShoot;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.MultiPartPath;

public class GeneralAuto extends AutoWithInit {
    public GeneralAuto() {
    }

    static SendableChooser<String> chooser = new SendableChooser<String>();

    public static void putToDashboard() {
        SmartDashboard.putBoolean("Mid Line?", false);
        SmartDashboard.putBoolean("do 1?", false);
        SmartDashboard.putBoolean("do 2?", false);
        SmartDashboard.putBoolean("do 3?", false);

        chooser.addOption("Left", "Left");
        chooser.addOption("Speaker", "Speaker");
        chooser.addOption("Right", "Right");
        SmartDashboard.putData(chooser);
    }

    @Override
    public void initializeCommands() {
        // !PATHWEAVER_INFO: {"trackWidth":0.762,"gameName":"Crescendo"}
        boolean doLastPart = SmartDashboard.getBoolean("DoLastPart?", true);
        RobotContainer.driveTrain.resetAngle();

        DriveTrainConfig config = RobotContainer.driveTrain.getConfig().makeClone();
        config.maxVelocity = 2;
        config.maxAcceleration = 2;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;

        if (chooser.getSelected().equals("Right")) {
            MultiPartPath pathB = new MultiPartPath(RobotContainer.driveTrain, config, null);
            pathB.resetPosition(1.294, 5.564);
            pathB.addWaypoint(1.354, 3.786);
            pathB.addWaypoint(2.022, 1.746);
            pathB.addWaypoint(4.015, 3.035);
            pathB.addWaypoint(4.754, 3.929);
            pathB.addWaypoint(4.253, 5.277);
            pathB.addWaypoint(4.205, 7.079);
            pathB.addStop();
            addCommands(pathB.finalizePath());
        } else if (chooser.getSelected().equals("Speaker")) {
            MultiPartPath pathA = new MultiPartPath(RobotContainer.driveTrain, config, null);
            pathA.resetPosition(1.294, 5.564);
            pathA.addSequentialCommand(new SetShooter(2));// ENDPOS:1.664,5.564
            pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:1.664,5.504
            if (SmartDashboard.getBoolean("do 3?", false)) {//path on
                pathA.addWaypoint(1.640, 4.240);
                pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, false, 1000));// ENDPOS:2.929,4.156
                pathA.addWaypoint(2.452, 4.156);
                pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:2.380,4.180
                pathA.pivotInPlace(90);
            }
            if (SmartDashboard.getBoolean("do 2?", false)) {// path on
                //pathA.addWaypoint(1.855, 5.397);
                pathA.addSequentialCommand(new FollowNote(true, true, 2, 1, false, 500));// ENDPOS:2.917,5.576
                pathA.addWaypoint(2.547, 5.588);
                pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:2.476,5.695
                pathA.pivotInPlace(90);

            }
            if (SmartDashboard.getBoolean("do 1?", false)) {
                //pathA.addWaypoint(1.951, 6.828);
                pathA.addSequentialCommand(new FollowNote(true, true, 2, 1, false, 500));// ENDPOS:2.917,6.888
                pathA.addWaypoint(1.366, 6.566);
                pathA.addSequentialCommand(new TurnAndShoot());// ENDPOS:1.247,6.459
            }
            addCommands(pathA.finalizePath());


        } else if (chooser.getSelected().equals("Left")) {

        }
    }
}