package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoWithInit;
import frc.robot.commands.FollowNote;
import frc.robot.commands.Ploop;
import frc.robot.commands.SetShooter;
import frc.robot.commands.ShooterMode;
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

        SmartDashboard.putBoolean("far 3", false);
        SmartDashboard.putBoolean("far 4", false);
        SmartDashboard.putBoolean("far 5", false);
        SmartDashboard.putBoolean("ShootFirst", false);
        SmartDashboard.putBoolean("ShootSecond", false);

        chooser.addOption("Left", "Left");
        chooser.addOption("Speaker", "Speaker");
        chooser.addOption("Right", "Right");
        SmartDashboard.putData(chooser);
    }

    @Override
    public void initializeCommands() {
        // !PATHWEAVER_INFO: {"trackWidth":0.9144,"gameName":"Crescendo"}
        boolean doLastPart = SmartDashboard.getBoolean("DoLastPart?", true);
        RobotContainer.driveTrain.resetAngle();
        RobotContainer.driveTrain.setAngleOffset(RobotContainer.AllianceAngleDeg);

        RobotContainer.intake.resetIntake();
        DriveTrainConfig config = RobotContainer.driveTrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 3;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;
        double correctSeek = RobotContainer.isRed ? 0.2 : -0.2;
        if (chooser.getSelected().equals("Right")) {
            MultiPartPath pathB = new MultiPartPath(RobotContainer.driveTrain, config, null);
            pathB.resetPosition(1.426, 3.727);
            if (SmartDashboard.getBoolean("ShootFirst", false)) { // path off
                pathB.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:1.211,3.715
                pathB.addWaypoint(1.772, 4.156);
                pathB.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.569,4.037
            } else {
                pathB.addSequentialCommand(new SetShooter(ShooterMode.PLOOP));// ENDPOS:1.593,4.049
                pathB.addSequentialCommand(new Ploop());// ENDPOS:0.000,6.200

            }
            pathB.addSequentialCommand(new SetShooter(ShooterMode.OFF));// ENDPOS:1.593,4.049

            if (SmartDashboard.getBoolean("far 3", false)) {// path off
                pathB.addWaypoint(2.845, 2.832);
                pathB.addWaypoint(3.698, 2.770);
                pathB.addWaypoint(4.854, 3.990);
                pathB.addWaypoint(7.105, 4.037);
                pathB.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.274,4.038
                pathB.addWaypoint(6.620, 4.022);
                pathB.addWaypoint(4.742, 3.990);
                pathB.addWaypoint(3.669, 2.844);
                pathB.addWaypoint(2.130, 3.214);
                pathB.addWaypoint(2.058, 4.932);
                pathB.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:2.034,4.955
                pathB.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.998,4.979

            } else if (SmartDashboard.getBoolean("far 4", false)) { // path off

                pathB.addWaypoint(1.139, 2.868);
                pathB.addWaypoint(3.108, 1.555);
                pathB.addWaypoint(3.848, 1.722);
                pathB.addWaypoint(6.269, 1.997);
                pathB.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.262,2.390
                pathB.addWaypoint(7.128, 3.023);
                pathB.addWaypoint(5.745, 4.240);
                pathB.addWaypoint(4.468, 3.225);
                pathB.addWaypoint(2.130, 3.214);
                pathB.addWaypoint(1.927, 4.311);
                pathB.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:2.022,4.430
                pathB.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.951,4.490
            } else if (SmartDashboard.getBoolean("far 5", false)) {// path on
                pathB.addWaypoint(1.139, 2.868);
                pathB.addWaypoint(3.108, 1.555);
                pathB.addWaypoint(5.983, 0.887);
                pathB.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.345,0.732
                pathB.addWaypoint(6.830, 0.792);
                pathB.addWaypoint(3.645, 2.402);
                pathB.addWaypoint(2.130, 3.214);
                pathB.addWaypoint(1.925, 5.068);
                pathB.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:1.925,4.954
                pathB.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.936,5.045
            }
            if (SmartDashboard.getBoolean("ShootSecond", false)) { // path off
                pathB.addSequentialCommand(new FollowNote(true, true, 1, .75, -correctSeek, 900));// ENDPOS:2.929,4.156
                pathB.addSequentialCommand(new TurnAndShoot(0));
            }
            pathB.addStop();
            if (RobotContainer.isRed) {
                pathB.flipAllX();
            }
            addCommands(pathB.finalizePath());
        } else if (chooser.getSelected().equals("Speaker")) {
            MultiPartPath pathA = new MultiPartPath(RobotContainer.driveTrain, config, null);
            pathA.resetPosition(1.402, 5.552);
            pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:1.664,5.564
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.664,5.504
            if (SmartDashboard.getBoolean("do 3?", false)) {// path on
                pathA.addWaypoint(1.640, 4.240);
                pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, correctSeek, 900));// ENDPOS:2.929,4.156
                pathA.addWaypoint(2.487, 4.442);
                pathA.addSequentialCommand(new TurnAndShoot(-3));// ENDPOS:2.380,4.180
                // pathA.pivotInPlace(90);
            }
            if (SmartDashboard.getBoolean("do 2?", false)) {// path on
                // pathA.addWaypoint(1.855, 5.397);
                pathA.addSequentialCommand(new FollowNote(true, true, 2, 2, correctSeek, 400));// ENDPOS:2.917,5.576
                pathA.addWaypoint(2.547, 5.588);
                pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:2.476,5.695
                // pathA.pivotInPlace(90);

            }
            if (SmartDashboard.getBoolean("do 1?", false)) {
                // pathA.addWaypoint(1.951, 6.828);
                pathA.addSequentialCommand(new FollowNote(true, true, 2, 2, correctSeek, 400));// ENDPOS:2.917,6.888
                pathA.addWaypoint(2.523, 6.208);
                pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:2.320,6.160
            }

            if (RobotContainer.isRed) {
                pathA.flipAllX();
            }
            addCommands(pathA.finalizePath());

        } else if (chooser.getSelected().equals("Left")) {

        }
    }
}