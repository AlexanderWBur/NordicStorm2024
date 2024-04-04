package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
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
        boolean doOne = SmartDashboard.getBoolean("do 1?", false);
        boolean doTwo = SmartDashboard.getBoolean("do 1?", false);
        boolean doThree = SmartDashboard.getBoolean("do 3?", false);
        boolean farThree = SmartDashboard.getBoolean("far 3", false);
        boolean farFour = SmartDashboard.getBoolean("far 4", false);
        boolean farFive = SmartDashboard.getBoolean("far 5", false);

        RobotContainer.intake.triggered = true;
        DriveTrainConfig config = RobotContainer.driveTrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 3;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;
        double correctSeek = RobotContainer.isRed ? 0.2 : -0.2;
        MultiPartPath pathA = new MultiPartPath(RobotContainer.driveTrain, config, null);
        if (chooser.getSelected().equals("Right")) { // path on
            pathA.resetPosition(1.426, 3.727);
            pathA.setHeading(RobotContainer.AllianceAngleDeg);
            if (SmartDashboard.getBoolean("ShootFirst", false)) { // path off
                pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:1.211,3.715
                pathA.addWaypoint(1.545, 4.729);
                pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.712,4.764
            } else { // path on
                pathA.addSequentialCommand(new SetShooter(ShooterMode.PLOOP));// ENDPOS:1.593,4.049
                pathA.addSequentialCommand(new Ploop());// ENDPOS:1.569,4.228
            }
        }
        if (chooser.getSelected().equals("Speaker")) { // path off
            pathA.resetPosition(1.402, 5.552);
            pathA.setHeading(RobotContainer.AllianceAngleDeg);
            if (SmartDashboard.getBoolean("ShootFirst", false)) { // path on
                pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:1.664,5.564
                pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.664,5.504
            } else { // path off
                pathA.addSequentialCommand(new SetShooter(ShooterMode.PLOOP));// ENDPOS:1.593,4.049
                pathA.addSequentialCommand(new Ploop());

            }
        }

        if (doThree) {// path off
            pathA.addWaypoint(1.640, 4.240);
            pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, correctSeek, 900));// ENDPOS:2.929,4.156
            pathA.addWaypoint(2.487, 4.442);
            pathA.addSequentialCommand(new TurnAndShoot(-3));// ENDPOS:2.380,4.180
            // pathA.pivotInPlace(90);
        }
        if (doTwo) {// path off
                    // pathA.addWaypoint(1.855, 5.397);
            pathA.addSequentialCommand(new FollowNote(true, true, 2, 2, correctSeek, 400));// ENDPOS:2.917,5.576
            pathA.addWaypoint(2.547, 5.588);
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:2.476,5.695
            // pathA.pivotInPlace(90);

        }
        if (doOne) { // path off
                     // pathA.addWaypoint(1.951, 6.828);
            pathA.addSequentialCommand(new FollowNote(true, true, 2, 2, correctSeek, 400));// ENDPOS:2.917,6.888
            pathA.addWaypoint(2.523, 6.208);
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:2.320,6.160
        }
        if (farFive || farThree || farFour) { // path on
            pathA.addSequentialCommand(new SetShooter(ShooterMode.OFF));// ENDPOS:1.593,4.049
        }
        if (farThree) {// path on
            pathA.addWaypoint(2.845, 2.832);
            pathA.addWaypoint(3.698, 2.770);
            pathA.addWaypoint(4.854, 3.990);
            pathA.addWaypoint(7.105, 4.037);
            pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.274,4.038
            pathA.addWaypoint(6.620, 4.022);
            pathA.addWaypoint(4.742, 3.990);
            pathA.addWaypoint(3.669, 2.844);
            pathA.addWaypoint(2.130, 3.214);
            pathA.addWaypoint(2.058, 4.932);
            pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:2.034,4.955
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.998,4.979

        } else if (farFour) { // path off

            pathA.addWaypoint(1.139, 2.868);
            pathA.addWaypoint(3.108, 1.555);
            pathA.addWaypoint(3.848, 1.722);
            pathA.addWaypoint(6.269, 1.997);
            pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.262,2.390
            pathA.addWaypoint(7.128, 3.023);
            pathA.addWaypoint(5.745, 4.240);
            pathA.addWaypoint(4.468, 3.225);
            pathA.addWaypoint(2.130, 3.214);
            pathA.addWaypoint(1.927, 4.311);
            pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:2.022,4.430
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.951,4.490
        } else if (farFive) {// path off
            pathA.addWaypoint(1.139, 2.868);
            pathA.addWaypoint(3.108, 1.555);
            pathA.addWaypoint(5.983, 0.887);
            pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.345,0.732
            pathA.addWaypoint(6.830, 0.792);
            pathA.addWaypoint(3.645, 2.402);
            pathA.addWaypoint(2.130, 3.214);
            pathA.addWaypoint(1.927, 4.311);
            pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// ENDPOS:2.022,4.430
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.951,4.490
        }
        if (SmartDashboard.getBoolean("ShootSecond", false)) { // path on
            pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, -correctSeek, 900));// ENDPOS:1.306,4.096
            pathA.addSequentialCommand(new TurnAndShoot(0));// ENDPOS:1.294,4.144
        }

        pathA.addStop();
        if (RobotContainer.isRed) {
            pathA.flipAllX();
        }

        addCommands(pathA.finalizePath());

    }
}