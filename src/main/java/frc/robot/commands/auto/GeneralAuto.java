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
        SmartDashboard.putBoolean("far 1", false);
        SmartDashboard.putBoolean("far 2", false);
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
        boolean farOne = SmartDashboard.getBoolean("far 1", false);
        boolean farTwo = SmartDashboard.getBoolean("far 2", false);
        boolean farThree = SmartDashboard.getBoolean("far 3", false);
        boolean farFour = SmartDashboard.getBoolean("far 4", false);
        boolean farFive = SmartDashboard.getBoolean("far 5", false);

        RobotContainer.intake.resetIntake();
        DriveTrainConfig config = RobotContainer.driveTrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 3;
        config.maxCentripetalAcceleration = 11;
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;
        double correctSeek = RobotContainer.isRed ? -0.2 : 0.2;
        MultiPartPath pathA = new MultiPartPath(RobotContainer.driveTrain, config, null);
        if (chooser.getSelected().equals("Left")) { // path off
            pathA.resetPosition(1.366, 6.554);
            pathA.setHeading(RobotContainer.AllianceAngleDeg);
            pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
            pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
            if (doOne) { // path on
                pathA.addWaypoint(1.795, 7.019);
                pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, correctSeek, 900));// ENDPOS:2.929,7.007
                pathA.addWaypoint(1.366, 6.554);
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
            } else { // path off
                pathA.addWaypoint(2.106, 7.628);
                pathA.addWaypoint(2.666, 7.699);
                pathA.addWaypoint(3.263, 7.664);

            }
            pathA.addSequentialCommand(new SetShooter(ShooterMode.OFF)); // NOMOVE
            if (farThree || farTwo || farOne) {
                pathA.addWaypoint(4.027, 7.616);
                pathA.addWaypoint(5.589, 7.282);
            }
            if (farOne) { // path on
                pathA.addWaypoint(6.592, 7.353);
                pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.333,7.461
                pathA.addWaypoint(5.625, 7.055);
                pathA.addWaypoint(4.874, 6.459);
            } else if (farTwo) { // path off
                pathA.addWaypoint(6.735, 5.898);
                pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.310,5.791
                pathA.addWaypoint(6.055, 6.339);
                pathA.addWaypoint(4.874, 6.459);
            } else if (farThree) { // path off
                pathA.addWaypoint(6.675, 6.113);
                pathA.addWaypoint(6.866, 4.049);
                pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.286,4.120
                pathA.addWaypoint(6.293, 4.013);
                pathA.addWaypoint(4.850, 4.061);
                pathA.addWaypoint(4.038, 5.671);
            }
            if (doOne) { // path off
                pathA.addWaypoint(2.905, 7.007);
                pathA.addWaypoint(1.473, 6.566);
            } else { // path on
                pathA.addWaypoint(2.881, 5.600);
                pathA.addWaypoint(1.438, 6.482);

            }
            pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
            pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE

        } else { // path on
            if (chooser.getSelected().equals("Right")) { // path off
                pathA.resetPosition(1.438, 4.216);
                pathA.setHeading(RobotContainer.AllianceAngleDeg);
                if (SmartDashboard.getBoolean("ShootFirst", false)) { // path on
                    pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
                    pathA.addWaypoint(1.008, 4.502);
                    pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
                } else { // path off
                    pathA.addSequentialCommand(new SetShooter(ShooterMode.PLOOP));// NOMOVE
                    pathA.addSequentialCommand(new Ploop());// NOMOVE
                }
            }
            if (chooser.getSelected().equals("Speaker")) { // path on
                pathA.resetPosition(1.402, 5.552);
                pathA.setHeading(RobotContainer.AllianceAngleDeg);
                if (SmartDashboard.getBoolean("ShootFirst", false)) { // path on
                    pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
                    pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
                } else { // path off
                    pathA.addSequentialCommand(new SetShooter(ShooterMode.PLOOP));// NOMOVE
                    pathA.addSequentialCommand(new Ploop());

                }
            }

            if (doThree) {// path on
                pathA.addWaypoint(1.640, 4.240);
                pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, correctSeek, 900));// ENDPOS:2.929,4.156
                pathA.addWaypoint(2.165, 4.884);
                pathA.addSequentialCommand(new TurnAndShoot(-3));// NOMOVE
                // pathA.pivotInPlace(90);
            }
            if (doTwo) {// path on
                        // pathA.addWaypoint(1.855, 5.397);
                pathA.addSequentialCommand(new FollowNote(true, true, 2, 2, correctSeek, 400));// ENDPOS:2.917,5.576
                pathA.addWaypoint(2.201, 5.647);
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
                // pathA.pivotInPlace(90);

            }
            if (doOne) { // path off
                         // pathA.addWaypoint(1.951, 6.828);
                pathA.addSequentialCommand(new FollowNote(true, true, 2, 2, correctSeek, 400));// ENDPOS:2.917,6.888
                pathA.addWaypoint(2.010, 5.946);
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
            }
            if (farFive || farThree || farFour) { // path on
                pathA.addSequentialCommand(new SetShooter(ShooterMode.OFF));// NOMOVE
            }
            if (farThree) {// path on
                pathA.addWaypoint(2.845, 2.832);
                pathA.addWaypoint(3.811, 2.979);
                pathA.addWaypoint(4.854, 3.990);
                pathA.addWaypoint(7.105, 4.037);
                pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.274,4.038
                pathA.addWaypoint(6.620, 4.022);
                pathA.addWaypoint(4.742, 3.990);
                pathA.addWaypoint(4.276, 3.123);
                pathA.addWaypoint(2.130, 3.214);
                pathA.addWaypoint(2.058, 4.932);
                pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE

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
                pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
            } else if (farFive) {// path off
                pathA.addWaypoint(1.139, 2.868);
                pathA.addWaypoint(3.108, 1.555);
                pathA.addWaypoint(5.983, 0.887);
                pathA.addSequentialCommand(new FollowNote(true, true, 3, 2, correctSeek, 300));// ENDPOS:8.345,0.732
                pathA.addWaypoint(6.830, 0.792);
                pathA.addWaypoint(3.645, 2.402);
                pathA.addWaypoint(2.130, 3.214);
                pathA.addWaypoint(1.927, 4.311);
                pathA.addSequentialCommand(new SetShooter(ShooterMode.SHOOT));// NOMOVE
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
            }
            if (SmartDashboard.getBoolean("ShootSecond", false)) { // path off
                pathA.addSequentialCommand(new FollowNote(true, true, 1, .75, -correctSeek, 900));// ENDPOS:1.306,4.096
                pathA.addSequentialCommand(new TurnAndShoot(0));// NOMOVE
            }
        }
        pathA.addStop();
        if (RobotContainer.isRed) {
            pathA.flipAllX();
        }

        addCommands(pathA.finalizePath());

    }
}