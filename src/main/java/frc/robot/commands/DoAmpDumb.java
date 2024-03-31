package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DoAmpDumb extends SequentialCommandGroup {

    public DoAmpDumb() {
        addRequirements(RobotContainer.driveTrain);
        addCommands(new Command() {
            // snaps wheels to amp
            @Override
            public boolean isFinished() {

                return Math.abs(snapToSide()) < 3;
            }

        }, new Command() {
            long timeToEnd = 0;

            // send to shooter
            @Override
            public void initialize() {
                timeToEnd = 0;
                RobotContainer.shooterSubsystem.setShooterAngle(-70); // need measure before enable

            }

            @Override
            public void execute() {
                if (Math.abs(RobotContainer.shooterSubsystem.getAngleError()) < 3) {
                    RobotContainer.intake.sendToShooter();
                    timeToEnd = 0;
                }
                if (!RobotContainer.intake.hasNote) {
                    timeToEnd = System.currentTimeMillis() + 500;
                }
            }

            @Override
            public void end(boolean isInterupted) {
                RobotContainer.shooterSubsystem.setShooterAngle(-2);
            }

            @Override
            public boolean isFinished() {

                return timeToEnd != 0 && System.currentTimeMillis() > timeToEnd;
            }

        });
    }

    PIDController strafePID = new PIDController(3, 0, .1);
    public double getStrafeSpeed(double targetX) {
        double currentX = RobotContainer.driveTrain.getPose().getX();
        return -Util.absClamp(strafePID.calculate(currentX, targetX), 3);
    }

    public double snapToSide() {
        double angleNeeded = 90;

        double angleDiff = Util.angleDiff(RobotContainer.driveTrain.getGyroDegrees(), angleNeeded);

        RobotContainer.driveTrain.setRotationSpeed(RobotContainer.driveTrain.getTurnToTarget(angleNeeded), 1);
        return angleDiff;
    }
}
