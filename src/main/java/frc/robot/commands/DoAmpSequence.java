package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DoAmpSequence extends SequentialCommandGroup {

    public DoAmpSequence() {
        addRequirements(RobotContainer.driveTrain);
        addCommands(new Command() {
            // snaps wheels to amp
            @Override
            public void initialize() {

            }

            @Override
            public void execute() {

            }

            @Override
            public void end(boolean isInterupted) {

            }

            @Override
            public boolean isFinished() {

                return Math.abs(snapToSide()) < 3;
            }

        }, new Command() {
            boolean isDone = false;

            // strafe until aligned
            @Override
            public void initialize() {
                isDone = false;
            }

            @Override
            public void execute() {
                double targetX = 1.842;
                double currentX = RobotContainer.driveTrain.getPose().getX();
                double error = targetX - currentX;
                RobotContainer.driveTrain.drive(0, getStrafeSpeed(targetX), 0);
                snapToSide();
                isDone = Math.abs(error) < .1;

            }

            @Override
            public void end(boolean isInterupted) {

            }

            @Override
            public boolean isFinished() {

                return isDone;
            }

        }, new Command() {
            long timeToEnd = 0;

            // drive Forward
            @Override
            public void initialize() {
                timeToEnd = 0;
            }

            @Override
            public void execute() {
                double targetX = RobotContainer.isRed ? 5.93: 1.842;
                double currentX = RobotContainer.driveTrain.getPose().getX();
                double error = targetX - currentX;
                double forward;
                if (RobotContainer.driveTrain.isRangeValid()) {
                    forward = 0.75; //RobotContainer.driveTrain.getRange() * 0.001;
                    if (RobotContainer.driveTrain.getRange() < 150 && timeToEnd == 0) {
                        timeToEnd = System.currentTimeMillis() + 100;
                    }
                } else {
                    forward = 2;
                }
                RobotContainer.driveTrain.drive(forward, getStrafeSpeed(targetX), 0);
                snapToSide();

            }

            @Override
            public void end(boolean isInterupted) {
                RobotContainer.driveTrain.drive(0, 0, 0);
            }

            @Override
            public boolean isFinished() {
                return timeToEnd != 0 && System.currentTimeMillis() > timeToEnd;
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
                if (!RobotContainer.intake.hasNote()) {
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
        // if (vision.canSeeTarget) {
        // angleDiff = -vision.bestTarget.getYaw();
        // }
        RobotContainer.driveTrain.setRotationSpeed(RobotContainer.driveTrain.getTurnToTarget(angleNeeded), 1);
        return angleDiff;
    }
}
