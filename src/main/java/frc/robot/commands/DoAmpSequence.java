package frc.robot.commands;

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
                RobotContainer.shooterSubsystem.setShooterAngle(-70); // need measure before enable
            }

            @Override
            public void execute() {
                double targetX = 1.842;
                double currentX = RobotContainer.driveTrain.getPose().getX();
                double error = targetX - currentX;
                double forward;
                if (RobotContainer.driveTrain.isRangeValid()) {
                    forward = RobotContainer.driveTrain.getRange() * 0.0005;
                    if (forward < .1) {
                        forward = .1;
                    }
                    if (RobotContainer.driveTrain.getRange() < 150) {
                        timeToEnd = System.currentTimeMillis() + 1000;
                    }
                } else {
                    forward = 1;
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

                return timeToEnd != 0 && System.currentTimeMillis() > timeToEnd
                        && Math.abs(RobotContainer.shooterSubsystem.getAngleError()) < 2;
            }

        }, new Command() {
            long timeToEnd = 0;

            // drive Forward
            @Override
            public void initialize() {
                timeToEnd = 0;
               RobotContainer.intake.sendToShooter();
            }

            @Override
            public void execute() {
                if(!RobotContainer.intake.hasNote()){
                    timeToEnd = System.currentTimeMillis() + 500;
                }
            }

            @Override
            public void end(boolean isInterupted) {
                RobotContainer.shooterSubsystem.setShooterAngle(-5);
            }

            @Override
            public boolean isFinished() {

                return timeToEnd != 0 && System.currentTimeMillis() > timeToEnd;
            }

        });
    }

    public double getStrafeSpeed(double targetX) {
        double currentX = RobotContainer.driveTrain.getPose().getX();
        double error = targetX - currentX;
        return Util.absClamp(-error, 1.5);
    }

    public double snapToSide() {
        double angleNeeded = 90;

        double angleDiff = Util.angleDiff(RobotContainer.driveTrain.getGyroDegrees(), angleNeeded);
        double p = 0.115;
        // if (vision.canSeeTarget) {
        // angleDiff = -vision.bestTarget.getYaw();
        // }
        double correction = angleDiff * p; // rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);

        correction = Util.absClamp(correction, 5);
        RobotContainer.driveTrain.setRotationSpeed(correction, 1);
        return angleDiff;
    }
}
