// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 *
 */
public class FollowNote extends CommandPathPiece {

    int camWidth = 320;
    int camHeight = 240;

    double turnValue = 0;
    double forwardValue = 0;
    double maxTurn = 3;
    double pVal = 3; // 5
    double proxPVal = 0.07 * 0;
    double stopWidth = 80;
    double forwardMod = 2;
    boolean fullAuto = true;
    boolean hasGotABall = false;
    boolean shouldStop = false;
    int currentFollowingID = -1;
    long timeToEndDrive = 0;
    int widthMetFor = -1; // this takes an ID if the ball becomes wide enough to be chargeworthy

    long chargeTime = 0;

    boolean doIntake;
    boolean endWhenClose;
    boolean canAbort = false;
    double chargeSpeed = 0;

    public int targetColor = -1;

    public FollowNote(boolean handleIntake, boolean endWhenClose,
            double forwardMod, double chargeSpeed, boolean canAbort, long chargeTime) {

        this.doIntake = handleIntake;
        this.endWhenClose = endWhenClose;
        this.forwardMod = forwardMod;
        this.targetColor = targetColor;

        this.chargeSpeed = chargeSpeed;
        this.canAbort = canAbort;
        // chargeTime = (long) ((2/chargeSpeed)*300);
        this.chargeTime = chargeTime;
        addRequirements(RobotContainer.driveTrain);

    }

    // Called just before this Command runs the first time
    DriveToObject targetTracker;

    @Override
    public void initialize() {
        targetTracker = new DriveToObject(pVal, forwardMod, maxTurn, stopWidth * 0, proxPVal, camWidth, camHeight);
        targetTracker.setOffset(-55);
        RobotContainer.intake.doIntake(99999999);
    }

    private ProcessedTarget findTarget(List<PhotonTrackedTarget> possibleTargets) {

        if (possibleTargets.size() > 0) {
            return new ProcessedTarget(possibleTargets.get(0));
        }
        return null;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        if (timeToEndDrive < System.currentTimeMillis()) {
            if (turnValue == 0) {
                turnValue = 0.2;
            }
            if (hasGotABall && endWhenClose) {
                shouldStop = true;// The timer has run out after we have grabbed a ball

            }
            List<PhotonTrackedTarget> objects = RobotContainer.visionSubsystem.getTargets();
            ProcessedTarget object = findTarget(objects);
            if (object != null) {
                System.out.println("width:" + object.width);
                System.out.println("height:" + object.height);
                System.out.println("y:" + object.y);
                if (object.y > 220) {
                    if (endWhenClose) {
                        hasGotABall = true;
                    }
                    // System.out.println("charge!");

                    timeToEndDrive = System.currentTimeMillis() + chargeTime;
                }
            }
            if (object != null) {
                double[] speeds = targetTracker.execute(object.x, object.width);
                turnValue = speeds[0];
                forwardValue = speeds[1];
            } else {
                if (turnValue < 0) {
                    turnValue = -maxTurn * 0.75;
                } else if (turnValue > 0) {
                    turnValue = maxTurn * 0.75;
                }
                // turnValue = 0;
                forwardValue = 0;
                widthMetFor = -1;

            }
            if (Math.abs(turnValue) > 0.1 && Math.abs(turnValue) < 0.15) {
                if (turnValue < 0) {
                    turnValue = -0.15;
                } else {
                    turnValue = 0.15;
                }
            }
        } else {
            forwardValue = chargeSpeed;
            turnValue = 0;
        }
        turnValue *= Math.abs(RobotContainer.rightJoystick.getY());
        forwardValue *= Math.abs(RobotContainer.rightJoystick.getY());
        RobotContainer.driveTrain.limitDrive(new ChassisSpeeds(forwardValue, -turnValue * 0.1, -turnValue), 2);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {

        return shouldStop;// || barrel.hasBottomBall();
    }

    @Override
    public void end(boolean isInterupted){
        RobotContainer.intake.stopIntake();
    }

    @Override
    public double getRequestedStartSpeed() {
        return 1;
    }

    private class ProcessedTarget {
        double x;
        double y;
        double width;
        double height;

        public ProcessedTarget(PhotonTrackedTarget target) {
            double minX = 1000;
            double maxX = 0;
            double minY = 1000;
            double maxY = 0;

            for (var corner : target.getMinAreaRectCorners()) {
                System.out.println(corner.x);
                if (corner.x < minX) {
                    minX = corner.x;
                }
                if (corner.y < minY) {
                    minY = corner.y;
                }
                if (corner.x > maxX) {
                    maxX = corner.x;
                }
                if (corner.y > maxY) {
                    maxY = corner.y;
                }
            }
            y = minY;
            width = maxX - minX;
            height = maxY - minY;
            x = minX + (width/2);
        }
    }
}
