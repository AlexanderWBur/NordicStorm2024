package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.RobotContainer; // When to use this...
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Util;

public class AprilTagLockOn extends CommandPathPiece {
    /*
     * For future purposes:
     * This command has been made for the event that we will need to lock on to
     * areas with april tags.
     * The thinking behind this is that we may have the ability to offset the
     * distance of our target by a distance equal to half the radius form the center
     * of the tag. (There is a constant distance in between where we want to aim at
     * and the center of the april tag)
     */

    public static boolean currentlyRunning = false;
    private DriveTrainSubsystem drivetrain;
    private VisionSubsystem vision;
    private long timeout;
    private long endingTime;

    public boolean rotateTowardTarget() {
        double angleOffset = 0 * Math.atan2(0.154, 0); // aim 6in to the side **these have been set to 0
        double angleNeeded = 0; // **

        double angleDiff = Util.angleDiff(drivetrain.getGyroDegrees(), angleNeeded + angleOffset);
        double p = 0.000000; // **

        if (vision.canSeeTarget) {
            // angleDiff = -vision.best.getYaw()-angleOffset;
        }
        double correction = angleDiff * p; // rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);

        correction = Util.absClamp(correction, 5);
        drivetrain.setRotationSpeed(correction, 1);
        return Math.abs(angleDiff) < 3;
    }
}
