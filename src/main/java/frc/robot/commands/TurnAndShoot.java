package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

// import com.ctre.phoenix.Util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnAndShoot extends CommandPathPiece {

    boolean hasSent;

    public TurnAndShoot() {
        addRequirements(RobotContainer.intake);
        // rotationPID.enableContinuousInput(-180, 180);
    }

    public static double getNeededTurnAngle(){
        Pose2d futurePose = RobotContainer.driveTrain.getPose();

        double angleNeeded = Util.angleBetweenPoses(futurePose, RobotContainer.targetLocation)+Math.PI;

        return Math.toDegrees(angleNeeded);

    }
     private PIDController rotationPID = new PIDController(0.115,0 , 0.007);
    public boolean rotateTowardTarget() {
        double angleNeeded = getNeededTurnAngle();

        double angleDiff = Util.angleDiff(RobotContainer.driveTrain.getGyroDegrees(), angleNeeded);
        // if (vision.canSeeTarget) {
        //     angleDiff = -vision.bestTarget.getYaw();
        // }
        double correction = rotationPID.calculate(-angleDiff , 0); // rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);

        correction = Util.absClamp(correction, 5);
        RobotContainer.driveTrain.setRotationSpeed(correction, 1);
        return Math.abs(angleDiff) < 3;
    }

    @Override
    public void initialize() {
        hasSent = false;
        RobotContainer.shooterSubsystem.setShooterAngle(-30);
    }

    @Override
    public void execute() {
       boolean isAngleGood = rotateTowardTarget();
        if (Math.abs(RobotContainer.shooterSubsystem.getAngleError()) < .5 && !hasSent && isAngleGood) {
            RobotContainer.intake.sendToShooter();
            hasSent = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooterSubsystem.setShooterAngle(-5);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
