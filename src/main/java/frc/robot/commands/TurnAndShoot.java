package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.Util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnAndShoot extends CommandPathPiece {

    boolean hasSent;
    static double angleAdjust;
    long timeToEnd = 0;
    public TurnAndShoot(double angleAdjust) {
        addRequirements(RobotContainer.intake);
        SmartDashboard.putNumber("targetRPM", 50);
        SmartDashboard.putNumber("targetPitch", 40);
        this.angleAdjust = angleAdjust;
    }

    public static double getNeededTurnAngle() {
        Pose2d futurePose = RobotContainer.driveTrain.getPose();

        double angleNeeded = Util.angleBetweenPoses(futurePose, RobotContainer.aimingLocation) + Math.PI;

        return Math.toDegrees(angleNeeded) + 10 + angleAdjust; // 10

    }

    public boolean rotateTowardTarget() {
        double angleNeeded = getNeededTurnAngle();
        double angleDiff = Util.angleDiff(RobotContainer.driveTrain.getGyroDegrees(), angleNeeded);
        RobotContainer.driveTrain.setRotationSpeed(RobotContainer.driveTrain.getTurnToTarget(angleNeeded), 1);
        return Math.abs(angleDiff) < 3;
    }

    @Override
    public void initialize() {
        RobotContainer.shooterSubsystem.resetHasShot();
        hasSent = false;
        timeToEnd = 0;
    }

    @Override
    public void execute() {
        RobotContainer.driveTrain.drive(0,0,0);
        double targetPitch = RobotContainer.shooterSubsystem.getAngleForDist(RobotContainer.shooterSubsystem.distance);
        
        RobotContainer.shooterSubsystem.setShooterAngle(-targetPitch);

        boolean isAngleGood = rotateTowardTarget();
        if (Math.abs(RobotContainer.shooterSubsystem.getAngleError()) < .5 &&
         !hasSent && isAngleGood && Math.abs(RobotContainer.shooterSubsystem.getShooterError()) < 3 
         && Math.abs(RobotContainer.shooterSubsystem.getAmpError()) < 3) {
            RobotContainer.intake.sendToShooter();
            hasSent = true;
            timeToEnd = System.currentTimeMillis() + 1000;
        }

    
    }

    @Override
    public void end(boolean interrupted) {
        
        RobotContainer.shooterSubsystem.setShooterAngle(-2);
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() > timeToEnd  && hasSent) || RobotContainer.shooterSubsystem.hasShot();
    }
}
