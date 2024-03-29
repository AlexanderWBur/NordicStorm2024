package frc.robot.commands;

//import com.ctre.phoenix.Util;
//import frc.robot.Utils.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.DriveTrainConfig;
//import frc.robot.subsystems.ShooterSubsystem;

public class OperatorControl extends Command {
    private DriveTrainConfig config;

    public OperatorControl() {
        this.config = RobotContainer.driveTrain.getConfig();
        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


        var leftStick = RobotContainer.leftJoystick;
        var rightStick = RobotContainer.rightJoystick;
        // double forward = -rightStick.getY();
        // double sideways = -rightStick.getX();
        // double rot = -rightStick.getTwist();
        // SmartDashboard.putNumber("stick", rot);
        var xbox = RobotContainer.xbox;
        double forward = -xbox.getLeftY();
        double sideways = -xbox.getLeftX();
        double rot = -xbox.getRightX();

        double throttle = rightStick.getThrottle();
        throttle = Util.map(throttle, 1, -1, 0.1, 1);
        // if (forward < 0.008 && rot >= 0.14) {// weird thing with joystick
        //     // rot = Util.map(rot, 0.15, in_max, out_min, out_max);
        //     rot -= 0.14;
        // }
        if (forward > 0.7 && false) {
            RobotContainer.driveTrain.driveVolts(new ChassisSpeeds(12, 0, 0));
            return;
        }
        throttle *= config.maxVelocity;

        forward = Util.applyDeadzone(forward, 0.1) * throttle;
        sideways = Util.applyDeadzone(sideways, 0.1) * throttle;
        rot = Util.applyDeadzone(rot, 
        0.2);
        rot = Util.signedSquare(rot);
        rot *= 5;

        ChassisSpeeds localSpeeds = Util.rotateSpeeds(new ChassisSpeeds(forward, sideways, rot),
        RobotContainer.driveTrain.getGyroRadians() + RobotContainer.AllianceAngleRad);
        // SmartDashboard.putNumber("Local Speed", localSpeeds.vyMetersPerSecond);
        RobotContainer.driveTrain.limitDrive(localSpeeds, 0);

        
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
