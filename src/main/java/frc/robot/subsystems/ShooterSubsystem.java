package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

private CANSparkMax bottomWheel = new CANSparkMax(Constants.lowerFlywheelID, MotorType.kBrushless);
private SparkPIDController bottomWheelPID = bottomWheel.getPIDController();
private RelativeEncoder bottomWheelEncoder = bottomWheel.getEncoder();

private CANSparkMax topWheel = new CANSparkMax(Constants.upperFlywheelID, MotorType.kBrushless);
private SparkPIDController topWheelPID = topWheel.getPIDController();
private RelativeEncoder topWheelEncoder = topWheel.getEncoder();

    public ShooterSubsystem(){
        configureFlywheelMotor(topWheel);
        configureFlywheelMotor(bottomWheel);

    }

    private void configureFlywheelMotor(CANSparkMax motor) {
        motor.setInverted(true);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic(){
        updateShooter(); // top then bottom
        setFlywheelsRaw(.25* RobotContainer.leftJoystick.getY(),-.4 *RobotContainer.leftJoystick.getY());
    }

    private double topCurrentRPM;
    private double bottomCurrentRPM;

    public void setFlywheelsRaw(double top, double bottom){
        topWheelPID.setReference(top, CANSparkMax.ControlType.kDutyCycle);
        bottomWheelPID.setReference(bottom, CANSparkMax.ControlType.kDutyCycle);

    }

    private void updateShooter() {
        topCurrentRPM = topWheelEncoder.getVelocity();
        bottomCurrentRPM = bottomWheelEncoder.getVelocity();

        SmartDashboard.putNumber("Top wheel velocity",topCurrentRPM);
        SmartDashboard.putNumber("Bottom wheel velocity", bottomCurrentRPM);


    }

    private double topTargetRPM;
    private double bottomTargetRPM;

    public void updateFlywheels(double topRPM, double bottomRPM){
        if(topRPM < 0 || bottomRPM < 0) {
            System.out.println("Error tried to set shooter to negative RPM");
        }

        topTargetRPM = topRPM;
        bottomTargetRPM = bottomRPM;

        topWheelPID.setReference(topRPM, CANSparkMax.ControlType.kVelocity);
        bottomWheelPID.setReference(bottomRPM, CANSparkMax.ControlType.kVelocity);

    }
}

