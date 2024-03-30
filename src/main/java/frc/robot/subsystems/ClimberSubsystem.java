package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax left = new CANSparkMax(Constants.climberLeftID, MotorType.kBrushless);
    private SparkPIDController leftPID = left.getPIDController();
    private RelativeEncoder leftEncoder = left.getEncoder();

    private CANSparkMax right = new CANSparkMax(Constants.climberRightID, MotorType.kBrushless);
    private SparkPIDController rightPID = right.getPIDController();
    private RelativeEncoder rightEncoder = right.getEncoder();

    public ClimberSubsystem() {
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
        left.setSoftLimit(SoftLimitDirection.kForward, 350);
        left.setSoftLimit(SoftLimitDirection.kReverse, 2);
        right.setSoftLimit(SoftLimitDirection.kForward, 350);
        right.setSoftLimit(SoftLimitDirection.kReverse, 2);

    }

    public void setLimitsEnabled(boolean enabled) {
        left.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        left.enableSoftLimit(SoftLimitDirection.kReverse, enabled);
        right.enableSoftLimit(SoftLimitDirection.kForward, enabled);
        right.enableSoftLimit(SoftLimitDirection.kReverse, enabled);

    }

    public void resetPosition(){
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        if(RobotContainer.leftJoystick.getRawButton(7)){
            left.set(-Util.leftDebug());
        } else {
            left.set(0);
        }
        if(RobotContainer.leftJoystick.getRawButton(10)){
            right.set(-Util.leftDebug());
        } else {
            right.set(0);
        }
        SmartDashboard.putNumber("Right Amps", right.getOutputCurrent());
        SmartDashboard.putNumber("Left Amps", left.getOutputCurrent());
        SmartDashboard.putNumber("Right pos", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left pos", leftEncoder.getPosition());

    }

}
