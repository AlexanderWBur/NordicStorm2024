package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ClimberSubsystem extends SubsystemBase {
    
    private CANSparkMax left = new CANSparkMax(Constants.climberLeftID, MotorType.kBrushless);
    private SparkPIDController leftPID = left.getPIDController();
    private RelativeEncoder leftEncoder = left.getEncoder();

    private CANSparkMax right = new CANSparkMax(Constants.climberRightID, MotorType.kBrushless);
    private SparkPIDController rightPID = right.getPIDController();
    private RelativeEncoder rightEncoder = right.getEncoder();

    public ClimberSubsystem(){

    }

    @Override
    public void periodic(){
        left.set(Util.leftDebug());
        right.set(Util.leftDebug());
    }
    
}
