package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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

    double targetAngle = 0;

    private CANSparkMax pinion = new CANSparkMax(Constants.shooterPitchID, MotorType.kBrushless);
   private SparkPIDController pinionPID = pinion.getPIDController();
   private RelativeEncoder pinionEncoder = pinion.getEncoder();

    private CANSparkMax shooter = new CANSparkMax(Constants.shooterID, MotorType.kBrushless);
    private SparkPIDController shooterPID = shooter.getPIDController();
    private RelativeEncoder shooterEncoder = shooter.getEncoder();

    private CANSparkMax amp = new CANSparkMax(Constants.ampID, MotorType.kBrushless);
    private SparkPIDController ampPID = amp.getPIDController();
    private RelativeEncoder ampEncoder = amp.getEncoder();

    

    public ShooterSubsystem() {
       
    }

    private void configureFlywheelMotor(CANSparkMax motor) {
        motor.setInverted(true);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void configurePinion(){
        pinion.setInverted(false);
        pinion.enableVoltageCompensation(12);
        pinion.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Pinion", pinionEncoder.getPosition());
        //  updateShooter(); // top then bottom
    // setFlywheelsRaw(.25 * RobotContainer.leftJoystick.getY(), -.4 * RobotContainer.leftJoystick.getY());
    }



    private double topCurrentRPM;
    private double bottomCurrentRPM;

    public void setFlywheelsRaw(double top, double bottom) {
       

    }

    private void updateShooter() {
       
    }


    public void setPinionRaw(double power){
        pinionPID.setReference(power, CANSparkMax.ControlType.kDutyCycle);
    }

    public double getAngleError(){
        return pinionEncoder.getPosition() - targetAngle;
    }

    private void setPinion(){

    }

    public void setShooterAngle(double ticks){
        targetAngle = ticks;
        pinionPID.setReference(ticks, CANSparkMax.ControlType.kPosition);
    }

    public void setShooterRaw(double power){
        shooterPID.setReference(power, CANSparkMax.ControlType.kDutyCycle);
    }

    public void setAmpRaw(double power){
        ampPID.setReference(power * .3, CANSparkMax.ControlType.kDutyCycle);
    }

    public void setAmp(){

    }

    private double topTargetRPM;
    private double bottomTargetRPM;

    public void updateFlywheels(double topRPM, double bottomRPM) {
        if (topRPM < 0 || bottomRPM < 0) {
            System.out.println("Error tried to set shooter to negative RPM");
        }

        topTargetRPM = topRPM;
        bottomTargetRPM = bottomRPM;

    
    }
}
