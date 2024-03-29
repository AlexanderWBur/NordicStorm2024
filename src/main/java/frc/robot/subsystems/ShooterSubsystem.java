package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {

    double targetAngle = 0;

    private DigitalInput prox = new DigitalInput(1);

    private CANSparkMax pinion = new CANSparkMax(Constants.shooterPitchID, MotorType.kBrushless);
    private SparkPIDController pinionPID = pinion.getPIDController();
    private RelativeEncoder pinionEncoder = pinion.getEncoder();

    private CANSparkMax indexer = new CANSparkMax(Constants.indexerID, MotorType.kBrushless);
    private SparkPIDController indexerPID = indexer.getPIDController();
    private RelativeEncoder indexerEncoder = indexer.getEncoder();

    private TalonFX shooter = new TalonFX(16);
    private VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0);

    private TalonFX amp = new TalonFX(17);
    private VelocityVoltage ampRequest = new VelocityVoltage(0).withSlot(0);

    public ShooterSubsystem() {
        setShooter(0);
        amp.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.5));  
        shooter.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.5));  
    }

    public void configurePinion() {
        pinion.setInverted(false);
        pinion.enableVoltageCompensation(12);
        pinion.setIdleMode(IdleMode.kBrake);
    }

    double targetAmp = 0;

    @Override
    public void periodic() {

        indexer.set(.2);
        if (Math.signum(amp.getVelocity().getValueAsDouble()) != Math.signum(targetAmp)
                && Math.abs(amp.getVelocity().getValueAsDouble()) > 10) {
            amp.setControl(new StaticBrake());
        } else {
            amp.setControl(ampRequest.withVelocity(targetAmp));

        }
        SmartDashboard.putNumber("Pinion", pinionEncoder.getPosition());
        SmartDashboard.putNumber("Am", amp.getVelocity().getValue());
        SmartDashboard.putNumber("Sh", shooter.getVelocity().getValue());
        SmartDashboard.putNumber("AmTe", amp.getDeviceTemp().getValue());
        SmartDashboard.putNumber("ShTe", shooter.getDeviceTemp().getValue());
        SmartDashboard.putNumber("AmCur", amp.getTorqueCurrent().getValue());
        SmartDashboard.putNumber("ShCur", shooter.getTorqueCurrent().getValue());
    }

    private double topCurrentRPM;
    private double bottomCurrentRPM;

    public void setFlywheelsRaw(double top, double bottom) {

    }

    private void updateShooter() {

    }

    public double getAmpError() {
        return amp.getVelocity().getValueAsDouble() - targetAmp;
    }

    public double getShooterError() {
        return shooter.getClosedLoopError().getValueAsDouble();
    }

    public void setPinionRaw(double power) {
        pinionPID.setReference(power, CANSparkMax.ControlType.kDutyCycle);
    }

    public double getAngleError() {
        return pinionEncoder.getPosition() - targetAngle;
    }

    public void setShooter(double velocity) {
        shooter.setControl(shooterRequest.withVelocity(velocity));
    }

    public void setShooterAngle(double ticks) {
        targetAngle = ticks;
        pinionPID.setReference(ticks, CANSparkMax.ControlType.kPosition);
    }

    public void setAmp(double velocity) {
        targetAmp = velocity;
    }

}
