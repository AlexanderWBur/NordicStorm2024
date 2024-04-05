package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.Util;
import frc.robot.commands.ShooterMode;

public class ShooterSubsystem extends SubsystemBase {

    private boolean hasSeenSensor;
    double targetAngle = 0;

    private DigitalInput sensor = new DigitalInput(1);

    private CANSparkMax pinion = new CANSparkMax(Constants.shooterPitchID, MotorType.kBrushless);
    private SparkPIDController pinionPID = pinion.getPIDController();
    private RelativeEncoder pinionEncoder = pinion.getEncoder();

    private TalonFX shooter = new TalonFX(16);
    private VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0);

    private TalonFX amp = new TalonFX(17);
    private VelocityVoltage ampRequest = new VelocityVoltage(0).withSlot(0);
    public double distance;

    public ShooterSubsystem() {
        setShooter(0);
        amp.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        shooter.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        amp.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.75));
        shooter.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.75));
    }

    public void configurePinion() {
        pinion.setInverted(false);
        pinion.enableVoltageCompensation(12);
        pinion.setIdleMode(IdleMode.kBrake);
    }

    public void resetHasShot() {
        hasSeenSensor = false;
    }

    public boolean hasShot(){

        return hasSeenSensor && sensor.get();
    }

    public double getRPM(double distance) {
        double x = distance;
        // double
        double result = 5.22845233503428*x*x + -15.647280208158447*x + 66.71118046450191; // CURVE:rpm,10:05,04/05
        // result = SmartDashboard.getNumber("targetRPM", 0);
        SmartDashboard.putNumber("curveRPM", result);
        result = SmartDashboard.getNumber("targetRPM", 0);
        return result;
    }

    public double getAngleForDist(double distance) {
        double x = distance;
        double result = 1.1036465538680458*x*x + -11.836409900141415*x + 61.64766671927669; // CURVE:angle,10:05,04/05
        result = Util.clamp(result, 1, 70); // +3, same ang
        SmartDashboard.putNumber("curvePitch", result);
        result = SmartDashboard.getNumber("targetPitch", 0);
        return result;
    }

    double targetAmp = 0;
    private ShooterMode mode = ShooterMode.OFF;

    @Override
    public void periodic() {
        if(!sensor.get()){
            hasSeenSensor = true;
        }
        distance = Util.distance(RobotContainer.driveTrain.getPose(), RobotContainer.targetLocation);

        SmartDashboard.putNumber("distance", distance);

        if (mode == ShooterMode.OFF) {
            setAmp(0);
            setShooter(0);
        } else if (mode == ShooterMode.AMP) {
            setAmp(-25);
            setShooter(25);
        } else if (mode == ShooterMode.SHOOT) {
            double targetRPM = getRPM(distance);
            setAmp(targetRPM); // was target RPM changed when shooter broke
            setShooter(targetRPM);

        } else if (mode == ShooterMode.PLOOP){
            setAmp(15);
            setShooter(15);
        } else if(mode == ShooterMode.PASS){
            setAmp(30);
            setShooter(30);
            setShooterAngle(-40);
        }

        if (Math.signum(amp.getVelocity().getValueAsDouble()) != Math.signum(targetAmp)
                && Math.abs(amp.getVelocity().getValueAsDouble()) > 10 && mode != ShooterMode.OFF) {
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

    public void setAmpMode() {
        mode = ShooterMode.AMP;
    }

    public void setShooterMode() {
        mode = ShooterMode.SHOOT;
    }

    public void setOffMode() {
        mode = ShooterMode.OFF;
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

    private void setShooter(double velocity) {
        shooter.setControl(shooterRequest.withVelocity(-velocity));
    }

    public void setShooterAngle(double ticks) {
        targetAngle = ticks;
        pinionPID.setReference(ticks, CANSparkMax.ControlType.kPosition);
    }

    private void setAmp(double velocity) {
        targetAmp = -velocity;
    }

    public void setMode(ShooterMode mode) {
        this.mode = mode;
    }

}