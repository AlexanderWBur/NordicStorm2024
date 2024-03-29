package frc.robot.subsystems;

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
import frc.robot.commands.SetRumble;

public class IntakeSubsystem extends SubsystemBase {

    private DigitalInput prox = new DigitalInput(0);
    private CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkPIDController motorPID = motor.getPIDController();
    private RelativeEncoder motorEncoder = motor.getEncoder();

    public boolean hasNote;
    private long timeOut;
    private long timeToStop = 0;
    private boolean triggered = true;
    private double ticksToStopIntake;
    private double ticksToStopFeed;

    public IntakeSubsystem() {
        configureIntakeMotor(motor);
    }

    private void configureIntakeMotor(CANSparkMax motor) {
        motor.setInverted(true);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void doIntake(long timeOut) {
        triggered = hasNote;
        timeToStop = System.currentTimeMillis() + timeOut;
        if(hasNote){
            timeToStop = 0;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("feed", ticksToStopFeed);
        SmartDashboard.putNumber("stop", ticksToStopIntake);
        SmartDashboard.putNumber("current", motorEncoder.getPosition());

        // indexerPID.setReference(Util.leftDebug(),
        // CANSparkMax.ControlType.kDutyCycle);
        updateMotorStats();
        hasNote = !prox.get();
        if (!triggered && hasNote) {
            ticksToStopIntake = motorEncoder.getPosition() + 0.5 + 0.2*(Util.clamp(2.5 - RobotContainer.driveTrain.getSpeeds().vxMetersPerSecond, 0, 2.5));
            timeToStop = 0;
            new SetRumble(1).schedule();
        }
        if (hasNote) {
            triggered = true;
        }

        if (motorEncoder.getPosition() < ticksToStopFeed) {
            setMotorRaw(1);

        } else if (motorEncoder.getPosition() < ticksToStopIntake) {
            setMotorRaw(Constants.minIntakePower);

        } else if (System.currentTimeMillis() < timeToStop) {
            setMotorRaw(Constants.minIntakePower);

        } else {
            setMotorRaw(0);

        }

        if (motorEncoder.getPosition() > ticksToStopFeed) {
        }
        SmartDashboard.putBoolean("has note", hasNote);
    }

    public void sendToShooter() {
        ticksToStopFeed = motorEncoder.getPosition() + 30;

    }

    public void stopIntake() {
        timeToStop = 0;
    }

    public void idleIndexer(){
        
    }

    private double currentRPM;

    public void setMotorRaw(double rpm) {
        motorPID.setReference(rpm, CANSparkMax.ControlType.kDutyCycle);

    }

    private void updateMotorStats() {
        currentRPM = motorEncoder.getVelocity();
        SmartDashboard.putNumber("Top wheel velocity", currentRPM);

    }

    public boolean hasNote() {

        return false;
    }

    public boolean noteDetected() {

        return false;
    }

    private double targetRPM;

    public void updateRPM(double rpm) {
        if (rpm < 0) {
            System.out.println("Error tried to set shooter to negative RPM");
        }

        targetRPM = rpm;

        motorPID.setReference(rpm, CANSparkMax.ControlType.kVelocity);

    }
}
