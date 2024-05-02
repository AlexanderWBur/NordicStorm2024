package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

    private CANSparkMax indexer = new CANSparkMax(Constants.indexerID, MotorType.kBrushless);
    private SparkPIDController indexerPID = indexer.getPIDController();
    private RelativeEncoder indexerEncoder = indexer.getEncoder();

    public boolean hasNote;
    private long timeOut;
    private long timeToStop = 0;
    public boolean triggered = true;
    private double ticksToStopIntake;
    private double ticksToStopFeed;

    public IntakeSubsystem() {
        configureIntakeMotor(motor);
    }

    private void configureIntakeMotor(CANSparkMax motor) {
        motor.setInverted(true);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void doIntake(long timeOut) {
        triggered = hasNote;
        timeToStop = System.currentTimeMillis() + timeOut;
        if (hasNote) {
            timeToStop = 0;
        }
    }

    public void resetIntake() {
        triggered = true;
        timeToStop = 0;
        ticksToStopFeed = -10000;
        ticksToStopIntake = -10000;
    }

    @Override
    public void periodic() {

        if (!hasNote) {
            if (RobotContainer.visionSubsystem.getTargets().size() > 0 && DriverStation.isTeleopEnabled()) {
                RobotContainer.xbox.setRumble(RumbleType.kRightRumble, 1);
            } else {
                RobotContainer.xbox.setRumble(RumbleType.kRightRumble, 0);
            }
        }
        SmartDashboard.putNumber("feed", ticksToStopFeed);
        SmartDashboard.putNumber("stop", ticksToStopIntake);
        SmartDashboard.putNumber("current", motorEncoder.getPosition());

        // indexerPID.setReference(Util.leftDebug(),
        // CANSparkMax.ControlType.kDutyCycle);
        updateMotorStats();
        hasNote = !prox.get();
        if (!triggered && hasNote) {
            ticksToStopIntake = motorEncoder.getPosition() + 0.2
                    + 0.2
                     * (Util.clamp(2.5
                            - RobotContainer.driveTrain.getSpeeds().vxMetersPerSecond, -0.5, 2.5));
            timeToStop = 0;
        }
        if (hasNote) {
            triggered = true;
        }
        indexer.set(0.2);
        if (motorEncoder.getPosition() < ticksToStopFeed) {
            setMotorRaw(1);
            // indexer.set(0.5);

        } else if (motorEncoder.getPosition() < ticksToStopIntake) {
            setMotorRaw(Constants.minIntakePower);
            // indexer.set(0);

        } else if (System.currentTimeMillis() < timeToStop) {
            setMotorRaw(Constants.minIntakePower);
            // indexer.set(0);

        } else {
            setMotorRaw(0);
            // indexer.set(0);

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

    public void idleIndexer() {

    }

    private double currentRPM;

    public void setMotorRaw(double rpm) {
        motorPID.setReference(rpm, CANSparkMax.ControlType.kDutyCycle);

    }

    private void updateMotorStats() {
        currentRPM = motorEncoder.getVelocity();
        SmartDashboard.putNumber("Top wheel velocity", currentRPM);

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
