package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANBus.ShooterIDs;
import frc.robot.Constants.Measured.ShooterMeasurements;
import frc.robot.Constants.Tunables.ShooterTunables;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax leadMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = leadMotor.getEncoder();
    private final SparkMax followMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, MotorType.kBrushless);

    /** input RPM, outputs volts */
    private final PIDController flywheelVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    /** input RPS, outputs volts */
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterMeasurements.FLYWHEEL_S, ShooterMeasurements.FLYWHEEL_V, ShooterMeasurements.FLYWHEEL_A);

    public final Trigger flywheelUpToSpeed = new Trigger(flywheelVelocityController::atSetpoint);
    public final Trigger inShootMode = new Trigger(this::inShootMode);

    /** the RPM calculated by the shot calculation system */
    private final DoubleSupplier shootSpeed;

    private boolean shootMode = false;

    public ShooterSubsystem(DoubleSupplier shootSpeed) {
        this.shootSpeed = shootSpeed;
        leadMotor.configure(
                ShooterTunables.FLYWHEEL_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // get the follower to follow the lead motor's CANID
        followMotor.configure(
                ShooterTunables.FLYWHEEL_MOTOR_CONFIG.follow(ShooterIDs.LEFT_MOTOR_ID, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        flywheelVelocityController.setTolerance(ShooterTunables.FLYWHEEL_RPM_TOLERANCE);

        flywheelVelocityController.setSetpoint(ShooterTunables.FLYWHEEL_IDLE_RPM);

        setDefaultCommand(runPID());
    }

    /** runs the velocity control, the RPM target will change if set to shoot mode */
    public Command runPID() {
        return run(() -> {
            // give the pid RPM
            double pidOutput;

            if (shootMode) {
                pidOutput = flywheelVelocityController.calculate(getFlywheelVelocity(), shootSpeed.getAsDouble());
            } else {
                pidOutput = flywheelVelocityController.calculate(getFlywheelVelocity());
            }

            double targetRPM = flywheelVelocityController.getSetpoint();

            // convert the rpm to rps because the feedforward takes units/s not units/m
            double feedforwardOutput = flywheelFeedforward.calculate(targetRPM / 60.0);

            double totalOutput = feedforwardOutput + pidOutput;

            double clampedOutput = MathUtil.clamp(totalOutput, -12.0, 12.0);

            leadMotor.setVoltage(clampedOutput);
        });
    }

    /** sets the RPM target to follow calculated RPM */
    public Command setShootMode() {
        return runOnce(() -> shootMode = true);
    }

    /** sets the RPM target to not change */
    public Command holdSpeed() {
        return runOnce(() -> {
            shootMode = false;
        });
    }

    /** sets the RPM target to the idle rpm */
    public Command setIdle() {
        return runOnce(() -> {
            shootMode = false;
            flywheelVelocityController.setSetpoint(ShooterTunables.FLYWHEEL_IDLE_RPM);
        });
    }

    /** sets the RPM target */
    public Command setHoldRPM(double targetRPM) {
        return runOnce(() -> {
            shootMode = false;
            flywheelVelocityController.setSetpoint(targetRPM);
        });
    }

    /** changes the RPM target by the amount */
    public Command changeHeldRPM(double change) {
        return runOnce(() -> {
            shootMode = false;
            double currentTargetRPM = flywheelVelocityController.getSetpoint();
            double newTargetRPM = currentTargetRPM + change;
            if (newTargetRPM > ShooterTunables.SHOOTER_RPM_CEILING) {
                newTargetRPM = ShooterTunables.SHOOTER_RPM_CEILING;
            }
            if (newTargetRPM < 0) {
                newTargetRPM = 0;
            }

            flywheelVelocityController.setSetpoint(newTargetRPM);
        });
    }

    /** a command that just sets the motor voltage and doesn't do anything fancy with pids */
    public Command setFlywheelMotorVoltage(double motorVoltage) {
        return runOnce(() -> {
            leadMotor.setVoltage(motorVoltage);
        });
    }

    /** returns the flywheel velocity in RPM */
    public double getFlywheelVelocity() {
        return flywheelEncoder.getVelocity();
    }

    /** returns if the target RPM is following the calculation */
    public boolean inShootMode() {
        return shootMode;
    }
}
