package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANBus.ShooterIDs;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.PIDTools.HotPIDFTuner;
import frc.robot.PIDTools.PIDCommandGenerator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax rightMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    /** input RPM, outputs volts */
    private final PIDController rightVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    /** input RPM, outputs volts */
    private final PIDController leftVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    /** setting the target velocity will also set the shooter to hold velocity */
    public final PIDCommandGenerator<AngularVelocity> velocityControllerCommands =
            new PIDCommandGenerator<AngularVelocity>(
                    (target) -> {
                        double targetRPM = target.in(RotationsPerSecond);
                        idle = true;
                        rightVelocityController.setSetpoint(targetRPM);
                        leftVelocityController.setSetpoint(targetRPM);
                    },
                    this,
                    rightVelocityController,
                    leftVelocityController);

    /** input RPS, outputs volts, this feedforward can be used for both motors */
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterTunables.FLYWHEEL_S, ShooterTunables.FLYWHEEL_V, ShooterTunables.FLYWHEEL_A);

    public final Trigger inShootMode = new Trigger(this::inShootMode);

    /** the RPM calculated by the shot calculation system */
    private DoubleSupplier rpmCalculation = () -> 0.0;

    private boolean idle = true;

    public ShooterSubsystem() {
        rightMotor.configure(
                ShooterTunables.FLYWHEEL_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // invert the left motor because it is oriented the other way
        leftMotor.configure(
                ShooterTunables.FLYWHEEL_MOTOR_CONFIG.inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        rightVelocityController.setTolerance(ShooterTunables.FLYWHEEL_RPM_TOLERANCE);
        leftVelocityController.setTolerance(ShooterTunables.FLYWHEEL_RPM_TOLERANCE);

        rightVelocityController.setSetpoint(ShooterTunables.FLYWHEEL_IDLE_RPM);
        leftVelocityController.setSetpoint(ShooterTunables.FLYWHEEL_IDLE_RPM);

        setDefaultCommand(runPIDs());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/At target", velocityControllerCommands.atSetpoint.getAsBoolean());
        Logger.recordOutput("Shooter/Right current", rightMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Left current", leftMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/Right Flywheel RPM", getRightFlywheelVelocity());
        Logger.recordOutput("Shooter/Left Flywheel RPM", getLeftFlywheelVelocity());

        HotPIDFTuner.logPIDDetails("Shooter", "left RPM controller", leftVelocityController);
        HotPIDFTuner.logPIDDetails("Shooter", "right RPM controller", rightVelocityController);
    }

    private void runFlywheelPID(PIDController pid, SparkMax motor, RelativeEncoder encoder) {
        // give the pid RPM
        double pidOutput;

        if (idle) {
            pidOutput = pid.calculate(encoder.getVelocity());
        } else {
            pidOutput = pid.calculate(encoder.getVelocity(), rpmCalculation.getAsDouble());
        }

        double targetRPM = pid.getSetpoint();

        // convert the rpm to rps because the feedforward takes units/s not units/m
        double feedforwardOutput = flywheelFeedforward.calculate(targetRPM / 60.0);

        double totalOutput = feedforwardOutput + pidOutput;

        double clampedOutput = MathUtil.clamp(totalOutput, -12.0, 12.0);

        motor.setVoltage(clampedOutput);
    }

    /** runs the velocity control, the RPM target will change if set to shoot mode */
    public Command runPIDs() {
        return run(() -> {
            runFlywheelPID(rightVelocityController, rightMotor, rightEncoder);
            runFlywheelPID(leftVelocityController, leftMotor, leftEncoder);
        });
    }

    /** sets the RPM target to follow calculated RPM */
    public Command stopIdling() {
        return runOnce(() -> idle = false);
    }

    /** sets the RPM target to not change */
    public Command holdSpeed() {
        return runOnce(() -> {
            idle = true;
        });
    }

    /** sets the RPM target to the idle rpm */
    public Command setIdle() {
        return velocityControllerCommands.setTarget(
                AngularVelocity.ofBaseUnits(ShooterTunables.FLYWHEEL_IDLE_RPM, RotationsPerSecond));
    }

    /** a command that sets the rpm supplier to the one provided */
    public Command setRPMSupplier(DoubleSupplier calculatedRPM) {
        return runOnce(() -> {
            this.rpmCalculation = calculatedRPM;
        });
    }

    /** changes the held RPM by the amount */
    public Command nudgeRPM(double rpmNudge) {
        return defer(() -> velocityControllerCommands.setTarget(AngularVelocity.ofBaseUnits(
                MathUtil.clamp(
                        (rpmNudge + rightVelocityController.getSetpoint()), 0, ShooterTunables.SHOOTER_RPM_CEILING),
                RotationsPerSecond)));
    }

    /** a command that just sets the motor voltage and doesn't do anything fancy with pids */
    public Command setFlywheelMotorVoltages(double motorVoltage) {
        return runOnce(() -> {
            rightMotor.setVoltage(motorVoltage);
            leftMotor.setVoltage(motorVoltage);
        });
    }

    /** returns the right flywheel velocity in RPM */
    public double getRightFlywheelVelocity() {
        return rightEncoder.getVelocity();
    }

    /** returns the right flywheel velocity in RPM */
    public double getLeftFlywheelVelocity() {
        return leftEncoder.getVelocity();
    }

    /** return the average rpm of both flywheels */
    public double getAverageRPM() {
        return getRightFlywheelVelocity() / 2 + getLeftFlywheelVelocity() / 2;
    }

    /** returns if the target RPM is following the calculation */
    public boolean inShootMode() {
        return !idle;
    }
}
