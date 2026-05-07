package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANBus.ShooterIDs;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.PIDTools.FFCommandGenerators.SimpleMotorFFCommandGenerator;
import frc.robot.PIDTools.HotPIDFTuner;
import frc.robot.PIDTools.PIDCommandGenerator;
import frc.robot.SparkMaxHelper;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax rightMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    @AutoLogOutput(key = "Shooter/right jammed")
    public final Trigger rightShooterJammed =
            new Trigger(() -> rightMotor.getOutputCurrent() >= 30.0 && Math.abs(rightEncoder.getVelocity()) <= 400);

    @AutoLogOutput(key = "Shooter/left jammed")
    public final Trigger leftShooterJammed =
            new Trigger(() -> leftMotor.getOutputCurrent() >= 30.0 && Math.abs(leftEncoder.getVelocity()) <= 400);

    /** input RPM, outputs volts */
    private final PIDController rightVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    /** input RPM, outputs volts */
    private final PIDController leftVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    private final Alert outOfBoundsRPMTarget = new Alert("Shooter/recieved out of bounds RPM target", AlertType.kError);

    /** setting the target velocity will also set the shooter to hold velocity
     * <p> the RPM target will be clamped between 0 and the shooter RPM limit
     */
    public final PIDCommandGenerator<AngularVelocity> velocityControllerCommands =
            new PIDCommandGenerator<AngularVelocity>(
                    (target) -> {
                        idle = true;
                        setTargetAngularVelocity(target);
                    },
                    this,
                    rightVelocityController,
                    leftVelocityController);

    /** input RPS, outputs volts, this feedforward can be used for both motors */
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterTunables.FLYWHEEL_S, ShooterTunables.FLYWHEEL_V, ShooterTunables.FLYWHEEL_A);

    public final SimpleMotorFFCommandGenerator flywheelFFCommands =
            new SimpleMotorFFCommandGenerator(this, flywheelFeedforward);

    @AutoLogOutput(key = "Shooter/using rpm calculation function")
    public final Trigger inShootMode = new Trigger(this::inShootMode);

    /** the RPM calculated by the shot calculation system */
    @AutoLogOutput(key = "Shooter/rpm calculation")
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

        Logger.recordOutput("Shooter/Right Flywheel RPM", getRightFlywheelVelocity());
        Logger.recordOutput("Shooter/Left Flywheel RPM", getLeftFlywheelVelocity());

        SparkMaxHelper.logMotorDetails("Shooter", "left motor", leftMotor);
        SparkMaxHelper.logMotorDetails("Shooter", "right motor", rightMotor);

        HotPIDFTuner.logPIDDetails("Shooter", "left RPM controller", leftVelocityController);
        HotPIDFTuner.logPIDDetails("Shooter", "right RPM controller", rightVelocityController);

        if (getCurrentCommand() == null) {
            Logger.recordOutput("Shooter/current command", "no active command");
        } else {
            Logger.recordOutput(
                    "Shooter/current command", this.getCurrentCommand().getName());
        }
    }

    /** set the target of both pid loops, doing unit conversion, clamping it, and sending an alert if it is out of bounds */
    private void setTargetAngularVelocity(AngularVelocity targetFlywheelSpeed) {
        Logger.recordOutput("Shooter/target angular velocity", targetFlywheelSpeed);

        double targetRPM = targetFlywheelSpeed.in(RPM);

        // send an alert if the RPM target is out of bounds then clamp it
        if (Math.abs(targetRPM) >= ShooterTunables.SHOOTER_RPM_CEILING) {
            outOfBoundsRPMTarget.set(true);
            targetRPM = MathUtil.clamp(
                    targetRPM, -ShooterTunables.SHOOTER_RPM_CEILING, ShooterTunables.SHOOTER_RPM_CEILING);
        } else {
            outOfBoundsRPMTarget.set(false);
        }

        rightVelocityController.setSetpoint(targetRPM);
        leftVelocityController.setSetpoint(targetRPM);
    }

    private void runFlywheelPID(PIDController pid, SparkMax motor, RelativeEncoder encoder) {
        // give the pid RPM
        double pidOutput;

        pidOutput = pid.calculate(encoder.getVelocity());

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
            if (!idle) {
                // update the target RPM if the shooter is NOT in idle mode
                setTargetAngularVelocity(RPM.of(rpmCalculation.getAsDouble()));
            }
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
        return velocityControllerCommands.setTarget(RPM.of(ShooterTunables.FLYWHEEL_IDLE_RPM));
    }

    /** a command that sets the rpm supplier to the one provided, and sets the shooter to follow it */
    public Command setRPMSupplier(DoubleSupplier calculatedRPM) {
        return runOnce(() -> {
            idle = false;
            this.rpmCalculation = calculatedRPM;
        });
    }

    /** sets the RPM target to be based off of the current robot position and a given target position */
    public Command setTarget(Supplier<Translation2d> robotPosition, Supplier<Translation2d> targetPosition) {
        return setRPMSupplier(() -> ShotCalculation.calculateRPM(robotPosition.get(), targetPosition.get()));
    }

    /** changes the held RPM by the amount */
    public Command nudgeRPM(double rpmNudge) {
        return defer(
                () -> velocityControllerCommands.setTarget(RPM.of(rpmNudge + rightVelocityController.getSetpoint())));
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

    /** disables the motors and sets their current limits to 0, their pid gains to 0, and their ff gains to 0 */
    public Command turnOff() {
        return runOnce(() -> {
            Logger.recordOutput("Shooter/turned off", true);
            rightMotor.disable();
            leftMotor.disable();

            SparkBaseConfig disabledConfig = new SparkMaxConfig()
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(0)
                    .secondaryCurrentLimit(0.0);
            rightMotor.configure(disabledConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            leftMotor.configure(disabledConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            flywheelFeedforward.setKa(0.0);
            flywheelFeedforward.setKs(0.0);
            flywheelFeedforward.setKv(0.0);

            rightVelocityController.setPID(0.0, 0.0, 0.0);
            leftVelocityController.setPID(0.0, 0.0, 0.0);

            setDefaultCommand(dontMove());
        });
    }

    public Command dontMove() {
        return run(() -> {
            rightMotor.stopMotor();
            leftMotor.stopMotor();
        });
    }
}
