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
import frc.robot.Constants.CANBus.SysIDIds;
import frc.robot.Constants.Measured.ShooterMeasurements;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Constants.Tunables.SysIDtunables;

import java.util.function.DoubleSupplier;


import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;



public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax rightMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    /** input RPM, outputs volts */
    private final PIDController rightVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    private final PIDController leftVelocityController =
            new PIDController(ShooterTunables.FLYWHEEL_P, ShooterTunables.FLYWHEEL_I, ShooterTunables.FLYWHEEL_D);

    /** input RPS, outputs volts, this feedforward can be used for both motors */
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterMeasurements.FLYWHEEL_S, ShooterMeasurements.FLYWHEEL_V, ShooterMeasurements.FLYWHEEL_A);

    public final Trigger flywheelsUpToSpeed =
            new Trigger(() -> rightVelocityController.atSetpoint() && leftVelocityController.atSetpoint());
    public final Trigger inShootMode = new Trigger(this::inShootMode);

    /** the RPM calculated by the shot calculation system */
    private final DoubleSupplier shootSpeed;

    private boolean shootMode = false;

    //SysId variables 

          // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    public ShooterSubsystem(DoubleSupplier shootSpeed) {

        this.shootSpeed = shootSpeed;
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

    private void runFlywheelPID(PIDController pid, SparkMax motor, RelativeEncoder encoder) {
        // give the pid RPM
        double pidOutput;

        if (shootMode) {
            pidOutput = pid.calculate(encoder.getVelocity(), shootSpeed.getAsDouble());
        } else {
            pidOutput = pid.calculate(encoder.getVelocity());
        }

        double targetRPM = pid.getSetpoint();

        // convert the rpm to rps because the feedforward takes units/s not units/m
        double feedforwardOutput = flywheelFeedforward.calculate(targetRPM / 60.0);

        double totalOutput = feedforwardOutput + pidOutput;

        double clampedOutput = MathUtil.clamp(totalOutput, -12.0, 12.0);

        motor.setVoltage(clampedOutput);
    }

    private void setSetpoints(double setpoint) {
        rightVelocityController.setSetpoint(setpoint);
        leftVelocityController.setSetpoint(setpoint);
    }

    /** runs the velocity control, the RPM target will change if set to shoot mode */
    public Command runPIDs() {
        return run(() -> {
            runFlywheelPID(rightVelocityController, rightMotor, rightEncoder);
            runFlywheelPID(leftVelocityController, leftMotor, leftEncoder);
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
            setSetpoints(ShooterTunables.FLYWHEEL_IDLE_RPM);
        });
    }

    /** sets the RPM target */
    public Command setHoldRPM(double targetRPM) {
        return runOnce(() -> {
            shootMode = false;
            setSetpoints(targetRPM);
        });
    }

    /** changes the RPM target by the amount */
    public Command changeHeldRPM(double change) {
        return runOnce(() -> {
            shootMode = false;
            // fetch the rpm of the right one (both are the same)
            double currentTargetRPM = rightVelocityController.getSetpoint();
            double newTargetRPM = currentTargetRPM + change;
            if (newTargetRPM > ShooterTunables.SHOOTER_RPM_CEILING) {
                newTargetRPM = ShooterTunables.SHOOTER_RPM_CEILING;
            }
            if (newTargetRPM < 0) {
                newTargetRPM = 0;
            }

            setSetpoints(newTargetRPM);
        });
    }

    /** a command that just sets the motor voltage and doesn't do anything fancy with pids */
    public Command setFlywheelMotorVoltages(double motorVoltage) {
        return runOnce(() -> {
            rightMotor.setVoltage(motorVoltage);
            leftMotor.setVoltage(motorVoltage);
        });
    }

    /** makes a command to set the velocity gains */
    public Command setVelocityPID(double p, double i, double d) {
        return runOnce(() -> {
            rightVelocityController.setPID(p, i, d);
            leftVelocityController.setPID(p, i, d);
        });
    }

    /** returns the right flywheel velocity in RPM */
    public double getRightFlywheelVelocity() {
        return rightEncoder.getVelocity();
    }

    /** returns the right flywheel velocity in RPM */
    public double getLeftFlywheelVelocity() {
        return rightEncoder.getVelocity();
    }

    /** return the average rpm of both flywheels */
    public double getAverageRPM() {
        return getRightFlywheelVelocity() / 2 + getLeftFlywheelVelocity() / 2;
    }

    /** returns if the target RPM is following the calculation */
    public boolean inShootMode() {
        return shootMode;
    }

    private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              rightMotor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("Right Shooter Motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(rightEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(rightEncoder.getVelocity(), RotationsPerSecond));
                log.motor("Left Shooter Motor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(leftEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(leftEncoder.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

    /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
