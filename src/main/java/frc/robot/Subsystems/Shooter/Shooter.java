package frc.robot.Subsystems.Shooter;

import java.util.function.Supplier;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.ShooterIDs;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Tunables.SysIDTunables;

// SysID Imports: 
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
    
    public static final class ShooterConstants{
        // all placeholder values pretty much
        public static final double FIRING_SPEED_ROTATIONS_PER_SECOND = 0; 
        public static final SparkBaseConfig INTAKE_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
        public static final SparkBaseConfig RIGHT_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);

        public static final double SHOOTER_PIDCONST_P = 0.0; 
        public static final double SHOOTER_PIDCONST_I = 0.0;
        public static final double SHOOTER_PIDCONST_D = 0.0;

        public static final double SHOOTER_FEEDFORWARDCONST_KS = 0.0;
        public static final double SHOOTER_FEEDFORWARDCONST_KV = 0.0;   

    }

    // private final SparkMax leftShooterMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    // private final RelativeEncoder leftShooterMotorEncoder = leftShooterMotor.getEncoder();

    private final SparkMax rightShooterMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder rightShooterMotorEncoder = rightShooterMotor.getEncoder();

    private final SparkMax leftShooterMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder leftShooterMotorEncoder = leftShooterMotor.getEncoder();

    private final PIDController shooterPIDController = new PIDController(ShooterConstants.SHOOTER_PIDCONST_P, ShooterConstants.SHOOTER_PIDCONST_I, ShooterConstants.SHOOTER_PIDCONST_D);
    private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_FEEDFORWARDCONST_KS, ShooterConstants.SHOOTER_FEEDFORWARDCONST_KV);
    private Supplier<Double> getDistanceToHub;

    private double distancePerPulse = 1;

     // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    public Shooter(Supplier<Double> getDToHub) {
        rightShooterMotor.configure(ShooterConstants.RIGHT_MOTOR_CONFIG, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);

        getDistanceToHub = getDToHub;
        
        rightShooterMotorEncoder.setPosition(0.0);

        shooterPIDController.setTolerance(.05); 
        
    }

    public Command setVelocity(double targetRPM){
        return runOnce(
            () -> {
                double feedforwardVoltage = shooterFeedforward.calculate(targetRPM / 60.0); 
                double pidOutput = shooterPIDController.calculate((getRightMotorSpeed() / 60.0));
                double totalOutput = feedforwardVoltage + pidOutput;

                if (totalOutput > 12.0) {
                    totalOutput = 12.0;
                } else if (totalOutput < -12.0) {
                    totalOutput = -12.0;
                }
                rightShooterMotor.setVoltage(totalOutput);
            }
        );
    }

    public Command runPID() {
        return run(() -> {
            setVelocity(FieldMeasurements.hubDistanceToFlywheelRPM(getDistanceToHub.get()));
        });
    }

    public double getRightMotorSpeed(){
        // in RPM
        return rightShooterMotorEncoder.getVelocity();
    }

  private final SysIdRoutine sysIdRoutine1 =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              rightShooterMotor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("right-shooter")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightShooterMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(getDistance(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(((Encoder) rightShooterMotorEncoder).getRate(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  
  // PID controller to run the shooter wheel in closed-loop, set the constants equal to those
  // calculated by SysId
  private final PIDController m_shooterFeedback = new PIDController(SysIDTunables.kP, 0, 0);
  
  // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
  // those calculated by SysId
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          SysIDTunables.kSVolts,
          SysIDTunables.kVVoltSecondsPerRotation,
          SysIDTunables.kAVoltSecondsSquaredPerRotation);

  /** Creates a new Shooter subsystem. */
  public void Shooter() {
    // Sets the distance per pulse for the encoders
    ((Encoder) rightShooterMotorEncoder).setDistancePerPulse(SysIDTunables.kEncoderDistancePerPulse);
  }

  /**
   * Returns a command that runs the shooter at a specifc velocity.
   *
   * @param shooterSpeed The commanded shooter wheel speed in rotations per second
   */
  public Command runShooter(DoubleSupplier shooterSpeed) {
    // Run shooter wheel at the desired speed using a PID controller and feedforward.
    return run(() -> {
          rightShooterMotor.setVoltage(
              m_shooterFeedback.calculate(((Encoder) rightShooterMotorEncoder).getRate(), shooterSpeed.getAsDouble())
                  + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));
          leftShooterMotor.set(SysIDTunables.kFeederSpeed);
        })
        .finallyDo(
            () -> {
              rightShooterMotor.stopMotor();
              leftShooterMotor.stopMotor();
            })
        .withName("run Test Motor");
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine1.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine1.dynamic(direction);
  }

  public double getDistance(){
    return rightShooterMotorEncoder.getPosition() * distancePerPulse;
  }
}
