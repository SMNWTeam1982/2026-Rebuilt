package frc.robot.Subsystems.Shooter;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.ShooterIDs;

//SysID Routine Imports 
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterConstants;


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

    private final PIDController shooterPIDController = new PIDController(ShooterConstants.SHOOTER_PIDCONST_P, ShooterConstants.SHOOTER_PIDCONST_I, ShooterConstants.SHOOTER_PIDCONST_D);
    private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_FEEDFORWARDCONST_KS, ShooterConstants.SHOOTER_FEEDFORWARDCONST_KV);
    private Supplier<Double> getDistanceToHub;


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

    public double getLeftMotorSpeed(){
        // in RPM
        return leftShooterMotorEncoder.getVelocity();
    }
}
