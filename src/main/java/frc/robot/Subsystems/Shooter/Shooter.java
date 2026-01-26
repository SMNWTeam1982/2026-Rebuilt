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
import frc.robot.Constants.Tunables.ShooterTunables;


public class Shooter extends SubsystemBase {
    


    // private final SparkMax leftShooterMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    // private final RelativeEncoder leftShooterMotorEncoder = leftShooterMotor.getEncoder();

    private final SparkMax rightShooterMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder rightShooterMotorEncoder = rightShooterMotor.getEncoder();

    private final PIDController shooterPIDController = new PIDController(ShooterTunables.SHOOTER_PIDCONST_P, ShooterTunables.SHOOTER_PIDCONST_I, ShooterTunables.SHOOTER_PIDCONST_D);
    private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ShooterTunables.SHOOTER_FEEDFORWARDCONST_KS, ShooterTunables.SHOOTER_FEEDFORWARDCONST_KV);
    private Supplier<Double> getDistanceToHub;


    public Shooter(Supplier<Double> getDToHub) {
        rightShooterMotor.configure(ShooterTunables.RIGHT_MOTOR_CONFIG, 
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
}
