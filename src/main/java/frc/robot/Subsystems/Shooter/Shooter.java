package frc.robot.Subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.ShooterIDs;

//SysID Routine Imports 
import edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class Shooter{
    
    public static final class ShooterConstants{
        
        public static final SparkBaseConfig INTAKE_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    private final SparkMax leftShooterMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder leftShooterMotorEncoder = leftShooterMotor.getEncoder();

    private final SparkMax rightShooterMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder rightShooterMotorEncoder = rightShooterMotor.getEncoder();
    
    public static final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.in(Volts)),
                        log -> {
                            log.motor("left")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(voltageSignal.getValueAsDouble(),
                                            Volts))
                                    .angularPosition(sysidPositionMeasure
                                            .mut_replace(positionSignal.getValueAsDouble(), Rotations))
                                    .angularVelocity(
                                            sysidVelocityMeasure.mut_replace(velocitySignal.getValueAsDouble(),
                                                    RotationsPerSecond));
                        },
                        this));
                        
        setDefaultCommand(spinAtVelocityCommand(() -> 0.0));

    public double getRightMotorSpeed(){
        return rightShooterMotor.get
    }

    public double getLeftMotorSpeed(){

    }


    

}
