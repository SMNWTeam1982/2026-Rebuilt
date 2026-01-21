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
import frc.robot.Constants.CANBus.IntakeIDS;
import frc.robot.Constants.CANBus.ShooterIDs;

public class Shooter{
    
    public static final class ShooterConstants{
        public static final double FIRING_SPEED_ROTATIONS_PER_SECOND = 0; 
        
        public static final SparkBaseConfig INTAKE_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);

    }

    private final SparkMax leftShooterMotor = new SparkMax(ShooterIDs.LEFT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder leftShooterMotorEncoder = leftShooterMotor.getEncoder();

    private final SparkMax rightShooterMotor = new SparkMax(ShooterIDs.RIGHT_MOTOR_ID, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder rightShooterMotorEncoder = rightShooterMotor.getEncoder();

    public double getRightMotorSpeed(){
        return rightShooterMotor.get
    }

    public double getLeftMotorSpeed(){

    }


    

}
