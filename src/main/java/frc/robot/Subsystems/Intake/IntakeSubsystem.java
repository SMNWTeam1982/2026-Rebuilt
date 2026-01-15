package frc.robot.Subsystems.Intake;

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
import frc.robot.CANIDs.IntakeIDS;


/**                                     IMPORTANT
 * All values/IDs are placeholders and are in need of change once robot design is complete and 
 * futher research has been conducted. 
 * 1/12/2026
 */

public class IntakeSubsystem extends SubsystemBase {
    public static final class IntakeConstants {
        public static final double intakeSpeed = 5;
        public static final double ejectSpeed = -5;  
        
        public static final Rotation2d STOW_POS = Rotation2d.fromDegrees(0); 
        public static final Rotation2d INTAKE_POS = Rotation2d.fromDegrees(0);


        public static final double PROPORTIONAL_GAIN = 0;  
        public static final double INTEGRAL_GAIN = 0;  
        public static final double DERIVITIVE_GAIN = 0;  

        public static final double MULTIPLIER = 0;  

        public static final SparkBaseConfig INTAKE_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kCoast);
    } 
    /** Motor on the Arm */
    private final SparkMax intakeMotor = new SparkMax(IntakeIDS.INTAKE, SparkMax.MotorType.kBrushless); 
    private final RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();

    private final SparkMax armMotor = new SparkMax(IntakeIDS.PIVOT, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder armMotorEncoder = armMotor.getEncoder();

    private final PIDController armController = new PIDController(
        IntakeConstants.PROPORTIONAL_GAIN,
        IntakeConstants.INTEGRAL_GAIN,
        IntakeConstants.DERIVITIVE_GAIN
    );

    public IntakeSubsystem() {
        armMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        intakeMotorEncoder.setPosition(0);
        armMotorEncoder.setPosition(0);

        armController.setTolerance(IntakeConstants.WRIST_PID_TOLERANCE);
        armController.setSetpoint(IntakeConstants.STOW_POS.getRadians());

        setDefaultCommand(runPID());
    }

    public Command setTargetAngle(
            Rotation2d targetAngle) { // Finds the target angle for the wrist based on button input
        return runOnce(() -> {
            armController.setGoal(targetAngle.getRadians());
        });
    }

    public Command setIntaking() {

    } 

    public Command setEjecting() {

    }

    public Command setStowPos() {
        return runOnce(
            intakeMotor.set(0);
            
        );
    }

    public Rotation2d getArmPos() {

    }



}
