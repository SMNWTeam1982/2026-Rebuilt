package frc.robot.Subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANBus.IntakeIDs;
import frc.robot.Constants.Tunables;
import frc.robot.Constants.Tunables.IntakeTunables;
import frc.robot.SparkMaxHelper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** since we are having issues with the intake pivot motor, a stripped down version without complicated PIDF control is being created as a backup */
public class IntakeSubsystem extends SubsystemBase {
    /** for pulling in fuel */
    private final SparkMax intakeMotor = new SparkMax(IntakeIDs.INTAKE, SparkMax.MotorType.kBrushless);

    /** for deploying the intake */
    private final SparkMax pivotMotor = new SparkMax(IntakeIDs.PIVOT, SparkMax.MotorType.kBrushless);

    @AutoLogOutput(key = "Intake/intake motor is hot")
    private final Trigger intakeMotorHot = new Trigger(() -> intakeMotor.getMotorTemperature() >= 60);

    @AutoLogOutput(key = "Intake Disabled")
    private boolean intakeDisabled = false;
    
    // /** the absolue throughbore encoder attatched to the hex shaft */
    // private final CANcoder pivotEncoder = new CANcoder(IntakeIDs.PIVOT_ENCODER);

    // @AutoLogOutput(key = "Intake/is stowed")
    // public final Trigger stowed = new Trigger(
    //                 () -> pivotEncoder.getAbsolutePosition().getValueAsDouble() < IntakeTunables.STOWED_THRESHOLD)
    //         .debounce(IntakeTunables.THRESHOLD_TIME.in(Seconds));

    // @AutoLogOutput(key = "Intake/is deployed")
    // public final Trigger deployed = new Trigger(
    //                 () -> pivotEncoder.getAbsolutePosition().getValueAsDouble() > IntakeTunables.DEPLOYED_THRESHOLD)
    //         .debounce(IntakeTunables.THRESHOLD_TIME.in(Seconds));

    public IntakeSubsystem() {
        pivotMotor.configure(
                IntakeTunables.PIVOT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotor.configure(
                Tunables.DEFAULT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("Intake/absolute position", pivotEncoder.getAbsolutePosition().getValueAsDouble());

        SparkMaxHelper.logMotorDetails("Intake", "pivot motor", pivotMotor);
        SparkMaxHelper.logMotorDetails("Intake", "intake motor", intakeMotor);

        if (getCurrentCommand() == null) {
            Logger.recordOutput("Intake/current command", "no active command");
        } else {
            Logger.recordOutput(
                    "Intake/current command", this.getCurrentCommand().getName());
        }
    }

    /** sets the intake motor to the intake speed */
    public Command startIntaking() {
        if (intakeDisabled == true){
            return turnOff();
        }
        else {
            return runOnce(() -> intakeMotor.set(IntakeTunables.INTAKE_SPEED));
        }
    }

    /** sets the intake motor to 0 */
    public Command stopIntaking() {
        if (intakeDisabled == true){
            return turnOff();
        }
        else {
            return runOnce(() -> intakeMotor.set(0));
        }
    }

    /** sets the pivot motor to move IN at a constant speed while running, then stops the motor when it ends */
    public Command moveIn() {
        if (intakeDisabled == true){
            return turnOff();
        }
        else {
        return startEnd(
                () -> {
                    pivotMotor.set(IntakeTunables.MOVE_IN_SPEED);
                },
                () -> {
                    pivotMotor.set(0.0);
                }
            );
        }
    }

    /** sets the pivot motor to move OUT at a constant speed while running, then stops the motor when it ends */
    public Command moveOut() {
        if (intakeDisabled == true){
            return turnOff();
        }
        else {
            return startEnd(
                    () -> {
                        pivotMotor.set(IntakeTunables.MOVE_OUT_SPEED);
                    },
                    () -> {
                        pivotMotor.set(0.0);
                    }
            );
        }
    }

    /** sets the intake to stop intaking, and to move in until it passes the stow threshold
     * <p> will automatically end after a tunable number of seconds (IntakeTunables.RETRACT_ATTEMPT_TIME)
     */
    public Command stow() {
        if (intakeDisabled == true){
            return turnOff();
        }
        else {
            return stopIntaking().andThen(moveIn()).withTimeout(IntakeTunables.RETRACT_ATTEMPT_TIME);
        }
    }

    /** sets the intake to start intaking, and to move out until it passes the deploy threshold
     * <p> will automatically end after a tunable number of seconds (IntakeTunables.DEPLOY_ATTEMPT_TIME)
     */
    public Command deploy() {
        if (intakeDisabled == true){
            return turnOff();
        }
        else {
            return startIntaking().andThen(moveOut()).withTimeout(IntakeTunables.DEPLOY_ATTEMPT_TIME);
        }
    }

    public Command turnOff(){
        return runOnce(
            () -> {
                intakeMotor.stopMotor();
                intakeDisabled = true;
            }
        );
    }

    public Command turnOn(){
        return runOnce(
            () -> {
                intakeDisabled = false; 
            }
        );
    }
}
