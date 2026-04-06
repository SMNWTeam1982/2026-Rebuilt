package frc.robot.Subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANBus.IntakeIDs;
import frc.robot.Constants.Tunables;
import frc.robot.Constants.Tunables.IntakeTunables;
import frc.robot.SparkMaxHelper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    /** for pulling in fuel */
    private final SparkMax intakeMotor = new SparkMax(IntakeIDs.INTAKE, SparkMax.MotorType.kBrushless);

    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    /** for deploying the intake */
    private final SparkMax pivotMotor = new SparkMax(IntakeIDs.PIVOT, SparkMax.MotorType.kBrushless);

    @AutoLogOutput(key = "Intake/intake motor is hot")
    private final Trigger intakeMotorHot = new Trigger(() -> intakeMotor.getMotorTemperature() >= 60);

    @AutoLogOutput(key = "Intake/intake motor disabled")
    private boolean intakeDisabled = false;

    @AutoLogOutput(key = "Intake/pivot motor disabled")
    private boolean pivotDisabled = false;

    @AutoLogOutput(key = "Intake/intake motor jammed")
    public final Trigger intakeMotorJammed =
            new Trigger(() -> intakeMotor.getOutputCurrent() >= IntakeTunables.INTAKE_MOTOR_JAM_CURRENT_THRESHHOLD
                    && Math.abs(intakeEncoder.getVelocity()) <= 400);

    private final SlewRateLimiter pivotOutputLimiter = new SlewRateLimiter(IntakeTunables.PIVOT_OUTPUT_RATE_LIMIT);

    public IntakeSubsystem() {
        pivotMotor.configure(
                IntakeTunables.PIVOT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotor.configure(
                Tunables.DEFAULT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SparkMaxHelper.logMotorDetails("Intake", "pivot motor", pivotMotor);
        SparkMaxHelper.logMotorDetails("Intake", "intake motor", intakeMotor);

        if (getCurrentCommand() == null) {
            Logger.recordOutput("Intake/current command", "no active command");
        } else {
            Logger.recordOutput(
                    "Intake/current command", this.getCurrentCommand().getName());
        }
    }

    private void setIntake(double intakeSpeed) {
        if (intakeDisabled) {
            intakeMotor.set(0);
        } else {
            intakeMotor.set(intakeSpeed);
        }
    }

    private void setPivot(double pivotSpeed) {
        if (pivotDisabled) {
            pivotMotor.set(0);
        } else {
            pivotMotor.set(pivotSpeed);
        }
    }

    /** sets the pivot using the rate limiter */
    private void movePivot(double desiredPivotSpeed) {
        setPivot(pivotOutputLimiter.calculate(desiredPivotSpeed));
    }

    /** sets the pivot motor to 0 and resets the rate limiter */
    private void stopPivot() {
        setPivot(0);
        pivotOutputLimiter.reset(0);
    }

    /** sets the intake motor to the intake speed */
    public Command startIntaking() {
        return runOnce(() -> setIntake(IntakeTunables.INTAKE_SPEED));
    }

    /** sets the intake motor to 0 */
    public Command stopIntaking() {
        return runOnce(() -> setIntake(0));
    }

    /** sets the pivot motor to move IN at a constant speed while running, then stops the motor when it ends */
    public Command moveIn() {
        return runEnd(() -> movePivot(IntakeTunables.PIVOT_MOVE_IN_SPEED), () -> stopPivot());
    }

    /** sets the pivot motor to move OUT at a constant speed while running, then stops the motor when it ends */
    public Command moveOut() {
        return runEnd(() -> movePivot(IntakeTunables.PIVOT_MOVE_OUT_SPEED), () -> stopPivot());
    }

    /**
     * sets the intake to stop intaking, and to move in for a tunable number of seconds (IntakeTunables.RETRACT_ATTEMPT_TIME)
     * <p> stops the intake and resets the rate limiter on interupt
     */
    public Command stow() {
        return stopIntaking()
                .andThen(moveIn().withTimeout(IntakeTunables.RETRACT_ATTEMPT_TIME))
                .handleInterrupt(this::stopPivot);
    }

    /**
     * sets the intake to start intaking, and to move out for a tunable number of seconds (IntakeTunables.DEPLOY_ATTEMPT_TIME)
     */
    public Command deploy() {
        return startIntaking()
                .andThen(moveOut().withTimeout(IntakeTunables.DEPLOY_ATTEMPT_TIME))
                .handleInterrupt(this::stopPivot);
    }

    /**
     * stops the intake then slowly ramps up and ramps down the speed of the pivot for a smoother retraction
     * <p> will stop the pivot and reset the rate limiter on end or interrupt
     */
    public Command smoothStow() {
        return Commands.sequence(
                        stopIntaking(), // stop intaking
                        runOnce(this::stopPivot), // stop & reset rate limiter
                        run(() -> movePivot(IntakeTunables.PIVOT_MOVE_IN_SPEED))
                                .withTimeout(IntakeTunables.RETRACT_ATTEMPT_TIME.div(2.0)), // accelerate
                        run(() -> movePivot(0)).withTimeout(IntakeTunables.RETRACT_ATTEMPT_TIME.div(2.0)) // deccelerate
                        )
                .finallyDo(this::stopPivot);
    }

    /**
     * starts the intake then slowly ramps up and ramps down the speed of the pivot for a smoother deploy
     * <p> will stop the pivot and reset the rate limiter on end or interrupt
     */
    public Command smoothDeploy() {
        return Commands.sequence(
                        startIntaking(), // start intaking
                        runOnce(this::stopPivot), // stop & reset rate limiter
                        run(() -> movePivot(IntakeTunables.PIVOT_MOVE_OUT_SPEED))
                                .withTimeout(IntakeTunables.DEPLOY_ATTEMPT_TIME.div(2.0)), // accelerate
                        run(() -> movePivot(0)).withTimeout(IntakeTunables.DEPLOY_ATTEMPT_TIME.div(2.0)) // deccelerate
                        )
                .finallyDo(this::stopPivot);
    }

    public Command suddenStow() {
        return Commands.sequence(
                        stopIntaking(),
                        runOnce(this::stopPivot),
                        runOnce(() -> setPivot(IntakeTunables.PIVOT_MOVE_IN_SPEED)),
                        Commands.waitTime(IntakeTunables.RETRACT_ATTEMPT_TIME))
                .finallyDo(this::stopPivot);
    }

    public Command suddenDeploy() {
        return Commands.sequence(
                        startIntaking(),
                        runOnce(this::stopPivot),
                        runOnce(() -> setPivot(IntakeTunables.PIVOT_MOVE_OUT_SPEED)),
                        Commands.waitTime(IntakeTunables.DEPLOY_ATTEMPT_TIME))
                .finallyDo(this::stopPivot);
    }

    public Command turnOff() {
        return runOnce(() -> {
            stopPivot();
            intakeMotor.stopMotor();
            pivotMotor.stopMotor();
            intakeDisabled = true;
            pivotDisabled = true;
        });
    }

    public Command turnOn() {
        return runOnce(() -> {
            intakeDisabled = false;
            pivotDisabled = false;
        });
    }
}
