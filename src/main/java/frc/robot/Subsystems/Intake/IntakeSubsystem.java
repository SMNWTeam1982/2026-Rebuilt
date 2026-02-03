package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.IntakeIDs;
import frc.robot.Constants.Tunables.IntakeTunables;
import org.littletonrobotics.junction.Logger;

/** controls the pivoting of the intake and the roller bar */
public class IntakeSubsystem extends SubsystemBase {
    /** for pulling in fuel */
    private final SparkMax intakeMotor = new SparkMax(IntakeIDs.INTAKE, SparkMax.MotorType.kBrushless);

    /** for deploying the intake */
    private final SparkMax pivotMotor = new SparkMax(IntakeIDs.PIVOT, SparkMax.MotorType.kBrushless);

    /** the absolue throughbore encoder attatched to the hex shaft */
    private final CANcoder pivotEncoder = new CANcoder(IntakeIDs.PIVOT_ENCODER);

    private final PIDController pivotController =
            new PIDController(IntakeTunables.PIVOT_P, IntakeTunables.PIVOT_I, IntakeTunables.PIVOT_D);

    /** for gravity compensation */
    public final ArmFeedforward pivotFeedforward =
            new ArmFeedforward(IntakeTunables.PIVOT_S, IntakeTunables.PIVOT_G, IntakeTunables.PIVOT_V);

    public IntakeSubsystem() {
        pivotMotor.configure(
                IntakeTunables.PIVOT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotController.setTolerance(IntakeTunables.PIVOT_TOLERANCE.getRadians());
        pivotController.setSetpoint(IntakeTunables.STOW_POSITION.getRadians());

        setDefaultCommand(runPID());
    }

    /** sets the pid setpoint to the desired angle */
    public Command setTargetAngle(
            Rotation2d targetAngle) { // Finds the target angle for the wrist based on button input
        return runOnce(() -> pivotController.setSetpoint(targetAngle.getRadians()));
    }

    @Override
    public void periodic() {
        // output
        Logger.recordOutput("intake/Target Angle (Radians)", pivotController.getSetpoint());
        // input
        Logger.recordOutput("intake/Current Angle (Radians)", getIntakePosition().getRadians());
    }

    /** runs the feedback and feedforward control and sets the motor */
    public Command runPID() {
        return runEnd(
                () -> {
                    double intakeRotation = getIntakePosition().getRadians();

                    // pid loop tuned to output in volts
                    double pidOutput = pivotController.calculate(intakeRotation);

                    double feedForwardOutput = pivotFeedforward.calculate(pivotController.getSetpoint(), 0);

                    double outputVoltage = MathUtil.clamp(feedForwardOutput + pidOutput, -12, 12);

                    pivotMotor.setVoltage(outputVoltage);
                },
                () -> {
                    pivotMotor.set(0);
                    pivotController.reset();
                });
    }

    /** sets the pivot pid to the constants */
    public Command setPIDValues(double p, double i, double d) {
        return runOnce(() -> {
            pivotController.setPID(p, i, d);
        });
    }

    /** sets the intake to start intaking and the intake target to the deploy position */
    public Command deploy() {
        return startIntaking().andThen(setTargetAngle(IntakeTunables.DEPLOY_POSITION));
    }

    /** sets the intake to stop intaking and the intake target to the stow position */
    public Command retract() {
        return stopIntaking().andThen(setTargetAngle(IntakeTunables.STOW_POSITION));
    }

    /** sets the intake motor to the intake speed */
    public Command startIntaking() {
        return runOnce(() -> intakeMotor.set(IntakeTunables.INTAKE_SPEED));
    }

    /** sets the intake motor to 0 */
    public Command stopIntaking() {
        return runOnce(() -> intakeMotor.set(0));
    }

    /** the absolute position of the intake from the throughbore encoder */
    public Rotation2d getIntakePosition() {
        return Rotation2d.fromRotations(pivotEncoder.getPosition().getValueAsDouble());
    }
}
