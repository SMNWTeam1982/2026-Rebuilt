package frc.robot.Subsystems.Kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.KickerIDs;
import frc.robot.Constants.Tunables.KickerTunables;
import frc.robot.SparkMaxHelper;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor = new SparkMax(KickerIDs.KICKER, MotorType.kBrushless);

    private boolean kickerDisabled = false;

    public KickerSubsystem() {
        kickerMotor.configure(
                KickerTunables.KICKER_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SparkMaxHelper.logMotorDetails("Kicker", "kicker motor", kickerMotor);

        if (getCurrentCommand() == null) {
            Logger.recordOutput("Kicker/current command", "no active command");
        } else {
            Logger.recordOutput(
                    "Kicker/current command", this.getCurrentCommand().getName());
        }
    }

    public Command setSpeed(double speed) {
        return runOnce(() -> {
            if (kickerDisabled) {
                kickerMotor.set(0.0);
            } else {
                kickerMotor.set(speed);
            }
        });
    }

    public Command setHigh() {
        return setSpeed(KickerTunables.HIGH_SPEED);
        // return runOnce(() -> kickerMotor.set(KickerTunables.HIGH_SPEED));
    }

    public Command setLow() {
        return setSpeed(KickerTunables.LOW_SPEED);
        // return runOnce(() -> kickerMotor.set(KickerTunables.LOW_SPEED));
    }

    public Command setReverse() {
        return setSpeed(KickerTunables.REVERSE_SPEED);
        // return runOnce(() -> kickerMotor.set(KickerTunables.REVERSE_SPEED));
    }

    public Command setIdle() {
        return setSpeed(KickerTunables.IDLE_SPEED);
        // return runOnce(() -> kickerMotor.set(KickerTunables.IDLE_SPEED));
    }

    /** while running, it switches the kicker's speed from high to low periodically */
    public Command kick() {
        return Commands.repeatingSequence(
                        setHigh(),
                        Commands.waitTime(KickerTunables.HIGH_TIME),
                        setLow(),
                        Commands.waitTime(KickerTunables.LOW_TIME))
                .finallyDo(() -> {
                    if (kickerDisabled) {
                        kickerMotor.set(0.0);
                    } else {
                        kickerMotor.set(KickerTunables.IDLE_SPEED);
                    }
                });
        // return runOnce(() -> kickerMotor.set(KickerTunables.HIGH_SPEED))
        //         .andThen(new WaitCommand(KickerTunables.HIGH_TIME))
        //         .andThen(runOnce(() -> kickerMotor.set(KickerTunables.LOW_SPEED)))
        //         .andThen(new WaitCommand(KickerTunables.LOW_TIME))
        //         .repeatedly()
        //         .finallyDo(() -> kickerMotor.set(KickerTunables.IDLE_SPEED));
    }

    /** disables the motors and sets their current limits to 0, their pid gains to 0, and their ff gains to 0 */
    public Command turnOff() {
        return runOnce(() -> {
            kickerMotor.stopMotor();
            kickerDisabled = true;
        });
    }

    public Command turnOn() {
        return runOnce(() -> {
            kickerDisabled = false;
        });
    }

    public Command dontMove() {
        return run(() -> {
            kickerMotor.stopMotor();
        });
    }
}
