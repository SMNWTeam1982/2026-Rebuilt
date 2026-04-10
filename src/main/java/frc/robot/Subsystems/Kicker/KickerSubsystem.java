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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax beltMotor = new SparkMax(KickerIDs.BELTS, MotorType.kBrushless);
    private final SparkMax wheelMotor = new SparkMax(KickerIDs.WHEELS, MotorType.kBrushless);

    @AutoLogOutput(key = "Kicker/kicker disabled")
    private boolean kickerDisabled = false;

    public KickerSubsystem() {
        beltMotor.configure(
                KickerTunables.BELT_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wheelMotor.configure(
            KickerTunables.WHEEL_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SparkMaxHelper.logMotorDetails("Kicker", "belt motor", beltMotor);
        SparkMaxHelper.logMotorDetails("Kicker", "wheel motor", wheelMotor);

        if (getCurrentCommand() == null) {
            Logger.recordOutput("Kicker/current command", "no active command");
        } else {
            String currentCommandName = this.getCurrentCommand().getName();
            Logger.recordOutput(
                    "Kicker/current command", currentCommandName);
        }
    }

    public void setBelts(double amount) {
        if (kickerDisabled) {
            beltMotor.set(0.0);
        } else {
            beltMotor.set(amount);
        }
    }

    public void setWheels(double amount) {
        if (kickerDisabled) {
            wheelMotor.set(0.0);
        } else {
            wheelMotor.set(amount);
        }
    }

    public Command setHigh() {
        return runOnce(() -> {
            setBelts(KickerTunables.HIGH_BELT_SPEED);
            setWheels(KickerTunables.HIGH_WHEEL_SPEED);
        });
        // return runOnce(() -> kickerMotor.set(KickerTunables.HIGH_SPEED));
    }

    public Command setLow() {
        return runOnce(() -> {
            setBelts(KickerTunables.LOW_BELT_SPEED);
            setWheels(KickerTunables.LOW_WHEEL_SPEED);
        });
        // return runOnce(() -> kickerMotor.set(KickerTunables.LOW_SPEED));
    }

    public Command setReverse() {
        return runOnce(() -> {
            setBelts(KickerTunables.REVERSE_BELT_SPEED);
            setWheels(KickerTunables.REVERSE_WHEEL_SPEED);
        });
        // return runOnce(() -> kickerMotor.set(KickerTunables.REVERSE_SPEED));
    }

    public Command setIdle() {
        return runOnce(() -> {
            setBelts(KickerTunables.IDLE_BELT_SPEED);
            setWheels(KickerTunables.IDLE_WHEEL_SPEED);
        });
        // return runOnce(() -> kickerMotor.set(KickerTunables.IDLE_SPEED));
    }

    /** while running, it switches the kicker's speed from high to low periodically */
    public Command kick() {
        return Commands.repeatingSequence(
                        setHigh(),
                        Commands.waitTime(KickerTunables.HIGH_TIME),
                        setLow(),
                        Commands.waitTime(KickerTunables.LOW_TIME))
                .finallyDo(
                    () -> {
                        setBelts(KickerTunables.IDLE_BELT_SPEED);
                        setWheels(KickerTunables.IDLE_WHEEL_SPEED);
                    }
                )
                .withName("kick");
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
            beltMotor.stopMotor();
            wheelMotor.stopMotor();
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
            beltMotor.stopMotor();
            wheelMotor.stopMotor();
        });
    }
}
