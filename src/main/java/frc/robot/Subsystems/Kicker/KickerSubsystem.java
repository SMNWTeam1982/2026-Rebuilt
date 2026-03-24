package frc.robot.Subsystems.Kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CANBus.KickerIDs;
import frc.robot.Constants.Tunables.KickerTunables;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor = new SparkMax(KickerIDs.KICKER, MotorType.kBrushless);

    public KickerSubsystem() {
        kickerMotor.configure(
                KickerTunables.KICKER_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Kicker/kicker output", kickerMotor.getAppliedOutput());
        Logger.recordOutput("Kicker/kicker current", kickerMotor.getOutputCurrent());
    }

    public Command startKicker() {
        return runOnce(() -> kickerMotor.set(KickerTunables.HIGH_SPEED));
    }

    /** while running, it switches the kicker's speed from high to low periodically */
    public Command kick() {
        return runOnce(() -> kickerMotor.set(KickerTunables.HIGH_SPEED))
                .andThen(new WaitCommand(KickerTunables.HIGH_TIME))
                .andThen(runOnce(() -> kickerMotor.set(KickerTunables.LOW_SPEED)))
                .andThen(new WaitCommand(KickerTunables.LOW_TIME))
                .repeatedly();
    }

    public Command idleKicker() {
        return runOnce(() -> kickerMotor.set(KickerTunables.IDLE_SPEED));
    }
}
