package frc.robot.Subsystems.Kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.KickerIDs;
import frc.robot.Constants.Tunables;
import frc.robot.Constants.Tunables.KickerTunables;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor = new SparkMax(KickerIDs.KICKER, MotorType.kBrushless);

    public KickerSubsystem() {
        kickerMotor.configure(
                Tunables.DEFAULT_SPARK_MAX_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startKicker() {
        return runOnce(() -> kickerMotor.set(KickerTunables.KICKER_SPEED));
    }

    /** runs the kicker while active the stops it when it ends */
    public Command kick() {
        return startEnd(
                () -> kickerMotor.set(KickerTunables.KICKER_SPEED),
                () -> kickerMotor.set(KickerTunables.KICKER_IDLE_SPEED));
    }

    public Command idleKicker() {
        return runOnce(() -> kickerMotor.set(KickerTunables.KICKER_IDLE_SPEED));
    }
}
