package frc.robot.Subsystems.Kicker;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.KickerIDs;
import frc.robot.Constants.Tunables.KickerTunables;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor = new SparkMax(KickerIDs.KICKER, MotorType.kBrushless);

    public KickerSubsystem() {}

    public Command startKicker() {
        return runOnce(() -> kickerMotor.set(KickerTunables.KICKER_SPEED));
    }

    /** runs the kicker while active the stops it when it ends */
    public Command kick() {
        return startEnd(() -> kickerMotor.set(KickerTunables.KICKER_SPEED), () -> kickerMotor.set(0));
    }

    public Command stopKicker() {
        return runOnce(() -> kickerMotor.set(0));
    }
}
