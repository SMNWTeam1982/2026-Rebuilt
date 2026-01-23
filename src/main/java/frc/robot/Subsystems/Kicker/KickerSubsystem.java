package frc.robot.Subsystems.Kicker;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.kickerTuneables;
import frc.robot.Constants.CANBus.KickerIDs;;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor = new SparkMax(KickerIDs.KICKER_MOTER, MotorType.kBrushless);

    public KickerSubsystem(){}

     public Command startKicker() {
    return runOnce(() -> kickerMotor.set(kickerTuneables.INTAKE_KICKER_SPEED)
  );
  }
public Command stopKicker() {
    return runOnce(() -> kickerMotor.set(0)
  );
  }
}