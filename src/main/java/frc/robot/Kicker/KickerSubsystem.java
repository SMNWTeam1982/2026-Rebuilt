package frc.robot.Kicker;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.kickerTuneables;
import frc.robot.Constants.CANBus.KickerIDs;;

public class KickerSubsystem extends SubsystemBase {
    private final SparkMax kickerMotor = new SparkMax(KickerIDs.KICKER_MOTER, MotorType.kBrushless);

    public KickerSubsystem(){}

     public Command startIntaking() {
    return runOnce(() -> kickerMotor.set(kickerTuneables.INTAKE_SPEED)
  );
  }
public Command stopIntakeing() {
    return runOnce(() -> kickerMotor.set(0)
  );
  }
}