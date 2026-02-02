package frc.robot.Subsystems.Climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.ClimberIDs;
import frc.robot.Constants.Tunables.ClimberTunables;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(ClimberIDs.CLIMBER, MotorType.kBrushless);

    public ClimberSubsystem() {}

    /** moves the climber away from the robot and ready to be used to hold on to the ladder */
    public Command moveClimberOut() {
        return startEnd(
                () -> {
                    climberMotor.set(ClimberTunables.EXTEND_SPEED);
                },
                () -> {
                    climberMotor.set(0.0);
                });
    }

    /** moves the climber in, so it can be out of the way */
    public Command moveClimberIn() {
        return startEnd(
                () -> {
                    climberMotor.set(ClimberTunables.RETRACT_SPEED);
                },
                () -> {
                    climberMotor.set(0.0);
                });
    }
}
