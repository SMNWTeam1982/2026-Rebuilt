package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.LEDTunables;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class LEDSubsystem extends SubsystemBase {

    // Shooter Addressable LED Strip
    private final AddressableLED shooterLEDs;
    private final AddressableLEDBuffer shooterLEDBuffer;

    public LEDSubsystem() {
        // PWM Port 0
        shooterLEDs = new AddressableLED(0);
        shooterLEDBuffer = new AddressableLEDBuffer(LEDTunables.SHOOTER_LED_STRIP_LENGTH);
        shooterLEDs.setLength(LEDTunables.SHOOTER_LED_STRIP_LENGTH);
        shooterLEDs.start();
    }

    @Override
    public void periodic() {
        shooterLEDs.setData(shooterLEDBuffer);
        Logger.recordOutput("Test/ShooterLED", shooterLEDBuffer.getLED(0));
    }

    /**
     * Returns a Command which sets LED strips to corresponding animations.
     */
    public Command setLEDAnimation(Supplier<LEDTunables.LED_PATTERN> ledPattern) {
        return run(() -> {
            switch(ledPattern.get()) {
                case NO_VISION:
                    LEDTunables.RED_SOLID.applyTo(shooterLEDBuffer);
                    break;
                case HAS_VISION:
                    LEDTunables.GREEN_SOLID.applyTo(shooterLEDBuffer);
                    break;
                case IDLE_ANIMATED:
                    LEDTunables.RAINBOW_ANIMATION.applyTo(shooterLEDBuffer);
                    break;
                case BLUE_ALLIANCE:
                    LEDTunables.BLUE_ALLIANCE_ANIMATION.applyTo(shooterLEDBuffer);
                    break;
                case RED_ALLIANCE:
                    LEDTunables.RED_ALLIANCE_ANIMATION.applyTo(shooterLEDBuffer);
                    break;
                case SHOOTING:
                    LEDTunables.SHOOTING_ANIMATION.applyTo(shooterLEDBuffer);
                    break;
                default:
                    shooterLEDs.stop();
                    break;
            };
        }).ignoringDisable(true);
    }
}
