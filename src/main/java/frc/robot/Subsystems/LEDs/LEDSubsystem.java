package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.LEDTunables;
import java.util.function.BooleanSupplier;

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

        setDefaultCommand(runIdleAnimation());
    }

    @Override
    public void periodic() {
        shooterLEDs.setData(shooterLEDBuffer);
    }

    /**
     * Returns a Command which idles all Addressable LED strips with a Rainbow scroll animation
     */
    public Command runIdleAnimation() {
        return run(() -> {
                    LEDTunables.RAINBOW_ANIMATION.applyTo(shooterLEDBuffer);
                })
                .ignoringDisable(true);
    }

    /**
     * Returns a Command which sets the Shooter LED to green if vision has targets and red otherwise
     *
     * @param visionHasTargets Supplier containing vision status
     */
    public Command runVisionAnimation(BooleanSupplier visionHasTargets) {
        return run(() -> {
                    if (visionHasTargets.getAsBoolean()) {
                        LEDTunables.GREEN_SOLID.applyTo(shooterLEDBuffer);
                    } else {
                        LEDTunables.RED_SOLID.applyTo(shooterLEDBuffer);
                    }
                })
                .ignoringDisable(true);
    }

    /**
     * Returns a Command which sets the Shooter LED to a corresponding Alliance side animation
     *
     * @param onBlueAlliance Supplier indicating alliance side
     */
    public Command runAllianceAnimation(BooleanSupplier onBlueAlliance) {
        return run(() -> {
                    if (onBlueAlliance.getAsBoolean()) {
                        LEDTunables.BLUE_ALLIANCE_ANIMATION.applyTo(shooterLEDBuffer);
                    } else {
                        LEDTunables.RED_ALLIANCE_ANIMATION.applyTo(shooterLEDBuffer);
                    }
                })
                .ignoringDisable(true);
    }
}
