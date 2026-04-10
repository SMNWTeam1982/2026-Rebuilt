package frc.robot.Subsystems.LEDs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.LEDTunables;

public class LEDSubsystem extends SubsystemBase {

    // Global Addressable LED Strip Object
    private final AddressableLED ledStrips;

    // Global LED buffer
    private final AddressableLEDBuffer ledBuffer;
    // Hopper LED buffer views
    private final AddressableLEDBufferView leftHopperView;
    private final AddressableLEDBufferView rightHopperView;
    // Shooter LED buffer
    private final AddressableLEDBufferView shooterView;

    public LEDSubsystem() {
        // Configure global LED strip
        ledStrips = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(LEDTunables.GLOBAL_LED_STRIP_LENGTH);
        ledStrips.setLength(LEDTunables.GLOBAL_LED_STRIP_LENGTH);

        // Set up hopper LED views
        leftHopperView = ledBuffer.createView(0, 14);
        rightHopperView = ledBuffer.createView(15, 31);

        // Set up shooter LED view
        shooterView = ledBuffer.createView(32, 44);

        // Start LEDs
        ledStrips.start();
        setDefaultCommand(setIdleAnimation());
    }

    @Override
    public void periodic() {
        ledStrips.setData(ledBuffer);
    }

    /**
     * Command that sets all LED strips to the idle Animation
     */
    public Command setIdleAnimation() {
        return run(() -> {
            LEDTunables.RAINBOW_ANIMATION.applyTo(shooterView);
            LEDTunables.RAINBOW_ANIMATION.applyTo(leftHopperView);
            LEDTunables.RAINBOW_ANIMATION.applyTo(rightHopperView);
        }).ignoringDisable(true);
    }

    /**
     * Command that sets LEDs based on robot state in auto
     * @param hasVisionTargets if true, sets shooter LEDs to green & hopper LEDs to alliance color. otherwise, sets all to red
     */
    public Command setAutoAnimation(BooleanSupplier hasVisionTargets, BooleanSupplier onBlueAlliance) {
        return run(() -> {
            if (hasVisionTargets.getAsBoolean()) {
                LEDTunables.GREEN_SOLID.applyTo(shooterView);
                if (onBlueAlliance.getAsBoolean()) {
                    LEDTunables.BLUE_ALLIANCE_ANIMATION.applyTo(leftHopperView);
                    LEDTunables.BLUE_ALLIANCE_ANIMATION.applyTo(rightHopperView);
                } else {
                    LEDTunables.RED_ALLIANCE_ANIMATION.applyTo(leftHopperView);
                    LEDTunables.RED_ALLIANCE_ANIMATION.applyTo(rightHopperView);
                }
            } else {
                LEDTunables.RED_SOLID.applyTo(shooterView);
                LEDTunables.RED_SOLID.applyTo(leftHopperView);
                LEDTunables.RED_SOLID.applyTo(rightHopperView);
            }
        }).ignoringDisable(true);
    }

    /**
     * Command that sets LEDs based on robot state in teleop
     * @param isShooting if true, sets all LEDs to shooting animation, otherwise shooter to idle & hopper to alliance colors.
     */
    public Command setTeleopAnimation(BooleanSupplier isShooting, BooleanSupplier onBlueAlliance) {
        return run(() -> {
            if (isShooting.getAsBoolean()) {
                LEDTunables.SHOOTING_ANIMATION.applyTo(shooterView);
                LEDTunables.SHOOTING_ANIMATION.applyTo(leftHopperView);
                LEDTunables.SHOOTING_ANIMATION.applyTo(rightHopperView);
            } else {
                LEDTunables.RAINBOW_ANIMATION.applyTo(shooterView);
                if (onBlueAlliance.getAsBoolean()) {
                    LEDTunables.BLUE_ALLIANCE_ANIMATION.applyTo(leftHopperView);
                    LEDTunables.BLUE_ALLIANCE_ANIMATION.applyTo(rightHopperView);
                } else {
                    LEDTunables.RED_ALLIANCE_ANIMATION.applyTo(leftHopperView);
                    LEDTunables.RED_ALLIANCE_ANIMATION.applyTo(rightHopperView);
                }
            }
        }).ignoringDisable(true);
    }

}