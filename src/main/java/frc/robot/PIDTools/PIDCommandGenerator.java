package frc.robot.PIDTools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Consumer;

/**
 * generates commands for tuning pid loops
 * <p> instantiate this as a public final member in a subsystem, this can then be accessed outside of the subsystem for when you want to generate commands
 */
public class PIDCommandGenerator<T> {
    private final PIDController[] controllers;
    private final Consumer<T> setTarget;
    private final Subsystem requirementReference;

    public final Trigger atSetpoint;

    public PIDCommandGenerator(Consumer<T> setTarget, Subsystem requirement, PIDController... controllers) {
        this.controllers = controllers;
        this.setTarget = setTarget;
        requirementReference = requirement;

        atSetpoint = new Trigger(() -> {
            for (PIDController controller : controllers) {
                if (!controller.atSetpoint()) {
                    return false;
                }
            }
            return true;
        });
    }

    /** creates a command to set the target of the pid loop to this target */
    public Command setTarget(T target) {
        return requirementReference.runOnce(() -> setTarget.accept(target));
    }

    /** put the gains of the pid loop(s) onto the dashboard */
    public Command publishPIDGains() {
        return requirementReference.runOnce(() -> {
            HotPIDTuner.getInstance().publishGains(controllers[0]);
        });
    }

    /** update the pid loop(s) with the gains on the dashboard */
    public Command updatePIDGains() {
        return requirementReference.runOnce(() -> {
            double[] newGains = HotPIDTuner.getInstance().getGains();
            for (PIDController controller : controllers) {
                controller.setPID(newGains[0], newGains[1], newGains[2]);
            }
        });
    }
}
