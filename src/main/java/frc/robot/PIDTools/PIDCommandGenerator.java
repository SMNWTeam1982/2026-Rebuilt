package frc.robot.PIDTools;

import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** generates commands for tuning pid loops
 * <p> instantiate this as a public final member in a subsystem, this can then be accessed outside of the subsystem for when you want to generate commands
 */
public class PIDCommandGenerator<T> {
    private final PIDController[] controllers;
    private final Consumer<T> setPIDTarget;
    private final Subsystem requirementReference;

    public PIDCommandGenerator(Consumer<T> setPIDTarget, Subsystem requirement, PIDController... controllers){
        this.controllers = controllers;
        this.setPIDTarget = setPIDTarget;
        requirementReference = requirement;
    }

    /** set the target of the pid loop */
    public Command setTarget(T target){
        return requirementReference.runOnce(
            () -> setPIDTarget.accept(target)
        );
    }

    /** put the gains of the pid loop(s) onto the dashboard */
    public Command publishPIDGains(){
        return HotPIDTuner.publishGainsToNetworkTables(requirementReference, controllers[0]);
    }

    /** update the pid loop(s) with the gains on the dashboard */
    public Command updatePIDGains(){
        return HotPIDTuner.setGainsFromNetworkTables(requirementReference, controllers);
    }
}
