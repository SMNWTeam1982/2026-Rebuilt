package frc.robot.PIDTools.FFCommandGenerators;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.PIDTools.HotPIDFTuner;

/**
 * generates commands for tuning feedforwards
 * <p> instantiate this as a public final member in a subsystem, this can then be accessed outside of the subsystem for when you want to generate commands
 */
public class SimpleMotorFFCommandGenerator {
    private final SimpleMotorFeedforward[] feedforwards;
    private final Subsystem requirement;

    /**
     * @param requirement the subsystem that the generated commands with require
     * @param feedforwards a set of feedforwards that will have the same gains
     */
    public SimpleMotorFFCommandGenerator(Subsystem requirement, SimpleMotorFeedforward... feedforwards) {
        this.requirement = requirement;
        this.feedforwards = feedforwards;
    }

    /** put the gains from these feedforwards onto the dashboard */
    public Command publishGains() {
        return requirement.runOnce(() -> {
            HotPIDFTuner.getInstance().publishSimpleMotorFeedforwardGains(feedforwards[0]);
        });
    }

    /** update the feedforward gains with the ones from the dashboard */
    public Command updateGains() {
        return requirement.runOnce(() -> {
            double[] newGains = HotPIDFTuner.getInstance().getFeedforwardGains();
            for (SimpleMotorFeedforward feedforward : feedforwards) {
                feedforward.setKs(newGains[0]);

                feedforward.setKv(newGains[2]);
                feedforward.setKa(newGains[3]);
            }
        });
    }
}
