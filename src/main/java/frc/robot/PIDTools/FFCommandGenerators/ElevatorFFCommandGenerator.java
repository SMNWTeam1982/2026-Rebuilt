package frc.robot.PIDTools.FFCommandGenerators;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.PIDTools.HotPIDFTuner;

public class ElevatorFFCommandGenerator {
    private final ElevatorFeedforward[] feedforwards;
    private final Subsystem requirement;

    /**
     * @param requirement the subsystem that the generated commands with require
     * @param feedforwards a set of feedforwards that will have the same gains
     */
    public ElevatorFFCommandGenerator(Subsystem requirement, ElevatorFeedforward... feedforwards) {
        this.requirement = requirement;
        this.feedforwards = feedforwards;
    }

    /** put the gains from these feedforwards onto the dashboard */
    public Command publishGains() {
        return requirement.runOnce(() -> {
            HotPIDFTuner.getInstance().publishElevatorFeedforwardGains(feedforwards[0]);
        });
    }

    /** update the feedforward gains with the ones from the dashboard */
    public Command updateGains() {
        return requirement.runOnce(() -> {
            double[] newGains = HotPIDFTuner.getInstance().getFeedforwardGains();
            for (ElevatorFeedforward feedforward : feedforwards) {
                feedforward.setKs(newGains[0]);
                feedforward.setKg(newGains[1]);
                feedforward.setKv(newGains[2]);
                feedforward.setKa(newGains[3]);
            }
        });
    }
}
