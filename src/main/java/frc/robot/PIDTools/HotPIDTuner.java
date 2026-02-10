package frc.robot.PIDTools;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

/** while PIDController does implement Sendable, it does not have the functionality to wrap its setters in a Command */
public final class HotPIDTuner {
    private static HotPIDTuner instance;
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("PID helper");

    private static final DoubleEntry pEntry = table.getDoubleTopic("p gain").getEntry(0);
    private static final DoubleEntry iEntry = table.getDoubleTopic("i gain").getEntry(0);
    private static final DoubleEntry dEntry = table.getDoubleTopic("d gain").getEntry(0);

    /** put default values onto the dashboard */
    private HotPIDTuner() {
        pEntry.set(0);
        iEntry.set(0);
        dEntry.set(0);
    }

    public static synchronized HotPIDTuner getInstance() {
        if (instance == null) {
            instance = new HotPIDTuner();
        }
        return instance;
    }

    /** put the gains of the supplied pid controller onto the dashboard */
    public void publishGains(PIDController pid) {
        pEntry.set(pid.getP());
        iEntry.set(pid.getI());
        dEntry.set(pid.getD());
    }

    /** pulls the gains that are currently on the dashboard */
    public double[] getGains() {
        return new double[] {pEntry.get(), iEntry.get(), dEntry.get()};
    }

    /** logs the position of the pid, the target of the pid, the pid errors, and the last output of the controller  */
    public static void logPIDDetails(String subsystemName, String controllerName, PIDController controller) {
        String path = subsystemName + "/" + controllerName;

        Logger.recordOutput(path + "/position", controller.getSetpoint() - controller.getError());
        Logger.recordOutput(path + "/target", controller.getSetpoint());

        Logger.recordOutput(path + "/p error", controller.getError());
        Logger.recordOutput(path + "/i error", controller.getAccumulatedError());
        Logger.recordOutput(path + "/d error", controller.getErrorDerivative());

        double lastOutput = controller.getP() * controller.getError()
                + controller.getI() * controller.getAccumulatedError()
                + controller.getD() * controller.getErrorDerivative();

        Logger.recordOutput(path + "/last output", lastOutput);
    }
}
