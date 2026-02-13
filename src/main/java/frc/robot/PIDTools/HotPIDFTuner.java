package frc.robot.PIDTools;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

/** while PIDController does implement Sendable, it does not have the functionality to wrap its setters in a Command */
public final class HotPIDFTuner {
    private static HotPIDFTuner instance;
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("PID helper");

    private static final DoubleEntry pEntry = table.getDoubleTopic("p gain").getEntry(0);
    private static final DoubleEntry iEntry = table.getDoubleTopic("i gain").getEntry(0);
    private static final DoubleEntry dEntry = table.getDoubleTopic("d gain").getEntry(0);

    private static final DoubleEntry sEntry = table.getDoubleTopic("s gain").getEntry(0);
    private static final DoubleEntry gEntry = table.getDoubleTopic("g gain").getEntry(0);
    private static final DoubleEntry vEntry = table.getDoubleTopic("v gain").getEntry(0);
    private static final DoubleEntry aEntry = table.getDoubleTopic("d gain").getEntry(0);

    /** put default values onto the dashboard */
    private HotPIDFTuner() {
        pEntry.set(0);
        iEntry.set(0);
        dEntry.set(0);
    }

    public static synchronized HotPIDFTuner getInstance() {
        if (instance == null) {
            instance = new HotPIDFTuner();
        }
        return instance;
    }

    /** put the gains of the supplied pid controller onto the dashboard */
    public void publishPIDGains(PIDController pid) {
        pEntry.set(pid.getP());
        iEntry.set(pid.getI());
        dEntry.set(pid.getD());
    }

    /** pulls the gains that are currently on the dashboard */
    public double[] getPIDGains() {
        return new double[] {pEntry.get(), iEntry.get(), dEntry.get()};
    }

    public void publishSimpleMotorFeedforwardGains(SimpleMotorFeedforward feedforward){
        sEntry.set(feedforward.getKs());
        // no g entry for a simple motor feedforward
        vEntry.set(feedforward.getKv());
        aEntry.set(feedforward.getKa());
    }

    public void publishArmFeedforwardGains(ArmFeedforward feedforward){
        sEntry.set(feedforward.getKs());
        gEntry.set(feedforward.getKg());
        vEntry.set(feedforward.getKv());
        aEntry.set(feedforward.getKa());
    }

    public void publishElevatorFeedforwardGains(ElevatorFeedforward feedforward){
        sEntry.set(feedforward.getKs());
        gEntry.set(feedforward.getKg());
        vEntry.set(feedforward.getKv());
        aEntry.set(feedforward.getKa());
    }

    /** returns a double array of [Ks, Kg, Kv, Ka] */
    public double[] getFeedforwardGains(){
        return new double[] {
            sEntry.get(),
            gEntry.get(),
            vEntry.get(),
            aEntry.get()
        };
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
