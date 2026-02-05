package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PIDHelper {
    private static PIDHelper instance;
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("PID helper");

    private static final DoubleEntry pEntry = table.getDoubleTopic("/PID helper/p gain").getEntry(0);
    private static final DoubleEntry iEntry = table.getDoubleTopic("/PID helper/i gain").getEntry(0);
    private static final DoubleEntry dEntry = table.getDoubleTopic("/PID helper/d gain").getEntry(0);

    private PIDHelper(){
        pEntry.set(0);
        iEntry.set(0);
        dEntry.set(0);
    }

    public synchronized static PIDHelper getInstance(){
        if (instance == null){
            instance = new PIDHelper();
        }
        return instance;
    }

    public void publishGains(PIDController pid){
        pEntry.set(pid.getP());
        iEntry.set(pid.getI());
        dEntry.set(pid.getD());
    }

    public double[] getGains(){
        return new double[] {
            pEntry.get(),
            iEntry.get(),
            dEntry.get()
        };
    }
}
