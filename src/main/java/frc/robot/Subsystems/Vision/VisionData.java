package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** a class to bundle vision result data together */
public class VisionData {
    /** estimated pose from the vision hardware */
    public final Pose2d pose;

    /** this timestamp has the same epoch as the FPGA timestamp */
    public final double timestamp;

    /** error in the form of (x meters, y meters, heading radians) */
    public final Matrix<N3, N1> standardDeviations;

    /**
     * @param pose approximated pose from the vision hardware
     * @param timestamp timestamp in seconds that the result was taken. this should have the same
     *     epoch as the FPGA timestamp
     * @param standardDeviations error in the form of (x meters, y meters, heading radians)
     */
    public VisionData(Pose2d pose, double timestamp, Matrix<N3, N1> standardDeviations) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.standardDeviations = standardDeviations;
    }
}
