package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public interface VisionIO {
    default void updateInputs(VisionIOInputs inputs) {
    }

    @AutoLog
    class VisionIOInputs implements LoggableInputs, Cloneable {
        boolean driverMode = false;
        PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();
        List<PhotonTrackedTarget> targets = List.of();
        double latencyMillis = 0;
        double timestampSeconds = 0;
        boolean hasTargets = false;
        VisionLEDMode LEDMode;
        Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
        PhotonPipelineResult cameraResult = new PhotonPipelineResult();
        Optional<Matrix<N5, N1>> distanceCoefficients = Optional.empty();

        @Override
        public void toLog(LogTable table) {
            table.put("DriverMode", driverMode);
            table.put("LatencyMillis", latencyMillis);
            table.put("TimestampSeconds", timestampSeconds);
            table.put("HasTargets", hasTargets);
        }

        @Override
        public void fromLog(LogTable table) {
            driverMode = table.getBoolean("DriverMode", driverMode);
            latencyMillis = table.getDouble("LatencyMillis", latencyMillis);
            timestampSeconds = table.getDouble("TimestampSeconds", timestampSeconds);
            hasTargets = table.getBoolean("HasTargets", hasTargets);
        }

        public VisionIOInputs clone() {
            VisionIOInputs copy = new VisionIOInputs();
            copy.driverMode = this.driverMode;
            copy.latencyMillis = this.latencyMillis;
            copy.timestampSeconds = this.timestampSeconds;
            copy.hasTargets = this.hasTargets;
            return copy;
        }
    }
}
