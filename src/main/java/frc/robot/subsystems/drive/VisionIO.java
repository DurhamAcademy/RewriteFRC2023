package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public interface VisionIO {
    default void updateInputs(VisionIOInputs inputs) {
    }

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

        public double[] positionToArray(Pose3d... value) {
            double[] data = new double[value.length * 7];
            for (int i = 0; i < value.length; i++) {
                data[i * 7] = value[i].getX();
                data[i * 7 + 1] = value[i].getY();
                data[i * 7 + 2] = value[i].getZ();
                data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
                data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
                data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
                data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
            }
            return data;
        }

        public Transform3d[] arrayToPosition(double[] value) {
            var length = ((int) (value.length / 7.0));
            Transform3d[] data = new Transform3d[length + 1];
            for (int i = 0; i < length; i++) {
                var X = value[i * 7];
                var Y = value[i * 7 + 1];
                var Z = value[i * 7 + 2];
                var rotW = value[i * 7 + 3];
                var rotX = value[i * 7 + 4];
                var rotY = value[i * 7 + 5];
                var rotZ = value[i * 7 + 6];
                data[i] = new Transform3d(new Translation3d(X, Y, Z), new Rotation3d(new Quaternion(rotW, rotX, rotY, rotZ)));
            }
            return data;
        }

        public double[] positionToArray(Transform3d... value) {
            double[] data = new double[value.length * 7];
            for (int i = 0; i < value.length; i++) {
                data[i * 7] = value[i].getX();
                data[i * 7 + 1] = value[i].getY();
                data[i * 7 + 2] = value[i].getZ();
                data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
                data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
                data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
                data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
            }
            return data;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("DriverMode", driverMode);
            table.put("LatencyMillis", latencyMillis);
            table.put("TimestampSeconds", timestampSeconds);
//            System.out.println(targets.size());
            table.put("TargetCount", targets.size());
            table.put("HasTargets", hasTargets);
            var bestTargetTable = table.getSubtable("BestTarget");
            if (bestTarget != null) {
                bestTargetTable.put("yaw", bestTarget.getYaw());
                bestTargetTable.put("pitch", bestTarget.getPitch());
                bestTargetTable.put("area", bestTarget.getArea());
                bestTargetTable.put("skew", bestTarget.getSkew());
                bestTargetTable.put("fiducialId", bestTarget.getFiducialId());

                bestTargetTable.put("bestCameraToTarget", positionToArray(bestTarget.getBestCameraToTarget()));

                bestTargetTable.put("altCamToTarget", positionToArray(bestTarget.getAlternateCameraToTarget()));

                bestTargetTable.put("poseAmbiguity", bestTarget.getPoseAmbiguity());
//
                List<TargetCorner> minAreaRectCorners = bestTarget.getMinAreaRectCorners();
                if (minAreaRectCorners != null) {
                    var minAreaRectCornersXValues = new double[minAreaRectCorners.size()];
                    var minAreaRectCornersYValues = new double[minAreaRectCorners.size()];
                    for (int i = 0, minAreaRectCornersSize = minAreaRectCorners.size(); i < minAreaRectCornersSize; i++) {
                        TargetCorner minAreaRectCorner = minAreaRectCorners.get(i);
                        minAreaRectCornersXValues[i] = minAreaRectCorner.x;
                        minAreaRectCornersYValues[i] = minAreaRectCorner.y;
                    }
                    bestTargetTable.put("minAreaRectCorners_X", minAreaRectCornersXValues);
                    bestTargetTable.put("minAreaRectCorners_Y", minAreaRectCornersYValues);
                }
                List<TargetCorner> detectedCorners = bestTarget.getDetectedCorners();
                if (detectedCorners != null) {
                    var detectedCornersXValues = new double[detectedCorners.size()];
                    var detectedCornersYValues = new double[detectedCorners.size()];
                    for (int i = 0, minAreaRectCornersSize = detectedCorners.size(); i < minAreaRectCornersSize; i++) {
                        TargetCorner detectedCorner = detectedCorners.get(i);
                        detectedCornersXValues[i] = detectedCorner.x;
                        detectedCornersYValues[i] = detectedCorner.y;
                    }
                    bestTargetTable.put("detectedCorners_X", detectedCornersXValues);
                    bestTargetTable.put("detectedCorners_Y", detectedCornersYValues);
                }
            }
            var targetsTable = table.getSubtable("Targets");
            if (targets != null) {
                var allDetectedCornersXValues = new double[targets.size() * 4];
                var allDetectedCornersYValues = new double[targets.size() * 4];
                for (int i = 0; i < targets.size(); i++) {
                    var target = targets.get(i);
                    var targetTable = targetsTable.getSubtable("Target " + i);
                    targetTable.put("FiducialId", target.getFiducialId());
                    targetTable.put("HashCode", target.hashCode());
                    targetTable.put("Skew", target.getSkew());
                    targetTable.put("Yaw", target.getYaw());
                    targetTable.put("Area", target.getArea());
                    targetTable.put("Pitch", target.getPitch());
                    targetTable.put("PoseAmbiguity", target.getPoseAmbiguity());
                    targetTable.put("BestCameraToTarget", positionToArray(target.getBestCameraToTarget()));
                    targetTable.put("AlternateCameraToTarget", positionToArray(target.getAlternateCameraToTarget()));
                    List<TargetCorner> detectedCorners = target.getDetectedCorners();
                    if (detectedCorners != null) {
                        for (int b = 0, minAreaRectCornersSize = detectedCorners.size(); b < minAreaRectCornersSize; b++) {
                            TargetCorner detectedCorner = detectedCorners.get(b);
                            allDetectedCornersXValues[(i * 4) + b] = detectedCorner.x;
                            allDetectedCornersYValues[(i * 4) + b] = detectedCorner.y;
                        }
                    }
                }
                targetsTable.put("detectedCorners_X", allDetectedCornersXValues);
                targetsTable.put("detectedCorners_Y", allDetectedCornersYValues);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            driverMode = table.getBoolean("DriverMode", driverMode);
            latencyMillis = table.getDouble("LatencyMillis", latencyMillis);
            timestampSeconds = table.getDouble("TimestampSeconds", timestampSeconds);
            hasTargets = table.getBoolean("HasTargets", hasTargets);

            var bestTargetTable = table.getSubtable("BestTarget");
            if (bestTarget != null) {
                var yaw = bestTargetTable.getDouble("yaw", 0.0);
                var pitch = bestTargetTable.getDouble("pitch", 0.0);
                var area = bestTargetTable.getDouble("area", 0.0);
                var skew = bestTargetTable.getDouble("skew", 0.0);
                int fiducialId = (int) bestTargetTable.getInteger("fiducialId", -1);

                var bestCameraToTarget = arrayToPosition(bestTargetTable.getDoubleArray("bestCameraToTarget", new double[0]));

                var altCamToTarget = arrayToPosition(bestTargetTable.getDoubleArray("altCamToTarget", new double[0]));

                var poseAmbiguity = bestTargetTable.getDouble("poseAmbiguity", 0.0);

                var minAreaRectCornersXValues = bestTargetTable.getDoubleArray("minAreaRectCorners_X", new double[0]);
                var minAreaRectCornersYValues = bestTargetTable.getDoubleArray("minAreaRectCorners_Y", new double[0]);
                var minAreaRectCorners = new ArrayList<TargetCorner>();
                for (int i = 0,
                     minAreaRectCornersSize = Math.min(minAreaRectCornersXValues.length, minAreaRectCornersYValues.length);
                     i < minAreaRectCornersSize;
                     i++)
                    minAreaRectCorners.add(new TargetCorner(
                            minAreaRectCornersXValues[i],
                            minAreaRectCornersYValues[i]
                    ));


                var detectedCornersXValues = bestTargetTable.getDoubleArray("detectedCorners_X", new double[0]);
                var detectedCornersYValues = bestTargetTable.getDoubleArray("detectedCorners_Y", new double[0]);
                var detectedCornersSize = Math.min(detectedCornersXValues.length, detectedCornersYValues.length);
                var detectedCorners = new ArrayList<TargetCorner>(detectedCornersSize);
                for (int i = 0;
                     i < detectedCornersSize;
                     i++)
                    detectedCorners.add(new TargetCorner(
                            detectedCornersXValues[i],
                            detectedCornersYValues[i]
                    ));

                bestTarget = new PhotonTrackedTarget(yaw,
                        pitch,
                        area,
                        skew,
                        fiducialId,
                        bestCameraToTarget[0],
                        altCamToTarget[0],
                        poseAmbiguity,
                        minAreaRectCorners,
                        detectedCorners);
            }
            var targetsTable = table.getSubtable("Targets");
            if (targets != null) {
                var allDetectedCornersXValues = new double[targets.size() * 4];
                var allDetectedCornersYValues = new double[targets.size() * 4];
                for (int i = 0; i < targets.size(); i++) {
                    var target = targets.get(i);
                    var targetTable = targetsTable.getSubtable("Target " + i);
                    targetTable.put("FiducialId", target.getFiducialId());
                    targetTable.put("HashCode", target.hashCode());
                    targetTable.put("Skew", target.getSkew());
                    targetTable.put("Yaw", target.getYaw());
                    targetTable.put("Area", target.getArea());
                    targetTable.put("Pitch", target.getPitch());
                    targetTable.put("PoseAmbiguity", target.getPoseAmbiguity());
                    targetTable.put("BestCameraToTarget", positionToArray(target.getBestCameraToTarget()));
                    targetTable.put("AlternateCameraToTarget", positionToArray(target.getAlternateCameraToTarget()));
                    List<TargetCorner> detectedCorners = target.getDetectedCorners();
                    if (detectedCorners != null) {
                        for (int b = 0, minAreaRectCornersSize = detectedCorners.size(); b < minAreaRectCornersSize; b++) {
                            TargetCorner detectedCorner = detectedCorners.get(b);
                            allDetectedCornersXValues[(i * 4) + b] = detectedCorner.x;
                            allDetectedCornersYValues[(i * 4) + b] = detectedCorner.y;
                        }
                    }
                }
                targetsTable.put("detectedCorners_X", allDetectedCornersXValues);
                targetsTable.put("detectedCorners_Y", allDetectedCornersYValues);
            }
        }

        public VisionIOInputs clone() {
            try {
                VisionIOInputs cloned = (VisionIOInputs) super.clone();
                // For fields that are mutable objects, create deep copies if necessary.
                cloned.bestTarget = this.bestTarget;//.clone();
                cloned.targets = new ArrayList<>(this.targets);
                cloned.cameraResult = this.cameraResult;//.clone();
                cloned.cameraMatrix = this.cameraMatrix;
                cloned.distanceCoefficients = this.distanceCoefficients;
                return cloned;
            } catch (CloneNotSupportedException e) {
                // This should never happen since we implement Cloneable.
                throw new InternalError(e);
            }
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof VisionIOInputs)) return false;

            VisionIOInputs that = (VisionIOInputs) o;

            if (driverMode != that.driverMode) return false;
            if (Double.compare(latencyMillis, that.latencyMillis) != 0) return false;
            if (Double.compare(timestampSeconds, that.timestampSeconds) != 0) return false;
            if (hasTargets != that.hasTargets) return false;
            if (!Objects.equals(bestTarget, that.bestTarget)) return false;
            if (!targets.equals(that.targets)) return false;
            if (LEDMode != that.LEDMode) return false;
            if (!cameraMatrix.equals(that.cameraMatrix)) return false;
            if (!cameraResult.equals(that.cameraResult)) return false;
            return distanceCoefficients.equals(that.distanceCoefficients);
        }

        @Override
        public int hashCode() {
            int result;
            long temp;
            result = (driverMode ? 1 : 0);
            result = 31 * result + (bestTarget != null ? bestTarget.hashCode() : 0);
            result = 31 * result + targets.hashCode();
            temp = Double.doubleToLongBits(latencyMillis);
            result = 31 * result + (int) (temp ^ (temp >>> 32));
            temp = Double.doubleToLongBits(timestampSeconds);
            result = 31 * result + (int) (temp ^ (temp >>> 32));
            result = 31 * result + (hasTargets ? 1 : 0);
            result = 31 * result + LEDMode.hashCode();
            result = 31 * result + cameraMatrix.hashCode();
            result = 31 * result + cameraResult.hashCode();
            result = 31 * result + distanceCoefficients.hashCode();
            return result;
        }
    }
}
