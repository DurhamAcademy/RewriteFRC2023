package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.RepeatedTest;
import org.littletonrobotics.junction.LogTable;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;

class VisionIOInputsTest {
    Random rand = new Random();

    PhotonTrackedTarget randomTrackedTargetFactory() {
        return new PhotonTrackedTarget(rand.nextDouble(),
                rand.nextDouble(),
                rand.nextDouble(),
                rand.nextDouble(),
                rand.nextInt(),
                new Transform3d(new Translation3d(rand.nextDouble(), rand.nextDouble(), rand.nextDouble()),
                        new Rotation3d(rand.nextDouble(),
                                rand.nextDouble(),
                                rand.nextDouble())),
                new Transform3d(new Translation3d(rand.nextDouble(), rand.nextDouble(), rand.nextDouble()),
                        new Rotation3d(rand.nextDouble(),
                                rand.nextDouble(),
                                rand.nextDouble())),
                rand.nextDouble(),
                List.of(new TargetCorner(rand.nextDouble(), rand.nextDouble()),
                        new TargetCorner(rand.nextDouble(), rand.nextDouble()),
                        new TargetCorner(rand.nextDouble(), rand.nextDouble()),
                        new TargetCorner(rand.nextDouble(), rand.nextDouble())),
                List.of(new TargetCorner(rand.nextDouble(), rand.nextDouble()),
                        new TargetCorner(rand.nextDouble(), rand.nextDouble()),
                        new TargetCorner(rand.nextDouble(), rand.nextDouble()),
                        new TargetCorner(rand.nextDouble(), rand.nextDouble())));
    }

    void testConversion(VisionIO.VisionIOInputs old, VisionIO.VisionIOInputs fresh) {
        old.bestTarget = randomTrackedTargetFactory();
        old.targets = new ArrayList<>();
        for (int i = 0, max = rand.nextInt(5); i < max; i++) {
            old.targets.add(randomTrackedTargetFactory());
        }
        old.driverMode = rand.nextBoolean();

        var dCoef = new Matrix<>(Nat.N5(), Nat.N1());
        for (int i = 0; i < 5; i++) {
            dCoef.set(i, 0, rand.nextDouble());
        }
        old.distanceCoefficients = Optional.of(dCoef);


        var camMatrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                camMatrix.set(row, col, rand.nextDouble());
            }
        }
        old.cameraMatrix = Optional.of(camMatrix);

        old.LEDMode = VisionLEDMode.values()[rand.nextInt(VisionLEDMode.values().length)];
        old.hasTargets = true;
        old.timestampSeconds = rand.nextDouble();
        old.latencyMillis = rand.nextDouble();

        old.cameraResult = new PhotonPipelineResult(old.latencyMillis, old.targets);
        old.cameraResult.setTimestampSeconds(old.timestampSeconds);

        LogTable table = new LogTable(rand.nextLong());
        old.toLog(table);

        fresh.fromLog(table);

    }

    @RepeatedTest(10)
    void testConversionLatency() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.latencyMillis, fresh.latencyMillis);
    }

    @RepeatedTest(10)
    void testConversionBestTarget() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.bestTarget, fresh.bestTarget);
    }

    @RepeatedTest(10)
    void testConversionTimestampSeconds() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.timestampSeconds, fresh.timestampSeconds);
    }

    @RepeatedTest(10)
    void testConversionCameraMatrix() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.cameraMatrix, fresh.cameraMatrix);
    }

    @RepeatedTest(10)
    void testConversionCameraResult() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);
//        var oldString = "PhotonPipelineResultString{" +
//                "latency=" + old.cameraResult.getLatencyMillis() +
//                ", timestamp=" + old.cameraResult.getTimestampSeconds() +
//                ", targets=" + old.cameraResult.targets +
//                '}';
//        var newString = "PhotonPipelineResultString{" +
//                "latency=" + fresh.cameraResult.getLatencyMillis() +
//                ", timestamp=" + fresh.cameraResult.getTimestampSeconds() +
//                ", targets=" + fresh.cameraResult.targets +
//                '}';
        assertEquals(old.cameraResult, fresh.cameraResult);
    }

    @RepeatedTest(10)
    void testConversionDistanceCoefficients() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.distanceCoefficients, fresh.distanceCoefficients);
    }

    @RepeatedTest(10)
    void testConversionDriverMode() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.driverMode, fresh.driverMode);
    }

    @RepeatedTest(10)
    void testConversionHasTargets() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.hasTargets, fresh.hasTargets);
    }

    @RepeatedTest(10)
    void testConversionLEDMode() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);

        assertEquals(old.LEDMode, fresh.LEDMode);
    }

    @RepeatedTest(10)
    void testConversionTargets() {
        var old = new VisionIO.VisionIOInputs();
        var fresh = new VisionIO.VisionIOInputs();
        testConversion(old, fresh);
        if (old.targets.size() == fresh.targets.size()) {
            for (int i = 0; i < old.targets.size(); i++) {
                var oldTarget = old.targets.get(i);
                var newTarget = fresh.targets.get(i);
                assertEquals(oldTarget, newTarget);
            }
        }
        assertIterableEquals(old.targets, fresh.targets);
    }
}