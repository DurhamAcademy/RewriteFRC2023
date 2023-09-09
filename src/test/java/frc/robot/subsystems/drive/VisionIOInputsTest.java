package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
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

import static org.junit.jupiter.api.Assertions.*;

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

    @RepeatedTest(10)
    void testConversion() {
        var old = new VisionIO.VisionIOInputs();
        old.bestTarget = randomTrackedTargetFactory();
        old.targets = new ArrayList<PhotonTrackedTarget>();
        for (int i = 0, max = rand.nextInt(5); i < max; i++) {
            old.targets.add(randomTrackedTargetFactory());
        }
        old.driverMode = rand.nextBoolean();

        var dCoef = new Matrix<N5, N1>(Nat.N5(), Nat.N1());
        for (int i = 0; i < 5; i++) {
            dCoef.set(i, 0, rand.nextDouble());
        }
        old.distanceCoefficients = Optional.of(dCoef);

        old.cameraResult = new PhotonPipelineResult(rand.nextDouble(), old.targets);

        var camMatrix = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
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


        LogTable table = new LogTable(rand.nextLong());
        old.toLog(table);

        var fresh = new VisionIO.VisionIOInputs();
        fresh.fromLog(table);
        assertAll(
                () -> assertEquals(old.latencyMillis, fresh.latencyMillis),
                () -> assertEquals(old.bestTarget, fresh.bestTarget),
                () -> assertEquals(old.timestampSeconds, fresh.timestampSeconds),
                () -> assertEquals(old.cameraMatrix, fresh.cameraMatrix),
                () -> assertEquals(old.cameraResult, fresh.cameraResult),
                () -> assertEquals(old.distanceCoefficients, fresh.distanceCoefficients),
                () -> assertEquals(old.driverMode, fresh.driverMode),
                () -> assertEquals(old.hasTargets, fresh.hasTargets),
                () -> assertEquals(old.LEDMode, fresh.LEDMode),
                () -> assertIterableEquals(old.targets, fresh.targets)
        );

    }
}