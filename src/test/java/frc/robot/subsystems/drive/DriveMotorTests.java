package frc.robot.subsystems.drive;

import org.junit.jupiter.api.*;

import java.io.IOException;
import java.util.Random;

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
public class DriveMotorTests {

    /**
     * Should be constant throughout all tests
     */
    Random rand = new Random();

    ModuleIO.ModuleIOInputs driveIOInputs;
    ModuleIO.ModuleIOInputs newDriveIOInputs;

    Drive drive;

    //    @BeforeEach
    @BeforeAll
    void setUp() throws IOException {
        driveIOInputs = null;
        newDriveIOInputs = null;
        var moduleIOs = new ModuleIO[4];
//        var gyroSim = new GyroIOSim(new Translation2d[]{new Translation2d(1.0, 1.0),
//                new Translation2d(1.0, -1.0),
//                new Translation2d(-1.0, 1.0),
//                new Translation2d(-1.0, -1.0)});
        var gyroSim = new GyroIO() {
        };
        for (int i = 0; i < 4; i++) {
            moduleIOs[i] = new ModuleIOSim();
        }
        drive = new Drive(moduleIOs[0],
                moduleIOs[1],
                moduleIOs[2],
                moduleIOs[3],
                gyroSim,
                new VisionIO() {
                });
    }

    @RepeatedTest(10)
    @DisplayName("Test Turn PID")
    void testPID() {
        var x = rand.nextDouble() * 2 - 1;
        var y = rand.nextDouble() * 2 - 1;
        var rot = rand.nextDouble() * 2 - 1;
        System.out.println("x = " + x);
        System.out.println("y = " + y);
        System.out.println("rot = " + rot);
        for (int i = 0; i < 50; i++) {
            drive.periodic();
            drive.driveArcade(x, y, rot, true);
            int finalI = i;
            drive.anglePIDs.forEach(pidController -> {
                System.out.printf("%d = %s%n", finalI, pidController.getPositionError());
            });
        }
        drive.anglePIDs.forEach(pidController -> {
            Assertions.assertEquals(0.0, pidController.getPositionError(), 0.05);
        });
    }
}
