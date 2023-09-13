package frc.robot.subsystems.drive;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.util.Random;

class ModuleIOFalcon500Test {
    @Test
    @DisplayName("Test Constructor")
    void init() {
        var rand = new Random();
        var moduleIO = new ModuleIOFalcon500(
                rand.nextInt(30),
                rand.nextInt(30),
                rand.nextBoolean(),
                rand.nextBoolean(),
                rand.nextInt(30)
        );
    }
}