package frc.robot.subsystems.battery;

import org.littletonrobotics.junction.AutoLog;

public interface BatteryIO {
    default void updateInputs(BatteryIOInputs inputs) {
    }

    @AutoLog
    class BatteryIOInputs {
        public double voltage = 0.0;
    }
}
