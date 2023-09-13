package frc.robot.subsystems.battery;

import edu.wpi.first.wpilibj.RobotController;

public class BatteryIORio implements BatteryIO {
    @Override
    public void updateInputs(BatteryIOInputs inputs) {
        inputs.voltage = RobotController.getBatteryVoltage();
    }
}
