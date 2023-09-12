package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class ManipulatorIOSparkMax implements ManipulatorIO {
    public static final int deviceId = 31;
    private final CANSparkMax motor;

    public ManipulatorIOSparkMax(){
        motor = new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    }
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setPercentage(double percentage) {
        motor.set(percentage);
    }


    @Override
    public void stop() {
        motor.stopMotor();
    }

}
