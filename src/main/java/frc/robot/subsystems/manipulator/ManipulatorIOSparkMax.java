package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.flywheel.FlywheelIO;

public class ManipulatorIOSparkMax implements ManipulatorIO {
    public static final double GEAR_RATIO = 1;
    public static final int deviceId = 6; //TODO fill this in
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    public ManipulatorIOSparkMax(){
        motor = new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless);

        encoder = motor.getEncoder();
        pid = motor.getPIDController();

        motor.restoreFactoryDefaults();

        motor.setInverted(false);

        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(30);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                encoder.getVelocity() / GEAR_RATIO);
        inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    }

    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts) {
        pid.setReference(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
                        * GEAR_RATIO,
                CANSparkMax.ControlType.kVelocity, 0, ffVolts, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setP(kP, 0);
        pid.setI(kI, 0);
        pid.setD(kD, 0);
        pid.setFF(0, 0);
    }
}
