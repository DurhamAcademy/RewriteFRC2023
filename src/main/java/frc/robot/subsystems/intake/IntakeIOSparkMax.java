package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import static com.revrobotics.CANSparkMax.IdleMode.kBrake;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax intakeMotor;

    private final CANSparkMax modeMotor;
    private final RelativeEncoder modeEncoder;

    private final CANSparkMax deployMotor;
    private final SparkMaxAbsoluteEncoder deployEncoder;

    public IntakeIOSparkMax() {
        // —— Intake ——
        intakeMotor = new CANSparkMax(
                48,
                kBrushless
        );
        intakeMotor.setSmartCurrentLimit(0); // add current limit to limit the torque
        intakeMotor.setIdleMode(kBrake);

        // —— Mode ——
        modeMotor = new CANSparkMax(
                50,
                kBrushless
        );
        modeMotor.setSmartCurrentLimit(20); // add current limit to limit the torque
        modeMotor.setIdleMode(kBrake);

        modeEncoder = modeMotor.getEncoder();
        modeEncoder.setPositionConversionFactor(1.0 / 20.0);

        // —— Deploy ——
        deployMotor = new CANSparkMax(
                49,
                kBrushless
        );
        deployMotor.setSmartCurrentLimit(0); // add current limit to limit the torque
        deployMotor.setIdleMode(kBrake);

        deployEncoder = deployMotor.getAbsoluteEncoder(
                kDutyCycle
        );
        deployEncoder.setInverted(true);
        deployEncoder.setZeroOffset(0.1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.deployEncoderPosition = deployEncoder.getPosition();
        inputs.deployMotorOutputCurrent = deployMotor.getOutputCurrent();

        inputs.intakeMotorSpeed = intakeMotor.getEncoder().getVelocity();
        inputs.intakeMotorOutputCurrent = intakeMotor.getOutputCurrent();

        inputs.modeEncoderPosition = modeEncoder.getPosition();
        inputs.modeMotorOutputCurrent = modeMotor.getOutputCurrent();
    }

    @Override
    public void setDeployVoltage(double voltage) {
        deployMotor.setVoltage(voltage);
    }

    @Override
    public void setIntakePercentage(double percentage) {
        intakeMotor.set(percentage);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setModeEncoderPosition(double position) {
        modeEncoder.setPosition(position);
    }

    @Override
    public void setModeVoltage(double voltage) {
        modeMotor.setVoltage(voltage);
    }
}
