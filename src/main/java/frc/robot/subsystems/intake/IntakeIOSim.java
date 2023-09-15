package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

import static java.lang.Math.PI;

public class IntakeIOSim implements IntakeIO {
    SingleJointedArmSim deploySim = new SingleJointedArmSim(DCMotor.getNEO(1),
            /*1 /*/ 20.0,
            0.1,
            .8,
            -PI / 4,
            PI / 2,
            true);

    SingleJointedArmSim modeSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
            /*1 /*/ 50.0,
            0.1,
            .2,
            -PI / 2,
            3 * PI / 4,
            true);

    public IntakeIOSim() {
        modeSim.setInput(2.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (RobotState.isDisabled()) {
            modeSim.setInputVoltage(0.0);
            deploySim.setInputVoltage(0.0);
        }

        deploySim.update(Constants.loopPeriodSecs);
        modeSim.update(Constants.loopPeriodSecs);

        inputs.modeMotorOutputCurrent = modeSim.getCurrentDrawAmps();
        inputs.deployMotorOutputCurrent = deploySim.getCurrentDrawAmps();
        inputs.modeEncoderPosition = modeSim.getAngleRads();
        inputs.deployEncoderPosition = deploySim.getAngleRads();
    }

    @Override
    public void setModeEncoderPosition(double position) {
        IntakeIO.super.setModeEncoderPosition(position);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        IntakeIO.super.setIntakeVoltage(voltage);
    }

    @Override
    public void setModeVoltage(double voltage) {
        modeSim.setInputVoltage(voltage);
    }

    @Override
    public void setDeployVoltage(double voltage) {
        deploySim.setInputVoltage(voltage);
    }

    @Override
    public void setIntakePercentage(double percentage) {
        IntakeIO.super.setIntakePercentage(percentage);
    }
}
