package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

import java.util.Optional;

public class ModuleIOSim implements ModuleIO {
    private Optional<Integer> id = Optional.empty();
    private Optional<GyroIOSim> gyroSim = Optional.empty();

    private final FlywheelSim driveSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private final FlywheelSim turnSim =
            new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(int id, GyroIOSim gyroSim) {
        this.id = Optional.of(id);
        this.gyroSim = Optional.ofNullable(gyroSim);
    }

    public ModuleIOSim() {
    }

    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);

        double angleDiffRad =
                turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.drivePositionRad = inputs.drivePositionRad
                + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveOutputVoltage = driveAppliedVolts;
        inputs.driveCurrentAmps =
                new double[]{Math.abs(driveSim.getCurrentDrawAmps())};
        inputs.driveTempCelsius = new double[]{};

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnPositionRad = turnRelativePositionRad;
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnOutputVoltage = turnAppliedVolts;
        inputs.turnCurrentAmps =
                new double[]{Math.abs(turnSim.getCurrentDrawAmps())};
        inputs.turnTempCelsius = new double[]{};
        gyroSim.ifPresent(gyroIOSim -> gyroIOSim.setInput(id.orElseThrow(), inputs));
    }

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts - (driveSim.getAngularVelocityRadPerSec() * 0.02));
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -1.0, 1.0);
        turnSim.setInputVoltage(turnAppliedVolts - (turnSim.getAngularVelocityRadPerSec() * 0.005));
    }
}