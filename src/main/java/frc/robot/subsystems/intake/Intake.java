package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.clamp;
import static java.lang.Math.PI;

public class Intake extends SubsystemBase {
    IntakeIO intakeIO;
    IntakeIOInputsAutoLogged inputs;

    ProfiledPIDController deployPID = new ProfiledPIDController(
            4.0,
            0.0,
            0.05,
            new TrapezoidProfile.Constraints(
                    12.0,
                    18.0
            )
    );

    ProfiledPIDController modePID = new ProfiledPIDController(
            3.05,
            0.0,
            0.035,
            new TrapezoidProfile.Constraints(
                    16.0,
                    32.0
            )
    );
    double modeVoltage = 0.0;
    boolean modeZeroed = false;
    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
        inputs = new IntakeIOInputsAutoLogged();

        deployPID.setTolerance(
                0.0,
                0.0
        );
        deployPID.enableContinuousInput(0.0, PI * 2);

        modePID.setTolerance(
                0.0,
                0.0
        );
    }

    @Override
    public void periodic() {
        super.periodic();
        this.intakeIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);

        var deployVoltage = deployPID.calculate(
                inputs.deployEncoderPosition,
                0.0
        );

        var modeV = (modeZeroed) ? modePID.calculate(
                inputs.modeEncoderPosition,
                0.0
        ) : modeVoltage;

        intakeIO.setDeployVoltage(clamp(deployVoltage, -6.0, 6.0));
        intakeIO.setModeVoltage(clamp(modeV, -3.0, 3.0));
    }
}
