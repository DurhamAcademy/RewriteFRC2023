package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final MechanismLigament2d mechanismDeployArm = new MechanismLigament2d("Deploy Arm", 0.6, 90.0);
    private final Mechanism2d mechanism2d = new Mechanism2d(5.0, 5.0);
    private final MechanismLigament2d mechanismModeArm = new MechanismLigament2d("Mode Arm", 0.2, 170.0);
    private final CommandXboxController controller = new CommandXboxController(0);
    private double modeVoltage = 0.0;
    private boolean modeZeroed = false;

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

        MechanismRoot2d mechanismRoot = mechanism2d.getRoot("Intake", 2.5, 2.5);
        mechanismRoot.append(mechanismDeployArm);
        mechanismDeployArm.append(mechanismModeArm);
    }

    public void setZeroModeVoltage(double voltage) {
        modeVoltage = voltage;
    }

    public void startModeMotorZeroing() {
        modeZeroed = false;
    }

    public void endModeMotorZeroing() {
        intakeIO.setModeEncoderPosition(0.0);
        modeZeroed = true;
        setZeroModeVoltage(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        this.intakeIO.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);

        var deployVoltage = deployPID.calculate(
                inputs.deployEncoderPosition,
                controller.getLeftX()
        );

        var modeV = (modeZeroed) ? modePID.calculate(
                inputs.modeEncoderPosition,
                controller.getLeftY() * 2 * PI
        ) : modeVoltage;

        intakeIO.setDeployVoltage(clamp(deployVoltage, -6.0, 6.0));
        intakeIO.setModeVoltage(clamp(modeV, -3.0, 3.0));

        mechanismModeArm.setAngle(Units.radiansToDegrees(inputs.modeEncoderPosition) - 160.0);
        mechanismDeployArm.setAngle(Units.radiansToDegrees(inputs.deployEncoderPosition) + 90.0);
        Logger.getInstance().recordOutput("Intake", mechanism2d);
    }

}
