package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

public class ZeroModeMotor extends CommandBase {

    Timer timer = new Timer();
    Intake intake;

    public ZeroModeMotor(Intake intake) {
        addRequirements(intake);
        this.intake = intake;
    }

    public void initialize() {
        timer.restart();
        intake.startModeMotorZeroing();
    }

    public void execute() {
        intake.setZeroModeVoltage(-3.0);
    }

    public void end(boolean interrupted) {
        intake.endModeMotorZeroing();
    }

    public boolean isFinished() {
        return timer.hasElapsed(1.0);
    }

}