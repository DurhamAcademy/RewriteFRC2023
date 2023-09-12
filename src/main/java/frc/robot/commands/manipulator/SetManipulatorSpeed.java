package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Manipulator;

public class SetManipulatorSpeed extends CommandBase {
    Manipulator manipulator;
    double speed;
    public SetManipulatorSpeed(Manipulator manipulator, double speed){
        this.manipulator = manipulator;
        this.speed = speed;
    }

    private void init() {
        addRequirements(manipulator);
    }

    public void execute() {
        manipulator.setMotorPercentage(speed);
    }
}
