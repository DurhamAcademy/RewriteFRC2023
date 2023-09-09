// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithFlywheelAuto;
import frc.robot.commands.SpinAuto;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private final LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() throws IOException {
      var FRDriveMotorId = 10;
      var BLDriveMotorId = 11;
      var FLDriveMotorId = 12;
      var BRDriveMotorId = 13;

      var FRTurnMotorId = 14;
      var BLTurnMotorId = 15;
      var FLTurnMotorId = 16;
      var BRTurnMotorId = 17;

      var FRTurnEncoderId = 6;
      var BLTurnEncoderId = 7;
      var FLTurnEncoderId = 8;
      var BRTurnEncoderId = 9;

      var gyroID = 20;
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
          drive = new Drive(
                  new ModuleIOFalcon500(FRTurnMotorId, FRDriveMotorId, false, false, FRTurnEncoderId),
                  new ModuleIOFalcon500(BLTurnMotorId, BLDriveMotorId, false, false, BLTurnEncoderId),
                  new ModuleIOFalcon500(FLTurnMotorId, FLDriveMotorId, false, false, FLTurnEncoderId),
                  new ModuleIOFalcon500(BRTurnMotorId, BRDriveMotorId, false, false, BRTurnEncoderId),
                  new GyroIOReal(gyroID, "rio"),
                  new VisionIOPhoton());
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
          var gyroSim = new GyroIOSim(new Translation2d[]{new Translation2d(1.0, 1.0),
                  new Translation2d(1.0, -1.0),
                  new Translation2d(-1.0, 1.0),
                  new Translation2d(-1.0, -1.0)});
          drive = new Drive(new ModuleIOSim(0, gyroSim), new ModuleIOSim(1, gyroSim), new ModuleIOSim(2, gyroSim), new ModuleIOSim(3, gyroSim), gyroSim, new VisionIO() {
          });
        flywheel = new Flywheel(new FlywheelIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
          drive = new Drive(new ModuleIO() {
          }, new ModuleIO() {
          }, new ModuleIO() {
          }, new ModuleIO() {
          }, new GyroIO() {
          }, new VisionIO() {
          });
        flywheel = new Flywheel(new FlywheelIO() {
        });
        break;
    }

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Spin", new SpinAuto(drive));
    autoChooser.addOption("Drive With Flywheel", new DriveWithFlywheelAuto(drive, flywheel));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
            new RunCommand(() -> drive.driveArcade(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), true), drive));
    controller.a()
        .whileTrue(new StartEndCommand(() -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
