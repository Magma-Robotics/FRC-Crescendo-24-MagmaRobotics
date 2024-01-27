// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.PullNote;
import frc.robot.commands.PushNote;
import frc.robot.commands.ReverseShootNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private DriveTrain driveTrain = new DriveTrain();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private JoystickButton buttonA, buttonB, buttonX, buttonY;
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private XboxController driverController, driverPartnerController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*//Register named commands
    NamedCommands.registerCommand("pullNote", new PullNote(intake));
    NamedCommands.registerCommand("stopIntake", new StopIntake(intake));*/

    //controllers
    driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    driverPartnerController = new XboxController(Constants.OperatorConstants.kDriverPartnerControllerPort);


    //buttons
    this.buttonA = new JoystickButton(driverController, Constants.Button.kA);
    this.buttonB = new JoystickButton(driverController, Constants.Button.kB);
    this.buttonX = new JoystickButton(driverController, Constants.Button.kX);
    this.buttonY = new JoystickButton(driverController, Constants.Button.kY);


    //default commands
    driveTrain.setDefaultCommand(new DriveTrainCommand(driveTrain, driverController));

    //autoChooser for pathplanner
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    buttonA.onTrue(new PullNote(intake)).onFalse(new StopIntake(intake));
    buttonB.onTrue(new PushNote(intake)).onFalse(new StopIntake(intake));
    buttonX.onTrue(new ShootNote(shooter)).onFalse(new StopShooter(shooter));
    buttonY.onTrue(new ReverseShootNote(shooter)).onFalse(new StopShooter(shooter));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
