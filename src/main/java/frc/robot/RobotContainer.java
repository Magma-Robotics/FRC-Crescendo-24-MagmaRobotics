// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.drive.AutoMovement;
import frc.robot.commands.drive.DriveTrainCommand;
import frc.robot.commands.intake.PullNote;
import frc.robot.commands.intake.PushNote;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.lift.LowerLift;
import frc.robot.commands.lift.RaiseLift;
import frc.robot.commands.lift.StopLift;
import frc.robot.commands.shooter.AutoShootNote;
import frc.robot.commands.shooter.ReverseShootNote;
import frc.robot.commands.shooter.ShootNote;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveTrain driveTrain = new DriveTrain();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Lift lift = new Lift();
  private NavX navx = new NavX();
  private JoystickButton buttonA, buttonB, buttonX, buttonY, leftBumper, rightBumper, leftTrigger, rightTrigger;
  private POVButton upPOV, downPOV, leftPOV, rightPOV;
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private XboxController driverController, driverPartnerController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Register named commands
    List<Pair<String, Command>> namedCommands = new ArrayList<Pair<String, Command>>();
    namedCommands.add(new Pair<String,Command>("autoShootNote", new AutoShootNote(shooter)));
    NamedCommands.registerCommands(namedCommands);

    //controllers
    driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    driverPartnerController = new XboxController(Constants.OperatorConstants.kDriverPartnerControllerPort);


    //buttons
    this.buttonA = new JoystickButton(driverController, XboxController.Button.kA.value);
    this.buttonB = new JoystickButton(driverController, XboxController.Button.kB.value);
    this.buttonX = new JoystickButton(driverPartnerController, XboxController.Button.kX.value);
    this.buttonY = new JoystickButton(driverPartnerController, XboxController.Button.kY.value);
    this.leftBumper = new JoystickButton(driverPartnerController, XboxController.Button.kLeftBumper.value);
    this.rightBumper = new JoystickButton(driverPartnerController, XboxController.Button.kRightBumper.value);
    this.leftTrigger = new JoystickButton(driverPartnerController, XboxController.Axis.kLeftTrigger.value);
    this.rightTrigger = new JoystickButton(driverPartnerController, XboxController.Axis.kRightTrigger.value);
    this.upPOV = new POVButton(driverPartnerController, Constants.POVButton.kUP);
    this.downPOV = new POVButton(driverPartnerController, Constants.POVButton.kDOWN);
    this.leftPOV = new POVButton(driverController, Constants.POVButton.kLEFT);
    this.rightPOV = new POVButton(driverController, Constants.POVButton.kRIGHT);

    //default commands
    driveTrain.setDefaultCommand(new DriveTrainCommand(driveTrain, navx, driverController));

    //autoChooser for pathplanner
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    buildShuffleboard();
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
    rightBumper.onTrue(new PullNote(intake)).onFalse(new StopIntake(intake));
    leftBumper.onTrue(new PushNote(intake)).onFalse(new StopIntake(intake));
    buttonX.onTrue(new ShootNote(shooter)).onFalse(new StopShooter(shooter));
    buttonY.onTrue(new ReverseShootNote(shooter)).onFalse(new StopShooter(shooter));
    upPOV.onTrue(new RaiseLift(lift)).onFalse(new StopLift(lift));
    downPOV.onTrue(new LowerLift(lift)).onFalse(new StopLift(lift));

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
    return /*AutoMovement(driveTrain, 0, 0, 0);*/autoChooser.getSelected();
  }

  private void buildShuffleboard() {
    buildDriveTestTab();
  }
  
  public void buildDriveTestTab() {
    ShuffleboardTab driveMMTab = Shuffleboard.getTab("Drive Testing");
    driveMMTab.add("kF", 0.1 )              .withPosition(0, 0).getEntry();
    driveMMTab.add("kP", 0.3 )              .withPosition(1, 0).getEntry();
    driveMMTab.add("kI", 0 )                .withPosition(2, 0).getEntry();
    driveMMTab.add("kD", 0 )                .withPosition(3, 0).getEntry();
    driveMMTab.add("Tgt. Inches", 0)        .withPosition(4, 0).getEntry();
    driveMMTab.add("Tgt. Degrees", 0)       .withPosition(5, 0).getEntry();
    driveMMTab.add("Finish Iterations", 5 ) .withPosition(6, 0).getEntry();
  }
}
