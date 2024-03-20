// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveEncoders;
import frc.robot.commands.drive.DriveTrainCommand;
import frc.robot.commands.drive.StopDriveMotors;
import frc.robot.commands.drive.TestMotorBackward;
import frc.robot.commands.drive.TestMotorForward;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;

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
  //private NavX navx = new NavX();
  //private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private CommandXboxController driverController, driverPartnerController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Register named commands
    List<Pair<String, Command>> namedCommands = new ArrayList<Pair<String, Command>>();
    namedCommands.add(new Pair<String,Command>("autoShootNote", new AutoShootNote(shooter, intake)));
    NamedCommands.registerCommands(namedCommands);

    //controllers
    driverController = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    driverPartnerController = new CommandXboxController(Constants.OperatorConstants.kDriverPartnerControllerPort);

    //default commands
    driveTrain.setDefaultCommand(new DriveTrainCommand(driveTrain, driverController));

    //autoChooser for pathplanner
    /*autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
*/
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
    driverPartnerController.rightBumper().onTrue(new PullNote(intake)).onFalse(new StopIntake(intake));
    driverPartnerController.leftBumper().onTrue(new PushNote(intake)).onFalse(new StopIntake(intake));
    driverPartnerController.x().onTrue(new ShootNote(shooter)).onFalse(new StopShooter(shooter));
    driverPartnerController.y().onTrue(new ReverseShootNote(shooter)).onFalse(new StopShooter(shooter));
    driverPartnerController.povUp().onTrue(new RaiseLift(lift)).onFalse(new StopLift(lift));
    driverPartnerController.povDown().onTrue(new LowerLift(lift)).onFalse(new StopLift(lift));

    driverController.povUp().onTrue(new TestMotorForward(driveTrain)).onFalse(new StopDriveMotors(driveTrain));
    driverController.povDown().onTrue(new TestMotorBackward(driveTrain)).onFalse(new StopDriveMotors(driveTrain));

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
    
    return new SequentialCommandGroup(
            new DriveEncoders(driveTrain, 0.3, 2)
        );
    //autoChooser.getSelected();
  }

  private void buildShuffleboard() {
    //buildDriveTestTab();
  }
  
  /*public void buildDriveTestTab() {
    ShuffleboardTab driveMMTab = Shuffleboard.getTab("Drive Testing");
    driveMMTab.add("kF", 0.1 )              .withPosition(0, 0).getEntry();
    driveMMTab.add("kP", 0.3 )              .withPosition(1, 0).getEntry();
    driveMMTab.add("kI", 0 )                .withPosition(2, 0).getEntry();
    driveMMTab.add("kD", 0 )                .withPosition(3, 0).getEntry();
    driveMMTab.add("Tgt. Inches", 0)        .withPosition(4, 0).getEntry();
    driveMMTab.add("Tgt. Degrees", 0)       .withPosition(5, 0).getEntry();
    driveMMTab.add("Finish Iterations", 5 ) .withPosition(6, 0).getEntry();

    // Result Values on row 2
    driveMMTab.add("Tgt. Ticks", 0)                                          .withPosition(0, 1);
    driveMMTab.addNumber("Left Encoder", driveTrain::getLeftEncoderPos)                   .withPosition(1, 1);
    driveMMTab.addNumber("Right Encoder", driveTrain::getRightEncoderPos)                 .withPosition(2, 1);
    driveMMTab.addNumber("Gyro Read", navx::getYaw)                                       .withPosition(3, 1);
    driveMMTab.add("Run Time", 0)                                            .withPosition(4, 1);
    driveMMTab.addNumber("Left SP", m_driveTrain::getLeftSetPoint).withPosition(5, 1).withSize(1, 1);
    driveMMTab.addNumber("Right SP", m_driveTrain::getRightSetPoint).withPosition(6, 1).withSize(1, 1);
   
    // Drive limiters on row 3
    driveMMTab.add("Forward Limiter", 2.5).withPosition(0, 2);
    driveMMTab.add("Rotation Limiter", 2.5).withPosition(1, 2);
    driveMMTab.add("Drive Max", .7).withPosition(2, 2);
    driveMMTab.add("Update Limits", new UpdateDriveLimiters(driveTrain)).withPosition(3, 2).withSize(2, 1);

    // Drive commands on row 4
    driveMMTab.add("Drive MM 100", new DriveMM(driveTrain, 100))        .withPosition(0, 3).withSize(2, 1);
    driveMMTab.add("Drive MM -100", new DriveMM(driveTrain, -100))      .withPosition(2, 3).withSize(2, 1);
    //driveMMTab.add("Drive MM Test", new DriveMMTest(driveTrain, 0))     .withPosition(4, 3).withSize(2, 1);

    // Turn commands on row 5
    driveMMTab.add("Turn MM 90", new TurnToAngle(driveTrain, 90))          .withPosition(0, 4).withSize(2, 1);
    driveMMTab.add("Turn MM -90", new TurnToAngle(driveTrain, -90))        .withPosition(2, 4).withSize(2, 1);
    driveMMTab.add("Turn MM Test", new TurnToAngleTest(driveTrain, 0))     .withPosition(4, 4).withSize(2, 1);
    
  }*/
}
