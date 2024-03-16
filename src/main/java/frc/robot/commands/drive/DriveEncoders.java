package frc.robot.commands.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveEncoders extends Command {
    private DriveTrain driveTrain;
    private double driveDistance;
    private double botSpeed;
    private double endDistance;

    public DriveEncoders(DriveTrain driveTrain, double speed, double userFeet) {
        driveTrain.resetEncoders();
        driveDistance = userFeet;
        botSpeed = speed;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
      endDistance = driveTrain.getLeftEncoderPos() + Units.feetToMeters(driveDistance);
    }
    @Override
    public void execute() {
      driveTrain.diffDrive(botSpeed, botSpeed);
      SmartDashboard.putNumber("Left Encoder Pos", driveTrain.getLeftEncoderPos());
    }

    @Override
    public boolean isFinished() {
      return (driveTrain.getLeftEncoderPos() >= endDistance);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
