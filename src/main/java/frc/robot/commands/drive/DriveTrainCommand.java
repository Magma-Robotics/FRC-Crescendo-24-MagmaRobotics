package frc.robot.commands.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

public class DriveTrainCommand extends Command {

    private final DriveTrain driveTrain;
    private final NavX navx;
    private final XboxController driveController;

    public DriveTrainCommand(DriveTrain driveTrain, NavX navx, XboxController driveController){
        this.driveTrain = driveTrain;
        this.navx = navx;
        this.driveController = driveController;
        driveController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
        addRequirements(driveTrain);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.driveTrain.diffDrive(
            driveController.getLeftY()*Constants.Drivetrain.DRIVETRAIN_SPEED, 
            driveController.getRightY()*Constants.Drivetrain.DRIVETRAIN_SPEED
            );

        SmartDashboard.putNumber("leftEncoderPos", driveTrain.getLeftEncoderPos());//*(-(Units.inchesToMeters(6) * Math.PI) / (8.46*42)));
        SmartDashboard.putNumber("leftEncoderVel", driveTrain.getLeftEncoderVel());//*(-(Units.inchesToMeters(6) * Math.PI) / (8.46)*42 * 60));
        SmartDashboard.putNumber("rightEncoderPos", driveTrain.getRightEncoderPos());//*(-(Units.inchesToMeters(6) * Math.PI) / (8.46*42)));
        SmartDashboard.putNumber("rightEncoderVel", driveTrain.getRightEncoderVel());//*(-(Units.inchesToMeters(6) * Math.PI) / (8.46*42 * 60)));
        SmartDashboard.putNumber("yaw", navx.getYaw());
        SmartDashboard.putNumber("CPR", driveTrain.leftDriveEncoder.getCountsPerRevolution());
        
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}
