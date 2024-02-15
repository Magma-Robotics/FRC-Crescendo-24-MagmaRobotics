package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainCommand extends Command {

    private final DriveTrain driveTrain;
    private final XboxController driveController;

    public DriveTrainCommand(DriveTrain driveTrain, XboxController driveController){
        this.driveTrain = driveTrain;
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
            driveController.getLeftY(), driveController.getRightY()
            );     
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}
