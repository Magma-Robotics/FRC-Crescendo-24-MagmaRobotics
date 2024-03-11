package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TestMotorBackward extends Command {

    private final DriveTrain driveTrain;

    public TestMotorBackward(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveTrain.testMotorBackward();
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}