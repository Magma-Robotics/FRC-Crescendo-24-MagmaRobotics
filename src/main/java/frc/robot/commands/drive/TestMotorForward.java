package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TestMotorForward extends Command {

    private final DriveTrain driveTrain;

    public TestMotorForward(DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveTrain.testMotorForward();
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}