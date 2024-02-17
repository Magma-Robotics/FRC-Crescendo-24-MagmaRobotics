package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoMovement extends Command {
    private final DriveTrain driveTrain;
    private double leftPower, rightPower, duration;

    public AutoMovement(DriveTrain driveTrain, double duration, double leftPower, double rightPower) {
        this.driveTrain = driveTrain;
        this.duration = duration;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        addRequirements(driveTrain);
    }
    
    @Override
    public void initialize() {
        double currentTime = System.currentTimeMillis() / 1000;
        duration = currentTime + duration;
    }

    @Override
    public void execute() {
        driveTrain.diffDrive(leftPower, rightPower);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }


    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() / 1000) >= duration;
    }
    
}
