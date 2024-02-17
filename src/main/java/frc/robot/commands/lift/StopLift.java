package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class StopLift extends Command {
    private final Lift lift;
    
    public StopLift(Lift lift){
        this.lift = lift;
        addRequirements(lift);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       lift.stopLift();
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
