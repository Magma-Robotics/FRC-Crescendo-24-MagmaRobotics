package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class RaiseLift extends Command {
    private final Lift lift;
    
    public RaiseLift(Lift lift){
        this.lift = lift;
        addRequirements(lift);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       lift.raiseLift();
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
