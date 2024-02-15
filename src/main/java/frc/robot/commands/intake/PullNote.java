package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class PullNote extends Command {
    private final Intake intake;
    
    public PullNote(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       intake.pullNote();
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
