package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootNote extends Command{
    private final Shooter shooter;
    private final Intake intake;

    public AutoShootNote(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.shootNote();
        new WaitCommand(3);
        intake.pullNote();
        new WaitCommand(3);
        intake.pushNote();
        new WaitCommand(2);
        intake.stop();
        shooter.stop();
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return true;
    }
    
}
