package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
    private CANSparkMax lift = new CANSparkMax(8, MotorType.kBrushless);

    public Lift() {
        lift.restoreFactoryDefaults();
        lift.setInverted(false);
        lift.burnFlash();
    }

    public void raiseLift() {
        lift.set(1);
    }
    public void lowerLift() {
        lift.set(-1);
    }
    public void stopLift() {
        lift.set(0);
    }
}
