package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void pullNote() {
        intakeMotor.set(0.5);
    }

    public void pushNote() {
        intakeMotor.set(-0.5);
    }

    
}
