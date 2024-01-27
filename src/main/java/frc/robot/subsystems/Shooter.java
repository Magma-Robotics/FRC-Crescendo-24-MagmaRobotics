package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private CANSparkMax leftShooterMotor = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax rightShooterMotor = new CANSparkMax(7, MotorType.kBrushless);

    public void stop() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public void shootNote() {
        leftShooterMotor.set(1);
        rightShooterMotor.set(-1);
    }

    public void reverseShootNote() {
        leftShooterMotor.set(-1);
        rightShooterMotor.set(1);
    }


}
