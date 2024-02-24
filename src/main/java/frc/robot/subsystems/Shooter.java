package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    private CANSparkMax leftShooterMotor = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax rightShooterMotor = new CANSparkMax(10, MotorType.kBrushless);

    public void stop() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public void shootNote() {
        leftShooterMotor.set(-Constants.Shooter.SHOOTER_SPEED);
        rightShooterMotor.set(Constants.Shooter.SHOOTER_SPEED);
    }

    public void reverseShootNote() {
        leftShooterMotor.set(Constants.Shooter.SHOOTER_SPEED);
        rightShooterMotor.set(-Constants.Shooter.SHOOTER_SPEED);
    }

    public void autoShootNote() {
        
    }

}
