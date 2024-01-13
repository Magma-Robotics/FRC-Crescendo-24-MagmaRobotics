package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase{

    private CANSparkMax leftFront = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightFront = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(0, MotorType.kBrushless);

    private DifferentialDrive diffDrive;

    public DriveTrain() {
        leftBack.follow(leftFront, true);
        rightBack.follow(rightFront, true);
        diffDrive = new DifferentialDrive(leftFront, rightFront);
    }

    public void stop() {
        diffDrive.stopMotor();
    }

    public void diffDrive(double leftJoystick, double rightJoystick) {
        diffDrive.tankDrive(-leftJoystick, rightJoystick);
    }


    

}
