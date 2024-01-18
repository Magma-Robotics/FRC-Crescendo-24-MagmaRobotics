package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase{

    private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);

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

    private final DifferentialDrivePoseEstimator m_PoseEstimator =
        new DifferentialDrivePoseEstimator(null, null, 0, 0, null);

    

}
