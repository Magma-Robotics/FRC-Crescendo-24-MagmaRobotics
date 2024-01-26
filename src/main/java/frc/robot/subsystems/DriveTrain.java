package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase{

    private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);

    private DifferentialDrive diffDrive;
    private NavX navx;

    public DriveTrain() {
        leftBack.follow(leftFront, true);
        rightBack.follow(rightFront, true);
        diffDrive = new DifferentialDrive(leftFront, rightFront);

        final AutoBuilder autoBuilder = new AutoBuilder().configureRamsete(
            m_PoseEstimator.getEstimatedPosition(), //Robot pose supplier
            resetPose(), //resets odometry
            wheelSpeeds, //wheelspeed supplier
            diffDrive, //drives robot
            new ReplanningConfig(), // default path replanning config
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, //if path is mirrored or not
            this // reference to this subsystem to set requirements
            );
        
    }

    public void stop() {
        diffDrive.stopMotor();
    }

    public void diffDrive(double leftJoystick, double rightJoystick) {
        diffDrive.tankDrive(-leftJoystick, rightJoystick);
    }

    private final DifferentialDriveKinematics m_kinematics = 
        new DifferentialDriveKinematics(0/*track width in meters*/);


    private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);

    private DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(null, null);

    private final DifferentialDrivePoseEstimator m_PoseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics, 
            navx.getRotation2d(),
            0, //encoders
            0, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


    public void resetPose() {
        navx.resetYaw();
        m_PoseEstimator.resetPosition(navx.getRotation2d(), wheelPositions, null);
    }

}
