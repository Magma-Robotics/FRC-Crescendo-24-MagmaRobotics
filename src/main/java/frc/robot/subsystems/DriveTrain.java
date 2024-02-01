package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase{
    private NavX navx;

    private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftFront, rightFront);

    private RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
    private RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
    private RelativeEncoder leftBackEncoder = leftBack.getEncoder();
    private RelativeEncoder rightBackEncoder = rightBack.getEncoder();

    public DriveTrain() {
        leftFront.setInverted(false);
        rightFront.setInverted(false);

        leftBack.follow(leftFront, true);
        rightBack.follow(rightFront, true);

        leftFront.burnFlash();
        leftBack.burnFlash();
        rightFront.burnFlash();
        rightBack.burnFlash();

        AutoBuilder.configureRamsete(
            this::getPose, //Robot pose supplier
            this::resetPose, //resets odometry
            this::getSpeeds, //chassisspeeds supplier
            (chassisSpeeds) -> {
                DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
                diffDrive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
            }, //drives robot
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

    public Pose2d getPose() {
        return m_PoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_PoseEstimator.resetPosition(getRotation2d(), wheelPositions, pose);
    }

    public ChassisSpeeds getSpeeds() {
        return chassisSpeeds;
    }

    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    
    private final DifferentialDriveKinematics m_kinematics = 
        new DifferentialDriveKinematics(22.5/39.37); //22.5 in to meters

    private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);

    private ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

    private DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(0, );

    private final DifferentialDrivePoseEstimator m_PoseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics, //track width
            getRotation2d(),
            0, //encoders
            0, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    

}
