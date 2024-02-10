package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase{
    private NavX navx;

    private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftFront, rightFront);

    private RelativeEncoder leftEncoder = leftFront.getEncoder();
    private RelativeEncoder rightEncoder = rightFront.getEncoder();

    private final double wheelDiameter = Constants.Drivetrain.WHEEL_DIAMETER_IN_METERS;
    private final double trackWidth = Constants.Drivetrain.TRACK_WIDTH_IN_METERS;

    private Field2d field = new Field2d();

    public DriveTrain() {
        leftFront.setInverted(false);
        rightFront.setInverted(false);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        
        //records encoders in terms of meters, 8.46:1 gear ratio
        leftEncoder.setPositionConversionFactor((wheelDiameter * Math.PI) / (8.46));
        rightEncoder.setPositionConversionFactor((wheelDiameter * Math.PI) / (8.46));

        leftEncoder.setVelocityConversionFactor((wheelDiameter * Math.PI) / (8.46 * 60));
        rightEncoder.setVelocityConversionFactor((wheelDiameter * Math.PI) / (8.46 * 60));

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

        AutoBuilder.configureLTV(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            (chassisSpeeds) -> {
                    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
                    diffDrive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
                }, 
            0.02, 
            new ReplanningConfig(), 
            () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
            this);

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    public LTVDifferentialDriveController ltvController = new LTVDifferentialDriveController(null, trackWidth, null, null, trackWidth);

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
    
    private DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition()); //encoder ticks in meters
   
    private final DifferentialDriveKinematics m_kinematics = 
        new DifferentialDriveKinematics(trackWidth); //track width
    
    private final DifferentialDrivePoseEstimator m_PoseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics, //track width
            getRotation2d(),
            wheelPositions.leftMeters, //encoders
            wheelPositions.rightMeters, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    public ChassisSpeeds getSpeeds() {
        return chassisSpeeds;
    }
    
    private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity()); //rpm in m/s

    private ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }
}
