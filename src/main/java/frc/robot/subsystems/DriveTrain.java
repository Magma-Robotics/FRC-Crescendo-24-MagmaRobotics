package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase{
    //private AHRS navx = new AHRS(SPI.Port.kMXP);
    //private final DifferentialDriveOdometry m_odometry;

    private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftFront::set, rightFront::set);

    public RelativeEncoder leftDriveEncoder = leftFront.getEncoder();
    private RelativeEncoder rightDriveEncoder = rightFront.getEncoder();

    private DifferentialDriveKinematics m_kinematics = Constants.Drivetrain.kDriveKinematics;

    private PIDController leftWheelsController = new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0); 
    private PIDController rightWheelsController = new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0);

    private SimpleMotorFeedforward leftWheelFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts, Constants.Drivetrain.ksVoltSecondsPerMeter, Constants.Drivetrain.kaVoltSecondsSquaredPerMeter);
    private SimpleMotorFeedforward rightWheelFeedforward = new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts, Constants.Drivetrain.ksVoltSecondsPerMeter, Constants.Drivetrain.kaVoltSecondsSquaredPerMeter);

    private Field2d field = new Field2d();

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0)); 
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
            
    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            leftFront.setVoltage(volts.in(Volts));
            rightFront.setVoltage(volts.in(Volts));
        }, 
        log -> {
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftFront.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftDriveEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftDriveEncoder.getVelocity(), MetersPerSecond));
            log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightFront.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightDriveEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightDriveEncoder.getVelocity(), MetersPerSecond));
              },
        this));


    public DriveTrain() {
        leftFront.restoreFactoryDefaults();
        leftBack.restoreFactoryDefaults();
        rightFront.restoreFactoryDefaults();
        rightBack.restoreFactoryDefaults();

        leftFront.setInverted(false);
        rightFront.setInverted(true);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);


        //records encoders in terms of meters, 8.46:1 gear ratio
        leftDriveEncoder.setPositionConversionFactor(Constants.Drivetrain.kLinearDistanceConversionFactor);
        rightDriveEncoder.setPositionConversionFactor(Constants.Drivetrain.kLinearDistanceConversionFactor);

        leftDriveEncoder.setVelocityConversionFactor(Constants.Drivetrain.kLinearDistanceConversionFactor / 60);
        rightDriveEncoder.setVelocityConversionFactor(Constants.Drivetrain.kLinearDistanceConversionFactor / 60);

        /*leftDriveEncoder.setPositionConversionFactor((wheelDiameter * Math.PI) / (8.46));
        rightDriveEncoder.setPositionConversionFactor((wheelDiameter * Math.PI) / (8.46));

        leftDriveEncoder.setVelocityConversionFactor((wheelDiameter * Math.PI) / (8.46 * 60));
        rightDriveEncoder.setVelocityConversionFactor((wheelDiameter * Math.PI) / (8.46 * 60));*/

        leftFront.burnFlash();
        leftBack.burnFlash();
        rightFront.burnFlash();
        rightBack.burnFlash();
 
        /*AutoBuilder.configureRamsete(
            this::getPose, //Robot pose supplier
            this::resetPose, //resets odometry
            this::getSpeeds, //chassisspeeds supplier
            (chassisSpeeds) -> {
                DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
                diffDrive.tankDrive(leftWheelsController.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond), 
                    rightWheelsController.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond));
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
            );*/

        /*AutoBuilder.configureLTV(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            (chassisSpeeds) -> {
                driveConsumer(chassisSpeeds);
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

        navx.reset();
        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftDriveEncoder.getPosition(), rightDriveEncoder.getPosition());
        m_PoseEstimator.resetPosition(null, wheelPositions, getPose());*/
    }

    //public LTVDifferentialDriveController ltvController = new LTVDifferentialDriveController(null, trackWidth, null, null, trackWidth);

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public void stop() {
        diffDrive.stopMotor();
    }

    public void testMotorForward() {
        leftFront.set(0);
        rightFront.set(0);
    }

    public void testMotorBackward() {
        leftFront.set(-0);
        rightFront.set(-0);
    }

    public void stopDriveMotors() {
        leftFront.stopMotor();
        rightFront.stopMotor();
    }

    public void diffDriveJoystick(double leftJoystick, double rightJoystick) {
        diffDrive.tankDrive(-leftJoystick, -rightJoystick);
    }

    public void diffDrive(double leftPower, double rightPower) {
        diffDrive.tankDrive(leftPower, rightPower);
    }


    public void driveConsumer(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
        leftFront.setVoltage(leftWheelFeedforward.calculate(wheelSpeeds.leftMetersPerSecond)
            + leftWheelsController.calculate(leftDriveEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond));
        rightFront.setVoltage(rightWheelFeedforward.calculate(wheelSpeeds.rightMetersPerSecond)
            + rightWheelsController.calculate(rightDriveEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond));
    }

    public void resetEncoders() {
        leftDriveEncoder.setPosition(0);
        rightDriveEncoder.setPosition(0);
    }
 
    /*public Pose2d getPose() {
        return m_PoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_PoseEstimator.resetPosition(navx.getRotation2d(), wheelPositions, pose);
    }
    
    private DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(leftDriveEncoder.getPosition(), rightDriveEncoder.getPosition()); //encoder ticks in meters
   
    
    private final DifferentialDrivePoseEstimator m_PoseEstimator =
        new DifferentialDrivePoseEstimator(
            m_kinematics, //track width
            navx.getRotation2d(),
            wheelPositions.leftMeters, //encoders
            wheelPositions.rightMeters, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    public ChassisSpeeds getSpeeds() {
        return chassisSpeeds;
    }
    
    private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftDriveEncoder.getVelocity(), rightDriveEncoder.getVelocity()); //rpm in m/s

    private ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);*/

    public double getLeftEncoderPos() {
        return leftDriveEncoder.getPosition();
    }
    public double getLeftEncoderVel() {
        return leftDriveEncoder.getVelocity();
    }
    public double getRightEncoderPos() {
        return rightDriveEncoder.getPosition();
    }
    public double getRightEncoderVel() {
        return rightDriveEncoder.getVelocity();
    }

    /*@Override
    public void periodic() {
        m_odometry.update(navx.getRotation2d(), leftDriveEncoder.getPosition(), rightDriveEncoder.getPosition());
    }*/
}
