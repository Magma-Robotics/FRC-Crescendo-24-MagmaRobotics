package frc.robot.commands.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveMM extends Command {
    DriveTrain driveTrain;
    Shuffleboard driveMMTab;
    double m_targetPosition;
    double m_targetTicks;
    int count;
    double m_start_time;
    double m_drive_kP, m_kI, m_kD, m_kF;
    NetworkTableEntry m_drivekPEntry, m_kIEntry, m_kDEntry, m_kFEntry, m_targetPosEntry, m_targetTicksEntry, m_iterationEntry, m_drivedurationEntry;
    //the number of times motion magic is on target before the command finishes
    int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
    public DriveMM(DriveTrain driveTrain, double targetInches) {
        this.driveTrain = driveTrain;
        m_targetPosition = targetInches;
        NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");

        m_drivekPEntry = driveTab.getEntry("kP");
        m_kIEntry = driveTab.getEntry("kI");
        m_kDEntry = driveTab.getEntry("kD");
        m_kFEntry = driveTab.getEntry("kF");
        m_iterationEntry = driveTab.getEntry("Finish Iterations");
        m_targetPosEntry = driveTab.getEntry("Tgt. Inches");
        m_targetTicksEntry = driveTab.getEntry("Tgt. Ticks");
        m_drivedurationEntry = driveTab.getEntry("Run Time");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }
    
}
