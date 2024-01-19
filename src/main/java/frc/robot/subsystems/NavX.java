package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {

    private AHRS navx;

    public NavX() {
        try {
            this.navx = new AHRS(SerialPort.Port.kUSB1);
        } catch (Exception e) {
            System.out.println("NavX not connected.");
        }
    }


    public double getYaw() {
        return this.navx.getYaw();
    }

    public double getPitch() {
        return this.navx.getPitch();
    }

    public double getRoll() {
        return this.navx.getRoll();
    }

    public Rotation2d getRotation2d() {
        return this.navx.getRotation2d();
    }

    public void resetYaw() {
        this.navx.zeroYaw();
    }


    
}
