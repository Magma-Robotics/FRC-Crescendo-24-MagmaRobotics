package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {

    private AHRS navx;

    public NavX() {
        try {
            this.navx = new AHRS(SPI.Port.kMXP);
        } catch (Exception e) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + e.getMessage(), true);
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
