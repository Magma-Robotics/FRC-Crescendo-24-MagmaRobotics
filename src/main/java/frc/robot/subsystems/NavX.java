package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {

    private AHRS navx;

    public NavX() {
        try {
            navx = new AHRS()
            
        }
    }
    
}
