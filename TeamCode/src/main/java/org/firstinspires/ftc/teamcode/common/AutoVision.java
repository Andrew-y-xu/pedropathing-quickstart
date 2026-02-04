package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class AutoVision {

    private Limelight3A limelight;
    LLResult limelightResult;
    double limelight_tx = 0;
    double limelight_ty = 0;
    boolean aprilTagFound = false;

    public AutoVision(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void update() {
        limelightResult = this.limelight.getLatestResult();
        boolean aprilTagFound = false;

        if (limelightResult != null && limelightResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults2 = limelightResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults2) {
                this.limelight_tx = fr.getTargetXDegrees();
                this.limelight_ty = fr.getTargetYDegrees();
            }
        }
    }


    public double getLimelight_Tx() {
        return this.limelight_tx;
    }
    public double getLimelight_Ty() {
        return this.limelight_ty;
    }
    public boolean isAprilTagFound() {
        return this.aprilTagFound;
    }

    /*
    if (!doesiseeitfoundboi) {
        turretmotor.setPower(0);
        limelightMessage = "No data available";
    } else {
        limelightMessage = "Data available";
        flywheel.setPower(0.5); //Default running speed
    }
    */

    // --- End Limelight


}
