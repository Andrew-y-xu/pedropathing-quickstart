package org.firstinspires.ftc.teamcode.common;

public class AutoShooter {
    private double distantTy;
    private double shooterPowerValue;
    private double hoodPositionValue;
    private boolean shooterStopped;
    double flywheelPID = 10; // PID for AutoShoot FlyWheel to Prevent jerking
    double anglePID = 0.05; // PID for AutoShoot Angle to Prevent jerking
    private double defaultTy = 0.5;
    private double shooterPowerMinLimit = 0.00;
    private double shooterPowerMaxLimit = 0.90;
    private double hoodPositionMinLimit = 0.0;
    private double hoodPositionMaxLimit = 1.0;
    private double calPowerValue = shooterPowerMinLimit;
    private double calHoodValue = hoodPositionMinLimit;

    //--- Class init
    public AutoShooter() {
        this.distantTy         = 0;
        this.shooterPowerValue = shooterPowerMinLimit;
        this.hoodPositionValue = hoodPositionMinLimit;
        this.shooterStopped    = true;
    }

    /* Note on Control Flow
      1. LimeLight ty (From Implementation)
             ↓
      2. Distance calculation (From Shooter Class)
             ↓
      3. Lookup / math model (From Shooter Class)
             ↓
      4. Target RPM + Target Hood Angle (From Shooter Class)
             ↓
      5. PID controllers (From Shooter Class)
             ↓
      6. Motor/Servo (Return values back to Implementation)
     */
    public void advancedMathematics(double distant) {
        this.distantTy = distant;

        //--- Only stopping FlyWheel, not stopping for Hood
        if(this.shooterStopped) { //--- Stopped Shooter returns all Zeros
            this.calPowerValue = this.shooterPowerMinLimit;
            this.calHoodValue = this.hoodPositionMinLimit;
        } else { //--- Run the FlyWheel for Auto

            //--- Do the math here <TBD>
            double powerAConstant = 0.0147369;
            double powerBConstant = 0.629345;

            double hoodPositionConstant = 0.92;

            // Top small Triangle with distant(ty) greater than 2.5
            if( this.distantTy >= 2.5) {
                this.calPowerValue = 0.66;
                this.calHoodValue = 0.99; // 0.2 or 0.9
            } else {
                // Field large Triangle with distant(ty) under than 2.5
                this.calPowerValue = (powerAConstant * this.distantTy) + powerBConstant;
                this.calHoodValue = hoodPositionConstant;
            }

            //---Example
            /*
            double error = targetRPM - currentRPM;
            flywheelIntegral += error * dt;
            double derivative = (error - lastFlywheelError) / dt;
            double power =
                    kP * error +
                    kI * flywheelIntegral +
                    kD * derivative;
            */
            //---End Example

            //--- Poor man's logic
            //--- No Math Test, temporary without Algorithm
            //    1. Data is based on data collected from 1/4
            //    2. Ensure Battery voltage is over 12.00 V
            /*
            if( this.distantTy >= 2.5) {
                this.calPowerValue = 0.70;
                this.calHoodValue = 0.99; // 0.2 or 0.9
            } else if ( this.distantTy >= -3.5 && this.distantTy < 2.5) {
                this.calPowerValue = 0.65;
                this.calHoodValue = 0.5;
            } else if ( this.distantTy >= -2.0 && this.distantTy < -3.5) {
                this.calPowerValue = 0.60;
                this.calHoodValue = 0.2;
            } else { //--- all under -2.0
                this.calPowerValue = 0.55;
                this.calHoodValue = 0.0;
            }
            */

            //shooterPowerValue = Range.clip(calPowerValue, shooterPowerMinLimit, shooterPowerMaxLimit); //Limit the result within range
            //hoodPositionValue = Range.clip(calHoodValue, hoodPositionMinLimit, hoodPositionMaxLimit); //Limit the result within range
            this.shooterPowerValue = this.calPowerValue;
            this.hoodPositionValue = this.calHoodValue;

        }
    }
    public double getFlywheelPower() {
        return this.shooterPowerValue;
    }
    public double getAnglePosition() {
        return this.hoodPositionValue;
    }
    public void setFlywheelPower(double power) {
        this.shooterPowerValue = power;
    }
    public void setAnglePosition(double position) {
        this.hoodPositionValue = position;
    }
    public void stopShooter() {
        this.shooterStopped = true;
        this.advancedMathematics(this.defaultTy);
    }
    public void startShooter() {
        this.shooterStopped = false;
        this.advancedMathematics(this.defaultTy); // default Ty to get started regardless of camera finding
    }
    public boolean isShooterStopped() {
        return this.shooterStopped;
    }
}