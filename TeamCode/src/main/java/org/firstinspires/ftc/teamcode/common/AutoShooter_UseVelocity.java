package org.firstinspires.ftc.teamcode.common;

public class AutoShooter_UseVelocity {
    private double distantTy;
    //private double shooterPowerValue;
    private double shooterVelocityValueRPM;

    private double hoodPositionValue;
    private boolean shooterStopped;
    double flywheelPID = 10; // PID for AutoShoot FlyWheel to Prevent jerking
    double anglePID = 0.05; // PID for AutoShoot Angle to Prevent jerking
    private double defaultTy = 0.5;
    //private double shooterPowerMinLimit = 0.00;

    //*** FlyWheel-1 Max speed 6000 RPM

    //GoBILDA 5202 motors (the common FTC ones) ALL use: 28 ticks/second
    static final double TICKS_PER_REV = 28.0;
    static final double MAX_RPM = 6000.0;

    private double shooterVelocityMinLimit = 0.0;
    //private double shooterPowerMaxLimit = 0.90;
    private double shooterVelocityMaxLimit = 0.90;
    private double hoodPositionMinLimit = 0.0;
    private double hoodPositionMaxLimit = 1.0;
    //private double calPowerValue = shooterPowerMinLimit;
    private double calVelocityValue = shooterVelocityMinLimit;

    private double calHoodValue = hoodPositionMinLimit;

    //--- Class init
    public AutoShooter_UseVelocity() {
        this.distantTy         = 0;
        this.shooterVelocityValueRPM = shooterVelocityMinLimit;
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
            //this.calPowerValue = this.shooterPowerMinLimit;
            this.calVelocityValue = this.shooterVelocityMinLimit;
            this.calHoodValue = this.hoodPositionMinLimit;
        } else { //--- Run the FlyWheel for Auto

            //--- Do the math here <TBD>
            double velocityAConstant = 0.0136112;
            double velocityBConstant = 0.588231;

            //always stay in range (0.30, 0.50)
            double hoodPositionConstant = 0.7;

            // Top small Triangle with distant(ty) greater than 2.5
            if( this.distantTy >= 2.5) {
                this.calVelocityValue = 0.65;
                this.calHoodValue = 0.8; // 0.2 or 0.9
            } else {
                // Field large Triangle with distant(ty) under than 2.5
                this.calVelocityValue = (velocityAConstant * this.distantTy) + velocityBConstant +0.02;
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

            //shooterPowerValue = Range.clip(calPowerValue, shooterPowerMinLimit, shooterPowerMaxLimit); //Limit the result within range
            //hoodPositionValue = Range.clip(calHoodValue, hoodPositionMinLimit, hoodPositionMaxLimit); //Limit the result within range
            this.shooterVelocityValueRPM = this.calVelocityValue;
            this.hoodPositionValue = this.calHoodValue;

        }
    }
    public double getFlywheelVelocity() {
        return this.shooterVelocityValueRPM;
    }
    public double getAnglePosition() {
        return this.hoodPositionValue;
    }
    public void setFlywheelVelocity(double velocity) {
        this.shooterVelocityValueRPM = velocity;
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