
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain{
    private double speedScalar = 1; // used to control speed with bumpers
    
    //motor references
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    
    public boolean moving = true;
    
    public void init(HardwareMap hwMap)
    {
        // intialize motors
        frontLeft = hwMap.get(DcMotor.class, "FrontLeft");
        frontRight = hwMap.get(DcMotor.class, "FrontRight");
        backLeft = hwMap.get(DcMotor.class, "BackLeft");
        backRight = hwMap.get(DcMotor.class, "BackRight");

        //set their run modes to run without encoder, meaning they will not be using setTargetPos
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
    
    private double angleWrap(double rad)
    {
        while (rad > Math.PI)
        {
            rad -= 2 * Math.PI;
        }
        while (rad < -Math.PI)
        {
            rad += 2 * Math.PI;
        }
        return rad;
    }

    public void FieldOrientedTranslate(double targetPowerX, double targetPowerY, double rotation, double currentRotation)
    {
        double yaw = Math.toDegrees(angleWrap(Math.toRadians(currentRotation)));
        
        double stickRotation = 0;
        if (targetPowerY > 0 && targetPowerX < 0) //quad2
        {
            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI/2) * 180/Math.PI;   
        }
        else if (targetPowerY < 0 && targetPowerX < 0) //quad3
        {
            stickRotation = (Math.atan2(Math.abs(targetPowerY),Math.abs(targetPowerX)) + Math.PI) * 180/Math.PI;
        }
        else //quad1 and quad4
        {
            stickRotation = Math.atan2(targetPowerY,targetPowerX) * 180/Math.PI;
        }
        
        // angle of imu yaw supplemented by the stick's rotation, determined by atan
        double theta = (360-yaw) + stickRotation;
        double power = Math.hypot(targetPowerX,targetPowerY); //get hypotenuse of x and y tgt, getting the power
        
        // if at max power diag, limit to magnitude of 1
        // with the normalizing code, the diag movement had a bug where max power (being magnitude sqrt(2))-- 
        // --would cause wheels to flip polarity
        // to counteract this the power is limited to a proper magnitude
        if (power > 1)
        {
            power = 1;
        }
        else if (power < -1) 
        {
            power = -1;
        }
        
        //get the sin and cos of theta
        //math.pi/4 represents 45 degrees, accounting for angular offset of mechanum
        double sin = Math.sin((theta * (Math.PI/180)) - (Math.PI/4));
        double cos = Math.cos((theta * (Math.PI/180)) - (Math.PI/4));
        //max of sin and cos, used to normalize the values for maximum efficiency
        double maxSinCos = Math.max(Math.abs(sin),Math.abs(cos));
    
        //same sign flip is to account for the inability of atan2, it typically only works for quadrants 1 and 4
        //by flipping the polarity when x < 0, we can use atan for every quadrant
        double flPower = 0;
        double frPower = 0;
        double blPower = 0;
        double brPower = 0;

        rotation *= -1;

        flPower = power * cos/maxSinCos+rotation;
        frPower = power * sin/maxSinCos-rotation;
        blPower = power * sin/maxSinCos+rotation;
        brPower = power * cos/maxSinCos-rotation;
        
        double frontMax = Math.max(Math.abs(flPower),Math.abs(frPower));
        double backMax = Math.max(Math.abs(blPower),Math.abs(brPower));
        
        //another normalization
        if ((power + Math.abs(rotation)) > 1)
        {
            flPower /= power + Math.abs(rotation);
            frPower /= power - Math.abs(rotation);
            blPower /= power + Math.abs(rotation);
            brPower /= power - Math.abs(rotation);
        }
        
        //set speed to calculated values * the speedScalar(determined by bumpers)
        if(moving)
        {
            if (frontLeft != null && frontRight != null && backLeft != null && backRight != null)
            {
                frontLeft.setPower(flPower * speedScalar);
                frontRight.setPower(frPower * speedScalar);
                backLeft.setPower(blPower * speedScalar);
                backRight.setPower(brPower * speedScalar);
            }
        }
    }
    
    public void Translate(double targetPowerX, double targetPowerY, double rotation)
    {
        // inputs target power on x and y, outputs proper power distribution
        
        double theta = Math.atan2(targetPowerY, targetPowerX);
        double power = Math.hypot(targetPowerX, targetPowerY);
        
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        rotation *= -1;

        double flPower = power * cos/max + rotation;
        double frPower = power * sin/max - rotation;
        double blPower = power * sin/max + rotation;
        double brPower = power * cos/max - rotation;

        if ((power + Math.abs(rotation)) > 1)
        {
            flPower /= power + Math.abs(rotation);
            frPower /= power - Math.abs(rotation);
            blPower /= power + Math.abs(rotation);
            brPower /= power - Math.abs(rotation);
        }
        
        //sets power of each motor
        if (frontLeft != null && frontRight != null && backLeft != null && backRight != null)
        {
            frontLeft.setPower(flPower * speedScalar);
            frontRight.setPower(frPower * speedScalar);
            backLeft.setPower(blPower * speedScalar);
            backRight.setPower(brPower * speedScalar);
        }
    }
    
    public void SetSpeedScalar(double change)
    {
        //sets the speed scalar of the motors to a specified value in starter bot
        speedScalar = change;
    }
}
