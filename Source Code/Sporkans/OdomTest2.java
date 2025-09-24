package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class OdomTest2 extends LinearOpMode{

    // todo: write your code here
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private SparkFunOTOS otos;
    private ElapsedTime time;
    
    public double kp = 1.5;
    public double kd = 0.1;
    public double kl = 0.25;
    public double integral, previous, output = 0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        
        otos = hardwareMap.get(SparkFunOTOS.class, "imu");
        time = new ElapsedTime();

        // Reset OTOS odometer and heading
        configureOtos();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward 24 inches
            moveForward(24);

            // Turn 90 degrees clockwise
            // turnByAngle(90);
        }
    }
    
    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(6.2992, -2.95276,0);
        otos.setOffset(offset);
        
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        
        otos.calibrateImu();
        otos.resetTracking();
    }
    
    private void moveForward(double inches) {
    // Reset OTOS odometer
        // otos.resetTracking();
        while (opModeIsActive()){
            SparkFunOTOS.Pose2D position = otos.getPosition();
            double[] errors = getError(0.0, inches, 0.0);
            double YError = pid(errors[1]);
            double XError = pid(errors[0]);
            double[] normalizedErrors = normalizeError(XError, YError);
            double power = normalizedErrors[1];
            setMotorPowers(power, power, -power, -power);
        }

        // while (opModeIsActive()) {
        //     SparkFunOTOS.Pose2D position = otos.getPosition();
        //     double distanceTraveled = position.y; // Positive Y-axis corresponds to forward movement

        //     if (distanceTraveled >= inches) {
        //         stopMotors();
        //         break;
        //     }

        //     // Set power to move forward
        //     setMotorPowers(0.5, 0.5, -0.5, -0.5);
        // }
    }

    private void turnByAngle(double targetAngle) {
        // Reset OTOS heading
        otos.resetTracking();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D position = otos.getPosition();
            double currentHeading = position.h; // Heading in degrees

            if (Math.abs(targetAngle - currentHeading) <= 1.0) { // Allow a margin of error
                stopMotors();
                break;
            }

            // Set power for clockwise turn
            setMotorPowers(-0.5, -0.5, -0.5, -0.5);
        }
    }
    
    private double[] getError(double targetX, double targetY, double targetH){
        double[] errors = new double[3];
        SparkFunOTOS.Pose2D position = otos.getPosition();
        
        errors[0] = targetX - position.x;
        errors[1] = targetY - position.y;
        errors[2] = targetH - position.h;
        
        return errors;
    }
    
    private double[] normalizeError(double errorX, double errorY){
        double biggestError = Math.max(errorX, errorY);
        double newX = 0;
        double newY = 0;
        if (biggestError > 1.0){
            newX = errorX/biggestError;
            newY = errorY/biggestError;
        } else {
            newX = errorX;
            newY = errorY;
        }
        
        double[] errors = {newX, newY};
        return errors;
    }
    
    private double pid(double error){
        double dt = time.seconds();
        double proportional = error;
        integral += error * dt;
        double derivative = (error - previous)/ dt;
        previous = error;
        double output = (kp * proportional) + (kl * integral) + (kd* derivative);
        time.reset();
        return output;
    }

    private void setMotorPowers(double lf, double lb, double rf, double rb) {
        FLMotor.setPower(lf);
        BLMotor.setPower(lb);
        FRMotor.setPower(rf);
        BRMotor.setPower(rb);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
    
}
