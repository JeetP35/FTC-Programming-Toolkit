package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Improved Odometer Test", group = "Tests")
public class OdometerTest extends LinearOpMode {

    // Motor declarations
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor armMotor;
    private DcMotor extendMotor;
    private Servo clawOCMotor;
    private Servo clawUDMotor;
    private SparkFunOTOS otos;
    private ElapsedTime timer;
    
    
    
    // Encoder constants
    private static final double ENCODER_TICKS_PER_REVOLUTION = 384.5; // Encoder ticks per revolution (based on motor and gear ratio)
    private static final double DEGREES_PER_REVOLUTION = 360.0;

    // PID constants
    private static final double LINEAR_KP = 1.63;
    private static final double LINEAR_KI = 0.27;
    private static final double LINEAR_KD = 0.120;
    
    private static final double TURN_KP = 0.025;
    private static final double TURN_KI = 0.01;
    private static final double TURN_KD = 0.005;

    // State variables
    private double linearIntegral = 0;
    private double turnIntegral = 0;
    private double previousLinearError = 0;
    private double previousTurnError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        extendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");
        
        clawOCMotor = hardwareMap.get(Servo.class, "ClawOCMotor");
        clawUDMotor = hardwareMap.get(Servo.class, "ClawUDMotor");

        // Reverse motors if necessary based on your robot configuration
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        
    //ArmPositions
    armMotor.setTargetPosition(0);
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //ExtendPosition
    extendMotor.setTargetPosition(0);
    extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        initializeHardware();
        configureOdometrySystem();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example sequence
            // moveForward(-15.0);
            // stopMotors();// Move 24 inches forward
            
            //  turnToHeading(-70.0);// Turn to 90 degrees
            // moveRight(24);
            // moveForward(-35.0);
            // raiseArm(750.0);
            // holdArm(750.0);
            // RaiseArm.setPower(-0.01);
            
            // closeClaw();
            // holdArm(900.0);
            // closeClaw();
            
            // shuffleRight(0.75, 300);
             moveRight(-1);
             armMotor.setPower(-1.00);
             extendMotor.setPower(1.00);
             moveForward(10);
             sleep(1);
             armMotor.setTargetPosition(-2000);
             extendMotor.setTargetPosition(-100);
             sleep(1);
             sleep(100);
    
            
            
            
            
        }
    }

    private void initializeHardware() {
        // Initialize motors with proper directions
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        extendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");
        
        clawOCMotor = hardwareMap.get(Servo.class, "ClawOCMotor");
        clawUDMotor = hardwareMap.get(Servo.class, "ClawUDMotor");

        // Reverse motors if necessary based on your robot configuration
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize odometry system
        otos = hardwareMap.get(SparkFunOTOS.class, "imu");
        timer = new ElapsedTime();
    }

    private void configureOdometrySystem() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        // otos.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte)0x0D));
        otos.setOffset(new SparkFunOTOS.Pose2D(6.2992, -2.95276, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
    }

    private void moveForward(double targetInches) {
        resetPID();
        final double tolerance = 1.0; // inches
        SparkFunOTOS.Pose2D startPosition = otos.getPosition();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D currentPosition = otos.getPosition();
            double error = targetInches - currentPosition.x;
            
            // Calculate PID output
            double output = calculateLinearPID(error);
            
            // Apply power to motors
            setMotorPowers(output, output, output, output);
            
            // Display telemetry
            telemetry.addData("Target", "%.2f inches", targetInches);
            telemetry.addData("Current", "%.2f inches", currentPosition.y);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", output);
            telemetry.update();

            // Exit condition
            if (Math.abs(error) < tolerance) {
                stopMotors();
                break;
            }
        }
    }
        
    private double degreesToEncoderCounts(double degrees) {
        return (double) ((degrees / DEGREES_PER_REVOLUTION) * ENCODER_TICKS_PER_REVOLUTION);
    }
    
//    private double getArmPositionDegrees() {
        //return -(RaiseArm.getCurrentPosition() / ENCODER_TICKS_PER_REVOLUTION) * DEGREES_PER_REVOLUTION;
//    }
    
//    private void openClaw(){
      //  ClawOpen.setPosition(-1);
//    }
    
//    private void closeClaw(){
       // ClawOpen.setPosition(1);
//    }
        
    
    private void moveRight(double targetInches) {
        resetPID();
        final double tolerance = 1.0; // inches
        SparkFunOTOS.Pose2D startPosition = otos.getPosition();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D currentPosition = otos.getPosition();
            double error = targetInches - currentPosition.y;
            
            // Calculate PID output
            double output = calculateLinearPID(error);
            
            // Apply power to motors
            setMotorPowers(-output, output, output, -output);
            
            // Display telemetry
            telemetry.addData("Target", "%.2f inches", targetInches);
            telemetry.addData("Current", "%.2f inches", currentPosition.y);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", output);
            telemetry.addData("armMotorPosition", armMotor.getCurrentPosition());
            telemetry.update();

            // Exit condition
            if (error < tolerance) {
                // stopMotors();
                break;
            }
        }
    }

    private void turnToHeading(double targetHeading) {
        resetPID();
        final double tolerance = 2.0; // degrees

        while (opModeIsActive()) {
            double currentHeading = otos.getPosition().h;
            currentHeading = currentHeading % 360;
            if (currentHeading < 0) currentHeading += 360;
            
            targetHeading = targetHeading % 360;
            if (targetHeading < 0) targetHeading += 360;

            double error = calculateAngleError(targetHeading, currentHeading);
            // double error = Math.toDegrees(angleWrap(Math.toRadians(targetHeading- currentHeading)));
            // double error = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
            
            
            // Calculate PID output
            double output = calculateTurnPID(error);
            
            // Apply turning power
            setMotorPowers(-output, -output, output, output);
            
            
            // Display telemetry
            telemetry.addData("Target", "%.1f°", targetHeading);
            telemetry.addData("Current", "%.1f°", currentHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Power", "%.2f", output);
            telemetry.update();

            // Exit condition
            if (Math.abs(error) < tolerance) {
                stopMotors();
                break;
            }
        }
    }

    private double calculateLinearPID(double error) {
        double dt = timer.seconds();
        linearIntegral += error * dt;
        double derivative = (error - previousLinearError) / dt;
        
        // Anti-windup clamp
        linearIntegral = Math.max(-1.0, Math.min(1.0, linearIntegral));
        
        double output = (LINEAR_KP * error) + 
                        (LINEAR_KI * linearIntegral) + 
                        (LINEAR_KD * derivative);
        
        previousLinearError = error;
        timer.reset();
        return Math.max(-0.7, Math.min(0.7, output)); // Limit max power
    }

    private double calculateTurnPID(double error) {
        final double deadband = 4.0;
        if (Math.abs(error) < deadband) {
            return 0.0;
        }
        double dt = timer.seconds();
        turnIntegral += error * dt;
        double derivative = (error - previousTurnError) / dt;
        
        // Anti-windup clamp
        turnIntegral = Math.max(-0.5, Math.min(0.5, turnIntegral));
        
        double output = (TURN_KP * error) + 
                        (TURN_KI * turnIntegral) + 
                        (TURN_KD * derivative);
        
        previousTurnError = error;
        timer.reset();
        return Math.max(-0.5, Math.min(0.5, output)); // Limit max turn power
    }
    
    private double calculateAngleError(double target, double current){
        double error = target - current;
        
        // if (error > 180) {
        //     error -= 360;
        // } else if (error < 180) {
        //     error += 360;
        // }
        // error = (error + 180) % 360 - 180;
        return error;
    }
    
    
    private double angleWrap(double radians){
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    private void resetPID() {
        linearIntegral = 0;
        turnIntegral = 0;
        previousLinearError = 0;
        previousTurnError = 0;
        timer.reset();
    }

    private void setMotorPowers(double fl, double bl, double fr, double br) {
        FLMotor.setPower(fl);
        BLMotor.setPower(bl);
        FRMotor.setPower(fr);
        BRMotor.setPower(br);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
        private void shuffleRight(double power, int durationMs) {
        FLMotor.setPower(-power);
        BRMotor.setPower(power);
        FRMotor.setPower(-power);
        BLMotor.setPower(power);
        sleep(durationMs);
     }
}