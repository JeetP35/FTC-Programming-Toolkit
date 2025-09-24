 package org.firstinspires.ftc.teamcode;
 
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 
 @Autonomous
 public class Auto extends LinearOpMode {
    // Declare hardware variables
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor armMotor;

    // Constants for arm control
    private static final double ENCODER_TICKS_PER_REVOLUTION = 384.5; // Encoder ticks per revolution (based on motor and gear ratio)
    private static final double DEGREES_PER_REVOLUTION = 504.0;
;
    private static double kp = 0.005;
    private static double maxPower = 0.5;
    private static double minPower = 0.1;
      
    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Display status on the Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (Driver presses PLAY)
        waitForStart();

        // Code to run during Autonomous period
        if (opModeIsActive()) {
            // shuffleLeft(0.75, 1400);
            // driveForward(0.5, 700);
            setArmPosition(40);
            
        }
    }

    // Initialize hardware
    private void initHardware() {
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Set motor behavior
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset and initialize the arm motor encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move the arm to a specific degree
    private void setArmPosition(double degrees) {
        
        // 
        int targetPosition = degreesToEncoderCounts(degrees);
        // int currentPosition = armMotor.getCurrentPosition();
        // int error = targetPosition - currentPosition;
        
        // double power = kP * error;
        // power = Math.max(-maxPower, Math.min(maxPower, power));
        
        
        // if (Math.abs(power) < minPower) {
        //     power = Math.signum(power) * minPower;
        // }
        
        // armMotor.setPower(power);
        
        // if (Math.abs(error) < 10) {
        //     // Threshold in encoder counts
        //     break;
        // }
        
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);

        // Wait until the arm motor reaches the target position
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("Target Arm Position (degrees)", degrees);
            telemetry.addData("Current Arm Position (degrees)", getArmPositionDegrees());
            telemetry.update();
        }

        // Stop the motor and hold the position
        // armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // armMotor.setPower(0.1);
    }

    // Calculate encoder counts from degrees
    private int degreesToEncoderCounts(double degrees) {
        return (int) ((degrees / DEGREES_PER_REVOLUTION) * ENCODER_TICKS_PER_REVOLUTION);
    }

    // Get the current arm position in degrees
    private double getArmPositionDegrees() {
        return (armMotor.getCurrentPosition() / ENCODER_TICKS_PER_REVOLUTION) * DEGREES_PER_REVOLUTION;
    }

    // Drive forward
    private void driveForward(double power, int durationMs) {
        FLMotor.setPower(-power);
        FRMotor.setPower(power);
        BRMotor.setPower(power);
        BLMotor.setPower(-power);
        sleep(durationMs);
    }

    // Reverse
    private void driveBack(double power, int durationMs) {
        FLMotor.setPower(power);
        FRMotor.setPower(-power);
        BRMotor.setPower(-power);
        BLMotor.setPower(power);
        sleep(durationMs);
    }

    // Turn left
    private void shuffleLeft(double power, int durationMs) {
        FLMotor.setPower(power);
        BRMotor.setPower(-power);
        FRMotor.setPower(power);
        BLMotor.setPower(-power);
        sleep(durationMs);
    }
    
    // Turn Right
    private void shuffleRight(double power, int durationMs) {
        FLMotor.setPower(-power);
        BRMotor.setPower(power);
        FRMotor.setPower(-power);
        BLMotor.setPower(power);
        sleep(durationMs);
     }

    // Stop all motors
    private void stopMotors() {
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }
}