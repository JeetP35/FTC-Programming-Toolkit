package org.firstinspires.ftc.teamcode;
 
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 
 @Autonomous
 public class Park extends LinearOpMode {
    // Declare hardware variables
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor armMotor;

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
            shuffleRight(0.75, 12);
            driveBack(0.1, 500);
        }
    }

    // Initialize hardware
    private void initHardware() {
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        // Set motor behavior
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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