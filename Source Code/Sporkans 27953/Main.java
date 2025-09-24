package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Main extends LinearOpMode {

    private Drive drive;
    private operations operations;

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;

    private Servo ClawUDMotor;
    private Servo ClawOCMotor;
    
    private DcMotor ArmMotor;
    private DcMotor ExtendMotor;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        ClawUDMotor = hardwareMap.get(Servo.class, "ClawUDMotor");
        ClawOCMotor = hardwareMap.get(Servo.class, "ClawOCMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ExtendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");

        // Create instances of the classes
        drive = new Drive(FLMotor, FRMotor, BLMotor, BRMotor);
        operations = new operations(ArmMotor, ExtendMotor, ClawUDMotor, ClawOCMotor);

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad Drive inputs
            double ForwardBack = -gamepad1.left_stick_y;
            double LeftRight = gamepad1.left_stick_x;
            double Rotate = -gamepad1.right_stick_x;

            // Extension control inputs
            double extendUpInput = gamepad2.left_trigger;
            double extendDownInput = gamepad2.right_trigger;

            // Dead zone
            if (Math.abs(ForwardBack) < 0.1) ForwardBack = 0;
            if (Math.abs(LeftRight) < 0.1) LeftRight = 0;
            if (Math.abs(Rotate) < 0.1) Rotate = 0;

            // Speed multiplier(LeftBumber is half speed, RightBumber is 1/4 speed)
            double SpeedMultiplier = 1.0;
            if (gamepad1.left_bumper) {
                SpeedMultiplier = 0.5;
            } else if (gamepad1.right_bumper) {
                SpeedMultiplier = 0.25;
            }

            // Control driving
            // Calculate power for each motor
            double FLP = (ForwardBack + LeftRight + Rotate) * SpeedMultiplier;
            double FRP = -(ForwardBack - LeftRight - Rotate) * SpeedMultiplier;
            double RLP = (ForwardBack - LeftRight + Rotate) * SpeedMultiplier;
            double RRP = -(ForwardBack + LeftRight - Rotate) * SpeedMultiplier;
            drive.drive(ForwardBack, LeftRight, Rotate, SpeedMultiplier, FLP, FRP, RLP, RRP);

            // Control claw
            operations.moveClawUpDown(gamepad2.dpad_up, gamepad2.dpad_down);
            operations.openCloseClaw(gamepad2.dpad_left, gamepad2.dpad_right);

            // Control arm
            double armInput = -(gamepad2.left_stick_y);
            double armPosition = ArmMotor.getCurrentPosition();
            double armMinLimit = 1;
            double armMaxLimit = -1300;
            operations.moveArm(armInput, armPosition, armMinLimit, armMaxLimit);

            // Control extension
            double extendUpInput = gamepad2.right_trigger;
            double extendDownInput = gamepad2.left_trigger;
            double extendPosition = ExtendMotor.getCurrentPosition();
            double extendLimit = -2520;
            operations.moveExtend(extendUpInput, extendDownInput, extendPosition, extendLimit);

            if (armPosition > -1300) {
                extendLimit = -400;  // Adjust limit based on arm position
            }
            else {
                extendLimit = -2520;
            }

            //showcase
            telemetry.addData("FL Power", FLP);
            telemetry.addData("FR Power", FRP);
            telemetry.addData("BL Power", RLP);
            telemetry.addData("BR Power", RRP);
            telemetry.addData("Speed Multiplier", SpeedMultiplier);  // Show current multiplier
            telemetry.addData("ArmMotor Position", ArmMotor.getCurrentPosition());
            telemetry.addData("ExtendMotor Position", ExtendMotor.getCurrentPosition());
            telemetry.addData("IntakeMotor Position", IntakeMotor.getCurrentPosition());
            telemetry.addData("ArmMotor Power", ArmMotor.getPower());
            telemetry.addData("ExtendMotor Power", ExtendMotor.getPower());
            telemetry.addData("IntakeMotor Power", IntakeMotor.getPower());
            telemetry.addData("clawUpDown Position", ClawUDMotor.getPosition());
            telemetry.addData("clawOpenClose position",ClawOCMotor.getPosition());
            telemetry.update();
        }
    }
}
