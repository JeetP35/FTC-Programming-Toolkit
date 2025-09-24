package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Drive_Only extends LinearOpMode{

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

    @Override
    public void runOpMode() {
        flMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        frMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        blMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        brMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        while (opModeIsActive()) {
            double ForwardBack = -gamepad1.left_stick_y;
            double LeftRight = gamepad1.left_stick_x;
            double Rotate = -gamepad1.right_stick_x;

            //left bumper is half speed right bumper is 1/4
            double SpeedMultiplier = 1.0;
            if (gamepad1.left_bumper) {
                SpeedMultiplier = 0.5;
            } else if (gamepad1.right_bumper) {
                SpeedMultiplier = 0.25;
            }

            // Calculate power for each motor
            double FLP = (ForwardBack + LeftRight + Rotate) * SpeedMultiplier;
            double FRP = -(ForwardBack - LeftRight - Rotate) * SpeedMultiplier;
            double RLP = (ForwardBack - LeftRight + Rotate) * SpeedMultiplier;
            double RRP = -(ForwardBack + LeftRight - Rotate) * SpeedMultiplier;




            // Clip power to ensure it stays within -1.0 to 1.0
            FLP = Range.clip(FLP, -1.0, 1.0);
            FRP = Range.clip(FRP, -1.0, 1.0);
            RLP = Range.clip(RLP, -1.0, 1.0);
            RRP = Range.clip(RRP, -1.0, 1.0);
    


            // Set motor power
            flMotor.setPower(FLP);
            frMotor.setPower(FRP);
            blMotor.setPower(RLP);
            brMotor.setPower(RRP);


            // showcase
            telemetry.addData("FL Power", FLP);
            telemetry.addData("FR Power", FRP);
            telemetry.addData("BL Power", RLP);
            telemetry.addData("BR Power", RRP);
            telemetry.update();
        }
    }
}
