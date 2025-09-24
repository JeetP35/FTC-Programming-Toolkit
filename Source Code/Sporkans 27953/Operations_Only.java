package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Operations_Only extends LinearOpMode{
    private DcMotor armMotor;
    private DcMotor extendMotor;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        extendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");

        while (opModeIsActive()) {
            //setting controls
            double armInput = -gamepad1.left_stick_y;
            double extendUpInput = gamepad1.left_trigger;
            double extendDownInput = gamepad1.right_trigger;

            //giving power
            double operationPower = 1.0;
            if (gamepad1.right_bumper) {
                operationPower = 0.25;
            } else if (gamepad1.left_bumper) {
                operationPower = 0.5;
            }

            //arm
            if (armInput > 0.1) {
                armMotor.setPower(1.00 * operationPower);
                armMotor.setTargetPosition(-1600); }
            else if (armInput < -0.1) {
                armMotor.setPower(1.00 * operationPower);
                armMotor.setTargetPosition(0); }
            else {
                armMotor.setPower(1.00);
                armMotor.setTargetPosition(armMotor.getCurrentPosition()); }

            //extend
            if (extendUpInput > 0.1) {
                extendMotor.setPower(1.00 * operationPower);
                extendMotor.setTargetPosition(-500); }
            else if (extendDownInput > 0.1) {
                extendMotor.setPower(1.00 * operationPower);
                extendMotor.setTargetPosition(0); }
            else {
                extendMotor.setPower(1.00);
                extendMotor.setTargetPosition(extendMotor.getCurrentPosition()); }

            //showcase
            telemetry.addData("power", operationPower);
            telemetry.addData("extension position", extendMotor.getCurrentPosition());
            telemetry.addData("arm position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
