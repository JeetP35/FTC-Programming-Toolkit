package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Wrist_Testing extends LinearOpMode{

    private CRServo rightGearServo;
    private CRServo leftGearServo;
    private Servo clawOCServo;

    @Override
    public void runOpMode() {
        rightGearServo = hardwareMap.get(CRServo.class, "rightServo");
        leftGearServo = hardwareMap.get(CRServo.class, "leftServo");
        clawOCServo = hardwareMap.get(Servo.class, "ClawOCMotor");


        waitForStart();

        while (opModeIsActive()) {
        //Claw Up, Down, Left, Right

            //up
            if (gamepad1.dpad_up) {
                rightGearServo.setPower(1.00);
                leftGearServo.setPower(1.00); }
            //down
            else if (gamepad1.dpad_down) {
                rightGearServo.setPower(-1.00);
                leftGearServo.setPower(-1.00); }
            //right
            else if (gamepad1.dpad_right) {
                rightGearServo.setPower(-1.00);
                leftGearServo.setPower(1.00); }
            //left
            else if (gamepad1.dpad_left) {
                rightGearServo.setPower(1.00);
                leftGearServo.setPower(-1.00); }


        // claw open, close

            //open
            if (gamepad1.x) {
                clawOCServo.setPosition(-1.00); }
            //close
            if (gamepad1.b) {
                clawOCServo.setPosition(0); }


            //showcase
            telemetry.addData("clawRightGearPower", rightGearServo.getPower());
            telemetry.addData("clawLeftGearPower", leftGearServo.getPower());
            telemetry.addData("clawOpenClosePosition", clawOCServo.getPosition());
            telemetry.update();
        }
    }
}
