package org.firstinspires.ftc.teamcode.FTC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "name")
public class Test extends LinearOpMode {

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor ArmMotor;
    private DcMotor ExtendMotor;
    private Servo clawOCMotor;
    private Servo clawUDMotor;
    // private SparkFunOTOS odoSensor;
    // private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ExtendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");
        clawOCMotor = hardwareMap.get(Servo.class, "ClawOCMotor");
        clawUDMotor = hardwareMap.get(Servo.class, "ClawUDMotor");
        
        //ReverseLeftSide
        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //ArmPositions
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ExtendPosition
        ExtendMotor.setTargetPosition(0);
        ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        //showcase
        telemetry.addData("FlMotor", FLMotor.getCurrentPosition());
        telemetry.addData("FRMotor", FRMotor.getCurrentPosition());
        telemetry.addData("BLMotor", BLMotor.getCurrentPosition());
        telemetry.addData("BRMotor", BRMotor.getCurrentPosition());
        telemetry.addData("arm Motor", ArmMotor.getCurrentPosition());
        telemetry.addData("extend Motor", ArmMotor.getCurrentPosition());
        telemetry.update();

        moveForwardTest(1.00, 10000);
    }

    public void moveForwardTest (double power,int time) {
        ArmMotor.setPower(-power);
        ArmMotor.setTargetPosition(-1000);
        ExtendMotor.setPower(-1.00);
        ExtendMotor.setTargetPosition(-50);
        clawUDMotor.setPosition(-1.00);
        sleep(time);
    }
}
