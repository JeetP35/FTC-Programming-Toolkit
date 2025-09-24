package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp

public class MotorTester extends LinearOpMode {
    Extension extension = new Extension();
//    Limelight lime = new Limelight();
    public DcMotor armLiftL = null;
    public DcMotor armLiftR = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor extend = null;

    private Servo claw = null;
    private Servo wrist = null;
    private Servo wristL = null;
    private Servo extendPitch = null;
    private Servo extendRoll = null;
    private Servo extendClaw = null;

    private SparkFunOTOS odometry;

    @Override
    public void runOpMode() {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

//        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
//        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        odometry = hardwareMap.get(SparkFunOTOS.class,"otos");
//        wristR = hardwareMap.get(Servo.class, "wristR");
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        claw = hardwareMap.get(Servo.class, "claw");
//        extend = hardwareMap.get(DcMotor.class, "extension");
        extendPitch = hardwareMap.get(Servo.class, "extendPitch");
        extendRoll = hardwareMap.get(Servo.class, "extendRoll");
        extendClaw = hardwareMap.get(Servo.class, "extendClaw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        extension.init(hardwareMap);
//        lime.init(hardwareMap);

        armLiftL = hardwareMap.get(DcMotor.class, "armLiftL");
        armLiftL.setTargetPosition(0);

        armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armLiftR = hardwareMap.get(DcMotor.class, "armLiftR");
        armLiftR.setTargetPosition(0);

        armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armLiftL.setDirection(DcMotor.Direction.FORWARD); // REVERSE
        armLiftR.setDirection(DcMotor.Direction.REVERSE); // FORWARD

        odometry.resetTracking();
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.calibrateImu();
        odometry.setLinearScalar(1.02); // 1.0115 drifted 1 inch left over 100inch movement
        odometry.setAngularScalar(1);
        odometry.setOffset(new SparkFunOTOS.Pose2D(0,0,0)); // 0.4375, 3.625
        odometry.begin();
        
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        amR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        amL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        amR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);
        
        
        waitForStart();
        while (opModeIsActive()) {

//            if (gamepad2.y)
//            {
//                extendRoll.setPosition(0);
////                extendPitch.setPosition(0);
////                extendClaw.setPosition(0);
//            }
//            else if (gamepad2.b)
//            {
//                extendRoll.setPosition(1);
////                extendPitch.setPosition(0.33);
////                extendClaw.setPosition(0.4);
//            }
//
//            if (gamepad1.a)
//            {
//                extension.MoveToPosition(-1200);
//            }
//            else if (gamepad1.b)
//            {
//                extension.MoveToPosition(-100);
//            }
//            else
//            {
//                extension.MoveToPosition(extension.GetExtensionPosition());
//            }

            if (gamepad2.a)
            {
                wrist.setPosition(1);
            }
            if (gamepad2.b)
            {
                wrist.setPosition(0);
            }
            extendPitch.setPosition(0.05);
            extendClaw.setPosition(0);
            extendRoll.setPosition(0);


            // 1.15 error
            telemetry.addData("odometryx", odometry.getPosition().x);
            telemetry.addData("odometryy", odometry.getPosition().y);
            telemetry.addData("odometryh", odometry.getPosition().h);
            telemetry.addData("armLiftL", armLiftL.getCurrentPosition());
            telemetry.addData("armLiftR", armLiftR.getCurrentPosition());
//            telemetry.addData("extension", extension.GetExtensionPosition());
//            telemetry.addData("extension output", extension.GetOutput());
//            telemetry.addData("limelight localization converted", lime.GetLimelightData(false,odometry.getPosition().h));
            telemetry.update();
        }
    }
}
