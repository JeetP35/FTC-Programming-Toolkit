package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Config
@Autonomous
public class CommandSequence extends LinearOpMode
{
    MoveCommand moveCmd = new MoveCommand();
    Drivetrain dt = new Drivetrain();
    SharkDrive shark = new SharkDrive();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();

    boolean moving = true;
    double speed = 0.75; // 0.7

    double grabDistance = 0; // old 4.5
    double hookDistance = 25; // 22.5
    int grabHeight = 60;
    int lowMoveHeight = grabHeight + 200;
    int hookHeight = 1100;

    double stopLenience = 0.6;
    double preciseLenience = 2; // 0.6
    double arcLenience = 10;

    double circleXOffset = 34 + 2;
    double circleYOffset = 30;
    double robotHalfLength = 7.5;

    int pushExtendDist = -1421;

    double pitchUpPos = 0.05;
    double pitchDownPos = 0.63;

    double xAutoOffset = 0;

    double yRehomeConstant = -2.5; // 0 on the competition field

    // push circle radius = 25.75 inches

    private void ContinueMovement()
    {
        moving = true;
    }

    private void Stop()
    {
        dt.FieldOrientedTranslate(0,0,0,0);
        moving = false;
    }

    private void SpecimenSequence()
    {
        moveCmd.MoveToPosition(0,0,0,0,arcLenience,2,1,lowMoveHeight, 0,0,pitchUpPos,true,'C'); //5

        moveCmd.MoveToPosition(speed,0-xAutoOffset,25,0,stopLenience,1,1,lowMoveHeight, 0,0,pitchUpPos,true,'C'); //5
        moveCmd.MoveToPosition(0,0-xAutoOffset,hookDistance,0,arcLenience,2,1,hookHeight, 0,0,pitchUpPos,true,'C'); //5
//        sleep(200);
        moveCmd.MoveToPosition(0,0-xAutoOffset,hookDistance,0,arcLenience,2,0.5,hookHeight, 0,0,pitchUpPos,false,'C'); //5
        sleep(200);
        dt.FieldOrientedTranslate(0,-1,0,shark.GetImuReading());
        sleep(300);

        moveCmd.MoveToPosition(speed,36-xAutoOffset,24,0,preciseLenience,2,1,lowMoveHeight, 0,0,pitchUpPos,false,'G');
        moveCmd.MoveToPosition(speed,36-xAutoOffset,46,0,preciseLenience,1,1,lowMoveHeight, 0,0,pitchUpPos,false,'G');
//
////        Push(0);
////        Push(10);
        LateralPush(0-xAutoOffset);
        moveCmd.MoveToPosition(speed, 46-xAutoOffset,46,0,stopLenience,1,1,lowMoveHeight, 0,0,pitchUpPos,false,'G');
        LateralPush(10-xAutoOffset);
        moveCmd.MoveToPosition(speed, 56-xAutoOffset,48,0,stopLenience,1,1,lowMoveHeight, 0,0,pitchUpPos,false,'G');
        LateralPush(20-xAutoOffset);

//
//        moveCmd.MoveToPosition(speed,52,44,0,preciseLenience,2,0.5,lowMoveHeight, 0,0,0,false,'G'); //5
//        moveCmd.MoveToPosition(speed,58,44,0,preciseLenience,2,0.5,lowMoveHeight, 0,0,0,false,'G'); //6
//        moveCmd.MoveToPosition(speed,58,grabDistance+8,0,preciseLenience,1,0.5,grabHeight, 0,0,0,false,'G'); //6

//        Grab(18);
        Grab(26-xAutoOffset);
        Hook(8-xAutoOffset);
//        dt.FieldOrientedTranslate(0,-1,0,shark.GetImuReading());
//        sleep(300);

        Grab(0-xAutoOffset);
        Hook(8-xAutoOffset);
//        dt.FieldOrientedTranslate(0,-1,0,shark.GetImuReading());
//        sleep(300);

        Grab(0-xAutoOffset);
        Hook(8-xAutoOffset);
//        dt.FieldOrientedTranslate(0,-1,0,shark.GetImuReading());
//        sleep(300);

        Grab(0-xAutoOffset);
        Hook(8-xAutoOffset);

//        moveCmd.MoveToPosition(speed,48,grabDistance+8,0,preciseLenience,2,0.5,50, 0,0,pitchUpPos,false,'G');
        moving = false;
    }

    private void Hook(double offset) // add offset constant
    {
        dt.FieldOrientedTranslate(0,1,0,shark.GetImuReading());
        sleep(300);
        moveCmd.MoveToPosition(speed, 0 + offset, shark.GetPositionY(), 0,preciseLenience,0,1,lowMoveHeight,0, 0,pitchUpPos,true, 'C');

//        moveCmd.MoveToPosition(speed, 0 + offset, grabDistance+12, 0,stopLenience,0,1,lowMoveHeight,0, 0,pitchUpPos,true, 'C');
        moveCmd.MoveToPosition(speed, 0 + offset, hookDistance, 0,stopLenience,1,1,lowMoveHeight,0, 0,pitchUpPos,true, 'C');

        moveCmd.MoveToPosition(0, 0 + offset, hookDistance, 0,arcLenience,2,1, hookHeight,0, 0,pitchUpPos,true, 'C');
//        sleep(200);;
        dt.FieldOrientedTranslate(-1,0,0.1,shark.GetImuReading());
        sleep(200);

        moveCmd.MoveToPosition(0, 0 + offset, hookDistance, 0, arcLenience,1,1,hookHeight,0, 0,pitchUpPos,false, 'C');
        dt.FieldOrientedTranslate(0,-1,0,shark.GetImuReading());
        sleep(300);
    }

    private void Grab(double offset)
    {
        moveCmd.MoveToPosition(speed, 40 + offset, grabDistance + 8 + yRehomeConstant, 0,preciseLenience,2,1, grabHeight, 0, 0,pitchUpPos,false, 'G');
//        moveCmd.MoveToPosition(1, 40 + offset, grabDistance, 0,preciseLenience, 2,1,grabHeight, 0, 0,0,false, 'G');

        dt.FieldOrientedTranslate(0,-0.6,0,shark.GetImuReading());
        sleep(400);

        SparkFunOTOS.Pose2D overridePos = new SparkFunOTOS.Pose2D(shark.GetPositionX(),yRehomeConstant,shark.GetImuReading()); // y rehome constant set to -2.5 from 0
        shark.OverrideOtosPos(overridePos);

        moveCmd.MoveToPosition(0, 40 + offset, grabDistance, 0,arcLenience, 2,1,grabHeight, 0, 0,pitchUpPos,true, 'G');
        sleep(200);
        moveCmd.MoveToPosition(0, 40 + offset, grabDistance,0,arcLenience,2,1,lowMoveHeight,0, 0,pitchUpPos,true, 'G');
//        moveCmd.MoveToPosition(speed,42 + offset,grabDistance,0,0,-850,true,'G'); //8
    }

    // could try rotational push for speed
    private void Push(double offset)
    {
        moveCmd.MoveToPosition(speed,circleXOffset + offset,circleYOffset-robotHalfLength,-30,preciseLenience,3,1,lowMoveHeight, pushExtendDist,0.05,pitchUpPos,false,'G');
        moveCmd.MoveToPosition(0,circleXOffset + offset,circleYOffset-robotHalfLength,-30,arcLenience,2,1,lowMoveHeight, pushExtendDist,0.05,pitchDownPos,false,'G');
        sleep(200);
        moveCmd.MoveToPosition(0,circleXOffset + offset,circleYOffset-robotHalfLength,-30,arcLenience,2,1,lowMoveHeight, pushExtendDist,0.05,pitchDownPos,true,'G');
        sleep(200);

        moveCmd.MoveToPosition(speed,circleXOffset + offset,circleYOffset-robotHalfLength,-150,preciseLenience,4,1,lowMoveHeight, pushExtendDist,0.05,pitchDownPos,true,'G'); //6
        moveCmd.MoveToPosition(0,circleXOffset + offset,circleYOffset-robotHalfLength,-150,arcLenience,2,1,lowMoveHeight, pushExtendDist,0.05,pitchDownPos,false,'G'); //6
        sleep(200);
        moveCmd.MoveToPosition(speed,circleXOffset + offset,circleYOffset-robotHalfLength,-30,preciseLenience,4,1,lowMoveHeight, pushExtendDist,0.05,pitchUpPos,false,'G'); //6
    }

    private void LateralPush(double offset)
    {
        moveCmd.MoveToPosition(speed, 46+offset,46,0,stopLenience,2,1,lowMoveHeight, 0,0,pitchUpPos,false,'G');
        moveCmd.MoveToPosition(speed, 46+offset,15,0,preciseLenience,1,1,lowMoveHeight, 0,0,pitchUpPos,false,'G');
    }

    private void BasicPark()
    {
        dt.Translate(0.5,0,0);
        cs.SetClawOpen(true);
        cs.Update();
        sleep(7500);
        Stop();
    }

    private void UpdateTelemetry()
    {
        telemetry.addData("x", shark.GetPositionX());
        telemetry.addData("y", shark.GetPositionY());
        telemetry.addData("h", shark.GetImuReading());
        telemetry.addData("ex", shark.GetErrorX());
        telemetry.addData("eh", shark.GetErrorH());
        telemetry.addData("ey", shark.GetErrorY());
        telemetry.addData("PROPORTIONAL", shark.GetPorportionalX());
        telemetry.addData("DERIVATIVE", shark.GetDerivativeX());
        telemetry.update();
    }

    private void TestSequenceUpX()
    {
        shark.TuningUp();
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,1,1275,0,0,pitchUpPos,false,'/');
        while (true)
        {
            shark.OdometryControl(1,12,0,0,0.5,2);
            UpdateTelemetry();
        }
    }

    private void TestSequenceUpY()
    {
        shark.TuningUp();
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,1,1275,0,0,pitchUpPos,false,'/');
        while (true)
        {
            shark.OdometryControl(1,0,12,0,0.5,2);
            UpdateTelemetry();
        }
    }

    private void TestSequenceUpXY()
    {
        shark.TuningUp();
        moveCmd.MoveToPosition(speed,0,0,0,0.5,2,1,1275,0,0,pitchUpPos,false,'/');
        while (true)
        {
            shark.OdometryControl(1,12,12,0,0.5,2);
            UpdateTelemetry();
        }
    }

    private void TestSequenceDownX()
    {
        shark.TuningDown();
        shark.OdometryControl(1,12,0,0,0.5,2);
        UpdateTelemetry();
    }

    private void TestSequenceDownY()
    {
        shark.TuningDown();
        shark.OdometryControl(1,0,12,0,0.5,2);
        UpdateTelemetry();
    }

    private void TestSequenceDownXY()
    {
        shark.TuningDown();
        shark.OdometryControl(1,12,12,0,0.5,2);
        UpdateTelemetry();
    }

    private void RotationSequence()
    {
        moveCmd.MoveToPosition(0,0,0,0,10,2,1,1275,0,0,pitchUpPos,false,'/');
        while (true)
        {
            moveCmd.MoveToPosition(speed,0,0,180,1,2,0.5,1275,0,0,pitchUpPos,false,'/');
        }
    }

//    private void IntegralTest()
//    {
//        for (int i = 0; i < 5; i++)
//        {
//            moveCmd.MoveToPosition(speed,0,12,0,0.5,2,0,0,0,0,0,false,'/');
//            moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0,0,0,0,0,false,'/');
//        }
//        for (int i = 0; i < 5; i++)
//        {
//            moveCmd.MoveToPosition(speed,12,0,0,0.5,2,0,0,0,0,0,false,'/');
//            moveCmd.MoveToPosition(speed,0,0,0,0.5,2,0,0,0,0,0,false,'/');
//        }
//        while (true)
//        {
//            shark.OdometryControl(1,0,0,0,0.5,2);
//            UpdateTelemetry();
//        }
//    }

    private void PushTest()
    {
        Push(0);
        Push(10);
        moving = false;
    }

    private void HoldPosition()
    {
        shark.OdometryControl(speed,0,0,0,0.6,3);
        telemetry.addData("shark imu", shark.GetImuReading());
        telemetry.update();
    }

    @Override
    public void runOpMode()
    {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
        dt.init(hardwareMap);
        moveCmd.init(hardwareMap, true);
        shark.init(hardwareMap,true);
        cs.init(hardwareMap, true);
        am.init(hardwareMap, true);

//        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
//        // removed for maximum speed possible
//        double normalizedVolts = (1 - volts/14) + volts/14 * 0.857; // reduced to account for volt drops during auton
//        speed *= normalizedVolts;

        waitForStart();
        ContinueMovement(); //just in case
        while(opModeIsActive())
        {
            if (moving)
            {
                // maybe add arm speed change when hover/moving
//                NetZoneAuto();

                SpecimenSequence();

//                PushTest();
//                HoldPosition();

//                BasketSequence();
//                TestSequenceUp();

//                TestSequenceUpX();
//                TestSequenceUpY();
//                TestSequenceUpXY();

//                TestSequenceDownX();
//                TestSequenceDownY();
//                TestSequenceDownXY();

//                IntegralTest();
//                RotationSequence();
                // BasicPark();
            }
        }
    }
}
