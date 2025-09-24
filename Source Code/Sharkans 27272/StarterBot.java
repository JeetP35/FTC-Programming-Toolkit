package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Config
@TeleOp
public class StarterBot extends LinearOpMode{
    //hook offset currently off - five inches from sub
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand();
    ElapsedTime runtime = new ElapsedTime();
//    opencv cv = new opencv();
    Extension extend = new Extension();

    boolean canShift = true;
    double currentSpeed = 1;
    double speedInterval = 0.4;

    double macroSpeed = 0.7;

    double diffSpeed = 1;

    double localOffset = 0;
    double localOffsetIncrement = 2;
    double grabDistance = 0; // 11
    double hookDistance = 29; // 43

    boolean canShiftArm = true;

    boolean normalControl = true;


    boolean canStartSubMacro = true;
    boolean canStartGrabMacro = true;
    boolean canStartHookMacro = true;
    boolean canStartTune = true;
    double localWristPos = 0;

    double preciseLenience = 0.6;
    double arcLenience = 10;

    double xLocalRehomePos = 0;

    double yLocalRehomePos = 0;

    double xRehomePos = 0; // 2
    double yRehomePos = 0; // 31.5

    boolean canGrab = false;
    boolean canSetDiffPos = true;

    double extensionRollSpeed = 1;

    int hookHeight = 1100;
    int grabHeight = 50;
    int extensionSafetyThreshold = -200;

    // inches
    double extendLength = 19.5;

    int extendPos = 0;

    double clawLength = 3.5; // inches

    boolean canPitchSwap = true;
    boolean pitchDown = false;
    boolean canIncrement = true;
    boolean canRetract = false;

    double pitchUpPos = 0.3;
    double pitchDownPos = 0.63;

    @Override
    public void runOpMode()
    {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        dt.init(hardwareMap);
        cs.init(hardwareMap, false);
        am.init(hardwareMap, false);
        shark.init(hardwareMap, false);
        moveCmd.init(hardwareMap, false);
//        cv.init(hardwareMap);
        extend.init(hardwareMap);

        waitForStart();
        am.SetArmSpeed(0.45);
        cs.SetDiffPos(0);
        am.Rotate(0,'T');
        am.SetLocalNeutral(50);
        am.Rotate(0,'T');

//        double volts = hardwareMap.getAll(VoltageSensor.class).get(0).getVoltage();
//        double normalizedVolts = (1 - volts/14) + volts/14 * 0.857; // reduced to account for volt drops during auton

//        cv.StartStream(telemetry);
        while(opModeIsActive())
        {
            // triggers should be extension
            // joystick should be wrist movement
            // swap between normal and custom in teleop for wrist
            double targetPowerX = gamepad1.left_stick_x;
            double targetPowerY = -gamepad1.left_stick_y;
            double targetRotation = gamepad1.right_stick_x;

            boolean aButtonPressed = gamepad1.a;
//            boolean xButtonPressed = gamepad1.x;
            boolean yButtonPressed = gamepad1.y;
            boolean bButtonPressed = gamepad1.b;

            boolean dpadUpPressed = gamepad1.dpad_up;
            boolean dpadDownPressed = gamepad1.dpad_down;

            boolean leftBumperPressed = gamepad1.left_bumper;
            boolean rightBumperPressed = gamepad1.right_bumper;

            double leftTriggerPressed = gamepad1.left_trigger;
            double rightTriggerPressed = gamepad1.right_trigger;

            //arm - add encoders later
            double wristInputX = gamepad2.right_stick_x; // usually left
            double wristInputY = gamepad2.right_stick_y;

            double armInput = gamepad2.left_stick_y;

            double clawOpen = gamepad2.left_trigger;
            double clawClose = gamepad2.right_trigger;

            boolean dpadUpPressed2 = gamepad2.dpad_up;
            boolean dpadDownPressed2 = gamepad2.dpad_down;
            boolean dpadLeftPressed2 = gamepad2.dpad_left;
            boolean dpadRightPressed2 = gamepad2.dpad_right;

            boolean armToggle = gamepad2.left_bumper;

            boolean aButtonPressed2 = gamepad2.a;
//            boolean bButtonPressed2 = gamepad2.b;
            boolean xButtonPressed2 = gamepad2.x;
            boolean yButtonPressed2 = gamepad2.y;

            TelemetryPrint();

            if (normalControl && canStartSubMacro)
            {
                targetPowerX *= currentSpeed;
                targetPowerY *= currentSpeed;
                targetRotation *= currentSpeed;

                dt.FieldOrientedTranslate(targetPowerX,targetPowerY,targetRotation * 0.5, shark.GetOrientation());
                SpeedShift(leftBumperPressed,rightBumperPressed);
//
//                Rehoming(dpadDownPressed);
//
//                ResetTopEncoder(yButtonPressed2);
//                ResetDownEncoder(xButtonPressed2);
                ArmSpeedToggle(armToggle);
                ClawControl(clawClose,clawOpen);
                ExtensionClawControl(clawClose, clawOpen);

                PrepareAscent(leftTriggerPressed);
//                ResetLocalOffset(leftTriggerPressed);
                IncrementLocalOffset(rightTriggerPressed);

                ExtensionControl(wristInputY);
                ExtensionManipulatorControl(wristInputX,aButtonPressed2);

                GrabRehome(aButtonPressed);

//                DropOff(dpadUpPressed2);
                PrepareGrab(dpadDownPressed2);

                if (!canGrab)
                {
                    ArmControl(-armInput);
                }
                DiffControl(dpadLeftPressed2,dpadRightPressed2);

                extend.Update();
            }

            runtime.reset();
//            SubmersibleGrab(aButtonPressed);
            HookMacro(yButtonPressed);
            GrabMacro(bButtonPressed);
//            PIDTune(yButtonPressed);
            cs.Update();
        }
//        cv.StopStream();
    }

    private void PrepareAscent(double input)
    {
        if (input > 0.8)
        {
            extend.SetLocalManipulatorState(0,0.1);
//            extend.MoveToPosition(0);s
            extend.UpdateOverride();
        }
    }

    private void PrepareGrab(boolean input)
    {
        if (!input) {return;}
        dt.FieldOrientedTranslate(0,0,0,0);
        extend.OpenExtendClaw();
        sleep(250);
        extend.SetLocalManipulatorState(extend.GetRollLocalPosition(),pitchDownPos);
        extend.UpdateOverride();

        sleep(250);
        extend.CloseExtendClaw();
        extend.UpdateOverride();
        sleep(250);
        extend.SetLocalManipulatorState(extend.GetRollLocalPosition(),pitchUpPos);
        extend.UpdateOverride();
        sleep(250);
    }

    private void DropOff(boolean input)
    {
        if (!input) {return;}
        extend.SetLocalManipulatorState(0.6,0);
        extend.UpdateOverride();
    }

    private void GrabRehome(boolean temp)
    {
        if (temp) {
            shark.OverrideOtosPos(new SparkFunOTOS.Pose2D(40,0,shark.GetImuReading()));
        }
    }

    private void PIDTune(boolean y)
    {
        if (y && canStartTune)
        {
            normalControl = false;
            canStartTune = false;

            TeleopMoveCommandY(0.7,0, 0, 0,preciseLenience,2,1,grabHeight,0, 0,pitchUpPos,false, 'G', false);
            TeleopMoveCommandY(0.7,12, 0, 0,preciseLenience,2,1,grabHeight,0, 0,pitchUpPos,false, 'G', false);
            TeleopMoveCommandY(0.7,12, 12, 0,preciseLenience,2,1,grabHeight,0, 0,pitchUpPos,false, 'G', false);
            TeleopMoveCommandY(0.7,0,12,0,preciseLenience,2,1,0,grabHeight,0,pitchUpPos,false,'G',false);
            TeleopMoveCommandY(0.7,0, 0, 0,preciseLenience,2,1,grabHeight,0, 0,pitchUpPos,false, 'G', false);

            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!y)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartTune = true;
        }
    }

    private void Rehoming(boolean pressed)
    {
        if (!pressed) {return;}

        shark.Rehome();
        xLocalRehomePos = xRehomePos;
        yLocalRehomePos = yRehomePos;
    }

    private void ResetTopEncoder(boolean pressed)
    {
        if (pressed)
        {
            am.ResetEncodersUp();
        }
    }

    private void ResetDownEncoder(boolean pressed)
    {
        if (pressed)
        {
            am.ResetEncoders();
        }
    }

    private void ResetLocalOffset(double input)
    {
        if (input > 0.8)
        {
            localOffset = 0;
        }
    }

    private void IncrementLocalOffset(double input)
    {
        if (input > 0.3 && canIncrement)
        {
            canIncrement = false;
            localOffset += localOffsetIncrement;
        }
        else if (input <= 0.3)
        {
            canIncrement = true;
        }
    }

    private void SpeedShift(boolean left, boolean right)
    {
        if (left && canShift)
        {
            if (currentSpeed - speedInterval > 0)
            {
                currentSpeed -= speedInterval;
                canShift = false;
            }
        }
        else if (right && canShift)
        {
            if (currentSpeed + speedInterval <= 1)
            {
                currentSpeed += speedInterval;
                canShift = false;
            }
        }
        else if (!left && !right)
        {
            canShift = true;
        }
    }

    private void HookMacro(boolean y)
    {
        if (y && canStartHookMacro)
        {
            normalControl = false;
            canStartHookMacro = false;
            TeleopMoveCommandY(1,localOffset, grabDistance+12, 0-shark.GetLastValidIMUReading(),preciseLenience,0,1,grabHeight+200,0, 0,pitchUpPos,true, 'C', false);
//            TeleopMoveCommandY(1,localOffset, hookDistance-5, 0-shark.GetLastValidIMUReading(),preciseLenience,1,0.7,grabHeight,0, 0,0,true, 'C', false);

            TeleopMoveCommandY(macroSpeed,localOffset, hookDistance, 0-shark.GetLastValidIMUReading(),preciseLenience,1,1,grabHeight+200,0, 0,pitchUpPos,true, 'C', false);
            TeleopMoveCommandY(0,localOffset, hookDistance, 0-shark.GetLastValidIMUReading(),arcLenience,2,1,hookHeight,0, 0,pitchUpPos,true, 'C', false);
            TeleopMoveCommandY(0,localOffset, hookDistance, 0-shark.GetLastValidIMUReading(),arcLenience,2,1,hookHeight,0, 0,pitchUpPos,false, 'C', false);
//            TeleopMoveCommandY(1,localOffset,hookDistance-8,0-shark.GetLastValidIMUReading(),preciseLenience,1,1,hookHeight,0,0,0,false,'C',false);

            localOffset += localOffsetIncrement;
            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!y)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartHookMacro = true;
        }
    }

    private void GrabMacro(boolean b)
    {
        if (b && canStartGrabMacro)
        {
            normalControl = false;
            canStartGrabMacro = false;
            boolean shouldReturn = false;
            shouldReturn = TeleopMoveCommandB(1, 40, grabDistance+8, 0-shark.GetLastValidIMUReading(),preciseLenience,2,1, grabHeight, 0,0,pitchUpPos, false, 'G', false);
            if (shouldReturn) {return;}

            dt.FieldOrientedTranslate(0,-0.35,0,shark.GetImuReading());
            sleep(750);

            if (!gamepad1.b) {return;}

            SparkFunOTOS.Pose2D overridePos = new SparkFunOTOS.Pose2D(shark.GetPositionX(),0,shark.GetImuReading());
            shark.OverrideOtosPos(overridePos);

//            shouldReturn = TeleopMoveCommandB(0.5, 40, grabDistance, 0-shark.GetLastValidIMUReading(),preciseLenience,1,1, grabHeight, 0,0,0, false, 'G', false);
//            if (shouldReturn) {return;}

            shouldReturn = TeleopMoveCommandB(0, 40, grabDistance, 0-shark.GetLastValidIMUReading(),arcLenience,2,1, grabHeight, 0,0,pitchUpPos, true, 'G', false);
            if (shouldReturn) {return;}
            sleep(400);

            shouldReturn = TeleopMoveCommandB(0, 40, grabDistance, 0-shark.GetLastValidIMUReading(), arcLenience,2,1, grabHeight + 200, 0,0,pitchUpPos, true, 'G', false);
            if (shouldReturn) {return;}

            am.SetLocalNeutral(am.GetTargetPosition());
            normalControl = true;
        }
        else if (!b)
        {
            am.SetLocalNeutral(am.GetTargetPosition());
            canStartGrabMacro = true;
        }
    }


    private void TeleopMoveCommandA(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        do
        {
//            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.a) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
    }

    private boolean TeleopMoveCommandB(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        shark.OdometryControl(0, x, y, h, d, axis);
        moveCmd.InitializeTeleopCommand();
        do
        {
            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed * macroSpeed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.b) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return true;
            }
        } while (!moveCmd.GetCommandState());
//        shark.DeactivateBoolsCompleted();
        return false;
    }

    private void TeleopMoveCommandY(double speed, double x, double y, double h, double d, int axis, double speedA, int a, int e, double roll, double pitch, boolean claw, char wrist, boolean extendClaw)
    {
        shark.OdometryControl(0, x, y, h, d, axis);
        moveCmd.InitializeTeleopCommand();
        do
        {
            TelemetryPrint();
            cs.SetClawOpen(claw);
            moveCmd.MoveToPositionCancellable(speed * macroSpeed,x,y,h,d,axis,speedA,a,e,roll, pitch, claw,wrist,extendClaw);
            if (!gamepad1.y) {
                normalControl = true;
                am.SetLocalNeutral(a);
                return;
            }
        } while (!moveCmd.GetCommandState());
//        shark.DeactivateBoolsCompleted();
    }

    private void ArmSpeedToggle(boolean armToggle)
    {
        // removing enum
        if (armToggle)
        {
            if (!canShiftArm) { return; }
            if (am.GetArmSpeed() == 1)
            {
                am.SetArmSpeed(0.45);
                canShiftArm = false;
            }
            else if (am.GetArmSpeed() == 0.45)
            {
                am.SetArmSpeed(1);
                canShiftArm = false;
            }
        }
        else
        {
            canShiftArm = true;
        }
    }

    private void ClawControl(double open, double close)
    {
        if (open > 0.5)
        {
            cs.SetClawOpen(true);
        }
        else if (close > 0.5)
        {
            cs.SetClawOpen(false);
        }
    }

    private void ArmControl(double input)
    {
        if (input > 0.2)
        {
            am.Rotate(1,'T');
        }
        else if (input < -0.2)
        {
            am.Rotate(-1,'T');
        }
        else
        {
            am.Rotate(0,'T');
        }
    }

    private void ExtensionControl(double stickInput)
    {
        if (stickInput < -0.1)
        {
            extend.MoveExtend(1,'T');
        }
        else if (stickInput > 0.1)
        {
            if (extend.GetPitchLocalPosition() == pitchDownPos && extend.GetExtensionPosition() > extensionSafetyThreshold)
            {
                extend.MoveExtend(0, 'T');
            }
            else
            {
                extend.MoveExtend(-1,'T');
            }
        }
        else
        {
            extend.MoveExtend(0,'T');
        }
    }

    private void ExtensionManipulatorControl(double rollInput,boolean downPitch)
    {
        double roll = extend.GetRollLocalPosition();
        double pitch = extend.GetPitchLocalPosition();

        double delta = 0;
        if (rollInput > 0.1)
        {
            delta = -runtime.seconds() * diffSpeed;
        }
        else if (rollInput < -0.1)
        {
            delta = runtime.seconds() * diffSpeed;
        }

        // roll
        if ((roll + delta >= 0 && localWristPos + delta <= 1))
        {
            roll += delta;
        }

        roll = Range.clip(roll, 0, 1);

        if (downPitch && canPitchSwap)
        {
            pitchDown = !pitchDown;
            canPitchSwap = false;

            // pitch
            if (pitchDown)
            {
                if (extend.GetExtensionPosition() < extensionSafetyThreshold)
                {
                    pitch = pitchDownPos;
                }
            }
            else
            {
                pitch = pitchUpPos;
            }
        }
        else if (!downPitch)
        {
            canPitchSwap = true;
        }

        extend.SetLocalManipulatorState(roll, pitch);
    }

    private void ExtensionClawControl(double left, double right)
    {
        if (left > 0.5)
        {
            extend.CloseExtendClaw();
        }
        else if (right > 0.5)
        {
            extend.OpenExtendClaw();
        }
    }

    private void DiffControl(boolean leftInput, boolean rightInput)
    {
        if (rightInput)
        {
            UpdateDiffPos(diffSpeed*runtime.seconds());
        }
        else if (leftInput)
        {
            UpdateDiffPos(-diffSpeed*runtime.seconds());
        }
    }

    private void UpdateDiffPos(double delta)
    {
        if ((localWristPos + delta >= 0 && localWristPos + delta <= 0.65))
        {
            localWristPos += delta;
        }
        cs.SpecifyDiffPos(localWristPos);
        cs.SetDiffPos(localWristPos);
    }

    private void TelemetryPrint()
    {
        telemetry.addData("error x", shark.GetErrorX());
        telemetry.addData("error y", shark.GetErrorY());
        telemetry.addData("error h", shark.GetErrorH());

        telemetry.addData("output x", shark.GetOutputX());
        telemetry.addData("output y", shark.GetOutputY());

        telemetry.addData("x position", shark.GetPositionX());
        telemetry.addData("y position", shark.GetPositionY());
        telemetry.addData("h position", shark.GetImuReading());

        telemetry.addData("shark completed?", shark.GetBoolsCompleted());
        telemetry.addData("command completed?", moveCmd.GetCommandState());

        telemetry.addData("arm speed", am.GetArmSpeed());
        telemetry.addData("can arm shift", canShiftArm);

        telemetry.update();

//        telemetry.addData("counting telemetry", shark.countingTelemetry);
//        telemetry.addData("local offset x", localOffset);

//        telemetry.addData("orientation", shark.GetOrientation());
//        telemetry.addData("last valid imu reading", shark.GetLastValidIMUReading());

//        telemetry.addData("raw localization x", shark.GetLocalizationX());
//        telemetry.addData("raw localization y", shark.GetLocalizationY());
//        telemetry.addData("cam is valid", shark.CamIsValid());
//        telemetry.addData("ODO pos", shark.PrintOdometryLocalization());
//        telemetry.addData("leftWrist", cs.GetWristLPosition());
//        telemetry.addData("rightWrist",cs.GetWristRPosition());
//        telemetry.addData("target extension", extendPos);
//        telemetry.addData("arm position", am.GetCurrentPosition());
//        telemetry.addData("ERROR", am.GetError());
//        telemetry.addData("PROPORTIONAL", am.GetProportional());
//        telemetry.addData("INTEGRAL", am.GetIntegral());
//        telemetry.addData("DERIVATIVE", am.GetDerivative());
//        telemetry.addData("SETPOINT", am.GetSetpoint());
//        telemetry.addData("localL", localDiffL);
//        telemetry.addData("localR", localDiffR);
//        telemetry.addData("INTEGRAL X", shark.GetIntegralSumX());
//        telemetry.addData("INTEGRAL Y", shark.GetIntegralSumY());
//        telemetry.addData("DiagonalScalar", shark.GetDiagonalScalar());
//        telemetry.addData("cv dist", cv.GetDistance());
//        telemetry.addData("cv cx", cv.GetCX());
//        telemetry.addData("cv xOffset", cv.GetXOffset());
//        telemetry.addData("x position", shark.GetPositionX());

//        telemetry.addData("errorX", shark.GetErrorX());
//        telemetry.addData("errorY", shark.GetErrorY());
//        telemetry.addData("autograb zerox", shark.GetAutograbZeroX());
//        telemetry.addData("autograb zeroy", shark.GetAutograbZeroY());
//        telemetry.addData("localWristPos", localWristPos);
//        telemetry.addData("cv points", cv.GetPoints());
//        telemetry.addData("xMin", cv.GetXMin());
//        telemetry.addData("xMax", cv.GetXMax());
//        telemetry.addData("minDiff", cv.GetMinDiff());
//        telemetry.addData("maxDiff", cv.GetMaxDiff());
//        telemetry.addData("yMin", cv.GetYMin());
//        telemetry.addData("yMinX", cv.GetYMinX());
//        telemetry.addData("theta", cv.GetTheta());
//        telemetry.addData("cv detecting", cv.GetDetecting());
//        telemetry.addData("using wrist", moveCmd.UsingWrist());
//        telemetry.addData("local roll position", extend.GetRollLocalPosition());
//        telemetry.addData("local pitch position", extend.GetPitchLocalPosition());
//        telemetry.addData("wrapped theta", cv.GetWrappedTheta());
    }
}
