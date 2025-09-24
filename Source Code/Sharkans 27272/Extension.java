package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Extension {
    private DcMotor extension = null;
    private Servo extendRoll = null;
    private Servo extendPitch = null;
    private Servo extendClaw = null;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime pidDT = new ElapsedTime();
//    private Servo extendYaw = null;


    int localNeutral = 0;
    boolean canUpdateLocalNeutral = true;

    int topLimit = -1500; // old value 1650
    int bottomLimit = -50;

    double localPitchPos = 0.3;
    double localRollPos = 0.63;

    double localClawPosition = 0;
    double closeClawPosition = 0.4;

    double kp,ki,kd, integral;
    double lastError = 0;
    double output = 0;

    double errorCrunchConstant = 12.5;

    double servoLenience = 0.01;


    public void init(HardwareMap hwMap) {
        extension = hwMap.get(DcMotor.class, "extension");

        extension.setTargetPosition(0);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setDirection(DcMotor.Direction.FORWARD);

        extendRoll = hwMap.get(Servo.class, "extendRoll");
        extendPitch = hwMap.get(Servo.class, "extendPitch");
        extendClaw = hwMap.get(Servo.class, "extendClaw");
//        extendYaw = hwMap.get(Servo.class, "extendYaw");

        kp = 0.1; // 0.1
        ki = 0; // 0
        kd = 0.0; // 0.01
    }

    public void ResetEncoders() {
        topLimit = 1500;
        bottomLimit = 50;
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ResetEncodersUp() {
        topLimit = -50;
        bottomLimit = -1500;
    }

    // intakeWrist down = 0
    // intake wrist perpendicular = 0.5

    public void MoveExtend(double targetPower, char mode) {
        if (mode == 'T') {
            if (targetPower > 0.1) {
                canUpdateLocalNeutral = true;
                MoveToPosition(topLimit);
            } else if (targetPower < -0.1) {
                canUpdateLocalNeutral = true;
                MoveToPosition(bottomLimit);
            } else {
                extension.setPower(0); // brake mode :)
                if (canUpdateLocalNeutral) {
                    localNeutral = extension.getCurrentPosition();
                    canUpdateLocalNeutral = false;
                }
//                MoveToPosition(localNeutral);
            }
        } else if (mode == 'A') {
            MoveToPosition((int) targetPower);
        }
    }

    public void SetLocalManipulatorState(double roll, double pitch) {
        localRollPos = roll;
        localPitchPos = pitch;
    }

    public void Update()
    {
        if (runtime.seconds() > 0.125)
        {
            runtime.reset();
            SetManipulatorState();
            extendClaw.setPosition(localClawPosition);
        }
    }

    public void UpdateOverride()
    {
        SetManipulatorState();
        extendClaw.setPosition(localClawPosition);
    }

    private double PID(double error)
    {
        double proportional = kp * error;
        integral += error * pidDT.seconds();
        double derivative = (error - lastError) / pidDT.seconds();

        double output = (proportional) + (integral * ki) + (derivative * kd);

        lastError = error;
        pidDT.reset();

        return Range.clip(output, -1, 1);
    }

    public void OpenExtendClaw()
    {
        localClawPosition = 0;
    }

    public void CloseExtendClaw()
    {
        localClawPosition = closeClawPosition;
    }

    public void SetManipulatorState()
    {
        extendRoll.setPosition(localRollPos);
        extendPitch.setPosition(localPitchPos);
//        extendYaw.setPosition(yaw);
    }

    public void MoveToPosition(int position)
    {
        double error = (position/errorCrunchConstant) - (GetExtensionPosition()/errorCrunchConstant);

        output = PID(error);

        extension.setPower(output);
    }

    public int GetCurrentPosition()
    {
        return extension.getCurrentPosition();
    }

    public boolean GetCompleted(int tgt)
    {
        // increased due to high range
        int lenience = 50;
        int error = Math.abs(tgt - extension.getCurrentPosition());

        return error < lenience;
    }

    public boolean GetRollAtPosition()
    {
        return Math.abs(extendRoll.getPosition() - localRollPos) < servoLenience;
    }

    public boolean GetPitchAtPosition()
    {
        return Math.abs(extendPitch.getPosition() - localPitchPos) < servoLenience;
    }

    public double GetRollPosition()
    {
        return extendRoll.getPosition();
    }

    public double GetPitchPosition()
    {
        return extendPitch.getPosition();
    }

    public double GetRollLocalPosition() { return localRollPos; }

    public double GetPitchLocalPosition() { return localPitchPos; }

    public double GetLocalClawPosition() { return localClawPosition; }

    public int GetExtensionPosition() {return extension.getCurrentPosition();}

    public int GetBottomLimit() { return bottomLimit; }

    public int GetTopLimit() { return topLimit; }

    public boolean GetExtendClawAtPosition(boolean tgt)
    {
        double error = 0;
        if (tgt)
        {
            error = Math.abs(closeClawPosition - extendClaw.getPosition());
        }
        else
        {
            error = Math.abs(0 - extendClaw.getPosition());
        }
        return error < servoLenience;
    }

    public double GetOutput()
    {
        return output;
    }
}
