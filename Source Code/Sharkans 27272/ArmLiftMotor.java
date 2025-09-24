package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ArmLiftMotor {
    ElapsedTime dxTime = new ElapsedTime();
    public DcMotor armLiftL = null;
    public DcMotor armLiftR = null;

    //    double speed = 0.3;
    double speed = 0.45;

    int localNeutral = 0;

    boolean canUpdateLocalNeutral = true;

    int topLimit = 1275; // old value 1650
    int bottomLimit = -10;

    double previous = 0;
    double integral = 0;
    double kp, ki, kd = 0;
    double average = 0;
    double error = 0;
    double derivative = 0;

    int setpoint = 0;

    public void init(HardwareMap hwMap, boolean isAuton) {
        kp = 0.2; //0.2
        ki = 0.1; //0.8
        kd = 0.05; // 0.033
        armLiftL = hwMap.get(DcMotor.class, "armLiftL");
        armLiftL.setTargetPosition(0);

        if (isAuton) {
            armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armLiftR = hwMap.get(DcMotor.class, "armLiftR");
        armLiftR.setTargetPosition(0);

        if (isAuton) {
            armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armLiftL.setDirection(DcMotor.Direction.FORWARD); // REVERSE
        armLiftR.setDirection(DcMotor.Direction.REVERSE); // FORWARD
    }

    // learning how enums work lol
//    public enum ArmSpeeds {
//        FAST(1),
//        SLOW(0.3);
//
//        public final double speed;
//
//        ArmSpeeds(double speed) {
//            this.speed = speed;
//        }
//    }

    public void ResetEncoders() {
        if (armLiftL != null && armLiftR != null) {
            topLimit = 1275;
            bottomLimit = -10;

            armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void ResetEncodersUp() {
        if (armLiftL != null && armLiftR != null) {
            topLimit = 10;
            bottomLimit = -1275;

            armLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void Rotate(double targetPower, char mode) {
        if (armLiftL != null && armLiftR != null) {
            if (mode == 'T') {
                if (targetPower > 0.1) {
                    canUpdateLocalNeutral = true;
                    MoveToPosition(topLimit);
                } else if (targetPower < -0.1) {
                    canUpdateLocalNeutral = true;
//                    MoveToPosition(bottomLimit);
                    MoveToPosition(GetCurrentPosition() - 300);
                } else {
                    if (canUpdateLocalNeutral) {
                        localNeutral = armLiftL.getCurrentPosition();
                        canUpdateLocalNeutral = false;
                    }
                    MoveToPosition(localNeutral);
                }
            } else if (mode == 'A') {
//            speed = 1;
                localNeutral = (int) targetPower;
                MoveToPosition((int) targetPower);
            }
        }
    }

    public void MoveToPosition(int position) {
//        double maxValue = Math.max(armLiftL.getCurrentPosition(),position);
//        double minValue = Math.min(armLiftL.getCurrentPosition(),position);
//        double positionDifference = 1 - minValue/maxValue;
//        double positionDifference = 1;
        if (armLiftL != null && armLiftR != null) {
            armLiftL.setPower(speed);
            armLiftR.setPower(speed);
//
            armLiftL.setTargetPosition(position);
            armLiftR.setTargetPosition(position);
        }
//        SetTargetPosition(position);
        setpoint = position;
    }

    public void SetLocalNeutral(int temp) {
        localNeutral = temp;
    }

    private void SetTargetPosition(int position) {
        error = position - armLiftL.getCurrentPosition();
        double output = PID(error / 20) * speed;
        armLiftL.setPower(output);
        armLiftR.setPower(output);
    }

    private double LowPass(double average, double newValue) {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    private double PID(double error) {
        double output = 0;

        integral += error * dxTime.seconds();

        if (previous * error < 0) {
            integral = 0;
        }

        derivative = (error - previous) / dxTime.seconds();
        derivative = LowPass(average, derivative);

        output = kp * error + ki * integral + kd * derivative;
        dxTime.reset();
        previous = error;

        output = Range.clip(output, -1, 1);
        return output;
    }


    public int GetSetpoint() {
        return setpoint;
    }

    public double GetError() {
        return error / 20;
    }

    public double GetProportional() {
        return error * kp;
    }

    public double GetDerivative() {
        return derivative;
    }

    public double GetIntegral() {
        return integral;
    }

    public int GetCurrentPosition() {
        return armLiftL.getCurrentPosition();
    }

    public int GetLocalNeutral() {
        return localNeutral;
    }

    public double GetArmSpeed() {
        return speed;
    }

    public void SetArmSpeed(double temp) {
        speed = temp;
    }

    public boolean GetCompleted(int tgt)
    {
        // increased due to high range
        int lenience = 30;
        int lError = Math.abs(tgt - armLiftL.getCurrentPosition());
        int rError = Math.abs(tgt - armLiftR.getCurrentPosition());
        return (lError + rError) / 2 < lenience;
    }

    public int GetTargetPosition()
    {
        return armLiftL.getTargetPosition();
    }
}
