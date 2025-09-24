package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

public class operations {
    private Servo ClawUDMotor;
    private Servo ClawOCMotor;
    private DcMotor ArmMotor;
    private DcMotor ExtendMotor;



    public operations(DcMotor ArmMotor, DcMotor ExtendMotor, Servo ClawUDMotor, Servo ClawOCMotor) {
        this.ClawUDMotor = ClawUDMotor;
        this.ClawOCMotor = ClawOCMotor;
        this.ArmMotor = ArmMotor;
        this.ExtendMotor = ExtendMotor;

        // Initialize ArmMotor
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize ExtendMotor
        ExtendMotor.setTargetPosition(0);
        ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


//Claw
    public void moveClawUpDown(boolean up, boolean down) {
        if (up) {
            ClawUDMotor.setPosition(1);
        }
        if (down) {
            ClawUDMotor.setPosition(-1);
        }
    }

    public void openCloseClaw(boolean open, boolean close) {
        if (open) {
            ClawOCMotor.setPosition(1);
        }
        if (close) {
            ClawOCMotor.setPosition(-1);
        }
    }


//Arm
    public void moveArm (double armInput, double armPosition, double armMinLimit, double armMaxLimit) {
        double rotations = 312;  // Motor's rotations per revolution
        double gear = 5;         // Gear ratio
        double countsRev = rotations * gear;  // Encoder counts per revolution
        double degree = countsRev / 360;  // Encoder counts per degree of movement
        ArmMotor.setPower(0.65);
        
        if (armInput > 0.1 && armPosition > armMaxLimit) {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (int)(degree * 90));
        }
        else if (armInput < -0.1 && armMinLimit > armPosition) {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() - (int)(degree * 90));
        }
        else {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition());
        }
    }

//Extend
    public void moveExtend (double extendUpInput, double extendDownInput, double extendPosition, double extendLimit) {
        ExtendMotor.setPower(0.65);
        
        if (extendUpInput > 0.1 && extendPosition > extendLimit) {
            ExtendMotor.setTargetPosition(ExtendMotor.getCurrentPosition() + 50);
        }
        else if (extendDownInput > 0.1 && extendPosition > extendLimit) {
            ExtendMotor.setTargetPosition(ExtendMotor.getCurrentPosition() - 50);
        }
        else {
            ExtendMotor.setTargetPosition(ExtendMotor.getCurrentPosition());
        }
    }
}

