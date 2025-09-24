package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Drive {
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;

    public Drive(DcMotor FLMotor, DcMotor FRMotor, DcMotor BLMotor, DcMotor BRMotor) {
        this.FLMotor = FLMotor;
        this.FRMotor = FRMotor;
        this.BLMotor = BLMotor;
        this.BRMotor = BRMotor;
    }

    public void drive(double ForwardBack, double LeftRight, double Rotate, double SpeedMultiplier, double FLP, double FRP, double RLP, double RRP) {
        // Calculate power for each motor
         FLP = (ForwardBack + LeftRight + Rotate) * SpeedMultiplier;
         FRP = -(ForwardBack - LeftRight - Rotate) * SpeedMultiplier;
         RLP = (ForwardBack - LeftRight + Rotate) * SpeedMultiplier;
         RRP = -(ForwardBack + LeftRight - Rotate) * SpeedMultiplier;

        // Clip power to ensure it stays within -1.0 to 1.0
        FLP = Range.clip(FLP, -1.0, 1.0);
        FRP = Range.clip(FRP, -1.0, 1.0);
        RLP = Range.clip(RLP, -1.0, 1.0);
        RRP = Range.clip(RRP, -1.0, 1.0);

        // Set motor power
        FLMotor.setPower(FLP);
        FRMotor.setPower(FRP);
        BLMotor.setPower(RLP);
        BRMotor.setPower(RRP);
    }
}