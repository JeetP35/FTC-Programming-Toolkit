package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawServo{
    ArmLiftMotor am = new ArmLiftMotor();
    ElapsedTime runtime = new ElapsedTime();
    
    private Servo claw = null;

    private Servo wrist = null;

    private double clawPos = 0.0;

    private double wristPos = 0;

    private char wristMode = 'D';

    private boolean auton = true;
    
    public void init(HardwareMap hwMap, boolean auton)
    {
        wrist = hwMap.get(Servo.class, "wrist");
        claw = hwMap.get(Servo.class, "claw");
        am.init(hwMap, false);
        this.auton = auton;
    }
    
    public void SetClawOpen(boolean temp)
    {
        if (temp)
        {
            clawPos = 0.35; //initially
        }
        else
        {
            clawPos = 0;
        }
    }

    // rezero
    private void MoveToPositionDiff()
    {
        // 0.5 is normal position

        if (wristMode == 'G')
        {
            wristPos = 0;
        }
        else if (wristMode == 'C')
        {
            wristPos = 0.65;
        }
        else if (wristMode == 'B')
        {
            wristPos = 0.96; // 1
        }
        else if (wristMode == 'S')
        {
            wristPos = 0.6;
        }
        else if (wristMode == 'M')
        {
            wristPos = 0.55;
        }
        else if (wristMode == 'D')
        {
            wristPos = 0.2;
        }
        else if (wristMode == 'P')
        {
            wristPos = 0.44;
        }
        else if (wristMode == 'T')
        {
            wristPos = 0.5;
        }
        else if (wristMode == 'N')
        {
            wristPos = 0.12;
        }
        else
        {
            wristPos = 0;
        }
    }

    public void SpecifyDiffPos(double position)
    {
        wristPos = position;
    }

    public void Update() {
        if (runtime.seconds() > 0.2)
        {
            if (auton)
            {
                if (wrist != null)
                {
                    SetDiffPos(wristPos);
                }
            }
            if (claw != null)
            {
                claw.setPosition(clawPos);
            }
        }
    }

    public void SetWristMode(char temp)
    {
        wristMode = temp;
        MoveToPositionDiff();
    }

    public void SetDiffPos(double position)
    {
        if (wrist != null)
        {
            wrist.setPosition(position);
        }
    }
    
    public boolean GetClawClosed()
    {
        return claw.getPosition() == 1;
    }
    
    public double GetClawPosition()
    {
        return claw.getPosition();
    }
    
    public char GetWristState()
    {
        return wristMode;
    }

    public double GetWristPosition()
    {
        return wrist.getPosition();
    }
}
