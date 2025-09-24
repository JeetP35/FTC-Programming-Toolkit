package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveCommand  {
    Drivetrain dt = new Drivetrain();
    ClawServo cs = new ClawServo();
    ArmLiftMotor am = new ArmLiftMotor();
    SharkDrive shark = new SharkDrive();
    CommandSystem command = new CommandSystem();
    ElapsedTime timeout = new ElapsedTime();
    Extension extend = new Extension();

    int lastA = 0;
    int lastE = 0;

    boolean usingWrist = false;
    
    public void init(HardwareMap hwMap, boolean isAuton)
    {
        dt.init(hwMap);
        cs.init(hwMap, true);
        am.init(hwMap, false);
        extend.init(hwMap);
        shark.init(hwMap, isAuton);
    }

    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, double d, int axis, double speedA, int tgtA, int tgtE, double roll, double pitch, boolean tgtClaw, char tgtWrist)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        // movement, will be driven by s1.GetBoolsCompleted()
        command.SetElementFalse('m'); 

        if (tgtA != lastA) // if arm movement must occur
        {
            command.SetElementFalse('a');
        }
        if (tgtE != lastE)
        {
            command.SetElementFalse('e');
        }

        lastA = tgtA;
        lastE = tgtE;
        
        shark.OdometryControl(speed,tgtX,tgtY,rot,d,axis);

        localCopy = command.GetMap();

        cs.SetWristMode(tgtWrist);
        cs.SetClawOpen(tgtClaw);
        am.SetArmSpeed(speedA);
        extend.SetLocalManipulatorState(roll, pitch);

        if (tgtClaw)
        {
            extend.CloseExtendClaw();
        }
        else
        {
            extend.OpenExtendClaw();
        }

        timeout.reset();

        while (!command.GetBoolsCompleted()) {
            // for every key (m, e, a, c, w)
            for (Character key : localCopy.keySet()) {
                // like an if else but more efficient
                // if a subsystem has reached it's position, set it to complete
                // otherwise, set it to false and stop moving it 
                switch (key) {
                    case 'm':
                        if (shark.GetStopBoolsCompleted()) {
                            shark.OdometryControl(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            // dt.FieldOrientedTranslate(0,0,0,0);
                            break;
                        }
                        else if (shark.GetBoolsCompleted())
                        {
                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            break;
                        }
                        else
                        {
                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementFalse('m');
                            break;
                        }
                    case 'a':
                        am.Rotate(tgtA, 'A');
                        if (am.GetCompleted(tgtA)) {
                            command.SetElementTrue('a');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('a');
                            break;
                        }
                    case 'e':
                        extend.MoveToPosition(tgtE);
                        if (extend.GetCompleted(tgtE))
                        {
                            command.SetElementTrue('e');
                            break;
                        }
                        else
                        {
                            command.SetElementFalse('e');
                            break;
                        }
                }
            }
//            if (timeout.seconds() > 3.5)
//            {
//                break;
//            }
            cs.Update();
            extend.Update();
        }
    }

    public void MoveToPositionCancellable(double speed, double x, double y, double h, double d, int axis, double speedA, int tgtA, int tgtE, double roll, double pitch, boolean tgtClaw, char tgtWrist, boolean extendClaw)
    {
        // reset for next command
//        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();
//
//        command.SetElementFalse('m');
//        command.SetElementFalse('a');
//        command.SetElementFalse('e');

        localCopy = command.GetMap();

        cs.SetWristMode(tgtWrist);
        cs.SetClawOpen(tgtClaw);
        am.SetArmSpeed(speedA);
        am.Rotate(tgtA,'A');
        extend.MoveExtend(tgtE, 'a');
        extend.SetLocalManipulatorState(roll, pitch);
        if (extendClaw){
            extend.OpenExtendClaw();
        }
        else{
            extend.CloseExtendClaw();
        }

        // for every key (m, e, a, c, w)
        for (Character key : localCopy.keySet())
        {
            // like an if else but more efficient
            // if a subsystem has reached it's position, set it to complete
            // otherwise, set it to false and stop moving it 
            switch (key)
            {
                case 'm':
                    if (shark.GetBoolsCompleted())
                    {
                        shark.OdometryControl(0,x,y,h,d,axis);
                        command.SetElementTrue('m');
                        // dt.FieldOrientedTranslate(0,0,0,0);
                        break;
                    }
                    else
                    {
                        shark.OdometryControl(speed,x,y,h,d,axis);
                        command.SetElementFalse('m');
                        break;
                    }
                case 'a':
                    if (am.GetCompleted(tgtA))
                    {
                        command.SetElementTrue('a');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('a');
                        break;
                    }
                case 'e':
                    extend.MoveExtend(tgtE,'a');
                    if (extend.GetCompleted(tgtE))
                    {
                        command.SetElementTrue('e');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('e');
                        break;
                    }
            }
        }
        cs.Update();
    }

    public void InitializeTeleopCommand()
    {
        shark.DeactivateBoolsCompleted();
        command.ResetMap();
        command.SetElementFalse('m');
        command.SetElementFalse('a');
        command.SetElementFalse('e');
    }


    public void MoveToPositionCV(double speed, double x, double y, double h, double d, int axis, double speedA, int tgtA, int tgtE, boolean tgtClaw, double roll, double pitch, double cx, double dist, boolean extendClaw, char type, boolean firstDetection)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();
        if (type == 'm')
        {
            command.SetElementFalse('m');
            command.SetElementFalse('a');
            extend.SetLocalManipulatorState(0,0);
            extend.OpenExtendClaw();
            extend.UpdateOverride();
        }
        else if (type == 'e')
        {
            command.SetElementFalse('e');
            extend.MoveToPosition(tgtE);
            extend.SetLocalManipulatorState(0,0);

            if (extendClaw)
            {
                extend.CloseExtendClaw();
            }
            else
            {
                extend.OpenExtendClaw();
            }

            extend.UpdateOverride();
        }
        else if (type == 'r')
        {
            command.SetElementFalse('r');
        }
        else if (type == 'p')
        {
            command.SetElementFalse('p');
        }
        else if (type == 'c')
        {
            command.SetElementFalse('c');
        }

        localCopy = command.GetMap();

//        cs.SpecifyDiffPos(wristL,wristR);
        cs.SetClawOpen(tgtClaw);
        am.SetArmSpeed(speedA);
        am.Rotate(tgtA,'A');
//        extend.MoveExtend(tgtE, 'a');

        if (firstDetection)
        {
            shark.SetAutograbZeroX(shark.GetPositionX());
            shark.SetAutograbZeroY(shark.GetPositionY());
        }

        // for every key (m, e, a, c, w)
        for (Character key : localCopy.keySet())
        {
            // like an if else but more efficient
            // if a subsystem has reached it's position, set it to complete
            // otherwise, set it to false and stop moving it
            switch (key)
            {
                case 'm':
                    if (shark.GetBoolsCompleted())
                    {
                        shark.CVControl(0,x,y,h,d,axis,cx,dist);
                        command.SetElementTrue('m');
                        // dt.FieldOrientedTranslate(0,0,0,0);
                        break;
                    }
                    else
                    {
                        shark.CVControl(speed,x,y,h,d,axis,cx,dist);
                        command.SetElementFalse('m');
                        break;
                    }
                case 'a':
                    if (am.GetCompleted(tgtA))
                    {
                        command.SetElementTrue('a');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('a');
                        break;
                    }
                case 'e':
                    if (extend.GetCompleted(tgtE))
                    {
                        extend.MoveExtend(0,'T');
                        command.SetElementTrue('e');
                        break;
                    }
                    else
                    {
                        command.SetElementFalse('e');
                        break;
                    }
                case 'r':
                    extend.OpenExtendClaw();
                    extend.SetLocalManipulatorState(roll, extend.GetPitchLocalPosition());
                    extend.UpdateOverride();

                    if (extend.GetRollAtPosition())
                    {
                        command.SetElementTrue('r');
                    }
                    break;
                case 'p':
                    extend.OpenExtendClaw();
                    extend.SetLocalManipulatorState(extend.GetRollLocalPosition(), pitch);
                    extend.UpdateOverride();

                    if (extend.GetPitchAtPosition())
                    {
                        command.SetElementTrue('p');
                    }
                    break;
                case 'c':
                    if (extend.GetExtendClawAtPosition(extendClaw))
                    {
                        command.SetElementTrue('c');
                    }
                    else
                    {
                        if (extendClaw)
                        {
                            extend.CloseExtendClaw();
                            extend.UpdateOverride();
                        }
                        else
                        {
                            extend.OpenExtendClaw();
                            extend.UpdateOverride();
                        }
                        command.SetElementFalse('c');
                    }
                    break;
            }
        }

        cs.Update();
    }

    public boolean GetCommandState()
    {
        return command.GetBoolsCompleted();
    }

    public boolean UsingWrist()
    {
        return usingWrist;
    }
}
