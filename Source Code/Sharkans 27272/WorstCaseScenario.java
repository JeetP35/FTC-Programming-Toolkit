// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.hardware.maxbotix.MaxSonarI2CXL;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.robot.Robot;
// 
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.hardware.IMU;
// //import org.firstinspires.ftc.robotcore.external.navigation;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// 
// import org.firstinspires.ftc.teamcode.Drivetrain;
// import org.firstinspires.ftc.teamcode.ClawServo;
// import org.firstinspires.ftc.teamcode.Extension_1;
// import org.firstinspires.ftc.teamcode.ArmLiftMotor;
// import org.firstinspires.ftc.teamcode.Sensor1;
// 
// @Autonomous
// 
// public class WorstCaseScenario extends LinearOpMode
// {
//     Drivetrain dt = new Drivetrain();
//     ClawServo cs = new ClawServo();
//     ArmLiftMotor am = new ArmLiftMotor();
//     Extension_1 e1 = new Extension_1();
//     Sensor1 s1 = new Sensor1(); 
//     ActionHandler action = new ActionHandler();
//     
//     MaxSonarI2CXL leftSensor = null;
//     MaxSonarI2CXL backSensor = null;
//     MaxSonarI2CXL rightSensor = null;
//     
//     IMU imu;
// 
//     double collisionDetectionRadius = 6.5;
//     public boolean moving = true;
//     boolean grabbing = true;
//     
//     int id = -1;
// 
//     DcMotor extension;
//     
//     private void ContinueMovement()
//     {
//         moving = true;
//     }
//     private void MoveForward(double speed, double rot)
//     {
//         double rotationRequired = GetRequiredCorrection(rot);
//         dt.fieldOrientedTranslate(0,speed,rotationRequired * speed);
//         UpdateClaw();
//     }
//     private void MoveBackward(double speed,double rot)
//     {
//         double rotationRequired = GetRequiredCorrection(rot);
//         dt.fieldOrientedTranslate(0,-speed,rotationRequired * speed);
//         UpdateClaw();
//     }
//     
//     private void MoveLeft(double speed, double rot)
//     {
//         double rotationRequired = GetRequiredCorrection(rot);
//         dt.fieldOrientedTranslate(-speed,0,rotationRequired * speed);
//         UpdateClaw();
//     }
//     private void MoveRight(double speed, double rot)
//     {
//         double rotationRequired = GetRequiredCorrection(rot);
//         dt.fieldOrientedTranslate(speed,0,rotationRequired * speed);
//         UpdateClaw();  
//     }
//     
//     private void Stop()
//     {
//         s1.Calibrate();
//         dt.fieldOrientedTranslate(0,0,0);
//         UpdateClaw();
//     }
//     
//     private void LeftRotate(double speed)
//     {
//         dt.fieldOrientedTranslate(0,0,-speed);
//         UpdateClaw();
//     }
//     private void RightRotate(double speed)
//     {
//         dt.fieldOrientedTranslate(0,0,speed);
//         UpdateClaw();
//     }
//     
//     private void OpenClaw()
//     {
//         cs.clawMove(false);
//         UpdateClaw();
//     }
//     private void CloseClaw()
//     {
//         cs.clawMove(true);
//         UpdateClaw();
//     }
//     
//     private void Extend(double power, int tgt)
//     {
//         UpdateClaw();
//         id++;
//         //if (action.GetActionId() != id) {return;}
//         e1.move(power,tgt,"A",id);
//     }
//     private void Retract(double power, int tgt)
//     {
//         UpdateClaw();
//         id++;
//         //if(action.GetActionId()!=id){return;}
//         e1.move(-power,tgt,"A",id);
//     }
//     
//     private void RotateArm(double rotation)
//     {
//         UpdateClaw();
//         id++;
//         //if(action.GetActionId()!=id){return;}
//         am.rotate(rotation,"A",id);
//     }
//     
//     private void ActivateGrab()
//     {
//         grabbing = true;
//     }
//     private void DeactivateGrab()
//     {
//         grabbing = false;
//     }
//     
//     private void UpdateClaw()
//     {
//         cs.update();
//     }
//     
//     private void MoveToPosition(double tgt,double spd,char dir)
//     {
//         id++;
//         //if (action.GetActionId()!=id){return;}
//         s1.moveToPositionR(tgt,spd,dir,id);
//     }
// 
//     private double GetRequiredCorrection(double tgt)
//     {
//         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//         double yaw = orientation.getYaw(AngleUnit.DEGREES);
// 
//         int lenience = 1; //lenience in degrees
//         if (yaw > tgt - lenience && yaw < tgt + lenience)
//         {
//             return 0;
//         }
//         else if (yaw < tgt - lenience)
//         {
//             return -0.05; //turn speed
//         }
//         else if (yaw > tgt + lenience)
//         {
//             return 0.05; //turn speed
//         }
//         return 0;
//     }
// 
//     private void IMU_RotationControl(int tgt, double speed, String dir)
//     {
//         id++;
//         //if (id != action.GetActionId()) {return;}
//         YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//         double yaw = orientation.getYaw(AngleUnit.DEGREES);
//         double lenience = 4;
//         if (dir == "R")
//         {
//             if (!(yaw > tgt - lenience && yaw < tgt + lenience))
//             {
//                 RightRotate(speed);
//             }
//             else
//             {
//                 Stop();
//             }
//         }
//         else if (dir == "L")
//         {
//             if (!(yaw > tgt - lenience && yaw < tgt + lenience))
//             {
//                 LeftRotate(speed);
//             }
//             else
//             {
//                 Stop();
//             }
//         }
//     }
//     
//     private void CommandSequence()
//     {
//         ActivateGrab(); //grab sample
//         CloseClaw(); //grab sample
//         sleep(700);
//         Extend(1,150);
//         sleep(1000);
//         RotateArm(90);//lift sample
//         Extend(1, 750);//try to reach high chamber
//         sleep(2500);
//         //MoveForward(0.55,0);//go to submersible
//         MoveToPosition(15,0.45,'F');
//         sleep(450);
//         Stop();//reset wheels   
//         sleep(1000);
//         RotateArm(70);//start to clip sample
//         sleep(400);
//         RotateArm(60);//pull 
//         sleep(1000);
//         RotateArm(50);//pull
//         sleep(500);
//         Retract(1,450);//secure latch
//         sleep(1500);
//         OpenClaw();//release specimen
//         DeactivateGrab();//release specimen
//         sleep(500);
//         Stop();
//         sleep(100);
//         MoveToPosition(10,0.4,'B');
//         sleep(400);
//         Stop();
//         sleep(50);
//         MoveToPosition(45,0.5,'R');
//         sleep(2000);
//         Stop();
//         sleep(50);
//         moving = false;
//     }
//  
//     @Override
//     public void runOpMode() 
//     {
//         dt.init(hardwareMap);
//         cs.init(hardwareMap);
//         am.init(hardwareMap);
//         e1.init(hardwareMap);
//         s1.init(hardwareMap);
//         
//         leftSensor = hardwareMap.get(MaxSonarI2CXL.class, "Sensor1");
//         backSensor = hardwareMap.get(MaxSonarI2CXL.class, "Sensor2");
//         rightSensor = hardwareMap.get(MaxSonarI2CXL.class, "Sensor3");
//         
//         imu = hardwareMap.get(IMU.class, "imu");
//         extension = hardwareMap.get(DcMotor.class, "extension_1");
//         
//         RevHubOrientationOnRobot.LogoFacingDirection logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
//         RevHubOrientationOnRobot.UsbFacingDirection usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//         RevHubOrientationOnRobot robotOrientationInit = new RevHubOrientationOnRobot(logoDir,usbDir);
//         imu.initialize(new IMU.Parameters(robotOrientationInit));
//         imu.resetYaw();
//         
//         waitForStart();
//         ContinueMovement(); //just in case
//         while(opModeIsActive())
//         {
//             if (moving)
//             {
//                 //IMU_RotationControl(-90,0.4,"R");
//                 CommandSequence();
//                 //MoveToPosition(10,0.2,'R');
//                 
//                 double dL = leftSensor.getDistanceSync(50,DistanceUnit.INCH);
//                 double dB = backSensor.getDistanceSync(50,DistanceUnit.INCH);
//                 double dR = rightSensor.getDistanceSync(50,DistanceUnit.INCH);
//                 
//                 telemetry.addData("l", dL);
//                 telemetry.addData("b", dB);
//                 telemetry.addData("r", dR);
//                 telemetry.addData("id",id);
//                 
//                 YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//                 double yaw = orientation.getYaw(AngleUnit.DEGREES);
//                 telemetry.addData("yaw",yaw);
//                 telemetry.update();
//                 /*telemetry.addData("extension pos", extension.getCurrentPosition());
//                 telemetry.update()*/
//             }
//         }
//     }
// }
// 
