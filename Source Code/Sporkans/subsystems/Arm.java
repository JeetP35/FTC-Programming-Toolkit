// package org.firstinspires.ftc.teamcode.subsystems;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.teamcode.Constants;
// 
// 
// public class Arm {
// 
//     private final DcMotor armPivot;
// 
//     private int armPos = Constants.ArmConstants.armPos_store;
// 
// 
// 
//     public Arm(DcMotor armPivot) {
//         this.armPivot = armPivot;
//         resetEncoders();
//     }
// 
// 
// 
//     public void resetEncoders() {
//         armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//     }
// 
//     
//     public void armPlace(String placeMode) {
//         switch (placeMode) {
//             case "frontLow":
//                 armPos = Constants.ArmConstants.armPos_place_frontLow;
//                 break;
//             case "backHigh":
//                 armPos = Constants.ArmConstants.armPos_place_backHigh;
//                 break;
//         }
//         
//     }
//     public void armStore() {
//         armPos = Constants.ArmConstants.armPos_store;
//     }
// 
// 
//     public void update() {
//         armPivot.setTargetPosition(armPos);
//         armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//         armPivot.setPower(Constants.ArmConstants.armSpeed);
//     }
//     
// }
// 