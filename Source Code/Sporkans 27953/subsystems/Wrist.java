// package org.firstinspires.ftc.teamcode.subsystems;
// 
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.teamcode.Constants;
// 
// 
// public class Wrist {
// 
//     private final Servo wristPivot;
// 
//     private double wristPos = Constants.ClawConstants.wristPos_store;
// 
// 
//     public Wrist(Servo wristPivot) {
//         this.wristPivot = wristPivot;
//     }
// 
// 
// 
//     public void wristPickup() {
//         wristPos = Constants.ClawConstants.wristPos_pickup;
//     }
// 
//     public void wristStore() {
//         wristPos = Constants.ClawConstants.wristPos_store;
//     }
// 
//     public void wristPlace(String placeMode) {
//         switch (placeMode) {
//             case "frontLow":
//                 wristPos = Constants.ClawConstants.wristPos_place_frontLow;
//                 break;
//             case "backHigh":
//                 wristPos = Constants.ClawConstants.wristPos_place_backHigh;
//                 break;
//         }
//     }
// 
// 
// 
//     public void update() {
//         wristPivot.setPosition(wristPos);
//     }
// 
// }
// 