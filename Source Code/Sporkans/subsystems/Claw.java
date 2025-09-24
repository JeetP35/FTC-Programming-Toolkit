// package org.firstinspires.ftc.teamcode.subsystems;
// 
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.teamcode.Constants;
// 
// 
// public class Claw {
// 
//     private final Servo clawLeft;
//     private final Servo clawRight;
// 
//     private double clawLeftPos = Constants.ClawConstants.clawLeftPos_clamp;
//     private double clawRightPos = Constants.ClawConstants.clawRightPos_clamp;
// 
// 
// 
//     public Claw(Servo clawLeft, Servo clawRight) {
//         this.clawLeft = clawLeft;
//         this.clawRight = clawRight;
//     }
// 
// 
// 
//     public void clawOpen() {
//         clawLeftPos = Constants.ClawConstants.clawPos_open;
//         clawRightPos = Constants.ClawConstants.clawPos_open;
//     }
// 
//     public void clawClamp() {
//         clawLeftPos = Constants.ClawConstants.clawLeftPos_clamp;
//         clawRightPos = Constants.ClawConstants.clawRightPos_clamp;
//     }
// 
//     public void clawClampLeft() {
//         clawLeftPos = Constants.ClawConstants.clawLeftPos_clamp;
//     }
// 
//     public void clawClampRight() {
//         clawRightPos = Constants.ClawConstants.clawRightPos_clamp;
//     }
// 
//     public void clawOpenLeft() {
//         clawLeftPos = Constants.ClawConstants.clawPos_open;
//     }
// 
//     public void clawOpenRight() {
//         clawRightPos = Constants.ClawConstants.clawPos_open;
//     }
// 
// 
// 
//     public void update() {
//         clawLeft.setPosition(clawLeftPos);
//         clawRight.setPosition(clawRightPos);
//     }
// 
// }