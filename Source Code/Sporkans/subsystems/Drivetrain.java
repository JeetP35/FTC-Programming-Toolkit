// package org.firstinspires.ftc.teamcode.subsystems;
// 
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import org.firstinspires.ftc.teamcode.Constants;
// 
// 
// public class Drivetrain {
// 
//     private final DcMotor frontLeft;
//     private final DcMotor frontRight;
//     private final DcMotor backLeft;
//     private final DcMotor backRight;
// 
// 
//     public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
//         this.frontLeft = frontLeft;
//         this.frontRight = frontRight;
//         this.backLeft = backLeft;
//         this.backRight = backRight;
//     }
// 
// 
//     public void addTelemetry() {
// 
//     }
// 
// 
//     public void runMecanum(double stickx, double sticky, double rot, double heading) {
// 
//         double translateX = stickx;
//         double translateY = sticky;
//         double rotate = rot;
//         double angle = heading;
//     
//         // Convert angles back
//         double x = translateX * Math.cos(-angle) - translateY * Math.sin(-angle) * 1.1;
//         double y = translateX * Math.sin(-angle) + translateY * Math.cos(-angle);
//         
//         double[] speeds = {
//             (y + x + rotate),
//             (y - x - rotate),
//             (y - x + rotate),
//             (y + x - rotate)
//         };
//         
//         double max = Math.abs(speeds[0]);
//         for (int i = 0; i < speeds.length; i++) {
//             if (max < Math.abs(speeds[i])) {
//                 max = Math.abs(speeds[i]);
//             }
//         }
//         
//         if (max > 1) {
//             for (int i = 0; i < speeds.length; i++) {
//                 speeds[i] /= max;
//             }
//         }
//     
//         frontLeft.setPower(-speeds[0]);
//         frontRight.setPower(speeds[1]);
//         backLeft.setPower(-speeds[2]);
//         backRight.setPower(speeds[3]);
//     }
// }
// 