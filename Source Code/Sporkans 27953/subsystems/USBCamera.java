// package org.firstinspires.ftc.teamcode.subsystems;
// 
// import org.firstinspires.ftc.teamcode.Constants;
// import java.lang.reflect.Array;
// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.List;
// 
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.opencv.core.Mat;
// import org.opencv.core.MatOfPoint;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.core.CvType;
// import org.opencv.core.Rect;
// import org.opencv.imgproc.Imgproc;
// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;
// import org.openftc.easyopencv.OpenCvPipeline;
// import org.openftc.easyopencv.OpenCvWebcam;
// 
// 
// public class USBCamera {
// 
//     private OpenCvWebcam webcam;
//     private Pipeline_Pixel pipeline_pixel;
// 
//     public USBCamera(OpenCvWebcam webcam) {
//         this.webcam = webcam;
//         pipeline_pixel = new Pipeline_Pixel();
// 
//         webcam.setPipeline(pipeline_pixel);
// 
//         webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
//         webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//             @Override
//             public void onOpened() {
//                 webcam.startStreaming(Constants.CameraConstants.camera_width, Constants.CameraConstants.camera_height, OpenCvCameraRotation.UPRIGHT);
//             }
// 
//             @Override
//             public void onError(int errorCode) {}
//         });
//         
//     }
//     
//     public double[] getBestDetectedPosition() {
//         double half_x = Constants.CameraConstants.camera_width / 2;
//         double calc_x = (pipeline_pixel.getDetectedX() - half_x) / half_x; // split x into 2 frames, find distance from middle line, percentage based on that distance to max
// 
//         double half_y = Constants.CameraConstants.camera_height / 2;
//         double calc_y = (pipeline_pixel.getDetectedY() - half_y) / half_y;
// 
//         double[] pos = {calc_x, calc_y};
//         return pos;
//     }
// 
// 
//     class Pipeline_Pixel extends OpenCvPipeline {
//         boolean viewportPaused;
// 
//         Mat gray;
//         Mat threshold;
//         List<MatOfPoint> contours;
//         Mat hier;
//         Rect bounds;
// 
//         int detected_x = 0;
//         int detected_y = 0;
// 
//         @Override
//         public Mat processFrame(Mat input) {
//             
//             gray = new Mat();
//             threshold = new Mat();
//             contours = new ArrayList<>();
//             hier = new Mat();
//             bounds = new Rect(); // x, y, width, height
// 
//             Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
//             Imgproc.threshold(gray, threshold, 180, 255, Imgproc.THRESH_BINARY);
// 
//             Imgproc.findContours(threshold, contours, hier, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
// 
//             for (int i = 0; i < contours.size(); i++) {
//                 if (Imgproc.contourArea(contours.get(i)) > 1000) { // get the area of first set of points
//                     bounds = Imgproc.boundingRect(contours.get(i));
//                     
//                     detected_x = bounds.x;
//                     detected_y = bounds.y;
// 
//                     Imgproc.rectangle(
//                         input,
//                         new Point(
//                             bounds.x, // x
//                             bounds.y // y
//                         ),
//                         new Point(
//                             bounds.x + bounds.width, // x + width
//                             bounds.y + bounds.height // y + height
//                         ),
//                         new Scalar(0, 255, 0),
//                         1
//                     );
//                     
//                     Imgproc.line(
//                         input,
//                         new Point(
//                             bounds.x + (bounds.width / 2) - 3, // x
//                             bounds.y + (bounds.height / 2) // y
//                         ),
//                         new Point(
//                             bounds.x + (bounds.width / 2) + 3, // x + width
//                             bounds.y + (bounds.height / 2) // y + height
//                         ),
//                         new Scalar(0, 200, 255),
//                         1
//                     );
//                     
//                     Imgproc.line(
//                         input,
//                         new Point(
//                             bounds.x + (bounds.width / 2), // x
//                             bounds.y + (bounds.height / 2) - 3 // y
//                         ),
//                         new Point(
//                             bounds.x + (bounds.width / 2), // x + width
//                             bounds.y + (bounds.height / 2) + 3 // y + height
//                         ),
//                         new Scalar(0, 200, 255),
//                         1
//                     );
//                 }            
//             }
// 
//             return input;
//         }
//         
//                 public int getDetectedX() {
//             return detected_x;
//         }
//         public int getDetectedY() {
//             return detected_y;
//         }
// 
//         @Override
//         public void onViewportTapped()
//         {
// 
//             viewportPaused = !viewportPaused;
// 
//             if (viewportPaused) {
//                 webcam.pauseViewport();
//             } else {
//                 webcam.resumeViewport();
//             }
//         }
//     }
// }
// 