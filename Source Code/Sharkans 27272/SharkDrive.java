package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SharkDrive {
    Drivetrain dt = new Drivetrain();
//    Limelight limelight = new Limelight();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime dxTime = new ElapsedTime();
    ElapsedTime dyTime = new ElapsedTime();
    ElapsedTime dhTime = new ElapsedTime();

    SparkFunOTOS odometry;
    SparkFunOTOS.Pose2D pos;
    SparkFunOTOS.Pose2D lastLimelightPosition = new SparkFunOTOS.Pose2D();

    double lastValidIMUReading = 0;

    boolean[] completedBools = new boolean[3];
    boolean[] completedStopBools = new boolean[3];

    double deltaTime, last_time;
    double integralX, integralY, integralH = 0;
    double kpx, kpy, kph, kix, kiy, kih, kdx, kdy, kdh = 0;
    double iX, iY, iH, pX, dX;
    double xAverage, yAverage, hAverage = 0;
    double[] output = new double[3];
    double[] errors = new double[3];
    double[] previous = new double[3];

    double maximumOutputX = 1;
    double maximumOutputY = 1;

    double diagonalScalar = 0;
    double angleLenience = 60;

    double autograbZeroX = 10;
    double autograbZeroY = 0;
    boolean lostSight = false;
    public int countingTelemetry = 0;

    double odometryInputMixPercentage = 0.7;

    public void init(HardwareMap hwMap, boolean isAuton) {
        kph = 0.34; // 0.34 1.12
        kih = 0.1; // 0.01
        kdh = 0; // 0.25 0.25

        last_time = 0;
        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.calibrateImu();
        odometry.setAngularScalar(0.987); // 0.987 0.989
        // -37 (clockwise, so 37)
        odometry.setLinearScalar(1.052); // 1.052 0.972
        /*
        49.875/52 slow
        0.959
        49.875/51.17 medium
        0.975
        49.875/50.739 fast
        0.983

        == 0.972
         */
        odometry.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));

        if (isAuton) {
            odometry.resetTracking();
            odometry.begin();
        }

        dt.init(hwMap);
//        limelight.init(hwMap);
    }

    public void TuningDown() {
//        kpx = 0.4; //0.38 1.14
//        kpy = 0.22; //0.2 0.6
//        kph = 0.34; // 0.34 1.12
//        kix = 0.0; // 0.01
//        kiy = 0.0; // 0.01
//        kih = 0.01; // 0.01
//        kdx = 0.32; // 0.32 0.16
//        kdy = 0.13; // 0.13 0.065
//        kdh = 0.25; // 0.25 0.25
    }

    public void TuningUp() {
//        kpx = 0.4; //0.38
//        kpy = 0.33; //0.24
//        kph = 0.34; // 0.34
//        kix = 0; // 0.01
//        kiy = 0; //  0
//        kih = 0.01; // 0.43
//        kdx = 0.29; // 0.29
//        kdy = 0.1; // 0.1
//        kdh = 0.25; // 0.25
    }

    private double LowPass(double average, double newValue) {
        average = (average * 0.85) + (0.15 * newValue);

        return average;
    }

    // maybe derivative not needed as velocity becomes near linear
    public double pid(double error, int index, double distanceLenience) {
        double output = 0;
        if (index == 0) {
            integralX += error * dxTime.seconds();

            if (previous[0] * error < 0) {
                integralX = 0;
            }
            iX = integralX;

            double derivative = (error - previous[0]) / dxTime.seconds();
            derivative = LowPass(xAverage, derivative);

//            output = kpx * error + kix * integralX + kdx * derivative;
            output = SampleConstants.KPX * error + kix * integralX + SampleConstants.KDX * derivative;

            dxTime.reset();
            previous[0] = error;

            NewHighestOutputX(Math.abs(output));
            output = Range.clip(output, -1, 1); // old coef 2*
            if (Math.max(Math.abs(maximumOutputX), Math.abs(maximumOutputY)) == Math.abs(maximumOutputY) && error > Math.abs(0.5)) {
                output = DiagonalScalar(Math.abs(maximumOutputX), Math.abs(maximumOutputY), 0.35) * output / Math.abs(output);
            }
        } else if (index == 1) {
            integralY += error * dyTime.seconds();

            if (previous[1] * error < 0) {
                integralY = 0;
            }

            double derivative = (error - previous[1]) / dyTime.seconds();
            derivative = LowPass(yAverage, derivative);
            iY = integralY;
            pX = kph * error;
            dX = derivative;

//            output = kpy * error + kiy * integralY + kdy * derivative;
            output = SampleConstants.KPY * error + kiy * integralY + SampleConstants.KDY * derivative;

            dyTime.reset();

            previous[1] = error;
            NewHighestOutputY(Math.abs(output));
            output = Range.clip(output, -1, 1); // old coef 2*

            if (Math.max(Math.abs(maximumOutputX), Math.abs(maximumOutputY)) == Math.abs(maximumOutputX) && error > Math.abs(0.5)) {
                output *= DiagonalScalar(Math.abs(maximumOutputX), Math.abs(maximumOutputY), 0.3) * output / Math.abs(output);
            }
        } else {
            integralH += error * dhTime.seconds();

            if (previous[2] * error < 0) {
                integralH = 0;
            }

            double derivative = (error - previous[2]) / dhTime.seconds();
            derivative = LowPass(hAverage, derivative);

            iH = integralH;

            output = SampleConstants.KPH * error + SampleConstants.KIH * integralH + SampleConstants.KDH * derivative;
            dhTime.reset();

            previous[2] = error;

            output = Range.clip(output, -1, 1); // old coef 2*
        }
        return output;
    }

    public double basicpid(double error, int index) {
        double output = 0;
        if (index == 0) {
            integralX += error * dxTime.seconds();

            if (previous[0] * error < 0) {
                integralX = 0;
            }

            double derivative = (error - previous[0]) / dxTime.seconds();
            derivative = LowPass(xAverage, derivative);

            output = kpx * error + kix * integralX + kdx * derivative;
            dxTime.reset();
            previous[0] = error;
        } else if (index == 1) {
            integralY += error * dyTime.seconds();

            if (previous[1] * error < 0) {
                integralY = 0;
            }

            double derivative = (error - previous[1]) / dyTime.seconds();
            derivative = LowPass(yAverage, derivative);

            output = kpy * error + kiy * integralY + kdy * derivative;
            dyTime.reset();

            previous[1] = error;
        } else {
            integralH += error * dhTime.seconds();

            if (previous[2] * error < 0) {
                integralH = 0;
            }

            double derivative = (error - previous[2]) / dhTime.seconds();
            derivative = LowPass(hAverage, derivative);

            output = kph * error + kih * integralH + kdh * derivative;
            dhTime.reset();

            previous[2] = error;
        }
        output = Range.clip(output, -1, 1); // old coef 2*
        return output;
    }

    public void CVControl(double speed, double tgtX, double tgtY, double tgtRot, double distanceLenience, int axis, double xOffset, double dist) {
        if (!odometry.isConnected()) {
            return;
        }
        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;

        pos = GetOdometryLocalization();
//        pos = GetLocalization();

        errors[0] = xOffset - (pos.x - autograbZeroX);
        errors[1] = dist - (pos.y - autograbZeroY); // handling distance calc other file
        errors[2] = (Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)))) / 10;

        output[0] = basicpid(errors[0], 0);
        output[1] = basicpid(errors[1], 1);
        output[2] = basicpid(errors[2], 2);

        if (tgtRot == 1) {
            completedBools[2] = true;
            output[2] = 0;
        } else {
            completedBools[2] = Math.abs(errors[2]) < angleLenience;
        }

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;

        if (axis == 0) {
            output[1] = output[1] / Math.abs(output[1]) * 0.05;
            completedBools[1] = true;
        } else if (axis == 1) {
            output[0] = output[0] / Math.abs(output[0]) * 0.05;
            completedBools[0] = true;
        }

        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], pos.h);
    }

    public void OdometryControl(double speed, double tgtX, double tgtY, double tgtRot, double distanceLenience, int axis) {
        if (!odometry.isConnected()) {
            return;
        }
//        distanceLenience; //best value 1.75

        double now = runtime.milliseconds();
        deltaTime = now - last_time;
        last_time = now;

//        pos = limelight.GetLimelightData(false, GetOdometryLocalization().h);
//        pos = GetLocalization();

//        pos = PoseEstimator();

        pos = GetOdometryLocalization();

        if (axis == 3 || axis == 4) {
            angleLenience = 5;
        } else {
            angleLenience = 60;
        }

        errors[0] = tgtX - pos.x;
        errors[1] = tgtY - pos.y;
        errors[2] = Math.toDegrees(angleWrap(Math.toRadians(tgtRot - pos.h)));

        completedBools[2] = Math.abs(errors[2]) < angleLenience;

        errors[2] /= 10;

//        if (new ArmLiftMotor().GetLocalNeutral() == 1250) {
//            TuningUp();
//        } else {
//            TuningDown();
//        }

        // normalized against one another
        // should create weird diagonal movement
        // might have to add increased magnitude to error, currently between -1 and 1
        output[0] = pid(errors[0], 0, distanceLenience);
        output[1] = pid(errors[1], 1, distanceLenience);
        output[2] = pid(errors[2], 2, angleLenience);

        completedBools[0] = Math.abs(errors[0]) < distanceLenience;
        completedBools[1] = Math.abs(errors[1]) < distanceLenience;

        completedStopBools[0] = Math.abs(errors[0]) < 0.6;
        completedStopBools[1] = Math.abs(errors[1]) < 0.6;

        if (axis == 0) {
//            output[1] = output[1] / Math.abs(output[1]) * 0.2;
            output[1] *= 0;
//            output[2] *= 0;
            completedBools[1] = true;
        } else if (axis == 1) {
            output[0] *= 0;
            completedBools[0] = true;
        }
        if (tgtRot == 1){
            completedBools[2] = true;
            output[2] = 0;
        }

        if (axis == 4)
        {
            completedBools[0] = true;
            completedBools[1] = true;
            output[0] = 0;
            output[1] = 0;
        }

//        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], GetOrientation());
        dt.FieldOrientedTranslate(speed * output[0], speed * output[1], speed * output[2], GetImuReading());

    }

    // creates odometry fallback if the limelight stops working
//    private SparkFunOTOS.Pose2D GetLocalization() {
//        SparkFunOTOS.Pose2D limelightPos = limelight.GetLimelightData(false, GetImuReading());
//        if (limelight.GetIsValid()) {
//            lostSight = false;
//            return limelightPos;
//        } else {
//            if (!lostSight) {
//                lastValidIMUReading = GetImuReading() + lastValidIMUReading;
//                Rehome();
//                lastLimelightPosition = limelight.GetLastPosition();
//                lostSight = true;
//            }
//            return GetOdometryLocalization();
//        }
//    }


//    private SparkFunOTOS.Pose2D PoseEstimator()
//    {
//        SparkFunOTOS.Pose2D output = new SparkFunOTOS.Pose2D();
//        SparkFunOTOS.Pose2D limelightPosition = limelight.GetLimelightData(false, GetOrientation());
//
//        if (limelightPosition.x == 0 && limelightPosition.y == 0)
//        {
//            odometryInputMixPercentage = 1;
//        }
//        else
//        {
//            odometryInputMixPercentage = 0.9;
//        }
//
//        output.x = (odometry.getPosition().x * odometryInputMixPercentage) + (limelightPosition.x * 1-odometryInputMixPercentage);
//        output.y = ((odometry.getPosition().y+15) * odometryInputMixPercentage) + (limelightPosition.y * 1-odometryInputMixPercentage);
//        output.h = odometry.getPosition().h;
//
//        return output;
//    }

//    public void SetKPX(double temp)
//    {
//        kpx = temp;
//    }
//
//    public void SetKPY(double temp)
//    {
//        kpy = temp;
//    }
//
//    public void SetKDX(double temp)
//    {
//        kdx = temp;
//    }
//
//    public void SetKDY(double temp)
//    {
//        kdy = temp;
//    }
//
//    public double GetKPX()
//    {
//        return kpx;
//    }
//
//    public double GetKPY()
//    {
//        return kpy;
//    }
//
//    public double GetKDX()
//    {
//        return kdx;
//    }
//
//    public double GetKDY()
//    {
//        return kdy;
//    }

    private SparkFunOTOS.Pose2D GetOdometryLocalization() {
        SparkFunOTOS.Pose2D output = new SparkFunOTOS.Pose2D();

        output.x = odometry.getPosition().x;
        output.y = odometry.getPosition().y;
        output.h = odometry.getPosition().h;

        return output;
    }

    public SparkFunOTOS.Pose2D PrintOdometryLocalization()
    {
        return GetOdometryLocalization();
    }

//    public void SetLastLimelightPosition(SparkFunOTOS.Pose2D value)
//    {
//        lastLimelightPosition = value;
//    }

    public void SetLastValidIMUReading()
    {
        lastValidIMUReading = GetImuReading() + lastValidIMUReading;
    }

    private void NewHighestOutputX(double x) {
        if (x > maximumOutputX) {
            maximumOutputX = x;
        }
    }

    private void NewHighestOutputY(double y) {
        if (y > maximumOutputY) {
            maximumOutputY = y;
        }
    }

    private double DiagonalScalar(double x, double y, double min) {
        diagonalScalar = Math.max((Math.min(x, y) / Math.max(x, y)), min);
        return diagonalScalar;
    }

    public void Rehome()
    {
        odometry.resetTracking();
    }

    public double GetDiagonalScalar()
    {
        return diagonalScalar;
    }

    public double angleWrap(double rad)
    {
        while (rad > Math.PI)
        {
            rad -= 2 * Math.PI;
        }
        while (rad < -Math.PI)
        {
            rad += 2 * Math.PI;
        }
        return -rad;
    }

    public void OverrideOtosPos(SparkFunOTOS.Pose2D position)
    {
        odometry.setPosition(position);
    }

    public double GetLastValidIMUReading()
    {
        return lastValidIMUReading;
    }

    public void SetAngleLenience(double temp)
    {
        angleLenience = temp;
    }

    public double GetImuReading()
    {
        return odometry.getPosition().h;
    }
    public double GetOrientation() {return GetImuReading() + lastValidIMUReading;}

    public double GetPositionX()
    {
        return odometry.getPosition().x;
    }

    public double GetPositionY()
    {
        return odometry.getPosition().y;
    }

//    public double GetLocalizationX() { return GetLocalization().x; }
//    public double GetLocalizationY() { return GetLocalization().y; }

    public double GetErrorX()
    {
        return errors[0];
    }

    public double GetErrorY()
    {
        return errors[1];
    }

    public double GetErrorH()
    {
        return errors[2];
    }

    public double GetOutputX() {return output[0];}
    public double GetOutputY() {return output[1];}

    public double GetAutograbZeroX() {return autograbZeroX;}
    public double GetAutograbZeroY() {return autograbZeroY;}

    public void SetAutograbZeroX(double temp) {autograbZeroX = temp;}
    public void SetAutograbZeroY(double temp) {autograbZeroY = temp;}

    public double GetDerivativeX() { return dX; }

    public double GetPorportionalX() {return pX;}
//    public boolean CamIsValid() {return limelight.GetIsValid();}

//    public SparkFunOTOS.Pose2D GetLastLimelightPosition()
//    {
//        return lastLimelightPosition;
//    }

    public boolean GetBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (!completedBools[i])
            {
                return false;
            }
        }
        integralX = 0;
        integralY = 0;
        integralH = 0;
        maximumOutputX = 1;
        maximumOutputY = 1;
        return true;
    }

    public boolean GetStopBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            if (!completedStopBools[i])
            {
                return false;
            }
        }
        integralX = 0;
        integralY = 0;
        integralH = 0;
        maximumOutputX = 1;
        maximumOutputY = 1;
        return true;
    }

    public void DeactivateBoolsCompleted()
    {
        for (int i = 0; i < 3; i++)
        {
            errors[i] = 0;
            output[i] = 0;
            completedBools[i] = false;
            completedStopBools[i] = false;
        }
    }
}

/*
Congratulations!! You found this useless comment

:D

IF YOU ARE PICKING UP THIS CODE BASE I AM GENUINELY SORRY
PS. I strongly recommend coffee when updating this code
 */
