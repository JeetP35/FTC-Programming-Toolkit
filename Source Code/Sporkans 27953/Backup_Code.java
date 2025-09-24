package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class Backup_Code extends LinearOpMode {
  private DcMotor FLMotor;
  private DcMotor FRMotor;
  private DcMotor BLMotor;
  private DcMotor BRMotor;
  private DcMotor ArmMotor;
  private DcMotor ExtendMotor;
  private DcMotor IntakeMotor;
  private Servo ClawUDMotor;
  private Servo ClawOCMotor;

  @Override
  public void runOpMode() {
    // find the data stuff
    FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
    FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
    BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
    BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
    ArmMotor = hardwareMap.get(DcMotor.class,"ArmMotor");
    ExtendMotor = hardwareMap.get(DcMotor.class, "ExtendMotor");
    IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
    ClawOCMotor = hardwareMap.get(Servo.class, "ClawOCMotor");
    ClawUDMotor = hardwareMap.get(Servo.class, "ClawUDMotor");

    //ArmPositions
    ArmMotor.setTargetPosition(0);
    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //ExtendPosition
    ExtendMotor.setTargetPosition(0);
    ExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    waitForStart();

    while (opModeIsActive()) {
      // setting the power and the equations for cool wheel things
      double ForwardBack = -gamepad1.left_stick_y;
      double LeftRight = gamepad1.left_stick_x;
      double Rotate = -gamepad1.right_stick_x;
      double SpeedMultiplier = 1.0;

      double rotations=312;
      double gear=5;
      double countsRev=rotations*gear;
      double degree=countsRev/360;
      
      double operationPower = 0.90;

      double ExtendDown = gamepad2.right_trigger;
      double ExtendUp = gamepad2.left_trigger;
      double extendParameter = ExtendMotor.getCurrentPosition();
      double extendLimit = -2520;

      double armInput = -gamepad2.right_stick_y;
      double armPosition = ArmMotor.getCurrentPosition();
      double armMaxLimit = -1300;
      double armMinLimit = 1;

      double intakeInput = -gamepad2.right_stick_y;
      
      //dead zone
      if (Math.abs(ForwardBack) < 0.1) ForwardBack = 0;
      if (Math.abs(LeftRight) < 0.1) LeftRight = 0;
      if (Math.abs(Rotate) < 0.1) Rotate = 0;

      //left bumper is half speed right bumper is 1/4
      if (gamepad1.left_bumper) {
        SpeedMultiplier = 0.5;
      } else if (gamepad1.right_bumper) {
        SpeedMultiplier = 0.25;
      }


    //PowerChange
      //OperationPower
      if (gamepad2.left_bumper) {
        operationPower = 0.50;
      }

//Claw

      if (gamepad2.dpad_up) {
        ClawUDMotor.setPosition(0);
      }
      else if (gamepad2.dpad_down) {
        ClawUDMotor.setPosition(0.35);
      }

      if (gamepad2.dpad_right) {
        ClawOCMotor.setPosition(1);
      }
      else if (gamepad2.dpad_left) {
        ClawOCMotor.setPosition(-1);
      }
      
//ArmMotor
      if (armInput < -0.1 && armPosition < armMaxLimit) {
        ArmMotor.setPower(0.40);
        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (int) (degree * 90));
      }
      else if (armInput > 0.1 && armPosition > armMinLimit) {
        ArmMotor.setPower(0.40);
        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() - (int) (degree * 90));
      }
      else {
        ArmMotor.setPower(1.00);
        ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition());
      }

      //ExtendMotor
      if (armPosition > -800) {
        extendLimit = -1593;
      }
      
      if (ExtendUp > 0.1 && extendParameter > extendLimit) {
        ExtendMotor.setTargetPosition(ExtendMotor.getCurrentPosition() + 50); // Example increment
        ExtendMotor.setPower(operationPower * 1.00);
      }
      else if (ExtendDown > 0.1  && extendParameter > extendLimit) {
        ExtendMotor.setPower(operationPower * 1.00);
        ExtendMotor.setTargetPosition(ExtendMotor.getCurrentPosition() - 50); // Example decrement
      }
      else {
        ExtendMotor.setPower(1.00);
        ExtendMotor.setTargetPosition(ExtendMotor.getCurrentPosition());
      }
//Intake
      if (intakeInput > 0.1) {
        IntakeMotor.setPower(operationPower * 1.00);
        IntakeMotor.setTargetPosition(IntakeMotor.getCurrentPosition() + (int) (degree * 90));
      }

      else  if (intakeInput < 0.1) {
        IntakeMotor.setPower(operationPower * 1.00);
        IntakeMotor.setTargetPosition(IntakeMotor.getCurrentPosition() - (int) (degree * 90));
      }

      else {
        IntakeMotor.setPower(0);
        IntakeMotor.setTargetPosition(IntakeMotor.getCurrentPosition());
      }

      // Calculate power for each motor
      double FLP = (ForwardBack + LeftRight + Rotate) * SpeedMultiplier;
      double FRP = -(ForwardBack - LeftRight - Rotate) * SpeedMultiplier;
      double RLP = (ForwardBack - LeftRight + Rotate) * SpeedMultiplier;
      double RRP = -(ForwardBack + LeftRight - Rotate) * SpeedMultiplier;

      // Clip power to ensure it stays within -1.0 to 1.0
      FLP = Range.clip(FLP, -1.0, 1.0);
      FRP = Range.clip(FRP, -1.0, 1.0);
      RLP = Range.clip(RLP, -1.0, 1.0);
      RRP = Range.clip(RRP, -1.0, 1.0);

      // Set motor power
      FLMotor.setPower(FLP * 0.75);
      FRMotor.setPower(FRP * 0.75);
      BLMotor.setPower(RLP * 0.75);
      BRMotor.setPower(RRP * 0.75);

      // showcase
      telemetry.addData("FL Power", FLP);
      telemetry.addData("FR Power", FRP);
      telemetry.addData("BL Power", RLP);
      telemetry.addData("BR Power", RRP);
      telemetry.addData("Speed Multiplier", SpeedMultiplier);  // Show current multiplier
      telemetry.addData("Operation Power", operationPower);
      telemetry.addData("ArmMotor Position", ArmMotor.getCurrentPosition());
      telemetry.addData("ExtendMotor Position", ExtendMotor.getCurrentPosition());
      telemetry.addData("IntakeMotor Position", IntakeMotor.getCurrentPosition());
      telemetry.addData("ArmMotor Power", ArmMotor.getPower());
      telemetry.addData("ExtendMotor Power", ExtendMotor.getPower());
      telemetry.addData("IntakeMotor Power", IntakeMotor.getPower());
      telemetry.addData("claw power", ClawUDMotor.getPosition());
      telemetry.addData("clawUpDown position",ClawOCMotor.getPosition());
      telemetry.update();
    }
  }
}

