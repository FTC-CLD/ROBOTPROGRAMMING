package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;

@TeleOp(name = "AutoBesturing (Blocks to Java)", group = "")
public class AutoBesturing extends LinearOpMode {

  private DcMotor right_drive;
  private AndroidAccelerometer androidAccelerometer;
  private AndroidOrientation androidOrientation;
  private DcMotor left_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float Speed;
    float Steering;

    right_drive = hardwareMap.dcMotor.get("right_drive");
    androidAccelerometer = new AndroidAccelerometer();
    androidOrientation = new AndroidOrientation();
    left_drive = hardwareMap.dcMotor.get("left_drive");

    // Reverse one of the drive motors. Jeroen was hrtr
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    androidAccelerometer.startListening();
    androidOrientation.startListening();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        Steering = gamepad1.left_stick_x;
        Speed = gamepad1.right_stick_y;
        if (Steering >= 0) {
          // The Y axis of a joystick ranges from -1 in its topmost position
          // to +1 in its bottommost position. We negate this value so that
          // the topmost position corresponds to maximum forward power.
          left_drive.setPower(Speed);
          right_drive.setPower(Speed * (1 - 2 * Steering));
        } else {
          // The Y axis of a joystick ranges from -1 in its topmost position
          // to +1 in its bottommost position. We negate this value so that
          // the topmost position corresponds to maximum forward power.
          left_drive.setPower(Speed * (1 + 2 * Steering));
          right_drive.setPower(Speed);
        }
        telemetry.addData("Left Pow", left_drive.getPower());
        telemetry.addData("Right Pow", right_drive.getPower());
        telemetry.addData("Left Pos", left_drive.getCurrentPosition());
        telemetry.addData("Android accel Jeorne", androidOrientation.getAngle());
        telemetry.update();
      }
    }
  }
}
