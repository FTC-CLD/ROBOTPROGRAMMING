package org.firstinspires.ftc.teamcode.LinearExtenderTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;

@TeleOp(name = "ExtenderTest", group = "")
public class ExtenderTest extends LinearOpMode {

  private DcMotor extender;


  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {


    extender = hardwareMap.dcMotor.get("Extender");
    extender.setMode(DcMotor.RunMode.RESET_ENCODERS);
    extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    
    // Reverse one of the drive motors. Jeroen was hrtr
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    waitForStart();

    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.

        telemetry.update();
      }
    }
  }
}
