package org.firstinspires.ftc.teamcode.LinearExtenderTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.teamcode.RobotController;

@TeleOp(name = "GravityConstants", group = "Extender")
public class GravityConstants extends LinearOpMode {

  private DcMotor extender;
  private RobotController robot;


  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {


    extender = hardwareMap.dcMotor.get("Extender");
    extender.setMode(DcMotor.RunMode.RESET_ENCODERS);
    extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    // Support class for robot control
    robot = new RobotController(this);
    robot.Init();
    
    
    
    
    // Reverse one of the drive motors. Jeroen was hrtr
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    waitForStart();
    double power = 0.2;
    // Going up from -0.20, going down from 0.04
    // Kg = (-0.20+0.04)/2  = -0.08
    // Kfriction = (0.20 + 0.04)/2 = 0.12 in the opposite direction of speed;
    while (opModeIsActive()) {
      robot.BasicLoopTele();
      
      // Put loop blocks here.
      power += -gamepad1.left_stick_y*0.05*robot.dt;
      telemetry.addData("power", power);
      extender.setPower(power);
      telemetry.update();
    }
  }
}
