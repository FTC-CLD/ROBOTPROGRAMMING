package org.firstinspires.ftc.teamcode.LinearExtenderTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "KvTest", group = "Extender")
public class KvTest extends LinearOpMode {


  private RobotController r;


  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {




    // Support class for robot control
    r = new RobotController(this);

    
    
    
    
    // Reverse one of the drive motors. 
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    r.InitAndWait();
    double speedpower = 0.06;

    double prevposition = 0;
    
    while (opModeIsActive()) {
      r.BasicLoopTele();
      double position = r.getExtenderPosition();
      double accelpower = 0;
      if (position > 55) {
        speedpower = -0.05;
        accelpower = speedpower*3;
      }
      if (position < 0) {
        speedpower = 0.2;
        accelpower = speedpower;
      }
      
      double power = accelpower+speedpower+r.getK(position, speedpower);
      // Put loop blocks here.
      
      telemetry.addData("position", position) ;
      telemetry.addData("delta time", r.dt);
      telemetry.addData("speed", (position-prevposition)/r.dt);
      telemetry.addData("power", power);
      r.extender.setPower(power);
      telemetry.update();
      prevposition = position;
      sleep(70);
    }
  }
}
