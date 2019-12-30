package org.firstinspires.ftc.teamcode.LinearExtenderTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.teamcode.NonOpModes.RobotController;
import org.firstinspires.ftc.teamcode.NonOpModes.MotionProfile;
import org.firstinspires.ftc.teamcode.NonOpModes.PIDFController;
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
    double prevposition = 0;
    boolean prevA = false;
    boolean prevB = false;
    double index = 0;
    r.resetExtender();
    PIDFController pidf = new PIDFController(0.15,0.02,0.01);
    pidf.enable();
    double position = r.getExtenderPosition();
    MotionProfile mp = new MotionProfile(position, 1, 5, 8);
    while (opModeIsActive()) {
      r.BasicLoopTele();
      position = r.getExtenderPosition();
      double t = r.runtime.time();
      double power = pidf.performPIDF(position, mp.getPos(t), mp.getV(t), mp.getA(t));
      if (gamepad1.a&&!prevA) {
        if (index<5) {
          index++;
        }
        mp = new MotionProfile(position, index*10, 20, 60);
        pidf.reset();
        r.runtime = new ElapsedTime();
        
      }
      else if(gamepad1.b&&!prevB) {
        if (index>0) {
          index--;
        }
        mp = new MotionProfile(position, index*10, 20, 10);
        pidf.reset();
        r.runtime = new ElapsedTime();
      }
      prevB = gamepad1.b;
      prevA = gamepad1.a;
      
      
      telemetry.addData("position", position) ;
      telemetry.addData("t", t);
      telemetry.addData("speed", (position-prevposition)/r.dt);
      telemetry.addData("power", power);
      telemetry.addData("error", pidf.getError());
      r.extender.setPower(power);
      telemetry.update();
      prevposition = position;
      sleep(50);
    }
  }
}
