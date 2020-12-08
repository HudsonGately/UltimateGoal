package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="ServoTest")
public class ServoTest extends OpMode {
//hardware initialization stuff
        Servo servo;
        double pos = 0.3;

        /**
         * User defined init method
         * <p>
         * This method will be called once when the INIT button is pressed.
         */
        @Override
        public void init() {
                servo= hardwareMap.get(Servo.class, "feed_servo");
        }

        /**
         * User defined loop method
         * <p>
         * This method will be called repeatedly in a loop while this op mode is running
         */
        @Override
        public void loop() {
                if(gamepad1.a){
                        pos -= 0.001;
                }
                else if(gamepad1.b){
                        pos += 0.001;
                }
                servo.setPosition(pos);
                telemetry.addData("servo pos",servo.getPosition());
                telemetry.update();

        }
}