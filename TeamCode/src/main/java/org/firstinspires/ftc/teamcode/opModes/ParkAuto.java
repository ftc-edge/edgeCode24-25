/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 package org.firstinspires.ftc.teamcode.opModes;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;
 
 /*
  * This OpMode illustrates the concept of driving a path based on time.
  * The code is structured as a LinearOpMode
  *
  * The code assumes that you do NOT have encoders on the wheels,
  *   otherwise you would use: RobotAutoDriveByEncoder;
  *
  *   The desired path in this example is:
  *   - Drive forward for 3 seconds
  *   - Spin right for 1.3 seconds
  *   - Drive Backward for 1 Second
  *
  *  The code is written in a simple form with no optimizations.
  *  However, there are several ways that this type of sequence could be streamlined,
  *
  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
  */
 
 @Autonomous()
 public class ParkAuto extends LinearOpMode {
 
     /* Declare OpMode members. */
     private DcMotor FRmotor;
     private DcMotor FLmotor;
     private DcMotor BRmotor;
     private DcMotor BLmotor;
     private DcMotor slideMotor;
     private DcMotor underSlide;

     public Servo outShoulder;
     public Servo outWrist;
     public Servo outFinger1;
     public Servo outFinger2;
     public Servo clawFinger1;
    public Servo clawFinger2;
     public Servo clawElbow;
     public Servo clawWrist;
     private ElapsedTime runtime = new ElapsedTime();
 
 
     static final double FORWARD_SPEED = 0.45;
     static final double TURN_SPEED = 0.5;
 
    private void place(){
    }
 
     @Override
     public void runOpMode() {
 
         // Initialize the drive system variables.
         FRmotor = hardwareMap.get(DcMotor.class, "FRmotor");
         FRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         FRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         FLmotor = hardwareMap.get(DcMotor.class, "FLmotor");
         FLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         FLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         BRmotor = hardwareMap.get(DcMotor.class, "BRmotor");
         BRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         BRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
 
         BLmotor = hardwareMap.get(DcMotor.class, "BLmotor");
         BLmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        
         BLmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
         FLmotor.setDirection(DcMotor.Direction.REVERSE);
         BLmotor.setDirection(DcMotor.Direction.REVERSE);

         slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
         slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         underSlide = hardwareMap.get(DcMotor.class, "underSlide");
         underSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         underSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         outShoulder = hardwareMap.get(Servo.class, "outShoulder");
         outWrist = hardwareMap.get(Servo.class, "outWrist");
         outFinger1 = hardwareMap.get(Servo.class, "outFinger1");
         outFinger2 = hardwareMap.get(Servo.class, "outFinger2");
 
        clawFinger1 = hardwareMap.get(Servo.class, "clawFinger1");
        clawFinger2 = hardwareMap.get(Servo.class, "clawFinger2");
        
        clawElbow = hardwareMap.get(Servo.class, "clawElbow");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        
        
        
         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
         // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
      
 
         // Send telemetry message to signify robot waiting;
         telemetry.addData("Status", "Ready to run");    //
         telemetry.update();
 
 
         // Wait for the game to start (driver presses PLAY)
         waitForStart();
        
        clawElbow.setPosition(0.69f);
        clawWrist.setPosition(0.5f);
        
        
        float outFing1Position = 0.3f;
        float outFing2Position = 0.3f;
        
        outFinger1.setPosition(outFing1Position);
        outFinger2.setPosition(outFing2Position);
        
        float outShoPosition = 0.593f;
        float outWriPosition = 0.479f;
        outShoulder.setPosition(outShoPosition);
        outWrist.setPosition(outWriPosition);
        
        //TODO: lock intake claw
        
         // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
 
         // Step 1:  Drive forward for 3 seconds
        // FRmotor.setPower(FORWARD_SPEED - 0.01);
        // FLmotor.setPower(FORWARD_SPEED + 0.01);
        // BRmotor.setPower(FORWARD_SPEED + 0.01);
        // BLmotor.setPower(FORWARD_SPEED - 0.01);
        // runtime.reset();
        
        
        // while (opModeIsActive() && (runtime.seconds() < 0.15)) {
        //     telemetry.addData("Path", "Leg 1: Elapsed", runtime.seconds());
        //     telemetry.update();
        // }
        runtime.reset();
        
        FRmotor.setPower(FORWARD_SPEED);
        FLmotor.setPower(FORWARD_SPEED);
        BRmotor.setPower(FORWARD_SPEED);
        BLmotor.setPower(FORWARD_SPEED);
        
        while (opModeIsActive() && (runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 1: Elapsed", runtime.seconds());
            telemetry.update();
        }
    
        FRmotor.setPower(0);
        FLmotor.setPower(0);
        BRmotor.setPower(0);
        BLmotor.setPower(0);
        runtime.reset();
        
        // // Opens the fingers1 just in case
        // float fing1Position = 0.8f;
        // float fing2Position = 0.4f;
        // clawFinger1.setPosition(fing1Position);
        // clawFinger2.setPosition(fing2Position);
        
        // // Rotate arm above robot
        //  outShoPosition = 0.98f;
        //  outWriPosition = 0.22f;

        // outShoulder.setPosition(outShoPosition);
        // outWrist.setPosition(outWriPosition);
        // runtime.reset();
        // while(opModeIsActive() && (runtime.seconds() < 1.0)){
        //     telemetry.addData("Path", "Waiting for ", runtime.seconds());
        //     telemetry.update();
        // }
        
        // // Close Fingers (To make sure)
        //  outFing1Position = 0.3f;
        //  outFing2Position = 0.3f;
        
        // outFinger1.setPosition(outFing1Position);
        // outFinger2.setPosition(outFing2Position);
        
        // // Turn the Robot
        
        //  FRmotor.setPower(-TURN_SPEED);
        //  BRmotor.setPower(-TURN_SPEED);
        //  FLmotor.setPower(TURN_SPEED);
        //  BLmotor.setPower(TURN_SPEED);
        //  runtime.reset();
        //  while (opModeIsActive() && (runtime.seconds() < 0.2)) {
        //      telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
        //      telemetry.update();
        //  }
         
        // // Slides
        // runtime.reset();
        // slideMotor.setPower(TURN_SPEED);
        // underSlide.setPower(TURN_SPEED);
        // while(opModeIsActive() && (runtime.seconds() < 2.65)){
        //     telemetry.addData("Path", "Leg 3: Elapsed", runtime.seconds());
        //     telemetry.update();
        // }
        // slideMotor.setPower(0);
        // underSlide.setPower(0);
        // runtime.reset();
        
        // // Opens the Claws
        // outFinger1.setPosition(0.0f);
        // outFinger2.setPosition(0.6f);
 
       
         
        //  slideMotor.setPower(TURN_SPEED);
        //  underSlide.setPower(TURN_SPEED);
        //  while(opModeIsActive() && (runtime.seconds() < 0.5)){
        //     telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
        //     telemetry.update();
        //  }

        //  outShoulder.setPosition(0.98f);
        //  outWrist.setPosition(0.22f);
        //  while(opModeIsActive() && (runtime.seconds() < 0.1)){
        //     telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
        //     telemetry.update();
        //  }

        //  while(opModeIsActive() && (runtime.seconds() < 0.1)){
        //     telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
        //     telemetry.update();
        //  }

        //  outShoulder.setPosition(0.593f);
        //  outWrist.setPosition(0.479f);
    
         telemetry.addData("Path", "Complete");
         telemetry.update();
         sleep(1000);

     }
 }
 