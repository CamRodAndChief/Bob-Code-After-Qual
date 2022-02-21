// 2021-2022 FTC Freight Frenzy
// LSO3 - Locate Special Object
// Current Version: 2059-14-12 - Brandon Kirbyson, Kai Rodriguez, and Luca Cipresso
//
// This is the code to test out the time of flight sensor
//
// List Code Inputs here
// List Code Outputs here
//
// List non-standard dependancies here
//
// Version History:
// Current Version:
// 2022-16-2: Working autonomous
//
// Previous Version:
// 2022-13-2: Working autonomous with sensor issues

//
// Written By: Kai Rodriguez, Brandon Kirbyson, Luca Cipresso, and Teddy Telanoff
// For Jams RoboVikings Team 9887
// 2021-2022 Freight Frenzy
//
// License / Use Terms - GPL
//


package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.logging.Level;
import java.util.*;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;




@Autonomous(name = "AutoBlue1", group="Autonomous")
public class AutoBlue1 extends LinearOpMode { //Locate Special Object

    // gamepad1
    // ********
    // left_stick = Front, Back, Turn Left, Turn Right
    // right_stick_x = Crab Left/Right
    // left_bumper =
    // right_bumper =
    // left_trigger = Turn Table Motor (Duck) CCW
    // right_trigger = Turn Table Motor (Duck) CW
    // A = TBD
    // B = TBD
    // X = Preloaded Block Dropper Down
    // Y = Preloaded Block Dropper Up
    //
    // ********
    // gamepad2
    // ********
    // left_stick = TBD
    // right_stick_x = TBD
    // left_bumper = TBD
    // right_bumper = TBD
    // left_trigger = TBD
    // right_trigger = TBD
    // A = TBD
    // B = TBD
    // X = TBD
    // Y = TBD
    //

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WHEEL_FL;
    private DcMotor WHEEL_FR;
    private DcMotor WHEEL_BL;
    private DcMotor WHEEL_BR;
    private DcMotor TTS_Motor;
    private DcMotor ARM;
    private DcMotor Flapper;
    private DistanceSensor TOF_Left;
    private DistanceSensor TOF_Right;
    private DistanceSensor TOF_Front;
    private DistanceSensor TOF_Back;
    private Servo Lid;           //lid
    private DcMotor IntakeArm;
    private Servo Eyes;

    String MOTORTYPE = "REV20"; // "REV20", "REV40", "REV60", "TETRIX"
    String ARMMOTOR = "REV20"; // "REV20", "REV40", "REV60", "TETRIX"

    int[] ArmLevelTicks = {0, 0, 0, 0};  // declares an array of integers for arm level heights
    // ArmLevelTicks[Level] -> Ticks to move the arm to
    // ArmLevelTicks[0] = Arm Rest Position after Initialization
    // ArmLevelTicks[Level] = Birdbath Levels

    // That is which bar code corresponds to which birdbath level, it switches depending on which side ofthe bird bath you are on
    int StartPosition = 3;         // 1 = Red Closest to duck spinner, 2 = Red Closest to Warehouse; 3 = Blue Closest to duck spinner, 4 = Blue Closest to Warehouse

    double WheelTicksPerRot; // DEFAULT to ANDYMARK
    double WheelDiameterMM;         // The Diameter of the drive wheels in mm
    double DistPerRot;              // Distance traveled, either forward or backward, per 1 complete wheel rotation
    double TickPerDegree;           // The number of motor ticks to rotate (tank-like turn) the robot, either clockwise (CW) or counter-clockwise (CCW)
    double DistCrabPerRot;          // Distance crabbed, either left or right, per 1 complete wheel rotation

    int Step = -1;            // Variable to track which step of the of autonomous to perform
    int[] BarCodeLevels = {1, 2, 3}; // An int array for which barcode level corresponds to which level on the birdbath
    int Level = BarCodeLevels[1];   // Need to update when updating start position that is if the BarCodeLevels is reversed based on starting position
    //int StartLevel = 2;
    boolean ObjDetected = false;    // Boolean, for if there Special Object has been detected
    double BC_Dist = 70; //67.5         // Bar Code Distance
    double LeftTOFcm = 0.0;             // Time of Flight Measurement in cm
    double RightTOFcm = 0.0;
    double FrontTOFcm = 0.0;
    double BackTOFcm = 0.0;


    double HubDist = 60;
    double PipeDist = 60;
    double GapDist = 150;
    double WarehouseDist = 120;

    int WHEEL_FR_pos = 0;           // Front Right Wheel Position in Encoder Steps
    int WHEEL_FL_pos = 0;           // Front Left Wheel Position in Encoder Steps
    int WHEEL_BR_pos = 0;           // Back Right Wheel Position in Encoder Steps
    int WHEEL_BL_pos = 0;           // Back Left Wheel Position in Encoder Steps

    double ArmTicksPerRot; // DEFAULT to ANDYMARK
    double ArmDownSpeed = 0.35;      // Speed to move the arm down 0.0<->1.0
    double ArmUpSpeed = 1.00;     // Speed to move the arm down 0.0<->1.0
    double ArmHoldSpeed = 0.1;      // Holding Current For the Arm
    double OpenLidPos = 0.5;
    double ClosedLidPos = 0.0;


    double Happy = 0.2;
    double Sad = 0.5;
    double Worried = 0.3;
    double Mad = 0.1;
    double Angry = 0.05;


    int IntakeDown = 300;
    int IntakeUp = 0;
    double IntakeSpeed = 0.15;

    boolean LidOpen = false;
    double ARM_pos;
    double closed = 0.0, open = 0.35;
    long ToTheSide;
    int TurnAmount;

    final double BoxOpenPos = 0.8;
    final double BoxFullOpenPos = 0.9;
    final double BoxClosedPos = 0.2;
    final double BoxOpenTopLevel = 0.8;
    final double BoxOpenMiddleLevel = 0.7;
    final double BoxOpenBottomLevel = 0.6;


    boolean SensorsWork = true;
    boolean LeftSensor = true;
    boolean RightSensor = true;
    boolean FrontSensor = true;
    boolean BackSensor = true;

    boolean GapFree = true;
    boolean BotForward = false;
    boolean BotNoAuto = false;
    boolean BotInWareHouse = false;
    boolean BotInGap = false;
    boolean AlternateHubRoute = true;



    @Override
    public void runOpMode() {

        switch (MOTORTYPE) {
            case "ANDY20":
                WheelTicksPerRot = 537.6;
                WheelDiameterMM = 103.5;
                TickPerDegree = 8.769; //
                DistCrabPerRot = 267.5; //mm
                break;
            case "REV20":
                WheelTicksPerRot = 430.77;
                WheelDiameterMM = 105.0;
                TickPerDegree = 5.578; //5.567 //6.69 5.9733
                DistCrabPerRot = 260; //still to be empiracally derived
                break;
            case "REV40":
                WheelTicksPerRot = 1120.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "REV60":
                WheelTicksPerRot = 1680.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "TETRIX":
                WheelTicksPerRot = 1440;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            default:
                telemetry.addData("Wheel Motors not specified", "defaulting to ANDY20");
                telemetry.update();

                WheelTicksPerRot = 537.6;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empirically derived
                break;
        } // switch(MOTORTYPE)

        switch (ARMMOTOR) {

            case "ANDY20":
                ArmTicksPerRot = 537.6;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV20":
                ArmTicksPerRot = 560;
                ArmLevelTicks = new int[]{0, 1080, 960, 800, 180, 1125, 750, 550}; //0, 1000, 880, 750, 200}; // ArmLevelTicks[0] = ticks for down, ArmLevelTicks[1] = ticks for level 1, ...
                break;
            case "REV40":
                ArmTicksPerRot = 1120.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV60":
                ArmTicksPerRot = 1680.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "TETRIX":
                ArmTicksPerRot = 1440;
                ArmLevelTicks = new int[]{-50, -250, -500, -770}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            default:
                telemetry.addData("Arm Motor not specified", "defaulting to ANDY20");
                telemetry.update();

                ArmTicksPerRot = 537.6;
                break;
        } // switch(ARMMOTOR)

        switch (StartPosition) {
            case 1: // Red closest to the duck spinner
                ToTheSide = 550;
                TurnAmount = -90;
                break;
            case 2: // Red closest to the warehouse
                ToTheSide = -650;
                TurnAmount = -90;
                break;
            case 3: // Blue closest to the duck spinner
                BarCodeLevels[0]=3;
                BarCodeLevels[2]=1;
                Level=BarCodeLevels[0];
                ToTheSide = -650;
                TurnAmount = 90;
                break;
            case 4: // Blue closest to the warehouse
                //Collections.reverse(Arrays.asList(BarCodeLevels));
                BarCodeLevels[0]=3;
                BarCodeLevels[2]=1;
                Level=BarCodeLevels[0];
                ToTheSide = -600;
                TurnAmount = 90;
                break;
            default:
                telemetry.addData("Unspecified StartPosition", "defaulting to 1"); // Red closest to the duck spinner
                telemetry.update();
                //Collections.reverse(Arrays.asList(BarCodeLevels));
                break;
        } // switch(StartPosition)

        DistPerRot = Math.PI * WheelDiameterMM; // Distance traveled per full wheel rotation in mm


        WHEEL_FL = hardwareMap.get(DcMotor.class, "wheelfl");           // Front Left
        WHEEL_FR = hardwareMap.get(DcMotor.class, "wheelfr");           // Front Right
        WHEEL_BL = hardwareMap.get(DcMotor.class, "wheelbl");           // Back Left
        WHEEL_BR = hardwareMap.get(DcMotor.class, "wheelbr");           // Back Right
        TTS_Motor = hardwareMap.get(DcMotor.class, "duck");             // Motor for Duck Turn Table Spinner
        ARM = hardwareMap.get(DcMotor.class, "Arm");                // Motor for raising and lowering the arm
        TOF_Left = hardwareMap.get(DistanceSensor.class, "TOF_Left");   // Time of Flight Sensor mounted on the left of the robot
        TOF_Right = hardwareMap.get(DistanceSensor.class, "TOF_Right");   // Time of Flight Sensor mounted on the right of the robot
        TOF_Front = hardwareMap.get(DistanceSensor.class, "TOF_Front");
        TOF_Back = hardwareMap.get(DistanceSensor.class, "TOF_Back");
        Lid = hardwareMap.get(Servo.class, "Lid");              // lid Servo
        Flapper = hardwareMap.get(DcMotor.class, "Flapper");
        IntakeArm = hardwareMap.get(DcMotor.class, "IntakeArm"); //Arm with flapper
        Eyes = hardwareMap.get(Servo.class, "Eyes");



        //Reseting encoder value
        WHEEL_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        TTS_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Still need to wire this up
        TTS_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ARM.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lid.setPosition(closed);

        IntakeArm.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Flapper.setDirection(DcMotorSimple.Direction.FORWARD);

        WHEEL_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TTS_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
        WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        // (MOTORTYPE == "REV20")
        //TTS_Motor.setDirection(DcMotorSimple.Direction.REVERSE); // Not sure if needed
        ARM.setDirection(DcMotorSimple.Direction.REVERSE);  // Not sure if needed


        // *******************************************************************************
        // Step == -1 //Initialization
        // *******************************************************************************

        Lid.setPosition(ClosedLidPos);
        LidOpen = false;
        // RunArm(ArmLevelTicks[Level], ArmUpSpeed);

        // telemetry.addData("fwd-left connection info", WHEEL_FL.getConnectionInfo());
        //telemetry.addData("Level: ", 0);
        telemetry.addLine("NOTICE: MAKE SURE THIS IS THE RIGHT AUTONOMOUS!!");
        telemetry.addData("Current Auto", "BLUE closest to CAROUSEL");
        telemetry.addLine(" ");
        telemetry.addLine(" ");
        telemetry.addData("Object Detection", TOF_Right.getDistance(DistanceUnit.CM));
        telemetry.addLine(" ");
        if(TOF_Left.getDistance(DistanceUnit.CM) > 15){
            telemetry.addLine("WARNING: It appears that you have either setup the robot wrong or picked the wrong autonomous");
            telemetry.addData("Check for", "Robot has DUCK SPINNER facing CAROUSEL and the color of the side is BLUE");
            telemetry.addData("Also Check", "If Right Sensor is Working (Less than 819 and facing wall)");
            telemetry.addData("Right Sensor", TOF_Left.getDistance(DistanceUnit.CM));
            telemetry.addLine(" ");
            telemetry.addLine("If these are both 100% true then ignore warning");
            bob(Sad);
        }else{
            telemetry.addLine("GREAT JOB!!!! Looks like this is the right autonomous!!! Still check to be sure though!");
            telemetry.addData("Also Check", "If Right Sensor is Working (Less than 819 and facing wall)");
            telemetry.addData("Right Sensor", TOF_Left.getDistance(DistanceUnit.CM));
            bob(Happy);
        }
        telemetry.update();

        //Step++;

        // *******************************************************************************


        // *******************************************************************************
        // Waiting for "Play" button to be pressed on the driver hub
        // *******************************************************************************

        waitForStart();

        // *******************************************************************************


        // *******************************************************************************
        // Step == 0 // Locate Barcode Object and Crab + Drive towards Duck Spinner
        // *******************************************************************************

        //IntakeArm.setPosition(80);

        //Step++;

        //***** TOF Check *****
        //LeftTOFcm = TOF_Left.getDistance(DistanceUnit.CM);





        if (TOF_Right.getDistance(DistanceUnit.CM) < BC_Dist) { // If Time of flight sensor is less than 55 cm do this
            Level = BarCodeLevels[1]; // Set level to two
            telemetry.addData("Level: ", Level);
            telemetry.addData("TOF: ", RightTOFcm);
            telemetry.update();
            bob(Angry);
            ObjDetected = true;
        } //(TOFcm < BC_Dist)


        Step++;

        MoveIntake(IntakeDown, IntakeSpeed);
        crab(-300,0.25); // 30 cm Left, speed 0.25

        Flapper.setPower(-0.5); // To help the box get up
        RunArm(ArmLevelTicks[7] - 125, ArmUpSpeed);

        drive(400, 0.5); // cm to the right, speed = 25%
        Flapper.setPower(0);
        MoveIntake(IntakeUp, IntakeSpeed);
        // If the Special Object was not Detected in Posistion 2 or 3, set to position 1.
        // If the object is in 1 of 3 locations, only need to measure 2 in order to determine the location
        if (!ObjDetected) {
            Level = BarCodeLevels[2];
            telemetry.addData("Level: ", Level);
            telemetry.addData("TOF: ", LeftTOFcm);
            telemetry.update();
            bob(Sad);
            ObjDetected = true;
        }else{
            bob(Angry);
        }

        if(TOF_Back.getDistance(DistanceUnit.CM) < 800){
            BackSensor = true;
        }else{
            BackSensor = false;
        }


        turn(-90, 0.5);
        bob(Sad);
        drive(-50, 0.25);
        crab(-350, 0.25);
        drive(100, 0.25);
        bob(Worried);

        Step++; // Move to Next Step

        // *******************************************************************************

        // *******************************************************************************
        // Step == 1 //Run the Duck Spiner
        // *******************************************************************************

        // *******************************************************************************

        TTS_Motor.setPower(-0.08);
        sleep(3000);
        TTS_Motor.setPower(0.0);

        bob(Happy);



        Step++; // Move to Next Step

        // *******************************************************************************

        // *******************************************************************************
        // Step == 2 //Drive towards the Alliance Hub
        // *******************************************************************************

        // *******************************************************************************

        //True up against the duck spinner wall first
        drive(-50, 0.5);
        bob(Happy);
        crab(50, 0.5);
        turn(-90, 0.25);
        bob(Sad);
        drive(-150, 0.25);

        if(TOF_Front.getDistance(DistanceUnit.CM) < 800){
            FrontSensor = true;
        }else{
            FrontSensor = false;
        }

        //Drive
        Step++;

        drive(600, 0.5);

        Step++;



        // Set arm to level indicated by the barcode
        RunArm(ArmLevelTicks[Level], ArmUpSpeed);
        bob(Worried);

        //Road Block Detection

        if (AlternateHubRoute){ //TOF_Back.getDistance(DistanceUnit.CM)*0 < HubDist
            //Do Something becuase there was an object in my path******
            AlternateHubRoute = true;
            crab(150, 0.5);
            turn(-45, 0.25);
            drive(425 + Level, 0.5);
        }
        else{
            AlternateHubRoute = false;
            drive(750, 0.25);
            turn(-90, 0.25);
            drive(140 + Level, 0.25);
        }

        //

        if(Level == 3) {
            Lid.setPosition(BoxOpenTopLevel);
        }else if(Level == 2) {
            Lid.setPosition(BoxOpenMiddleLevel);
        }else{
            Lid.setPosition(BoxOpenBottomLevel);
        }
        LidOpen = true;
        bob(Sad);
        //drive(10,0.5);
        sleep(500); // Time for the object to fall out
        // *******************************************************************************

        Step++;
        if(AlternateHubRoute){
            drive(-100, 0.25);
            turn(-55, 0.25);

        }else{
            drive(-80, 0.25);
            crab(-100, 0.25);
        }

        RunArm(ArmLevelTicks[7], ArmDownSpeed);
        bob(Happy);
        drive(-100, 0.5);

        if(TOF_Right.getDistance(DistanceUnit.CM) < PipeDist){
            BotForward = true;
            GapFree = true;
            BotNoAuto = false;
            bob(Happy);
        }else{
            BotForward = false;
            GapFree = false;
            BotNoAuto = true;
            bob(Angry);

        }


        drive(-600, 0.5);




        if(TOF_Right.getDistance(DistanceUnit.CM) < GapDist){
            GapFree = false;
            BotNoAuto = true;
        }else{
            BotNoAuto = false;
            GapFree = true;
        }








        if(GapFree){

            turn(90, 0.5);
            crab(-300, 0.5);
            if(TOF_Right.getDistance(DistanceUnit.CM) < 800){
                RightSensor = true;
            }else{
                RightSensor = false;
            }
            drive(600, 0.5);






            /*drive(350, 0.5);
            turn(90, 0.25);
            drive(-2000, 0.5);
            */

            /*

            MoveIntake(IntakeDown, IntakeSpeed);
            turn(-90, 0.25);
            Flapper.setPower(0.5);
            RunArm(0, ArmDownSpeed);
            drive(50, 0.5);
            Flapper.setPower(0);
            MoveIntake(IntakeUp, IntakeSpeed);
            crab(800, 0.5);
            crab(100, 0.25);
            */



        }else if(!GapFree){

            drive(-50, 0.25);
            drive(525, 0.5);
            MoveIntake(IntakeDown, IntakeSpeed);
            Flapper.setPower(0.5);
            RunArm(0, ArmDownSpeed);
            turn(90, 0.15);
            Flapper.setPower(0);
            MoveIntake(IntakeUp, IntakeSpeed);
            drive(350, 0.5);
            RunArm(150, ArmDownSpeed);
            IntakeArm.setPower(-0.05);
            drive(2000, 0.5);
            RunArm(0, ArmDownSpeed);
            MoveIntake(IntakeUp, IntakeSpeed);
            sleep(10000);




        }



        if(TOF_Back.getDistance(DistanceUnit.CM) < WarehouseDist){
            GapFree = false;
            BotInWareHouse = true;
            BotNoAuto = false;
            bob(Angry);

            drive(-200, 0.25);
            crab(-50, 0.5);
            crab(500, 0.5);
            drive(2300, 0.5);
            MoveIntake(IntakeDown, IntakeSpeed);
            Flapper.setPower(0.5);
            RunArm(0, ArmDownSpeed);
            turn(-90, 0.25);
            Flapper.setPower(0);

            drive(-50, 0.5);
            MoveIntake(IntakeUp, IntakeSpeed);
            crab(-400, 0.5);
            crab(-100, 0.25);





        }else{
            GapFree = true;
            BotInWareHouse = false;
            bob(Happy);

            MoveIntake(IntakeDown, IntakeSpeed);
            drive(200, 0.5);
            Flapper.setPower(0.3);
            RunArm(0, ArmDownSpeed);
            drive(1050, 0.5);
            Flapper.setPower(0);
            MoveIntake(IntakeUp, IntakeSpeed);
            sleep(1000);


        }






        sleep(10000);

        ///*

        //*/






        /*

        drive(-100, 0.5);

        drive(-300, 0.5);
        turn(90, 0.5);
        crab(-200, 0.5);
        drive(-1200, 0.5);

        Step++;


        crab(-100, 0.25);
        MoveIntake(IntakeUp, 1);
        crab(700, 0.5);
        drive(-400, 0.5);
        turn(-90, 0.25);
        crab(300, 0.5);

        MoveIntake(IntakeDown, 1);
        Lid.setPosition(0.8);
        RunArm(0,ArmDownSpeed);
        Flapper.setPower(-1);
        drive(-300, 0.25);
        drive(300, 0.5);
        Flapper.setPower(0.5);
        Lid.setPosition(closed);
        sleep(500);
        Flapper.setPower(0);



        MoveIntake(IntakeDown, 1);
        sleep(500);

        RunArm(0, ArmUpSpeed);

        MoveIntake(IntakeUp, 1);
        sleep(1000);
        */



        // *******************************************************************************


        // *******************************************************************************
        // Step == 4 // Spin Duck Spinner or Grab Freight from Warehouse - Depending on Start Position
        // *******************************************************************************

        // *******************************************************************************


        // *******************************************************************************
        // Step == 5 // Park in Warehouse or designated area  - Depending on Start Position
        // *******************************************************************************

        // *******************************************************************************

    } // End runOpMode

    //******************************
    //********** SubFXNs ***********
    //******************************


    private void bob(double face){
        Eyes.setPosition(face);
    }


    private void turn(double angle, double speed) {

        // int LeftRot = (int) Math.round( angle * TickPerDegree * TicksPerRot);
        // int RightRot = (int) Math.round( -angle * TickPerDegree * TicksPerRot);
        int LeftRot = (int) Math.round(angle * TickPerDegree);
        int RightRot = (int) Math.round(-angle * TickPerDegree);

        WHEEL_FR.setTargetPosition(RightRot + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(LeftRot + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(RightRot + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(LeftRot + WHEEL_BL_pos);

        RunMotors(speed);

    } // End turn


    private void crab(double distance, double speed) {

        int ROT = (int) Math.round((distance / DistCrabPerRot) * WheelTicksPerRot);

        WHEEL_FR.setTargetPosition(-ROT + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(-ROT + WHEEL_BL_pos);

        RunMotors(speed);

    } // End crab


    private void drive(double distance, double speed) {

        int ROT = (int) Math.round((distance / DistPerRot) * WheelTicksPerRot);

        WHEEL_FR.setTargetPosition(ROT + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(ROT + WHEEL_BL_pos);

        RunMotors(speed);

    } // End drive


    private void RunMotors(double speed) {

        WHEEL_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        WHEEL_FL.setPower(speed);
        WHEEL_FR.setPower(speed);
        WHEEL_BL.setPower(speed);
        WHEEL_BR.setPower(speed);

        while (opModeIsActive() && isWheelsMoving()) {
            DoStuffWhileMoving();
        }

        WHEEL_FL.setPower(0.0);
        WHEEL_FR.setPower(0.0);
        WHEEL_BL.setPower(0.0);
        WHEEL_BR.setPower(0.0);

        sleep(50); // Allow for wheels to settle before getting final Position

        WHEEL_FR_pos = WHEEL_FR.getCurrentPosition();
        WHEEL_FL_pos = WHEEL_FL.getCurrentPosition();
        WHEEL_BR_pos = WHEEL_BR.getCurrentPosition();
        WHEEL_BL_pos = WHEEL_BL.getCurrentPosition();

    } // End RunMotors


    // FXN to clean up the code
    private boolean isWheelsMoving() {
        return WHEEL_FL.isBusy() && WHEEL_FR.isBusy() && WHEEL_BL.isBusy() && WHEEL_BR.isBusy();
    } // End wheelIsMoving


    private void RunArm(int ArmTicks, double ArmSpeed) {

        ARM.setTargetPosition(ArmTicks);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARM.setPower(ArmSpeed);

        while (opModeIsActive() && ARM.isBusy()) {idle();} //

        ARM.setPower(ArmHoldSpeed);
        sleep(100); // Allow for wheels to settle before getting final Position
        ARM_pos = ARM.getCurrentPosition();

    } // End RunArm

    private void MoveIntake(int ticks, double speed) {
        IntakeArm.setTargetPosition(ticks);
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        IntakeArm.setPower(speed);


    }

    // Run Arm but without while loop
    private void RunArmAsync(int ArmTicks, double ArmSpeed) {
        if (ARM.isBusy())
            return;

        if (ARM.getCurrentPosition() == ArmTicks) {
            ARM.setPower(ArmHoldSpeed);
            ARM_pos = ARM.getCurrentPosition();
            return;
        }

        ARM.setTargetPosition(ArmTicks);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARM.setPower(ArmSpeed);
    } // End RunArmAsync


    private void DoStuffWhileMoving() {


        RightTOFcm = TOF_Right.getDistance(DistanceUnit.CM);
        FrontTOFcm = TOF_Front.getDistance(DistanceUnit.CM);
        BackTOFcm = TOF_Back.getDistance(DistanceUnit.CM);
        LeftTOFcm = TOF_Left.getDistance(DistanceUnit.CM);

        switch (Step) {
            case 0: // Crab towards bird bath and figure out where the bar code object
                //LeftTOFcm = TOF_Left.getDistance(DistanceUnit.CM);
                if (!ObjDetected) { // if object not detected then run this
                    if (RightTOFcm < BC_Dist) { // if all of a sudden it sees something than do this
                        Level = BarCodeLevels[0]; // set level to three
                        telemetry.addData("Level:", Level);
                        telemetry.addData("TOF: ", RightTOFcm);
                        telemetry.update();
                        ObjDetected = true;
                        Step += 1;
                    }
                }
                break;
            case 1:

//                if(!ObjDetected && Level == 3){
//                    if(RightTOFcm < 800){
//                        LeftSensor = true;
//                    }else{
//                        LeftSensor = false;
//                        //Level = 2;
//                    }
//                }

                telemetry.update();

                break;
            default:
                telemetry.addLine("Robot Detection");
                telemetry.addData("AlternateHubRoute", AlternateHubRoute);
                telemetry.addData("BotForward", BotForward);
                telemetry.addData("BotNoAuto", BotNoAuto);
                telemetry.addData("GapFree", GapFree);
                telemetry.addLine(" ");
                telemetry.addLine("Alliance Shipping Hub Level");
                telemetry.addData("Level", Level);
                telemetry.addLine(" ");
                telemetry.addLine("Robot Data");
                telemetry.addData("LeftSensor", LeftSensor);
                telemetry.addData("RightSensor", RightSensor);
                telemetry.addData("FrontSensor", FrontSensor);
                telemetry.addData("BackSensor", BackSensor);
                telemetry.update();
                break;
        } // end switch case



        // telemetry.addLine("TOF Sensor Distances");
        // telemetry.addData("Left-TOF: ", LeftTOFcm);
        // telemetry.addData("Right-TOF: ", RightTOFcm);
        // telemetry.addData("Front-TOF: ", FrontTOFcm);
        // telemetry.addData("Back-TOF: ", BackTOFcm);
        // telemetry.addLine(" ");


    } // end DoStuffWhileMoving


} // End DriveTest
