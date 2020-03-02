/*
    Code by team 1820 "ROB. Rover" from No.1 Middle School of Zhengzhou.
    Programmer:Wang JingYi(kexuedianxue@outlook.com)
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "黑珍珠号-手动程序", group = "1820")

public class Black_Pearl_TeleOp extends LinearOpMode {
   
    private Servo mouse;
    private Servo topLeft;
    private Servo topRight;
    private Servo leftLock;
    private Servo rightLock;
    private Servo ArmHand;
    private Servo ArmArm;
    private Servo Test;
    private Servo ballArm;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor frontDrive;
    private DcMotor ArmWheel;

    private double LFPower;
    private double LBPower;
    private double RFPower;
    private double RBPower;
    private double frontPower;
    private double topPosition;
    private double mousePosition;
    private double topLockPosition;
    private double testPos;
    private double  fArmHandPos;
    private double  fArmArmPos;
    private double game1LY;
    private double game1LX;
    private double game1RX;
    private double game2LY;

    static final int    CYCLE_MS    = 50;
    private double RATE        = 1.0;
    private double TURN_RATE   = 0.7;
    
    static final double MOUSE_MAX   = 0.4;
    static final double MOUSE_MIN   = 0;
    static final double TOP_INCREMENT=0.05;
    static final double TOP_HIGH_INCREMENT=0.08;
    static final double TOP_MIN     = 0.0;
    static final double TOP_MAX     = 0.85;
    static final double TOP_LOCK_INCREMENT =0.06;
    static final double TOP_LOCK_LEFT_MAX  =1.0;
    static final double TOP_LOCK_RIGHT_MAX =1.0;
    static final double TOP_LOCK_LEFT_MIN  =0.28;
    static final double TOP_LOCK_RIGHT_MIN =0.08;
    static final double ARM_MAX  = 1.0;
    static final double ARM_MIN  = 0.05;
    static final double ARM_HIGH_INCREMENT = 0.05;
    static final double ARM_LOW_INCREMENT  = 0.02;
    static final double ARM_HAND_MAX  = 0.40;
    static final double ARM_HAND_MIN  = 0;
    static final double ARM_HAND_INCREMENT = 0.03;
    double scaleArray[] = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,///Magic number :)
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

    private void Initialize(){  //初始化
        mouse     = hardwareMap.get( Servo.class , "mouse");
        topLeft   = hardwareMap.get( Servo.class , "TLeft" );
        topRight  = hardwareMap.get( Servo.class , "TRight" );
        leftLock  = hardwareMap.get( Servo.class , "LLock");
        rightLock = hardwareMap.get( Servo.class , "RLock");
        ArmHand   = hardwareMap.get( Servo.class , "ArmHand" );
        ArmArm    = hardwareMap.get( Servo.class , "ArmArm"  );
        Test      = hardwareMap.get( Servo.class , "Test");
        ballArm   = hardwareMap.get( Servo.class , "ball_arm" );

        leftFront   = hardwareMap.get( DcMotor.class , "leftFront" );
        leftBack    = hardwareMap.get( DcMotor.class , "leftBack" );
        rightFront  = hardwareMap.get( DcMotor.class , "rightFront" );
        rightBack   = hardwareMap.get( DcMotor.class , "rightBack" );
        frontDrive  = hardwareMap.get( DcMotor.class , "front_drive" );
        ArmWheel    = hardwareMap.get( DcMotor.class , "ArmWheel");

        ArmWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setDirection( Servo.Direction.FORWARD );
        topRight.setDirection( Servo.Direction.REVERSE );
        mouse.setDirection( Servo.Direction.REVERSE );
        topLockPosition=0.08;
        fArmHandPos=ARM_HAND_MAX;
        fArmArmPos=0.67;
    }
    
    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();
        ballArm.setPosition(0.11);sleep(500);
        ballArm = null;
        int ArmEncoder=0;
        while(opModeIsActive()){

            if(gamepad1.left_bumper){
                RATE = 0.5;
                TURN_RATE = 0.3;
            }else{
                RATE = 1.0;
                TURN_RATE = 0.7;
            }
            //转换手柄的输入
            game1LY=gamepad1.left_stick_y>0?scaleArray[(int)Math.floor(gamepad1.left_stick_y*16.0+0.5)]
                    :-scaleArray[(int)Math.floor(-gamepad1.left_stick_y*16.0+0.5)];
            game1LX=gamepad1.left_stick_x>0?scaleArray[(int)Math.floor(gamepad1.left_stick_x*16.0+0.5)]
                    :-scaleArray[(int)Math.floor(-gamepad1.left_stick_x*16.0+0.5)];
            game1RX=gamepad1.right_stick_x>0?scaleArray[(int)Math.floor(gamepad1.right_stick_x*16.0+0.5)]
                    :-scaleArray[(int)Math.floor(-gamepad1.right_stick_x*16.0+0.5)];
            game2LY=gamepad2.left_stick_y>0?scaleArray[(int)Math.floor(gamepad2.left_stick_y*16.0+0.5)]
                    :-scaleArray[(int)Math.floor(-gamepad2.left_stick_y*16.0+0.5)];
            //控制麦轮的前后、旋转、平移
            LFPower=-RATE*game1LY + RATE*game1LX + TURN_RATE*game1RX;
            LBPower=-RATE*game1LY - RATE*game1LX + TURN_RATE*game1RX;
            RFPower=-RATE*game1LY - RATE*game1LX - TURN_RATE*game1RX;
            RBPower=-RATE*game1LY + RATE*game1LX - TURN_RATE*game1RX;
            
            frontPower = gamepad2.left_trigger-gamepad2.right_trigger;
            frontDrive.setPower(0.8*frontPower);

            leftFront.setPower(-LFPower);
            leftBack.setPower(-LBPower);
            rightFront.setPower(RFPower);
            rightBack.setPower(RBPower);
            ArmWheel.setPower(-game2LY);

            if(gamepad1.dpad_up)
                testPos+=0.01;
            else if(gamepad1.dpad_down)
                testPos-=0.01;
            testPos=Range.clip(testPos,0.0,1.0);
            Test.setPosition(testPos);

            //翻版
            if (gamepad2.y)
                topPosition += TOP_INCREMENT ;
            else if(gamepad2.a)
                topPosition -= TOP_HIGH_INCREMENT ;
            topPosition=Range.clip(topPosition,TOP_MIN,TOP_MAX);
            
            //夹子
            if (gamepad2.left_bumper)
                topLockPosition+=TOP_LOCK_INCREMENT;
            else if(gamepad2.right_bumper)
                topLockPosition-=TOP_LOCK_INCREMENT;
            topLockPosition=Range.clip(topLockPosition,Math.min(TOP_LOCK_LEFT_MIN,TOP_LOCK_RIGHT_MIN),Math.max(TOP_LOCK_LEFT_MAX,TOP_LOCK_RIGHT_MAX));
            
            //嘴
            if(gamepad2.dpad_up)
                mousePosition=MOUSE_MAX;
            else mousePosition=MOUSE_MIN;
            
            //手
            if( gamepad2.dpad_right )
                fArmHandPos -= ARM_HAND_INCREMENT;
            else if( gamepad2.dpad_left )
                fArmHandPos += ARM_HAND_INCREMENT;
            fArmHandPos = Range.clip( fArmHandPos , ARM_HAND_MIN, ARM_HAND_MAX );
            
            //臂
            if( gamepad2.right_stick_y < -0.65 )
                fArmArmPos += ARM_HIGH_INCREMENT;
            else if( gamepad2.right_stick_y > 0.65 )
                fArmArmPos -= ARM_HIGH_INCREMENT;
            else if( gamepad2.right_stick_y < -0.15 )
                fArmArmPos += ARM_LOW_INCREMENT;
            else if( gamepad2.right_stick_y > 0.15 )
                fArmArmPos -= ARM_LOW_INCREMENT;
            fArmArmPos = Range.clip( fArmArmPos , ARM_MIN , ARM_MAX );

            if(gamepad2.x)ArmEncoder=ArmWheel.getCurrentPosition();//5050 -858 //4996 -922
            
            mouse.setPosition(mousePosition);
            leftLock.setPosition(Range.clip(topLockPosition,TOP_LOCK_LEFT_MIN,TOP_LOCK_LEFT_MAX));
            rightLock.setPosition(1-Range.clip(topLockPosition,TOP_LOCK_RIGHT_MIN,TOP_LOCK_RIGHT_MAX));
            topLeft.setPosition(topPosition);
            topRight.setPosition(topPosition);
            ArmHand.setPosition( fArmHandPos );
            ArmArm.setPosition( fArmArmPos  );
            
            //显示实时数据
            telemetry.addData("手动阶段","祝您操纵顺利，绿灯通过！");
            telemetry.addData("功率" , "[%.2f]--[%.2f]", LFPower ,RFPower );
            telemetry.addData("功率" , "  |               |");         
            telemetry.addData("功率" , "[%.2f]--[%.2f]", LBPower ,RBPower );
            telemetry.addData("功率:吸盘" , "%5.2f", frontPower );
            telemetry.addData("测试",testPos);
            telemetry.addData("翻板的位置" , "%5.2f", topPosition );
            telemetry.addData("翻板锁" , "左[%5.2f],右[%5.2f]",
                                Range.clip(topLockPosition,TOP_LOCK_LEFT_MIN,TOP_LOCK_LEFT_MAX),
                                Range.clip(topLockPosition,TOP_LOCK_RIGHT_MIN,TOP_LOCK_RIGHT_MAX) );
            telemetry.addData("嘴的位置" , "%5.2f", mousePosition );

            telemetry.addData("滑轨" , "转速:%5.2f,角度(按x更新):", game2LY,ArmEncoder);
            telemetry.addData("臂(ArmArm)的位置" , "%5.2f", fArmArmPos);
            telemetry.addData("手(ArmHand)的位置" , "%5.2f", fArmHandPos );   
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}