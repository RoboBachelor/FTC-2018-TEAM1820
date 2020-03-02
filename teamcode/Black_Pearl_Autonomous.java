package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import android.graphics.Color;

@Autonomous(name="黑珍珠号-自动程序", group="1820")

public class Black_Pearl_Autonomous extends LinearOpMode {
    private Servo ballArm;
    private Servo ballHand;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor frontDrive;
    private ColorSensor sensorRGB;
    
    //Vuforia instances/objects
    private VuforiaLocalizer            PhoneCameraInterface;
    private VuforiaLocalizer.Parameters param;
    private VuforiaTrackables           relicTrackable;
    private VuforiaTrackable            relicTemplate;
    private RelicRecoveryVuMark         Img;
    private enum COLOR{RED,BLUE}
    private float hsvValues[] = {0F,0F,0F};
    private ElapsedTime runtime = new ElapsedTime();

    private int chooseMode(){
        while(true){
            if(gamepad2.y){
                telemetry.addData("机器的起始位置","蓝1");
                telemetry.update();
                return 1;
            }if(gamepad2.a){
                telemetry.addData("机器的起始位置","蓝2");
                telemetry.update();
                return 2;
            }if(gamepad2.dpad_up){
                telemetry.addData("机器的起始位置","红1");
                telemetry.update();
                return 3;
            }if(gamepad2.dpad_down){
                telemetry.addData("机器的起始位置","红2");
                telemetry.update();
                return 4;
            }else{
                telemetry.addData("请选择机器的起始位置","等待选择…");
                telemetry.update();
            }
        }
    }
    private void Init(){
        ballArm   = hardwareMap.get( Servo.class , "ball_arm" );
        ballHand  = hardwareMap.get( Servo.class , "ball_hand");
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");

        leftFront   = hardwareMap.get( DcMotor.class , "leftFront" );
        leftBack    = hardwareMap.get( DcMotor.class , "leftBack" );
        rightFront  = hardwareMap.get( DcMotor.class , "rightFront" );
        rightBack   = hardwareMap.get( DcMotor.class , "rightBack" );
        frontDrive  = hardwareMap.get( DcMotor.class , "front_drive" );

        //设定电机初始状态为解除刹车
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void InitVuforia(){
        int cameraID = hardwareMap.appContext.getResources().getIdentifier( "cameraMonitorViewId" , "id" , hardwareMap.appContext.getPackageName() );
        param = new VuforiaLocalizer.Parameters(cameraID);

        //This serial number is a magic number XD (magic number such as 19260817 :P)
        param.vuforiaLicenseKey = "AXGoew7/////AAABmShfDDB9H0rhguyNhaFhsoUHwQx5qa/QnKkO+7SKHzJDx+vfTfABrsMiYxU6FuBDQccQt01Qy5n4dPc5NT1zT+HZUJXUCOrMsX8QQRPndIVA6lxHenTntPfO8Iuxx+eyI7c/dSMH+wZKWZ7N9L3pK6imfmZ6nHEqR7oV09vA8it21sb7+qhbpLEM7k1dMxU8XEt6WAGT0Kc1998pd+Wxkr/Nqj1VBGLHN4XgO0LVEmNj1Ad3tmxes0uRKQjkp6rWn/5OkUFp01DckuQ+fGcIWgBiutAEG6Yz38I+p++IPK0TjuDGFJLSoYMfXpVoO4FtTWYk4yS5o2CiF1cELVhfPUohosGqWL5WYFe2I4icv0ng";
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.PhoneCameraInterface = ClassFactory.createVuforiaLocalizer(param);
        
        relicTrackable = this.PhoneCameraInterface.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackable.get(0);
        
        relicTrackable.activate();
        Img = RelicRecoveryVuMark.UNKNOWN;
    }
    private void encoderDrive(double rightPower,int rightTarget, double aStartDis,double aEndDis, double timeoutS){
        int newRightTarget;
        int rightStart;
        double minPower=0.03;
        double RATE;
        double DIR=rightTarget>0?1.0:-1.0;
        if(opModeIsActive()) {
            rightStart = rightBack.getCurrentPosition();
            newRightTarget = rightStart + rightTarget;
            runtime.reset();
            while(opModeIsActive() && runtime.seconds() < timeoutS &&
                    Math.abs(rightBack.getCurrentPosition()-rightStart)<=Math.abs(rightTarget) ){
                if( Math.abs(rightBack.getCurrentPosition()-rightStart) <= (int)aStartDis){
                    RATE = Math.sqrt((double)Math.abs(rightBack.getCurrentPosition()-rightStart)/aStartDis);
                    minPower=0.1;
                }else if( Math.abs(rightBack.getCurrentPosition()-newRightTarget) <= (int)aEndDis){
                    RATE = (double)Math.abs(rightBack.getCurrentPosition()-newRightTarget)/aEndDis;
                    minPower=0.02;
                }else RATE = 1.0;
                rightBack.setPower(DIR*Math.max(RATE*rightPower,minPower));
                rightFront.setPower(DIR*Math.max(RATE*rightPower,minPower));
                leftFront.setPower(-DIR*Math.max(RATE*rightPower,minPower));
                leftBack.setPower(-DIR*Math.max(RATE*rightPower,minPower));
                telemetry.addData("【前后】编码器","起始:%d,当前:%d,目标:%d",
                        rightStart,rightBack.getCurrentPosition(),newRightTarget);
                telemetry.addData("当前","系数：%.2f,右轮功率：%.2f",RATE,DIR*Math.max(RATE*rightPower,minPower));
                telemetry.update();
            }if(!opModeIsActive())return;
            rightBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            leftBack.setPower(0);
            sleep(250);
        }
    }
    private void encoderTurn(double rightPower,int Target,double timeoutS){
        if(!opModeIsActive())return;
        Target *= 2;
        int DIR=Target>0?1:-1;
        int Start = rightBack.getCurrentPosition()+leftFront.getCurrentPosition();
        runtime.reset();
        while( opModeIsActive() && Math.abs(rightBack.getCurrentPosition()+
                leftFront.getCurrentPosition()-Start)<=Math.abs(Target) && runtime.seconds()<=timeoutS){
            rightBack.setPower(DIR*rightPower);
            rightFront.setPower(DIR*rightPower);
            leftBack.setPower(DIR*rightPower);
            leftFront.setPower(DIR*rightPower);
        }
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
        sleep(250);
    }
    private void timerDrive(double leftPower , double rightPower , double timeoutS){
        runtime.reset();
        while(opModeIsActive()&&runtime.seconds() < timeoutS){
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);
        }
    }
    private int runVuforia(double sleepS,double timeoutS){
        runtime.reset();
        while( opModeIsActive() && runtime.seconds()<timeoutS ){
            Img = RelicRecoveryVuMark.from(relicTemplate);                          //timeoutS秒钟无结果则跳过
            if( Img == RelicRecoveryVuMark.LEFT ){
                telemetry.addData( "图像识别" , "左" );
                telemetry.update();
                if(runtime.seconds()>sleepS)return 0;
            }else if( Img == RelicRecoveryVuMark.CENTER ){
                telemetry.addData( "图像识别" , "中" );
                telemetry.update();
                if(runtime.seconds()>sleepS)return 1;
            }else if( Img == RelicRecoveryVuMark.RIGHT ){
                telemetry.addData( "图像识别" , "右" );
                telemetry.update();
                if(runtime.seconds()>sleepS)return 2;
            }else{
                telemetry.addData("图像识别","正在识别中……");
                telemetry.update();
            }
        }if( Img == RelicRecoveryVuMark.UNKNOWN ){
            telemetry.addData("图像识别","3秒没有结果，强制使用中方案");
            telemetry.update();
            return 1;
        }
        return 1;
    }
    private void HitBall(COLOR color,double timeoutS){
        runtime.reset();
        while(opModeIsActive()){
            Color.RGBToHSV ((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            telemetry.addData("识别到的颜色(A,R,G,B)","(%d,%d,%d,%d)",
                    sensorRGB.alpha(), sensorRGB.red(),
                    sensorRGB.green(), sensorRGB.blue());
            telemetry.addData("Hue的值", hsvValues[0]);
            telemetry.update();
            if(hsvValues[0]>350 || hsvValues[0]<31){  //红色球
                telemetry.addData("Hue的值", hsvValues[0]);
                telemetry.addData("识别到的颜色为","红色");
                telemetry.update();
                if(color==COLOR.RED)ballHand.setPosition(0.85);//打掉红色球
                if(color==COLOR.BLUE)ballHand.setPosition(0.15);//打掉蓝色球
                sleep(1000);
                ballArm.setPosition(0.12); //收起
                break;
            }else if(hsvValues[0]>170 && hsvValues[0]<220){    //蓝色球
                telemetry.addData("Hue的值", hsvValues[0]);
                telemetry.addData("识别到的颜色为","蓝色");
                telemetry.update();
                if(color==COLOR.RED)ballHand.setPosition(0.15);//打掉红色球
                if(color==COLOR.BLUE)ballHand.setPosition(0.85);//打掉蓝色球
                sleep(1000);
                ballArm.setPosition(0.12); //收起
                break;
            }else if(runtime.seconds()>=timeoutS){ //Cannot have analysis of ball color
                telemetry.addData("Hue的值", hsvValues[0]);
                telemetry.addData("颜色识别","超过2秒没有结果，跳过");
                telemetry.update();
                ballArm.setPosition(0.12);
                break;
            }
        }
    }
    private void Blue1(int Dist){
        HitBall(COLOR.RED,2);
        if(!opModeIsActive())return;

        timerDrive(0.3,-0.3,1.0);
        timerDrive(0.0,0.0,0.3);
        timerDrive(-0.2,0.2,1.4);
        timerDrive(0.0,0.0,0.2);
        timerDrive(0.1,-0.1,0.3);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int runDistance = 1800;
        if(Dist==0)runDistance-=900;
        if(Dist==2)runDistance+=900;
        encoderDrive(0.40,-runDistance,500,500,5);
        encoderTurn(0.2,-2400,5);
        encoderDrive(0.30,900,400,400,4);

        frontDrive.setPower(-0.8);
        sleep(800);
        frontDrive.setPower(0);

        encoderDrive(0.25,-1000,200,500,8);
        encoderTurn(0.35,-4750,8);
        timerDrive(0.25,-0.25,0.8);
        timerDrive(0,0,0.4);
        timerDrive(-0.2,0.2,0.25);
    }
    private void Blue2(int Dist){
        HitBall(COLOR.RED,2);
        if(!opModeIsActive())return;

        timerDrive(0.3,-0.3,1.0);
        timerDrive(0.0,0.0,0.3);
        timerDrive(-0.2,0.2,1.4);
        timerDrive(0.0,0.0,0.2);
        timerDrive(0.1,-0.1,0.3);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int runDistance = 1100;
        if(Dist==0)runDistance-=920;
        if(Dist==2)runDistance+=920;

        encoderDrive(0.2,-800,50,50,8);
        encoderTurn(0.2,-2400,5);
        encoderDrive(0.3,-runDistance,50,50,8);
        encoderTurn(0.2,-2400,5);
        encoderDrive(0.30,750,200,200,4);

        frontDrive.setPower(-0.8);
        sleep(800);
        frontDrive.setPower(0);

        encoderDrive(0.25,-800,200,500,8);
        encoderTurn(0.35,-4750,8);
        timerDrive(0.25,-0.25,0.9);
        timerDrive(0,0,0.4);
        timerDrive(-0.2,0.2,0.25);

    }
    private void Red1(int Dist){
        HitBall(COLOR.BLUE,2);
        if(!opModeIsActive())return;

        timerDrive(-0.3,0.3,1.0);
        timerDrive(0.0,0.0,0.3);
        timerDrive(0.2,-0.2,1.4);
        timerDrive(0.0,0.0,0.2);
        timerDrive(-0.1,0.1,0.3);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int runDistance = 1990;
        if(Dist==0)runDistance+=900;
        if(Dist==2)runDistance-=900;
        encoderDrive(0.40,runDistance,500,500,5);
        encoderTurn(0.2,-2400,5);
        encoderDrive(0.30,750,300,300,4);

        frontDrive.setPower(-0.8);
        sleep(800);
        frontDrive.setPower(0);

        encoderDrive(0.25,-1000,200,500,8);
        encoderTurn(0.35,-4750,8);
        timerDrive(0.25,-0.25,0.8);
        timerDrive(0,0,0.4);
        timerDrive(-0.2,0.2,0.25);
    }
    private void Red2(int Dist){
        HitBall(COLOR.BLUE,2);
        if(!opModeIsActive())return;

        timerDrive(-0.3,0.3,1.0);
        timerDrive(0.0,0.0,0.3);
        timerDrive(0.2,-0.2,1.4);
        timerDrive(0.0,0.0,0.2);
        timerDrive(-0.1,0.1,0.3);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int runDistance = 1500;
        if(Dist==0)runDistance+=900;
        if(Dist==2)runDistance-=900;

        encoderDrive(0.2,800,50,50,8);
        encoderTurn(0.2,-2400,5);
        encoderDrive(0.3,-runDistance,50,50,8);
        encoderTurn(0.2,2400,5);
        encoderDrive(0.30,300,120,120,4);

        frontDrive.setPower(-0.8);
        sleep(800);
        frontDrive.setPower(0);

        encoderDrive(0.25,-800,200,500,8);
        encoderTurn(0.35,-4750,8);
        timerDrive(0.25,-0.25,0.9);
        timerDrive(0,0,0.4);
        timerDrive(-0.2,0.2,0.25);
    }
    @Override
    public void runOpMode() {

        int mode = chooseMode();    //等待手柄选择起始位置并返回
        Init();                     //初始化hardwareMap
        InitVuforia();              //加载Vuforia
        telemetry.addData("初始化完毕","可以开始");
        telemetry.update();
        waitForStart();             //等待按下开始键

        ballHand.setPosition(0.48); //放下拨球摆杆
        ballArm.setPosition (0.90); //放下拨球摆杆
        int ImgResult=runVuforia(0.8,3);//执行图像识别并返回结果
        if(!opModeIsActive())return;//防止程序无法退出

        //根据机器起始平衡板、图像识别结果执行对应程序
        if(mode==1)Blue1(ImgResult);
        if(mode==2)Blue2(ImgResult);
        if(mode==3)Red1(ImgResult);
        if(mode==4)Red2(ImgResult);

        //确保机器停下
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
}