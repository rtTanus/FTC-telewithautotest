package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AutonomoTeste", group = "LinearOpMode")
@Disabled
public class autonomous extends LinearOpMode{
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;

        @Override
        public void runOpMode(){
        motorEsquerdoF  = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF  = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        motorEsquerdoF.setDirection(DcMotor.Direction.FORWARD);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.REVERSE);

        motorEsquerdoF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitoF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdoT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitoF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetRuntime();
        waitForStart();

    }
        public void climbOpMode(){
        allMotorsPower(1,1,1,1);
        sleep(1000);
        allMotorsPower(0,0,0,0);
    }

    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
}

