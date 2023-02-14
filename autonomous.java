package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "AutonomoTeste", group = "LinearOpMode")

public class autonomous extends LinearOpMode{
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;


    @Override
    public void runOpMode() {
        motorEsquerdoF = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        motorEsquerdoF.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.FORWARD);

        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {

                allMotorsPower(0.4,0.4,0.4,0.4);
                sleep(1000);


                while (opModeIsActive() && motorDireitoF.isBusy() && motorDireitoT.isBusy() && motorEsquerdoF.isBusy() && motorEsquerdoT.isBusy()) {
                    idle();

            }
        }
    }

    public void allMotorsPower ( double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
}
