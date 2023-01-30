package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AutonomoTeste", group = "LinearOpMode")
@Disabled
public class autonomous extends LinearOpMode{
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public static PIDCoefficients pidcoeffs1, pidcoeffs2, pidcoeffs3,pidcoeffs4 = new PIDCoefficients(0,0,0);
    public  PIDCoefficients pidGains1, pidGains2, pidGains3, pidGains4 = new PIDCoefficients(0,0,0);
    public ElapsedTime TempoPID = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



        @Override
        public void runOpMode() {
            motorEsquerdoF = hardwareMap.get(DcMotor.class, "leftDriveUp");
            motorDireitoF = hardwareMap.get(DcMotor.class, "RightDriveUp");
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

            if (opModeIsActive()) {
                while(opModeIsActive() && TempoPID.time() >= 30000){
                    PID(1,-1,1,-1);
                    while (opModeIsActive() && motorDireitoF.isBusy() && motorDireitoT.isBusy() && motorEsquerdoF.isBusy() && motorEsquerdoT.isBusy()) {
                        idle();
                    }
                }
            }
        }

            public void allMotorsPower ( double paMEF, double paMDF, double paMET, double paMDT){
                motorEsquerdoF.setPower(paMEF);
                motorDireitoF.setPower(paMDF);
                motorEsquerdoT.setPower(paMET);
                motorDireitoT.setPower(paMDT);
            }
            double ultimoerro1 = 0;
            double ultimoerro2 = 0;
            double ultimoerro3 = 0;
            double ultimoerro4 = 0;
            double errosomado1 = 0;
            double errosomado2 = 0;
            double errosomado3 = 0;
            double errosomado4 = 0;

            public void PID ( double velocidade1, double velocidade2, double velocidade3,
            double velocidade4){
                double velocidadeatualMEF = motorEsquerdoF.getPower();
                double velocidadeatualMDF = motorDireitoF.getPower();
                double velocidadeatualMET = motorEsquerdoT.getPower();
                double velocidadeatualMDT = motorDireitoT.getPower();

                double erro1 = velocidade1 - velocidadeatualMEF;
                double erro2 = velocidade2 - velocidadeatualMDF;
                double erro3 = velocidade3 - velocidadeatualMET;
                double erro4 = velocidade4 - velocidadeatualMDT;

                double errocomerro1 = erro1 - ultimoerro1;
                double errocomerro2 = erro2 - ultimoerro2;
                double errocomerro3 = erro3 - ultimoerro3;
                double errocomerro4 = erro4 - ultimoerro4;

                double errocausado1 = errocomerro1 / TempoPID.time();
                double errocausado2 = errocomerro2 / TempoPID.time();
                double errocausado3 = errocomerro3 / TempoPID.time();
                double errocausado4 = errocomerro4 / TempoPID.time();

                errosomado1 += erro1 * TempoPID.time();
                errosomado2 += erro2 * TempoPID.time();
                errosomado3 += erro3 * TempoPID.time();
                errosomado4 += erro4 * TempoPID.time();

                pidGains1.p = erro1 * pidcoeffs1.p;
                pidGains2.p = erro2 * pidcoeffs2.p;
                pidGains3.p = erro3 * pidcoeffs3.p;
                pidGains4.p = erro4 * pidcoeffs4.p;

                pidGains1.i = errocomerro1 * pidcoeffs1.i;
                pidGains2.i = errocomerro2 * pidcoeffs2.i;
                pidGains3.i = errocomerro3 * pidcoeffs3.i;
                pidGains4.i = errocomerro4 * pidcoeffs4.i;

                pidGains1.d = errocausado1 * pidcoeffs1.d;
                pidGains2.d = errocausado2 * pidcoeffs2.d;
                pidGains3.d = errocausado3 * pidcoeffs3.d;
                pidGains4.d = errocausado4 * pidcoeffs4.d;

                allMotorsPower(pidGains1.p + pidGains1.i + pidGains1.d + velocidade1, pidGains2.p + pidGains2.i + pidGains2.d + velocidade2, pidGains3.p + pidGains3.i + pidGains3.d + velocidade3, pidGains4.p + pidGains4.i + pidGains4.d + velocidade4);

                ultimoerro1 = erro1;
                ultimoerro2 = erro2;
                ultimoerro3 = erro3;
                ultimoerro4 = erro4;


            }
        }

