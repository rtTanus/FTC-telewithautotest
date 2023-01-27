package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOpBetter", group="OpMode")
public class TeleOpBetter extends OpMode{
    public Servo servoMotor;
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public DcMotor Arm;
    double powServo;

    @Override
    public void init(){
        motorEsquerdoF  = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF  = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        motorEsquerdoF.setDirection(DcMotor.Direction.FORWARD);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.REVERSE);

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        servoMotor = hardwareMap.get(Servo.class, "Servo");

        resetRuntime();
    }
    public void loop(){
        linear();
        servo();
        mover();
        telemetry.update();
        telemetry.addData("O tempo em que o robô opera é de :", getRuntime());
    }
    @SuppressLint("SuspiciousIndentation")
    public void mover(){
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        double veltonotturb = 0.3;
        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        boolean boostpower = gamepad1.right_bumper;


        double motorEsquerdoFf  = (axial + lateral + yaw / denominador) - veltonotturb;
        double motorDiretoFf = (axial - lateral - yaw / denominador) - veltonotturb;
        double motorEsquerdoTf   = (axial - lateral + yaw / denominador) - veltonotturb;
        double motorDireitoTf  = (axial + lateral - yaw / denominador) - veltonotturb;


        if (!boostpower)

            allMotorsPower(motorEsquerdoFf,motorDiretoFf,motorEsquerdoTf,motorDireitoTf);

            telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFf);
            telemetry.addData("A potencia do motorDireitoF é de:", motorDiretoFf);
            telemetry.addData("A potencia do motorEsquerdoT é de:", motorEsquerdoTf);
            telemetry.addData("A potencia do motorDireitoT é de:", motorDireitoTf);
        if (boostpower){

             double motorEsquerdoFfr  = axial + lateral + yaw / denominador;
             double motorDiretoFfr = axial - lateral - yaw / denominador;
             double motorEsquerdoTfr   = axial - lateral + yaw / denominador;
             double motorDireitoTfr  = axial + lateral - yaw / denominador;

             allMotorsPower(motorEsquerdoFfr,motorDiretoFfr,motorEsquerdoTfr,motorDireitoTfr);

             telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFfr);
             telemetry.addData("A potencia do motorDireitoF é de:", motorDiretoFfr);
             telemetry.addData("A potencia do motorEsquerdoT é de:", motorEsquerdoTfr);
             telemetry.addData("A potencia do motorDireitoT é de:", motorDireitoTfr);
             
            }
        }

    public void linear() {
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean powMax = gamepad2.x;
        boolean powMin = gamepad2.y;
        double pow = 0;

        if (poderCima) {
            pow = pow + 0.1;
            Arm.setPower(pow);
            if (pow >= 1) {
                pow = 1;
                Arm.setPower(pow);
            }
        }
        else if (poderBaixo) {
            pow = pow - 0.1;
            Arm.setPower(pow);
            if (pow <= -1) {
                pow = -1;
                Arm.setPower(pow);
            }
        }
        else if (poderCima && powMax) {
            pow = 1;
            Arm.setPower(pow);
        }
        else if(poderBaixo && powMax) {
            pow = -1;
            Arm.setPower(pow);
        }
        else if (poderCima && powMin){
            pow = 0.1;
            Arm.setPower(pow);
        }
        else if (poderBaixo && powMin){
            pow = -0.1;
            Arm.setPower(pow);
        }
        telemetry.addData("A potencia do motor do sistema linear é de", pow);

    }
    public void servo() {
        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double aberto = 0;
        double fechado = 1;

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);
        }
        else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);
        }
        telemetry.addData("A potencia do motor do servo é de:", powServo);
    }
    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
}
