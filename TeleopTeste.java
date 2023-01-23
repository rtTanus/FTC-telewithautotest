package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOpInicial", group="OpMode")
public class TeleopTeste extends OpMode{
    public Servo servoMotor;
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public DcMotor Arm;

    @Override
    public void init(){
        motorEsquerdoF  = hardwareMap.get(DcMotor.class, "leftDriveUp");
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

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        servoMotor = hardwareMap.get(Servo.class, "Arm");

        resetRuntime();
    }
    public void loop(){
        linear();
        servo();
        mover();
    }
    public void mover(){
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);


        double motorEsquerdoFf  = axial + lateral + yaw / denominador;
        double motorDiretoFf = axial - lateral - yaw / denominador;
        double motorEsquerdoTf   = axial - lateral + yaw / denominador;
        double motorDireitoTf  = axial + lateral - yaw / denominador;

        double motorEsquerdoFt  = axial - lateral - yaw / denominador;
        double motorDiretoFt = axial + lateral + yaw / denominador;
        double motorEsquerdoTt   = axial + lateral - yaw / denominador;
        double motorDireitoTt  = axial - lateral + yaw / denominador;

        if (!gamepad1.a){
            allMotorsPower(motorEsquerdoFf,motorDiretoFf,motorEsquerdoTf,motorDireitoTf);
        }
        else if(gamepad1.a){
            allMotorsPower(motorEsquerdoFt,motorDiretoFt,motorEsquerdoTt,motorDireitoTt);
        }


        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFf);
        telemetry.addData("A potencia do motorDireitoF é de:", motorDiretoFf);
        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoTf);
        telemetry.addData("A potencia do motorEsquerdoF é de:", motorDireitoTf);
        telemetry.addData("O tempo em que o robô opera é de :", getRuntime());
        telemetry.update();
    }
    public void linear() {
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean powMax = gamepad2.x;
        boolean powMin = gamepad2.y;
        double pow = 0;

        while (poderCima) {
            pow = pow + 0.1;
            Arm.setPower(pow);
            if (pow >= 1) {
                pow = 1;
                Arm.setPower(pow);
            }
        }
        while (poderBaixo) {
            pow = pow - 0.1;
            Arm.setPower(pow);
            if (pow <= -1) {
                pow = -1;
                Arm.setPower(pow);
            }
        }
        if (poderCima && powMax) {
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
        telemetry.update();
    }
    public void servo() {
        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double powServo;
        double aberto = 0;
        double fechado = 1;

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);
            telemetry.addData("A potencia do motor do servo é de:", powServo);
            telemetry.update();
        }
        else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);
            telemetry.addData("A potencia do motor do servo é de:", powServo);
            telemetry.update();
        }
    }
    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
}
