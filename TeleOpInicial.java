package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TesteTO", group="OpMode")
public class TeleopTeste extends OpMode{
    public Servo servoMotor = null;
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    public DcMotor Arm = null;

    @Override
    public void init(){
        motorEsquerdoF  = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF  = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");

        motorEsquerdoF.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.FORWARD);

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        servoMotor = hardwareMap.get(Servo.class, "Servo");


    }

    public void loop(){
        servo();
        mover();
        linear();
    }

    public void mover(){

        double axial   = gamepad1.right_trigger - gamepad1.left_trigger;
        double lateral = gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);


        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        allMotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);

        if(gamepad1.dpad_down){
            allMotorsPower(-0.35,-0.35,-0.35,-35);
        }
        if(gamepad1.dpad_up){
            allMotorsPower(0.35,0.35,0.35,0.35);
        }
        if(gamepad1.dpad_right){
            allMotorsPower(0.35,-0.35,-0.35,0.35);
        }
        if(gamepad1.dpad_left){
            allMotorsPower(-0.35,0.35,0.35,-0.35);
        }

        telemetry.addData("A potencia do motorEsquerdoF é de:", motorEsquerdoFf);
        telemetry.addData("A potencia do motorDireitoF é de:", motorDireitoFf);
        telemetry.addData("A potencia do motorEsquerdoT é de:", motorEsquerdoTf);
        telemetry.addData("A potencia do motorDireitoT é de:", motorDireitoTf);

    }


    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        motorEsquerdoF.setPower(paMEF);
        motorDireitoF.setPower(paMDF);
        motorEsquerdoT.setPower(paMET);
        motorDireitoT.setPower(paMDT);
    }
    public void linear() {
        double poderCima = gamepad2.right_trigger;
        double poderBaixo = gamepad2.left_trigger;
        double pow = poderCima - poderBaixo;

        Arm.setPower(pow);

        if(gamepad1.right_bumper){
            Arm.setPower(0.5);
        }
        if(gamepad1.left_bumper){
            Arm.setPower(-0.1);
        }

        telemetry.addData("A potencia do motor do sistema linear é de", pow);

    }
    double powServo = 0;

    public void servo() {
        boolean poderAberto = gamepad2.a;
        boolean poderFechado = gamepad2.b;
        double aberto = 0;
        double fechado = 1;

        if (poderAberto) {
            powServo = aberto;
            servoMotor.setPosition(powServo);
            telemetry.addData("A potencia do motor do servo é de:", powServo);
        }
        else if (poderFechado) {
            powServo = fechado;
            servoMotor.setPosition(powServo);
            telemetry.addData("A potencia do motor do servo é de:", powServo);
        }
        else{
            telemetry.addData("A potencia do motor do servo é de:", powServo);
        }
    }
}

