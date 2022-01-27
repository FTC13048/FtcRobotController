private DcMotor linSlide;
private DcMotor intake;

protected Lift(HardwareMap hmap, Telemetry tele){
        super(tele);
        linSlide = hmap.get(DcMotor.class, "linSlide");

        linSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tele.addData("linear slide", "initialized");
        }

public void runLinSlide(double power) {
        linSlide.setPower(power);
        }

public int getLinSlidePos() {
        return linSlide.getCurrentPosition();
        }

public boolean setLinSlidePos(int numTicks){
        linSlide.setTargetPosition(numTicks);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runLinSlide(0.4);

        if(Math.abs(linSlide.getCurrentPosition() - numTicks) <= 5){
        runLinSlide(0.0);
        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return true;
        }

        return false;
        }