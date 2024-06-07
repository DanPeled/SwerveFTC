#if (${PACKAGE_NAME} && ${PACKAGE_NAME} != "")package ${PACKAGE_NAME};#end
#parse("File Header.java")
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.RobotTelemetry;


@TeleOp
public class ${NAME} extends Robot {
    @Override
    public void initRobot() {
    }
    
    @Override
    public void startRobot() {
    }
    
    @Override
    public void updateLoop() {
    }
}