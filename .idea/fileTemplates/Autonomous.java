#if (${PACKAGE_NAME} && ${PACKAGE_NAME} != "")package ${PACKAGE_NAME};#end
#parse("File Header.java")
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ${NAME} extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // INIT 
        
        waitForStart();
        // START
                
    }   
}