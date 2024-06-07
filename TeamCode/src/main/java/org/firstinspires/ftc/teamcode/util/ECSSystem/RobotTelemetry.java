package org.firstinspires.ftc.teamcode.util.ECSSystem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotation to mark a field for Robot Telemetry display.
 * The annotated field's value will be displayed in the robot telemetry during operation.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface RobotTelemetry {
}
