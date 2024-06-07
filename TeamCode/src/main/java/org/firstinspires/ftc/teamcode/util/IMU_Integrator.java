package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * A class that integrates IMU data for accurate position tracking and telemetry visualization.
 */
public class IMU_Integrator implements BNO055IMU.AccelerationIntegrator {

    private final double tile = 0.6;
    private final double meters_to_inches = 39.37008;
    private volatile DcMotorEx fl = null;
    private volatile DcMotorEx fr = null;
    private volatile DcMotorEx bl = null;
    private volatile DcMotorEx br = null;
    private int fl_startPos = 0;
    private int fr_startPos = 0;
    private int bl_startPos = 0;
    private int br_startPos = 0;
    private final double forwardTicksPerMeter;
    private final double strafeTicksPerMeter;
    private final boolean useDashBoard;
    private ArrayList<Double> pathx;
    private ArrayList<Double> pathy;
    private long lastTimestamp = 0;

    private final Point origin; // Origin point of action
    private final Point direction; // Direction in x,y for dashboard

    private double angularOffset = 0;

    private BNO055IMU imu = null;
    private BNO055IMU.Parameters parameters = null;

    private Position position = new Position();
    private Velocity velocity = new Velocity();
    private Acceleration acceleration = null;

    /**
     * Constructs an IMU integrator with the specified parameters.
     *
     * @param imu                  The IMU sensor.
     * @param hw                   The hardware map.
     * @param forwardTicksPerMeter Ticks per meter for forward movement.
     * @param strafeTicksPerMeter  Ticks per meter for strafing movement.
     * @param useDashboard         Whether to use the dashboard for visualization.
     * @param origin               The origin point of action.
     * @param direction            The direction for the dashboard.
     * @param angularOffset        The angular offset.
     */
    public IMU_Integrator(BNO055IMU imu, HardwareMap hw, double forwardTicksPerMeter, double strafeTicksPerMeter, boolean useDashboard, Point origin, Point direction, double angularOffset) {
        this.imu = imu;
        this.fl = hw.get(DcMotorEx.class, "fl");
        this.fr = hw.get(DcMotorEx.class, "fr");
        this.bl = hw.get(DcMotorEx.class, "bl");
        this.br = hw.get(DcMotorEx.class, "br");
        this.origin = origin;
        this.direction = direction;
        this.forwardTicksPerMeter = forwardTicksPerMeter;
        this.strafeTicksPerMeter = strafeTicksPerMeter;
        this.useDashBoard = useDashboard;

        if (this.useDashBoard) {
            this.pathx = new ArrayList<>();
            this.pathy = new ArrayList<>();
        }
        this.angularOffset = angularOffset;
    }

    /**
     * Gets the current position.
     *
     * @return The current position.
     */
    public Position getPosition() {
        return this.position;
    }

    /**
     * Gets the current velocity.
     *
     * @return The current velocity.
     */
    public Velocity getVelocity() {
        return this.velocity;
    }

    /**
     * Gets the current acceleration.
     *
     * @return The current acceleration.
     */
    public Acceleration getAcceleration() {
        return this.acceleration;
    }

    /**
     * Gets the x-coordinate of the current position.
     *
     * @return The x-coordinate.
     */
    public double getX() {
        return position.x;
    }

    /**
     * Gets the y-coordinate of the current position.
     *
     * @return The y-coordinate.
     */
    public double getY() {
        return position.y;
    }

    /**
     * Gets the change in distance in the forward and strafe directions since the last update.
     *
     * @return The change in distance (forward, strafe).
     */
    public FS getDeltaDistance() {
        int fl_tick = fl.getCurrentPosition();
        int fr_tick = fr.getCurrentPosition();
        int bl_tick = bl.getCurrentPosition();
        int br_tick = br.getCurrentPosition();

        double fl_dist = fl_tick - fl_startPos;
        double fr_dist = fr_tick - fr_startPos;
        double bl_dist = bl_tick - bl_startPos;
        double br_dist = br_tick - br_startPos;

        double f = (bl_dist + br_dist + fr_dist + fl_dist) / forwardTicksPerMeter / 4;
        double s = (-bl_dist + br_dist - fr_dist + fl_dist) / strafeTicksPerMeter / 4;

        fl_startPos = fl_tick;
        fr_startPos = fr_tick;
        bl_startPos = bl_tick;
        br_startPos = br_tick;

        return new FS(f, s);
    }

    /**
     * Resets the robot's position to (0, 0).
     */
    public void resetPosition() {
        position.x = 0;
        position.y = 0;
    }

    /**
     * Initializes the IMU integrator with provided parameters.
     *
     * @param parameters      The IMU parameters.
     * @param initialPosition The initial position.
     * @param initialVelocity The initial velocity.
     */
    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = (initialPosition != null) ? initialPosition : this.position;
        this.velocity = (initialVelocity != null) ? initialVelocity : this.velocity;
        this.acceleration = null;

        if (this.useDashBoard) {
            Point p = transformDashboard(this.position);
            this.pathx.add(p.x);
            this.pathy.add(p.y);
        }
    }

    /**
     * Transforms the position for display on the dashboard.
     *
     * @param pos The position to transform.
     * @return The transformed point.
     */
    public Point transformDashboard(Position pos) {
        double x = Math.signum(direction.x) * (pos.x + origin.x) * meters_to_inches;
        double y = Math.signum(direction.y) * (pos.y + origin.y) * meters_to_inches;
        return new Point(x, y);
    }

    /**
     * Gets the heading angle of the robot.
     *
     * @return The heading angle.
     */
    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return -orientation.firstAngle + angularOffset;
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        if (this.useDashBoard && FtcDashboard.getInstance() != null && pathx.size() > 2) {
            Telemetry t = FtcDashboard.getInstance().getTelemetry();
            t.addData("[-2]", Arrays.toString(new double[]{pathx.get(pathx.size() - 2), pathy.get(pathy.size() - 2)}));
            t.addData("[-1]", Arrays.toString(new double[]{pathx.get(pathx.size() - 1), pathy.get(pathy.size() - 1)}));
        }

        FS delta = getDeltaDistance();
        double a = -getHeading() / 180.0 * Math.PI;

        this.position.x += delta.s * Math.cos(a) - delta.f * Math.sin(a);
        this.position.y += delta.f * Math.cos(a) + delta.s * Math.sin(a);

        if (this.useDashBoard && linearAcceleration.acquisitionTime - this.lastTimestamp >= 5000000L) {
            Point p = transformDashboard(this.position);
            double lastx = pathx.get(pathx.size() - 1);
            double lasty = pathy.get(pathy.size() - 1);
            if (Math.abs(lastx - p.x) > 1 || Math.abs(lasty - p.y) > 1) {
                pathx.add(p.x);
                pathy.add(p.y);

                TelemetryPacket packet = new TelemetryPacket();
                Canvas canvas = packet.fieldOverlay();

                canvas.setStroke("tomato");
                canvas.strokePolyline(toDoubleArray(pathx), toDoubleArray(pathy));
                Point o = transformDashboard(new Position());
                canvas.fillCircle(o.x, o.y, 3);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            this.lastTimestamp = linearAcceleration.acquisitionTime;
        }
    }

    private double[] toDoubleArray(ArrayList<Double> arr) {
        double[] a = new double[arr.size()];
        for (int i = 0; i < arr.size(); i++) {
            a[i] = arr.get(i);
        }
        return a;
    }

    private double[] to_d_katan(ArrayList<Double> arr) {
        double[] a = new double[arr.size()];
        for (int i = 0; i < arr.size(); i++) {
            a[i] = arr.get(i);
        }
        return a;
    }

    private class FS {
        public double f = 0;
        public double s = 0;

        FS(double f, double s) {
            this.f = f;
            this.s = s;
        }
    }
}
