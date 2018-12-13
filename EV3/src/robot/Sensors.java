package robot;

import lejos.hardware.device.NXTCam;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.geometry.Rectangle2D;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.PathFinder;
import lejos.robotics.pathfinding.ShortestPathFinder;
/**
 * Class containing all the methods for the project.
 * @author Jeremias Korhonen
 *
 */
public class Sensors extends Thread {
	private int angle = 70, numObjects;
	/**
	 * Provides data from the IR- and touchsensor to use in our methods
	 */
	private SampleProvider distanceProvider, touchProvider;
	private float[] sample, sample2;
	/**
	 * Connect the EV3 sensorports.
	 */
	private Port irport, touchport, cameraport;
	/**
	 * Provide access to the modes supported by a sensor.
	 */
	private SensorModes irsensor, touchsensor;
	private NXTCam camera;
	/**
	 * Connect the EV3 motorports
	 */
	private RegulatedMotor cameramotor, koura, vasen, oikea;
	private Line[] linjat;
	/**
	 * Represents the location and heading(direction angle) of a robot.
	 * Used in conjunction with PoseProvider.
	 */
	private Pose pose;
	/**
	 * See leJOS API for details.
	 * http://www.lejos.org/nxt/nxj/api/lejos/robotics/navigation/Navigator.html
	 */
	private Navigator nav;
	/**
	 * See leJOS API for details.
	 * https://lejos.sourceforge.io/nxt/pc/api/lejos/robotics/pathfinding/PathFinder.html
	 */
	private PathFinder pathf;
	private PoseProvider poseProvider;
	/**
	 * A map of the room or other closed environment, represented by line segments.
	 */
	private LineMap kartta;
	private int camdir;
	private MovePilot robot;
	private Path polku;
	/**
	 * Represents the chassis of our wheeled robot.
	 * See leJOS API for more details. 
	 * http://www.lejos.org/ev3/docs/lejos/robotics/chassis/WheeledChassis.html
	 */
	private Chassis chassis;
	private Rectangle rajat;
	/**
	 * Here we specify the wheel diameter and distance from the center of the robot.
	 */
	private Wheel wheel1, wheel2;
	private boolean kasky = false;
	private boolean paluu = false;
	/**
	 * Constructor used in all of the Behavior classes.
	 * Sets up the motors, sensors, camera and navigator.
	 * Bounds for the map are currently 600x300.
	 * @throws DestinationUnreachableException
	 */
	public Sensors() throws DestinationUnreachableException {
		irport = LocalEV3.get().getPort("S2");
		touchport = LocalEV3.get().getPort("S4");
		cameraport = LocalEV3.get().getPort("S1");
		touchsensor = new EV3TouchSensor(touchport);
		irsensor = new EV3IRSensor(irport);
		cameramotor = new EV3MediumRegulatedMotor(MotorPort.A);
		touchProvider = ((EV3TouchSensor)touchsensor).getTouchMode();
		distanceProvider = ((EV3IRSensor)irsensor).getDistanceMode();
		
		koura = new EV3LargeRegulatedMotor(MotorPort.D);
		vasen = new EV3LargeRegulatedMotor(MotorPort.B);
		oikea = new EV3LargeRegulatedMotor(MotorPort.C);
		wheel1 = WheeledChassis.modelWheel(vasen, 3.2).offset(-6.24);
		wheel2 = WheeledChassis.modelWheel(oikea, 3.2).offset(6.24);
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		robot = new MovePilot(chassis);
		robot.setLinearSpeed(12); robot.setAngularSpeed(60);
		
		sample = new float[irsensor.sampleSize()];
		sample2 = new float[touchsensor.sampleSize()];
		camera = new NXTCam(cameraport);
		cameramotor.setSpeed(65);
		camera.sendCommand('U');
		camera.sendCommand('E');
		rajat = new Rectangle(0, 0, 600, 300);
		linjat = new Line[12];
		linjat [0] = new Line(0, 0, 600, 0);
		linjat [1] = new Line(0, 0, 0, 300);
		linjat [2] = new Line(600, 0, 600, 300);
		linjat [3] = new Line(0, 300, 600, 300);
		linjat [4] = new Line(150, 175, 200, 175);
		linjat [5] = new Line(150, 175, 150, 125);
		linjat [6] = new Line(200, 175, 200, 125);
		linjat [7] = new Line(150, 125, 200, 125);
		linjat [8] = new Line(350, 175, 400, 175);
		linjat [9] = new Line(350, 175, 350, 125);
		linjat [10] = new Line(400, 175, 400, 125);
		linjat [11] = new Line(350, 125, 400, 125);
		
		
		poseProvider = chassis.getPoseProvider();
		kartta = new LineMap(linjat, rajat);
		nav = new Navigator(robot, poseProvider);
		pathf = new ShortestPathFinder(kartta);
		pose = new Pose(10, 150, 0);
		System.out.println(pose);
		((ShortestPathFinder)pathf).lengthenLines(15);
		poseProvider.setPose(pose);
	}
	/**
	 * Closes and opens the grabber in set degrees.
	 * @param Angle in degrees. Negative to close and positive to open
	 */
	public void grab(int angle) {
		System.out.println("Grabbed");
		koura.rotateTo(angle);
	}
	/**
	 * Stops the robot, sets a new waypoint and calculates the path to it
	 * @param Waypoint
	 * @throws DestinationUnreachableException
	 */
	public void asetaWP(Waypoint point) throws DestinationUnreachableException {
		nav.stop();
		polku = pathf.findRoute(pose, point);
		nav.setPath(polku);
		System.out.println("Waypoint asetettu");
	}
	/**
	 * Commands the robot to follow the current path
	 * @throws DestinationUnreachableException if the location is out of bounds
	 */
	public void RobotMene() throws DestinationUnreachableException {
		nav.followPath(polku);
	}
	/**
	 * Calculates the x-coordinate of the scanned object
	 * Two calculations if the robot is moving to or from the x axis
	 * @return The calculated x-coordinate	
	 */
	public double laskeX() {
		if (paluu) {
			double x =  getX()-((Math.cos(Math.toRadians(getCamDir())))*80);
			return x;
		} else {
			double x =  getX()+((Math.cos(Math.toRadians(getCamDir())))*80);
			return x;
		}
	}
	/**
	 * Calculates the y-coordinate of the scanned object
	 * Two calculations if the robot is moving to or from the y axis
	 * @return The calculated y-coordinate	
	 */
	public double laskeY() {
		if (paluu) {
			double y = getY()-((Math.sin(Math.toRadians(getCamDir())))*80);
			return y;
		} else {
			double y = getY()+((Math.sin(Math.toRadians(getCamDir())))*80);
			return y;
		}
		
	}
	/**
	 * Sets a boolean value to true so we know, that the robot is returning
	 */
	public void Palaamassa() {
		paluu = true;
	}
	/**
	 * Rotates the camera's motor on a 140 degree angle
	 * @return True, if the camera detects an object
	 */
	public boolean Tutki() {
		
		if (cameramotor.getTachoCount() > 70) {
			angle = -70;
		}
		if (cameramotor.getTachoCount() < -70) {
			angle = 70;
		}
		cameramotor.rotate(angle, true);
		numObjects = camera.getNumberOfObjects();
		Rectangle2D rect = camera.getRectangle(numObjects);
		camdir = cameramotor.getTachoCount();
		if (numObjects >= 1 && rect.getHeight() >= 9 && rect.getCenterX() > 80 && rect.getCenterX() < 96) { 
			// Kameran resoluutio on 176x144, odotetaan kunnes kohde on kameran x-koordinaatin keskiosassa
			cameramotor.stop();
			robot.setLinearSpeed(18);
			System.out.println("Kohde loydetty asteessa " + camdir);
			return true;
		}
		return false;
	}
	/**
	 * Resets the camera's motor back to the starting position
	 */
	public void nollaaKamera() {
		cameramotor.rotate(camdir*-1);
		cameramotor.close();
	}
	/**
	 * Checks, if the robot reaches its end point
	 * @return A boolean value. True, if the robot has reached its end location
	 */
	public boolean onkoPaassa() {
		if (getX()>499 && getX()<501 && getY()>149 && getY()<151) {
			return true;
		}
		return false;
	}
	/**
	 * Sets a new location for the navigator
	 * @param X coordinate
	 * @param Y coordinate
	 * @throws InterruptedException
	 */
	public void asetaKohde(double dx, double dy) throws InterruptedException {
		nav.stop();
		Waypoint point = new Waypoint(dx, dy);
		try {
			System.out.println(poseProvider.getPose());
			polku = pathf.findRoute(poseProvider.getPose(), point);
			nav.setPath(polku);
			
		} catch (DestinationUnreachableException e) {
			e.printStackTrace();
		}
	}
	/**
	 * 
	 * Used in laskeX() and laskeY() methods
 	 * @return Cameramotor's direction in degrees
	 */
	public int getCamDir() {
		return camdir;
		
	}
	/**
	 * 
	 * @param Boolean value for the Camera class takeControl() method
	 */
	public void setKasky(boolean kasky) {
		this.kasky = kasky;
	}
	/**
	 * 
	 * @return Boolean value for the Camera class takeControl() method
	 */
	public boolean getKasky() {
		return kasky;
	}
	/**
	 * 
	 * @return Current location on the x axis using poseProvider
	 */
	public double getX() {
		return poseProvider.getPose().getX();
	}
	/**
	 * 
	 * @return Current location on the y axis using poseProvider
	 */
	public double getY() {
		return poseProvider.getPose().getY();
	}
	/**
	 * Uses the IR sensor to see, if there is anything in front of the robot
	 * @return Distance to the object
	 */
	public double haeTulos() {
		distanceProvider.fetchSample(sample, 0);
		return sample[0];
		
	}
	/**
	 * 
	 * Used in the Stop class takeControl() method
	 * @return True, if the emergency button has been pressed
	 */
	public boolean painettu() {
		touchProvider.fetchSample(sample2, 0);
		if (sample2[0] >=0.5) {
			return true;
		}
		return false;
	}
	/**
	 * 
	 * @return True, if an object is within a certain distance to the robot
	 */
	public boolean KohdeEdessa() {
		
		if (haeTulos() <= 7) {
    		return true;
    	}
    	return false;
	}
	
	
}
