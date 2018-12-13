package robot;

import lejos.robotics.subsumption.Behavior;
/**
 * This class implements the Behavior interface and controls the safety mechanism of the robot
 * @author Jeremias Korhonen
 *
 */
public class Stop implements Behavior{
	private Sensors sensor;
	private volatile boolean suppressed = false;
	public static int seis = 0;
	
	
	public Stop(Sensors sensor) {
		this.sensor = sensor;
	}
	/**
	 * This behavior takes control after the exit button has been pressed
	 * @return True, if the button has been pressed
	 */
	@Override
	public boolean takeControl() {
		return sensor.painettu();
	}
	/**
	 * Shuts down the robot
	 */
	@Override
	public void action() {
		System.exit(0);
		
	}
	@Override
	public void suppress() {
		suppressed = true;
		
	}
}
