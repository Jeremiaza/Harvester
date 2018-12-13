package robot;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
/**
 * Waits for data received from the JavaFX class. Currently only a Waypoint is received.
 * Starts an arbitrator with all our behaviors.
 * Socket is 1111 to connect the EV3. DataStream to transfer data.
 * See leJOS API for more information
 * http://www.lejos.org/ev3/docs/lejos/robotics/subsumption/Behavior.html
 * 
 * @author Jeremias Korhonen
 *
 */
public class Data {
	public static void main(String [] args) throws DestinationUnreachableException {
		DataInputStream in = null;
		DataOutputStream out = null;
		Socket s = null;
		ServerSocket server = null;
		
		try {
			System.out.println("Waiting for data...");
			server = new ServerSocket(1111);
			s= server.accept();
			in = new DataInputStream(s.getInputStream());
			out = new DataOutputStream(s.getOutputStream());
		} catch (IOException e) {
			e.printStackTrace();
		}
		Waypoint point = new Waypoint(0,0);
		try {
			point.loadObject(in);
		} catch (IOException e1) {
			e1.printStackTrace();
		}
		Sensors sensor = new Sensors();
		sensor.asetaWP(point);
		Behavior b1 = new Movement(sensor);
		Behavior b2 = new Camera(sensor);
		Behavior b3 = new Action(sensor);
		Behavior b4 = new Stop(sensor);
		Behavior b5 = new Loop(sensor);
		Behavior b6 = new Release(sensor);
		Behavior [] bArray = {b1,b5,b6,b3,b2,b4};
		Arbitrator arby = new Arbitrator(bArray);
		try {
			s.close();
			in.close();
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		arby.go();
		
		
		
	}
}
