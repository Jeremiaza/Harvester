import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.navigation.Waypoint;

public class Data extends Application{
	private Socket s;
	private DataInputStream in;
	private DataOutputStream out;
	public Data() {
		try {
			
			s = new Socket("10.0.1.1", 1111);
			in = new DataInputStream(s.getInputStream());
			out = new DataOutputStream(s.getOutputStream());
			
			
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void start(Stage primaryStage) throws IOException {
		// TODO Auto-generated method stub
		BorderPane root = new BorderPane();
		Button btn = new Button("Tilttaa");
		
		btn.setOnAction(new EventHandler<ActionEvent>() {
			@Override
			public void handle(ActionEvent event) {
				Waypoint point = new Waypoint(250,150);
				try {
					point.dumpObject(out);
					out.flush();
					out.close();
					s.close();
					in.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		
		BorderPane.setAlignment(btn, Pos.CENTER);
		root.setCenter(btn);
		
		primaryStage.setScene(new Scene(root, 300, 150));
		primaryStage.show();
		
		
		
	}
	
	
	
	public static void main(String[] args) {
		launch(args);
	}

}
