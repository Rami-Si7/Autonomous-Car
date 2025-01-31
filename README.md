# Autonomous Car Project
In this project, we developed a simulation for autonomous driving using the Godot game engine. The system integrates state-of-the-art machine learning and computer vision techniques, including the Nvidia Dave-2 model for steering prediction  and YOLO(You Only Look Once) for object detection. The simulation consists of seven tracks designed to test various autonomous driving capabilities, such as lane merging, speed sign detection, obstacle avoidance, lane change and autonomous parking. The results of our test tracks showed great performance although the server side ran on CPU, with hardware which has a GPU compatible with Cuda, the performance will be better.

Here is and over view of the system
![System Digram](system_images/systemDiagram.png)

As you can see we used multiple machine learinig, image proccessing and computer vision models to implement such system.

We used Nvidia Dave-2 model for steering command prediction, we basicly trained a CNN to predict steering command, Data was collected using our simulation in Godot game Engine.
To connect between the worlds of AI models which all in pyhton and simulation which in C# we used Server-Client approach.
We ran a Server using Socketio and eventlet. The client(simulation) send requests to the server like steering command prediction and YOLO object detection. Server proccess this requests by running the relevant model for each request. When a result is aquired, the server will send back to the client, then changes we be applied.

For more information about the funcuality of each subsystem, you can check the report.pdf.


## Run Project
There is 2 parts of running the project:
- Run the server:
       - To run the server, dependices file must be installed, requirments.txt
- Run the simulation:
       - To, lanuch the simulation, you will need to download Godot Mono using this [link for mac](https://godotengine.org/download/macos/), [link for windows](https://godotengine.org/download/windows/)
       - Then you will need to download this github repository on your computer, and then open godot using the downloaded repository




       
       
