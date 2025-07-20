# Turtlebot Pastry

"turtlebot_pastry" (PArtial Self TRYving) ist ein Package für ROS2 Humble, mit dem ein turtlebot einen Straßenparkour bewältigen soll. Der Turtlebot (Modell Hamburger) ist mit LIDAR-Scanner auf dem Kopf und einer 640p Kamera ausgestattet. Der Parkour enthält unter anderem Straßenzüge, Hindernisse, Ampeln und Kreuzungen.

Näheres zu den Anforderungen ist in den [Milestones]("/milestones") festgehalten.

Eine genauere Erklärung aller Nodes ist im [Wiki]("/wiki") zu finden.

---

## Installation

Um dieses Package laufen zu lassen, benötigt man erst ein Ubuntu 22.04 System, auf dem [ROS2 Humble Hawksbill installiert]("https://docs.ros.org/en/humble/Installation.html") ist.

Zuerst muss ein Workspace angelegt und das Projekt geclont werden:
```
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/wenik35/robotik-projekt-25-fen.git -b humble
```

Anschließend müssen noch vom root des Workspaces aus Dependencys installiert werden, dann kann  das package gebaut werden (wenn --packages-select weggelassen wird, werden alle Packages im Workspace gebaut):
```
rosdep install -i --from-path ./ --rosdistro humble -y
colcon build --packages-select turtlebot_pastry
```

In einem neuen Terminal (im root des Workspaces) kann jetzt das Package gesourcet und ausgeführt werden:
```
source install/setup.bash
ros2 launch turtlebot_pastry TrafficLightLaunch.py
```
um alle Nodes zu starten, oder
```
ros2 launch turtlebot_pastry latest.py
```
um den Roboter auch ohne grünes Licht losfahren zu lassen.
