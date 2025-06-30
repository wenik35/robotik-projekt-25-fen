"turtlebot_pastry" (PArtial Self TRYving) ist ein Package für ROS2 Humble, mit dem ein TurtleBot3 (Modell Burger) einen Straßenparkour bewältigen soll. Dieser Turtlebot ist mit LIDAR-Scanner auf dem Kopf und einer 640p Kamera ausgestattet und muss im Parkour unter anderem Straßenzüge, Hindernisse, Ampeln und Kreuzungen bewältigen können. Näheres zu den Anforderungen ist in den Milestones festgehalten.

---

Um dieses Package laufen zu lassen, benötigt man erst ein Ubuntu 22.04 System, auf dem [ROS2 Humble Hawksbill installiert]("https://docs.ros.org/en/humble/Installation.html") ist.

Zuerst muss ein Workspace angelegt werden:
```
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

In den src-Ordner wird dann dieses Repository geclont:
```
git clone https://github.com/wenik35/robotik-projekt-25-fen.git -b humble
```

Anschließend müssen noch vom root des Workspaces aus Dependencys installiert werden:
```
rosdep install -i --from-path ./ --rosdistro humble -y
```

Nun kann das package gebaut werden (wenn --packages-select weggelassen wird, werden alle Packages im Workspace gebaut):
```
colcon build --packages-select turtlebot_pastry
```

In einem neuen Terminal (im root des Workspaces) kann jetzt das Package gesourcet und ausgeführt werden:
```
source install/setup.bash
ros2 launch turtlebot_pastry latest.py
```