"turtlebot_pastry" (PArtial Self TRYving) ist ein Package für ROS2 Humble, mit dem ein turtlebot einen Straßenparkour bewältigen soll. Der Turtlebot (Modell Hamburger) ist mit LIDAR-Scanner auf dem Kopf und einer 640p Kamera ausgestattet. Der Parkour enthält unter anderem Straßenzüge, Hindernisse, Ampeln und Kreuzungen bewältigen können. Näheres zu den Anforderungen ist in den Milestones festgehalten.

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
```
und dann
```
ros2 launch turtlebot_pastry TrafficLightLaunch.py
```
um alle Nodes zu starten, oder
```
ros2 launch turtlebot_pastry latest.py
```
um alle Nodes außer trafficlight zu starten.

---

Im folgenden sind alle Nodes dieses Packages und ihre Funktion kurz aufgelistet:

<details>
<summary>Node-Dokumentation</summary>
<h2>stateMachine</h2>
Master-Node, die die Daten aller anderen Nodes managet und Fahrbefehle an den Roboter weiterleitet.
<h4>Publisher</h4>

* status: Gibt State-Updates als String aus
* cmd_vel: Fahrbefehle für den Roboter

<h4>Parameter</h4>

* force_stop: Verhindert, dass der Roboter fährt


<h2>detectObstacle</h2>
Nutzt den Laserscanner, um Hindernisse vor dem Roboter zu erkennen.
<h4>Publisher</h4>

- obstacle_in_path: Gibt einen entsprechenden Boolean aus, wenn ein Hindernis auftaucht oder entfernt wird

<h4>Parameter</h4>

- detection_distance: Distanz, ab der ein Objekt erkannt werden soll


<h2>followPath</h2>
Nutzt die Kamera, um in einer Spur zu fahren. Orientiert sich dabei an der rechten Begrenzungslinie.
<h4>Publisher</h4>

- follow_path_cmd: Fahrbefehle zur Vorwärtsbewegung innerhalb der Fahrbahn
<h4>Parameter</h4>

        self.declare_parameter('max_line_offset', 300)
        self.declare_parameter('steering_quotient', 10)
        self.declare_parameter('line_expected_at', 550)
        self.declare_parameter('speed_drive', 0.15)
        self.declare_parameter('canny_high', 600)
        self.declare_parameter('canny_low', 150)

- max_line_offset: Maximale Distanz in Pixeln, die die erkannte Linie von ihrer erwarteten Position abweichen darf.
- steering_quotient: Kleinere Werte lassen den Roboter schneller lenken.
- line_expected_at: Pixelindex, an dem die Linie erwartet wird.
- speed_drive: Geschwindigkeit, mit der der Roboter vorwärts fährt.
- canny_high: High-Parameter für den Canny-Algorithmus
- canny-low: Low-Parameter für den Canny-Algorithmus

## signRecognition
Nutzt die Kamera um Schilder zu erkennen
### Publisher

- sign_seen: Gibt erkanntest Schild als Integerwert zurück
  - 0: Parkplatz
  - 1: Geradeaus fahren
  - 2: Links abbiegen
  - 3: Rechts abbiegen
  - 4: Zebrastreifen

### Parameter

- lower_bound: Untergrenze Blauton in HSV
- upper_bound: Obergrenze Blauton in HSV
- scalar: Skalierungsfaktor
</details>
