# Turtlebot Pastry

"turtlebot_pastry" (PArtial Self TRYving) ist ein Package für ROS2 Humble, mit dem ein turtlebot einen Straßenparkour bewältigen soll. Der Turtlebot (Modell Hamburger) ist mit LIDAR-Scanner auf dem Kopf und einer 640p Kamera ausgestattet. Der Parkour enthält unter anderem Straßenzüge, Hindernisse, Ampeln und Kreuzungen bewältigen können. Näheres zu den Anforderungen ist in den Milestones festgehalten.

---

## Installation

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

# Node-Dokumentation

## stateMachine
Master-Node, die die Daten aller anderen Nodes managet und Fahrbefehle an den Roboter weiterleitet.

### Publisher
- status: Gibt State-Updates als String aus
- cmd_vel: Fahrbefehle für den Roboter
- overwright: Boolean der Befehlnodes unterbricht und zurücksetzt


### Parameter
- force_stop: Verhindert, dass der Roboter fährt


## changeLaneAtObstacle
Nutzt den Laserscanner, um Hindernisse vor dem Roboter zu erkennen.

### Publisher
- lane_change_in_process: Gibt einen entsprechenden Boolean aus, wenn ein Hindernis auftaucht und die Spur gewechselt wird
- change_lane: Fahrbefehle zum Spurwechsel

### Parameter
- detection_distance: Distanz, ab der ein Objekt erkannt werden soll


## followPath
Nutzt die Kamera, um in einer Spur zu fahren. Orientiert sich dabei an der rechten Begrenzungslinie.

### Publisher
- follow_path_cmd: Fahrbefehle zur Vorwärtsbewegung innerhalb der Fahrbahn

### Parameter

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

## parking
Parkt den Roboter in der ersten lehren Parklücke (max. 3)

### Publisher

- parking_in_progress: Booleanwert ob der Roboter parkbereit ist
- parking_cmd: Fahrbefehle zum einparken

### Parameter

- followPath Parameter für die Parklinienerkennung
- deadreconing_time: Fahrzeit zwischen Parklücken

## crossing
Navigiert Kreuzung

### Publisher
- crossing_in_progress: Booleanwert ob der Roboter die Kreuzung überquert
- crossing_cmd: Fahrbefehle zum überqueren der Kreuzung

### Parameter
- left_time: Fahrzeit für das Linksabbiegen
- straight_time: Fahrzeit für das geradeaus fahren
- right_time: Fahrzeit für das Rechtsabbiegen
- line_brightness: Helligkeitsschwellenwert für die Haltelinie

## trafficlight_start
Erkennt das grüne Licht der Ampel

### Publisher
- GreenLight: Boolean ob grüne Ampel erkant wurde

### Parameter
- lower_bound / upper_bound: Ober- und Untergrenze für grünes Licht



</details>
