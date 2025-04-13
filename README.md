Das ROS2-Modul "turtlebot_pastry" wird von Erik Langer, Felix Görg und Niklas Werner im Rahmen des Sommersemester-Moduls "Robotik-Projekt" an der TU Bergakademie Freiberg entwickelt.

"pastry" steht in diesem Fall für PArtial Self TRYving. Der Roboter soll einen Parkour mit Straßenzügen, Hindernissen, Ampeln und Kreuzungen bewältigen können. Näheres zu den Anforderungen ist in den Milestones festgehalten.

Im folgenden sind alle Nodes dieses Packages und ihre Funktion kurz aufgelistet:

<details>
<summary>stateMachine</summary>
Master-Node, die die Daten aller anderen Nodes managet und Fahrbefehle an den Roboter weiterleitet.
    <details>
    <summary>Publisher</summary>
    - status: Gibt State-Updates als String aus
    <br>
    - cmd_vel: Fahrbefehle für den Roboter
    </details>
</details>
<br>
<details>
<summary>detectObstacle</summary>
Nutzt den Laserscanner, um Hindernisse vor dem Roboter zu erkennen.
    <details>
    <summary>Publisher</summary>
    - obstacle_in_path: Gibt einen entsprechenden Boolean aus, wenn ein Hindernis auftaucht oder entfernt wird
    </details>
    <details>
    <summary>Parameter</summary>
    - detection_distance: Distanz, ab der ein Objekt erkannt werden soll
    </details>
</details>
<br>
<details>
<summary>followPath</summary>
Nutzt die Kamera, um in einer Spur zu fahren. Orientiert sich dabei an der rechten Begrenzungslinie.
    <details>
    <summary>Publisher</summary>
    - follow_path_cmd: Fahrbefehle zur Vorwärtsbewegung innerhalb der Fahrbahn
    </details>
    <details>
    <summary>Parameter</summary>
    - max_line_offset: Maximale Distanz in Pixeln, die die erkannte Linie von ihrer erwarteten Position abweichen darf.
    - steering_quotient: Kleinere Werte lassen den Roboter schneller lenken.
    - line_expected_at: Pixelindex, an dem die Linie erwartet wird.
    - speed_drive: Geschwindigkeit, mit der der Roboter vorwärts fährt.
    - speed_turn: Geschwindigkeit, mit der der Roboter lenkt.
    </details>
</details>