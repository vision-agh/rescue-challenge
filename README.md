# ROS 2 + Gazebo Harmonic + Px4 docker

Repozytorium zawiera docker z ROS 2 Humble, Gazebo Harmonic oraz Px4

## Wymagania

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]  
> System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

## Pierwsze uruchomienie

Otwórz VS Code w katalogu z projektem.
Przejdź do lewego dolnego rogu i kliknij niebieską ikonę z dwiema strzałkami skierowanymi do siebie. Z rozwijanego menu wybierz **"Open Folder in Container... ”** i poczekaj, aż docker się zbuduje. Może to potrwać do 10 minut przy wolniejszym połączeniu internetowym.

> [!TIP]
> Dla osób korzystających z Windowsa oraz WSL 2 przygotowano `Dockerfile.windows` oraz `compose.windows.yaml`. 


Run:
```
cd ..
sudo ./setup.sh
./build.sh
source install/setup.bash
```
Powyższe komendy zbudują Twoją przestrzeń roboczą. Następnie przejdź do katalogu PX4, aby zbudować oprogramowanie w wersji SITL (ang. *Software in the loop*).
``` bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
Po wywołaniu powyższych komend powinno pojawić się okno symulatora Gazebo z dodanym dronem.

> [!NOTE]
> Chcąc uruchomić symulację drona w Gazebo należy za każdym razem użyć polecenia `make px4_sitl gz_x500`.

Po uruchomieniu symulacji, otwórz nowy terminal i wykonaj:
``` bash
# ten krok mozna takze dodac do pliku ROS launch
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

W celu sprawdzenia dostępnych topiców, w nowym oknie terminala wykonaj:
``` bash
ros2 topic list
```
Teraz możesz sprawdzić i subskrybować wszystkie topici od drona.


Po zalogowaniu się do dockera będzie on działał w sposób podobny do uruchamiania ROS na komputerze hosta. Wszystkie aplikacje GUI będą korzystać z domyślnego menedżera okien hosta, będziesz mieć również dostęp do wszystkich urządzeń na hoście, a także akceleracji GPU.
Docker ma preinstalowany [ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials.html) i większość potrzebnych zależności oraz symulator [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/).

## Pierwsze kroki

Dla osób, które nie miały doczynienia ze środowiskiem ROS 2 + Gazebo, zachęcam do przerobienia tutorialu: [Gazebo Tutorial](https://gazebosim.org/docs/harmonic/tutorials/). Pozwoli to zaznajomić się z tym środowiskiem i tworzyć w przyszłości zaawansowane symulacje.

Następnie pomocne będzie odpowiednia kontrola robotami w środowisku symulacyjnym, na dobry start proszę zaznajomić się z repozytorium: [Gazebo ROS 2 Control](https://github.com/ros-controls/gz_ros2_control/).

Na sam koniec pewnym podsumowaniem, a także praktycznym podejściem do tematu jest dostarczony od [Husariona](https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/) tutorial dla ich kilku robotów.

> [!IMPORTANT] 
Należy pamiętać, aby po zbudowaniu wywołać komendę lub pracować w nowym terminalu:
>
> ```bash
> source ~/.bashrc
> ```
>
> W tym pliku dodane są już dwie ważne ścieżki:
>
> ```bash
> /opt/ros/$ROS_DISTRO/setup.bash
> /home/developer/ros2_ws/install/setup.bash
> ```

## Uruchomienie lotu drona po kwadracie
1. Zbuduj obszar roboczy wywołując: ./build.sh w localization ~/ros2_ws.
2. Następnie: source install/setup.bash
3. Uruchom lot drona po kwadracie wywołując polecenie: ros2 launch simple_flight x500.launch.py


## Dodatkowe materiały
* [Getting Started](getting_started.md)
* [ROS 2 Command Cheat Sheet](cheatsheet.md)
* [ROS 2 Example packages in Python](example.md)
* [Bridge communication between ROS and Gazebo](ros_gz_bridge.md)