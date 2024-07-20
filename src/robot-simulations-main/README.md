# robot-simulations
Robot simulations for teaching and research purposes

### Adicione a arquivo .bashrc
    source /home/<username>/ros_ws/install/setup.bash

execute o coppeliaSim a partir do arquivo coppeliaSim.sh 
### Depedências
 - pip install transforms3d
 - sudo apt-get install ros-<ros_distro>-tf-transformations
 - opencv >= 4.9.0
### Execute os comandos abaixo para adicionar novas interfaces de dados para os serviços 
- colcon build --packages-select custom_interfaces
- source install/setup.bash
- ros2 interface list | grep custom
### A saída deve ser
    custom_interfaces/srv/ArucoDetect
    custom_interfaces/srv/Teste
Depois localize o arquivo interfaces.txt dentro da pasta onde o coppeliasim está instalado e adicione a saída anterior a lista.
