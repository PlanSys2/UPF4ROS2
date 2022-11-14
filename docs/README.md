## UPF4ROS2

## Instalation

### Dependencies:

- Unified Planning

    $ pip install --pre unified-planning[pyperplan,tamer]

- Unified Planning sources

    $ vcs import . < upf.repos

### Extra (WIP)

- Documentation with Doxygen: 

Install Doxygen package and run command in docs folder
    

    $ doxygen
    

### Tests

Standalone option (Double check your ROS 2 working directory )

    $ python3 ./src/UPF4ROS2/upf4ros2/test/test_conversion.py
    $ python3 ./src/UPF4ROS2/upf4ros2/test/test_upf4ros2.py

### Launch

    $ ros2 launch  upf4ros2 upf4ros2.launch.py

