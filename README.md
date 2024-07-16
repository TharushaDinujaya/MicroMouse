
# Micro Mouse - RoboFest 23


Micromouse is a robotics competition where tiny robots navigate a maze (14x14). These autonomous mice use ToF sensors (also Sharp IR sensors can be used) to map the maze and find the fastest route to the center, testing design, programming, and problem-solving skills.

![Micromouse](https://github.com/TharushaDinujaya/MicroMouse/blob/main/docs/image_2.jpg)

## Required Components

[ESP32](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)

[ToF Sensors](https://www.st.com/resource/en/datasheet/vl53l3cx.pdf) or [Sharp IR sensors (low range)](https://global.sharp/products/device/lineup/selection/opto/haca/diagram.html)

[GA12 N20 Motors](esp32_datasheet_en.pdf (espressif.com))

[TB6612FNG](https://www.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf)

[LiPo 1500mAh Batteries](https://www.lipolbattery.com/LiPo-Battery-Datasheets.html) (Capacity maybe vary)

[LiPo 1500mAh Batteries](https://www.lipolbattery.com/LiPo-Battery-Datasheets.html) 



## Acknowledgements

 - [Arduino](https://www.arduino.cc/)
 - [Platform IO](https://platformio.org/)


## Getting Started
### Prerequisites
Platform PlatformIO
VS Code

### Installation

Clone the repository:

```bash
git clone https://github.com/TharushaDinujaya/MicroMouse
```

Open Project in VS Code

```bash
cd MicroMouse
code .
```

### Install PlatformIO IDE extension:

Go to the Extensions view (Ctrl+Shift+X).
Search for "PlatformIO IDE" and install it.
Restart VS Code if prompted.

### Prepare the Circuit.

Connect each ToF sensor in to I2C line as circuit schematics given below. We've used 2 batteries. You can use any number of batteries as required. But adding more batteries means more load to the N20 Motor and Motor's lifetime may be reduced. Also you can add an accelerometer for precise measurings. Use MPU6050 or MPU6000 for this purpose. both can be use with I2C line.

![Circuit Schematic Diagram](https://github.com/TharushaDinujaya/MicroMouse/blob/main/docs/Schematic.png)

## Flood Fill Algorithm

The Flood Fill algorithm in a micro mouse maze-solving robot is used to explore the maze and find the shortest path to the goal. Here’s a concise description:

- Initialization: The goal cell is set to a value of 0, and all other cells are assigned a high initial value.

- Flooding the Maze: Starting from the goal, adjacent cells are incremented by 1, propagating outward. This creates a gradient of values representing distances to the goal.

- Pathfinding: The robot navigates by moving to neighboring cells with decreasing values, ensuring it always heads towards the goal.

- Exploration: As the robot encounters new cells, it updates their values and adjusts the flood fill accordingly, continuously refining its path to optimize the route to the center.


## Related

Here are some related projects

[Micromouse 2024](https://github.com/)


## Screenshots

![Image 01 ](https://github.com/TharushaDinujaya/MicroMouse/blob/main/docs/image_3.jpg)

![Image 02 ](https://github.com/TharushaDinujaya/MicroMouse/blob/main/docs/image_1.jpg)


## License

[BSD](https://choosealicense.com/licenses/bsd/)


## Authors

- [@TharushaDinujaya](https://github.com/TharushaDinujaya)
- [@NidulaGunawardana](https://github.com/NidulaGunawardana)
- [@ShavinAnjithaAlpha](https://github.com/ShavinAnjithaAlpha)
- [@dilumin](https://github.com/dilumin)
- [@SachinBuwanindu]()

