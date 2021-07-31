# Sensei32: HDC1080 and BME688 Basic Sensor Read

Sensei32 is a high-quality ULP, Battery-Powered, ESP32 board with embedded environment sensors.

We are looking forward to define the next hardware revision specs and organise PCBs reservation for the following production run. 

✨ Follow the project on [Hacakday.io][HChan] ✨

[//]: # (✨ Check out our store on [Tindie][TChan] ✨)

✨ Join us on [Discord][DChan] ✨

![alt text](https://github.com/Sensei32-DevKit/media/blob/main/SENSEI32_V001_REV002_PinOut.jpg?raw=true)

## 🚀 Features

In Sensei32, every pin of ESP32 has been brought to the board perimeter through castellated holes, so that the user can decide to:

- Solder the provided **18650 battery holder** and pin-headers, using Sensei32 as a standard ESP32 DevKit, ready for your battery-operated DIY projects.
- Solder Sensei32 to a custom carrier PCB, using Sensei32 as a feature-packed ESP32 module, with **any Li-Ion or Li-Poly external battery** (up to 6000mAh).

Sensei32 has **world-class ULP performance**, consuming about **21uA in deep-sleep** while providing a **stable and regulated 3.3V** over the whole voltage range of a single cell Li-Ion. The integrated sensors for Temperature, Humidity, Pressure and Air Quality (**BME688**, AI Powered) and precision temperature and humidity (**HDC1080**) provide an exceptional environmental sensing ability.

![alt text](https://github.com/Sensei32-DevKit/media/blob/main/SENSEI32_V001_REV002_Boards.jpg?raw=true)

Other features of Sensei32 are:

- **ESP32-WROOM-32D**, with 4MB Flash
- FTDI **FT231XQ-R**, usb-serial orignal chip
- Texas Instruments **BQ24092**, Li-Ion/Li-Poly 600mA battery charger
- Texas Instruments **TPS63900**, buck-boost dc-dc converted
- Texas Instruments **HDC1080**, environment sensor (T, RH)
- BOSCH **BME688**, AI-Powered air-quality sensor (T, RH, P, eCO2, bVOC, IAQ)
- **USB Voltage Sensing**, routed to IO36
- **Battery Voltage Sensing**, routed to IO35
- **Battery Charging/Floating Sensing**, routed to IO39
- **Activity LED**, routed to IO2
- **USB Power Supply LED** and **Battery InCharge LED**, managed by BQ24092

[//]: # (PLACEHOLDER: Blink Gif with terminal...)
[//]: # (A quick demo on how to assembly the **18650 battery holder** and **pin-headers**, to use Sensei32 as a standard ESP32 DevKit:)
[//]: # (https://user-images.githubusercontent.com/14925798/126035802-63ded463-5af1-4854-b9ec-f1f7bcdcf78c.mov)

## 🤖 Tech

Sensei32 is designed around the **ESP32 WROOM**, a battery charger (**BQ24092**) and ultra-low power regulator (**TPS63900**) ICs. The choice of these devices provide a particularly stable 3.3V over the whole voltage range of a single cell Li-Ion, enabling the operation until the battery reaches the discharge threshold. The integrated sensors for Temperature, Humidity, Pressure and Air Quality (**BME688**, AI Powered) and precision temperature and humidity (**HDC1080**) provide an exceptional environmental sensing ability.

The **USB Micro AB** connector is used for both charging and programming, as data lines are routed to an original **FT231XQ-R** USB-serial chip, which is supplied with power only by the 5V USB-VBus line, thus avoiding unnecessary battery usage when not programming or charging. This unique feature enables a 5V power source to keep the battery replenished whilst consuming ultra-low power.  

Sensei32 shares the same form-factor (every mechanical feature including button placement) and pinout of [WEMOS ESP32 Board with 18650 Battery Holder][WEMOS], representing a drag & drop replacement for every project that is already using this very common board, but with a far better ULP performance, components quality and integrated sensors. Sensei32 also has two extra pins on the right-side connector to break out VBAT (Battery line) and GND, enabling the use of any external Li-Ion or Li-Poly battery (up to 6000mAh). The Sensei32 also has an in-built battery monitoring facilities, together with an NTC resistor that is used to monitor battery and battery charger temperatures. The **NTC resistor is not reachable from the ESP32**, as it is dedicated to the BQ24092 battery charger.

While the power architecture of Sensei32 can operate with VBAT voltages (Battery line) exceeding the [2.6V, 4.2V] operational range, it's recommended to stay between the limits specified by the Li-Ion or Li-Poly battery manufaturer and always use Li-Ion or Li-Poly batteries from a trusty manufacturer, like a Samsung INR18650-30Q and/or a INR18650-35E cell.

> To avoid any operating interference with the ESP32 Internal Flash, Sensei32 **do not** break out ESP32 Internal Flash QSPI Pins.
> To ensure this does not happen, **In place of** IO6 (FLASH_SCK), IO7 (FLASH_D0), IO8 (FLASH_D1), IO9 (FLASH_D2), IO10 (FLASH_D3), IO11 (FLASH_CMD) **Sensei32 has GND Pins**.

## ⚠️ Disclaimer and Safety Note

Sensei32 software is provided "as is" for private use only, it is not for commercial use in whole or part or concept. For personal use it is supplied without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose and noninfringement. In no event shall the author or copyright holder be liable for any claim, damages or other liability, whether in an action of contract, tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in the software. Sensei32 product is meant to be use only by "Sophisticated User" who understands electrical, electronics and battery technology and that it is their sole responsibility to determine suitability of the product described for their application and to take precautions for protection against any hazards attendant to the handling and use of the product. In no event shall the author or copyright holder be liable for any claim, damages or other liability, whether in an action of contract, tort or otherwise, arising from, out of or in connection with the product or the use or other dealings in with the product.

## 🚦 Compiling the sketch with Arduino

### 1. Install the latest Arduino IDE and ESP32 BSP

As of this publication, the latest Arduino IDE 1.8.13 can be downloaded from this [link](https://www.arduino.cc/download_handler.php). In order to compile the Sensei32 Starter Package, the ESP32 Arduino Board Support Package should be already installed from "Tools > Board Manager". There are plenty of guides online that show how to install and use an ESP32 Board in Arduino IDE, as a reference: [How to install ESP32 Board in Arduino IDE][arduesp]. 

Once installed the ESP32 Arduino Board Support Package, Sensei32 Arduino board configuration should match the following:

| Configuration | Setting |
| ------ | ------ |
| Board | ESP32 Wrover Module |
| Upload Speed | 921600 |
| Flash Frequency | 80MHz |
| Flash Mode | QIO |
| Partition Scheme | Default 4MB with spiffs |

![alt text](https://github.com/Sensei32-DevKit/media/blob/main/GUIDE_00_ArduConfig_SENSEI32.png?raw=true)

### 2. Install basic libraries

Sensei32 can be used with most third-party library designed for Arduino. In this example, the integrated Texas Instruments HDC1080 Temperature and Humidity sensor and the integrated Bosh BME688 Temperature, Humidity, Presssure and Air Quality sensor are read using standard libraries, that can be easily downloaded from the Arduino Library Manager:
- HDC1080 Library from ClosedCube, [Library Github][CCube]
- BME680 Library form Adafruit, [Library Github][AdaBME]

From Adafruit Library Git: "The BME680 takes those sensors to the next step in that it contains a small MOX sensor. The heated metal oxide changes resistance based on the volatile organic compounds (VOC) in the air, so it can be used to detect gasses & alcohols such as Ethanol, Alcohol and Carbon Monoxide, and perform air quality measurements. Note it will give you **one resistance value**, with overall VOC content, but it cannot differentiate gasses or alcohols."

### 3. You are ready to go!

Just upload the firmware to your Sensei32 board and enjoy!

## 🙏🏼 Special Thanks

We would like to acknowledge the assistance of [G6EJD][G6EJD_H] that has inspired us to design Sensei32. His knowledge and constructive advice has lead to many valuable contribution to this project.


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [arduesp]: <https://www.hackster.io/abdularbi17/how-to-install-esp32-board-in-arduino-ide-1cd571>
   [G6EJD_H]: <http://g6ejd.dynu.com>
   [WEMOS]: <https://wiki.geekworm.com/index.php/WEMOS_ESP32_Board_with_18650_Battery_Holder>
   [CCube]: <https://github.com/closedcube/ClosedCube_HDC1080_Arduino>
   [AdaBME]: <https://github.com/adafruit/Adafruit_BME680>   
   [DChan]: <https://discord.gg/HK5cx9yMgQ>
   [HChan]: <https://hackaday.io/project/180675-sensei32>
   [TChan]: <https://www.tindie.com/products/24139/>      
