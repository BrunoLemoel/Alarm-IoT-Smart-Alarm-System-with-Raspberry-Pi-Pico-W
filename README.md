# Alarm-IoT-Smart-Alarm-System-with-Raspberry-Pi-Pico-W
This project implements a complete IoT alarm system using the Raspberry Pi Pico W microcontroller. The system is capable of real-time monitoring, activating physical devices (relay, buzzer, LED), displaying information on an LCD, measuring temperature via an NTC sensor, and connecting to a Wi-Fi network for future IoT integrations.

<img width="828" height="522" alt="Alarm-IoT" src="https://github.com/user-attachments/assets/50bc1259-4b36-4a10-a565-c228f5f7f07c" />

## Impact and Importance

This project represents a significant advance in the democratization of home automation and security, allowing anyone—from electronics enthusiasts to professional developers—to build custom alarm and monitoring solutions. By combining sensors, actuators, and connectivity, **Alarm-IoT** enables:

- **Greater Security:** Allows for scheduling automatic alarms, activating buzzers, LEDs, and relays, as well as manual deactivation using a physical button. - **Environmental Monitoring:** Measures and displays ambient temperature in real time, facilitating the monitoring of environmental conditions.
- **Remote and Expandable Control:** Prepares the system for future integration via MQTT, enabling remote control and notifications.
- **Education and Prototyping:** Serves as a reference for students, teachers, and makers who want to learn about microcontrollers, sensors, actuators, and IoT.

The project encourages the development of safe, accessible, and open-source solutions, promoting technological innovation and strengthening the maker culture.

## Main Features

- **Programmable Alarm:** Alarm scheduling via console, with a customized time.
- **Device Triggering:** Relay control for sirens, lights, or other external devices.
- **Audible and Visual Indication:** Buzzer tone sequence via PWM and LED signaling. - I2C LCD Display: Shows date and time (RTC DS3231) and environmental information.
- NTC Temperature Sensor: Accurate ambient temperature reading via ADC.
- Wi-Fi Connection: Ready for integration with IoT/cloud platforms.
- Physical Interface: Button for manual alarm deactivation and direct interaction.
- Debounce and Security: Reliable button reading and protection against unauthorized activation.

- ## Technical Architecture

- **Microcontroller:** Raspberry Pi Pico W
- **RTC:** DS3231 via I2C
- **LCD:** 16x2 via I2C
- **Temperature Sensor:** NTC 10kΩ
- **Actuators:** Relay, buzzer (PWM), LED
- **Button:** Digital input with debounce
- **Software:** C (Pico SDK), integration with hardware libraries, logic for alarm control, sensor reading, and display.

## Applications

- **Home Automation:** Security alarm, environmental monitoring, lighting control.
- **Industrial Monitoring:** Process and equipment control, automated alerts.
- **Education:** Teaching projects on embedded systems, sensors, actuators, and IoT.
- **Prototyping and Research:** Basis for developing custom monitoring systems.

---

## Future Improvements

The project is open to improvements that may expand its functionality and impact:

- **MQTT/Cloud Integration:** Sending alarm and monitoring notifications to remote platforms.
- **Web Interface:** Configuration of alarms and monitoring via browser.
- **Multiple Sensor Support:** Expansion to presence, gas, smoke, etc. sensors.
- **Push, Email, or SMS Notifications:** Instant user alerts.
- **Mobile App Control:** Remote interaction via the app.
- **Consumption Optimization:** Implementation of energy-saving modes.
- **OTA Firmware:** Automatic updates via Wi-Fi.
- **Event Logging:** Storage of alarm and measurement logs.

- ## How to Use

1. **Clone the repository:**
```sh
git clone https://github.com/BrunoLemoel/Alarm-IoT.git
cd Alarm-IoT
```
2. **Assemble the hardware according to the diagrams and list the components.**
3. **Configure Wi-Fi and alarm time via the console.**
4. **Compile and upload the firmware to the Pico W.**
5. **Monitor alarms, temperature, and events via the LCD and console.**


