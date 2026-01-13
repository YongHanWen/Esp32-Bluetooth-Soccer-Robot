# Startathon Soccer Bot ⚽

<p align="center">
  <img src="Soccer%20Bot%20CAD%20Model.jpg" width="500">
</p>

A Bluetooth-controlled soccer robot powered by an ESP32, designed for fast response, efficient power usage, and competitive performance.

This project uses **Bluetooth Classic** for real-time control instead of web-based solutions (e.g. WebSocket/Wi-Fi). Bluetooth provides lower latency, faster connection time, lower power consumption, and more stable performance in crowded environments, making it better suited for robotics control.

The ESP32 receives Bluetooth commands and converts them into PWM signals for independent left and right motor control, enabling smooth differential steering and responsive movement. The code cleanly separates communication and motor control logic, improving reliability and ease of modification.

The robot is powered using a **higher-voltage battery configuration**, which reduces current draw, minimizes heat loss, and allows the motors to achieve higher speed and acceleration. A dedicated motor driver isolates high-current motor paths from the ESP32 for stable operation.

🏆 **Achievements**
- 🥇 1st Prize Winner  
- 💰 SGD $400 in prizes  

📚 **Usage**
- Used in **garage@EEE workshops**
- Used in **makerspace competitions**
