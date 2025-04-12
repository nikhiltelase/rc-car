# 🚗 Internet-Controlled Car using ESP32-CAM + Ngrok

This project enables full remote control and live video streaming of a small robotic car over the internet using an ESP32-CAM, a mobile hotspot, and Ngrok.

## 🔧 Features

- Control car movements (forward, reverse, left, right, stop)
- Live video feed from ESP32-CAM camera
- Global access via secure Ngrok tunnel
- No need for GSM or cloud services
- Budget-friendly and DIY-ready

## 🛠️ Tech Stack

- **Hardware**: ESP32-CAM, L293D Motor Driver, DC Motors, Battery, Chassis
- **Software**: Arduino IDE, Ngrok, HTML/CSS for Web UI
- **Connectivity**: Smartphone hotspot + Ngrok tunnel for public access

## 🔗 How It Works

1. ESP32-CAM connects to mobile hotspot (Wi-Fi).
2. It hosts a local web server with control buttons and live video stream.
3. Ngrok on a laptop forwards the local IP to a secure public URL.
4. You can control the car from anywhere using this URL.

## 📺 Demo Video

[![Watch the video](https://img.youtube.com/vi/WGUHVOzH4Zo/0.jpg)](https://www.youtube.com/watch?v=WGUHVOzH4Zo)

> 🎥 [Watch on YouTube](https://www.youtube.com/watch?v=WGUHVOzH4Zo)

## 🧩 Circuit Diagram

![Surveillance Car Circuit Diagram](https://github.com/nikhiltelase/rc-car/raw/main/Surveillance%20Car%20circuit%20diagram.png)

> [View on GitHub](https://github.com/nikhiltelase/rc-car/blob/main/Surveillance%20Car%20circuit%20diagram.png)

## 👨‍💻 Developed By

- **Name**: Nikhil Telase  
- **Email**: nikhiltelase@gmail.com  
- **Location**: Balaghat, Madhya Pradesh  
- **Guidance & Funding**: [Connect Shiksha](https://connectshiksha.com)

## 💡 Future Scope

- Add GPS and obstacle avoidance
- Secure login with OTP or QR code
- Control via voice or mobile app
