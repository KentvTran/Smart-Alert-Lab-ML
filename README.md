# **Smart-Alert-Lab-ML**  

## **Goal**  
The goal of this project is to develop a machine learning model to predict room occupancy based on sensor data, and eventually also predict room activity and emergencies.

## **Data Dictionary**  

| Feature          | Description |
|-----------------|-------------|
| **second**      | Number of seconds passed since the start (starts at 31 in raw data). |
| **distance**    | Distance data collected from the ADXL345 Accelerometer. |
| **motion**      | Motion data collected from the HC-SR501 Motion Sensor. |
| **lightIntensity** | Lux level data collected from the BH1750 Light Sensor. |
| **occupied**    | Room occupancy status detected using two HC-SR04 Ultrasonic Sensors (tracks when someone leaves through a door). |
| **occupied_note** | Manual input from the user indicating when the room is occupied. |

## **Work in Progress**  
- **Vibration Sensor**
- **Sound Sensor**
