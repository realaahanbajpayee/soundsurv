# Edge AI-Based Sound Surveillance and Alert Recognition System

## Project Overview
The **Edge AI-Based Sound Surveillance and Alert Recognition System** is an embedded, AI-driven solution designed for real-time sound classification and alert generation. This project aims to identify sound events, such as sirens, human cries, and general noise, using an STM32 microcontroller and an optimized machine learning model trained via **NanoEdge AI Studio**. The system processes data locally, ensuring minimal latency and improved privacy.

## Key Features
- **Real-Time Audio Classification**: Detects sound types and outputs results in real-time with low latency.
- **Embedded ML Implementation**: Runs an SVM model on an STM32 microcontroller for efficient sound classification.
- **Sound Intensity Measurement**: Utilizes an **LM393 audio sensor** to gauge sound intensity, complemented by visual feedback via a **servo motor**.
- **User Interface**: Integrates an **OLED display** for immediate feedback on the detected sound category.
- **Data Privacy**: Local processing eliminates the need for data transmission to external servers.

## System Components
### Hardware
- **STM32 F401RE Microcontroller**: Serves as the processing unit for ML inference and system management.
- **LM393 Audio Sensor**: Measures sound level and sends data to the microcontroller for intensity classification.
- **Microphone**: Captures raw sound data for input into the ML model.
- **OLED Display**: Displays the detected sound type and classification confidence.
- **Servo Motor**: Provides a physical representation of sound intensity levels.

### Software
- **NanoEdge AI Studio**: Utilized for ML model training, benchmarking, and optimization.
- **C/C++ Programming**: Core programming languages for firmware development and embedded logic.
- **STM32CubeIDE**: Integrated development environment for building and debugging the system's firmware.

## Installation and Setup
### Prerequisites
- **STM32CubeIDE**: Ensure it is installed and configured.
- **NanoEdge AI Studio**: Used for generating and training ML models.
- **Git**: For cloning the project repository.
