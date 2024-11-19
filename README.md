# Edge AI-Based Sound Surveillance and Alert Recognition System

## Project Overview
The **Edge AI-Based Sound Surveillance and Alert Recognition System** is an embedded solution developed for real-time sound classification. This project uses a lightweight machine learning (ML) model implemented on an STM32 microcontroller to distinguish between different sound types, including sirens, human cries, and general noise. The system is built using **NanoEdge AI Studio** for training and optimization, with **audio sensors** collecting sound data for real-time analysis.

## Features
- **Real-Time Sound Classification**: Identifies sound categories and displays results in real-time.
- **Optimized ML Model**: Deploys an SVM model designed for efficiency on embedded hardware.
- **Low-Latency Processing**: Conducts all processing on the STM32 microcontroller to avoid data transmission delays.
- **OLED Display Output**: Shows detected sound classes and levels on an integrated display.
- **Sound Intensity Measurement**: Employs an LM393 audio sensor to measure sound levels and display them via a servo motor.

## System Components
### Hardware
- **STM32 F401RE Microcontroller**: Runs the ML model and manages system operations.
- **LM393 Audio Sensor**: Collects sound intensity data for classification.
- **Microphone**: Captures audio data for ML model input.
- **OLED Display**: Provides visual feedback of detected sound categories.
- **Servo Motor**: Indicates sound intensity in a physical, easy-to-understand format.

### Software
- **NanoEdge AI Studio**: Used to train and benchmark the ML model for optimal performance.
- **C/C++**: Primary programming languages used for firmware development.
- **STM32CubeIDE**: Development environment for coding, compiling, and debugging the firmware.

## Installation and Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/edge-ai-sound-surveillance.git
