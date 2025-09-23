# 🎮 Cactus Runner - Embedded Game

## 📖 Overview
Cactus Runner is an engaging embedded systems game inspired by Chrome's dinosaur runner. Players navigate between top and bottom positions to avoid oncoming cacti, creating an exciting and challenging gameplay experience.

## 🌟 Features
- 🎯 Simple one-button gameplay
- 🎚️ Adjustable difficulty levels
- 🏆 High score system with initial recording
- 💾 Persistent storage using EEPROM
- 📺 LCD display interface
- ⚡ Dynamic speed control

## 🎮 How to Play
1. **Start Game**: Press the button to begin
2. **Controls**: 
   - Single button press to switch positions
   - Toggle between top and bottom lanes
3. **Scoring**:
   - Points awarded for each cactus avoided
   - High scores saved with player initials

## ⚙️ Technical Details
- **Microcontroller**: PIC16F877
- **Clock Speed**: 4MHz
- **Display**: 16x2 LCD
- **Storage**: External EEPROM
- **Input**: 
  - Push button
  - Potentiometer (difficulty adjustment)

## 🛠️ Hardware Requirements
- PIC16F877 Development Board
- 16x2 LCD Display
- Push Button
- Potentiometer
- EEPROM
- Basic passive components

## 📥 Installation
1. Clone the repository
2. Configure your PIC programmer
3. Build the hardware circuit
4. Program the microcontroller
5. Power up and play!

## 🔧 Configuration
- LCD connected to PORTD
- Button on RB0
- Potentiometer on RA0
- I2C EEPROM on appropriate pins
- Crystal oscillator: 4MHz


## 👥 Contributors
- Markel Biain
- Nathan Ewnetu
