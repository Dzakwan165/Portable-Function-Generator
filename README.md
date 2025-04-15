A simple and portable function generator using ESP32, designed for educational and prototyping purposes.

## ✨ Features
- Generates sine, square, triangle, and sawtooth waveforms
- Adjustable frequency and amplitude
- Adjustable voltage range up to 8Vpp
- OLED display for user interface


## 🔌 Hardware Used
- ESP32 Dev Board
- DAC (internal or external)
- LCD 16x2 or OLED display
- push buttons
- Battery pack / power source
- OP07 

## 📈 Schematic
bit.ly/schematicgensin

## ⚙️ How to Use
1. Upload the code to your ESP32 using Arduino IDE
2. Connect the OLED to ESP32 via I2C (SCL → D22, SDA → D21 by default)
3. Power the board via USB or battery
4. Monitor the waveform on oscilloscope or circuit

## 🛠️ Setup Instructions
1. Install Arduino IDE
2. Tambahkan ESP32 board di Board Manager
3. Install library berikut dari Library Manager:
   - `U8g2` by olikraus
4. Buka file `.ino` dan upload ke ESP32

## 📜 License
This project is licensed under the MIT License — free to use, modify, and distribute.

## ❤️ Author

- **Safarudin Ali Topan (18023030)**
- **Taqidito Ilham Pratama (18023039)**
- **Muhamad Dzakwan Musfajra (18023054)**

Dibimbing oleh dosen yang luar biasa:
- **Ir. Syarif Hidayat, M.T., Ph.D**
- **Dr. Bryan Denov, S.T., M.T**

_Tugas besar mata kuliah EP2004 - Sistem Pengukuran, Program Studi Teknik Tenaga Listrik, ITB_


