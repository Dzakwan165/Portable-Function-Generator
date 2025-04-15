    #include <Wire.h>
    #include <U8g2lib.h>
    #include <driver/dac.h>
    #include <esp32-hal-ledc.h>
    #include <Arduino.h>


    // Inisialisasi OLED menggunakan U8g2
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

    // Definisi tombol
    #define BTN_UP 13
    #define BTN_DOWN 12
    #define BTN_SELECT 14
    #define OLED_SDA 21
    #define OLED_SCL 22

    // Deklarasi tabel gelombang
    uint8_t sine_table[256];
    uint8_t triangle_table[256];
    uint8_t sawtooth_table[256];
    uint8_t square_table[256];
    uint8_t scaled_wave[256];
    volatile uint8_t phase = 0;
    int pembagi_resolusi = 1;
    portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
    hw_timer_t *timer = NULL;
    
    // Variabel untuk navigasi
    int selecturr = 0; 
    int jeniswave = 0; // 0 = sinewave, 1 = trianglewave, 2 = squarewave, 3 = sawtoothwave
    int channelselector = 0; // 0 = channel 1, 1= channel 2
    int producingwave = 0;
    int menuselector = 0; // 0 = main menu, 1 =freq domain, 2=amplitude domain
    int freq_cursor = 0; 
    uint8_t freqDigits[5] = {0, 0, 1, 5, 0}; // 7 digit angka
    uint8_t backupFreq[5]; // Untuk menyimpan angka sebelum masuk menu
    float myFreq = 0; 
    int volt_cursor = 0; 
    uint8_t voltDigits[2] = {5, 0}; // 2 digit angka
    uint8_t backupVolt[2]; // Untuk menyimpan angka sebelum masuk menu
    uint8_t myVolt = 0; 
    struct ChannelSettings {
        int jeniswave;
        float frequency;
        float voltage;
    };

    ChannelSettings channelData[2] = {
        {0, 150, 5},  // Default Channel 1: Sine, 50 Hz, 0V
        {0, 150, 5}   // Default Channel 2: Sine, 50 Hz, 0V
    };

    int current_wave = channelData[0].jeniswave;
    float current_amplitude = channelData[0].voltage/8.0;
    float current_frequency = channelData[0].frequency;



    // Array untuk menyimpan pointer ke gambar
    // Gambar ikon (XBM format)
    static const unsigned char sinewave[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x1f,0x00,0x00,0x80,0x33,0x00,0x00,0x80,0x61,0x00,0x00,0xc0,0x60,0x00,0x00,0x60,0xc0,0x00,0x00,0x60,0x80,0x01,0x00,0x20,0x80,0x01,0x04,0x00,0x00,0x03,0x06,0x00,0x00,0x03,0x06,0x00,0x00,0x06,0x03,0x00,0x00,0x8e,0x01,0x00,0x00,0xdc,0x01,0x00,0x00,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    static const unsigned char trianglewave[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x80,0x01,0x00,0x00,0x80,0x03,0x00,0x00,0xc0,0x03,0x00,0x00,0xc0,0x07,0x00,0x00,0xe0,0x07,0x00,0x00,0xe0,0x0f,0x00,0x00,0xf0,0x1e,0x00,0x00,0x70,0x1e,0x00,0x00,0x78,0x3c,0x00,0x00,0x38,0x78,0x00,0x00,0x3c,0x78,0x00,0x00,0x1c,0xf0,0x00,0x00,0x1e,0xf0,0x00,0x00,0x0e,0xe0,0x01,0x00,0x0f,0xc0,0x03,0x00,0x00,0xc0,0x03,0xf0,0x00,0x80,0x07,0x70,0x00,0x00,0x07,0x78,0x00,0x00,0x0f,0x38,0x00,0x00,0x1e,0x3c,0x00,0x00,0x1e,0x1c,0x00,0x00,0x3c,0x1e,0x00,0x00,0x78,0x0e,0x00,0x00,0x78,0x0f,0x00,0x00,0xf0,0x07,0x00,0x00,0xe0,0x07,0x00,0x00,0xe0,0x03,0x00,0x00,0xc0,0x03,0x00,0x00,0xc0,0x01,0x00,0x00,0x80,0x01,0x00,0x00,0x00,0x00};
    static const unsigned char squarewave[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xf8,0xff,0x01,0x00,0xf8,0xff,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x18,0x80,0x01,0x00,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0x01,0x18,0x00,0x80,0xff,0x1f,0x00,0x80,0xff,0x1f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    static const unsigned char sawtoothwave[] U8X8_PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x80,0x01,0x00,0x00,0xc0,0x01,0x00,0x00,0xe0,0x01,0x00,0x00,0xf0,0x01,0x00,0x00,0xbc,0x01,0x00,0x00,0x9c,0x01,0x00,0x00,0x8e,0x01,0x10,0x00,0x87,0x01,0x10,0x80,0x83,0x01,0x18,0xc0,0x81,0x01,0x1c,0xe0,0x80,0x01,0x0e,0x70,0x80,0x81,0x07,0x38,0x80,0x81,0x03,0x18,0x80,0xc1,0x01,0x08,0x80,0xe1,0x00,0x08,0x80,0x71,0x00,0x00,0x80,0x39,0x00,0x00,0x80,0x3d,0x00,0x00,0x80,0x0f,0x00,0x00,0x80,0x07,0x00,0x00,0x80,0x03,0x00,0x00,0x80,0x01,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    const unsigned char* waveIcons[] = {sinewave, trianglewave, squarewave, sawtoothwave};
    const char *waveNames[] = {"Sine Wave", "Triangle Wave", "Square Wave", "Sawtooth Wave"};
    const char *channelNames[] = {"Channel 1", "Channel 2"};
    static const unsigned char image_arrow_curved_right_down_bits[] U8X8_PROGMEM = {0x01,0x02,0x06,0x04,0x1f,0x0e,0x04};

    void generate_wave_tables() {
        for(int i=0; i<256; i++) {
        // Sine wave (0-255)
        sine_table[i] = 128 + 127 * sin(2 * PI * i / 256.0);
        
        // Triangle wave
        triangle_table[i] = (i < 128) ? i*2 : 255 - (i - 128)*2;
        
        // Sawtooth wave
        sawtooth_table[i] = i;
        
        // Square wave
        square_table[i] = (i < 128) ? 255 : 0;
        }
    }
    
    void update_resolusi(){
        current_frequency=(channelData[0].frequency)*2;
        if (current_frequency > 70400) {
            pembagi_resolusi = 128;
        }
        else if (current_frequency > 35200) {
            pembagi_resolusi = 64;
        }
        else if (current_frequency > 17600) {
            pembagi_resolusi = 32;
        }
        else if (current_frequency > 8800) {
            pembagi_resolusi = 16;
        }
        else if (current_frequency > 4400) {
            pembagi_resolusi = 8;
        }
        else if (current_frequency > 2200) {
            pembagi_resolusi = 4;
        }
        else if (current_frequency > 1100) {
            pembagi_resolusi = 2;
        }
        else {
            pembagi_resolusi = 1;
        }
        Serial.println("Pembagi resolusi sekarang adalah ");
        Serial.println(pembagi_resolusi);
        }


    void update_scaled_wave() {
        uint8_t *current_table;
        current_wave=channelData[0].jeniswave; ///konversi variabel dari kode lama ke kode baru
        switch(current_wave) {
        case 0: current_table = sine_table; break;
        case 1: current_table = triangle_table; break;
        case 2: current_table = square_table; break;
        case 3: current_table = sawtooth_table; break;
        default: current_table = sine_table; break;
        }
        
        uint16_t amp_factor = channelData[0].voltage/8.0 * 255.0;
        
        portENTER_CRITICAL(&timerMux);
        for(int i=0; i<256; i++) {
        scaled_wave[i] = (current_table[i] * amp_factor) >> 8;
        }
        portEXIT_CRITICAL(&timerMux);
    }

    void IRAM_ATTR onTimer() {
        phase = (phase + pembagi_resolusi) % 256;
        dacWrite(26, scaled_wave[phase]);

    }
    
    

    float getFrequency() {
        float freq = 0;
        for (int i = 0; i < 5; i++) {
            freq = freq * 10 + freqDigits[i]; // Geser tempat desimal & tambah angka baru
            if (freq>20000){freq=20000;}
            Serial.println("Frekuensi sekarang adalah");
            Serial.println(freq);
        }
        return freq;
    }

    float getVoltage() {
        float volt= voltDigits[0]*1 + voltDigits[1]/10.0;
        if (volt>8){volt=8;}
        return (volt);
    }
    // Fungsi untuk menggambar UI
    void drawUI() {
        float myFreq = channelData[channelselector].frequency;
        float myVolt = channelData[channelselector].voltage;
        uint8_t jeniswave = channelData[channelselector].jeniswave;

        u8g2.clearBuffer();
        u8g2.setFontMode(1);
        u8g2.setBitmapMode(1);
        u8g2.setFont(u8g2_font_4x6_tr);
        u8g2.drawStr(82, 62, "By POWER 23");

        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(2, 61, "START");

        // Konversi frekuensi ke string
        char freqText[11];  // Buffer 
        sprintf(freqText, "%.1f Hz", myFreq);

        u8g2.drawStr(2, 29, freqText); // Tampilkan frekuensi di OLED

        //jenis wave
        u8g2.drawStr(2, 10, waveNames[jeniswave]);

        char VoltText[5];  
        sprintf(VoltText, "%.1f Vpp", myVolt); // Format angka jadi string
        u8g2.drawStr(2, 47, VoltText); // Tampilkan voltage di OLED
        u8g2.drawXBMP(95, 0, 32, 32, waveIcons[jeniswave]);


        // Posisi frame berdasarkan nilai select
        int frameX = 0;
        int frameY = 0;
        int frameW = 89;
        int frameH = 14;

        if (selecturr == 0) frameY = 0;
        else if (selecturr == 1) frameY = 19;
        else if (selecturr == 2) frameY = 37;
        else if (selecturr == 3) { 
            frameY = 50;
            frameW = 61; // Ukuran berubah khusus select == 3
        }

        // Gambar frame berdasarkan nilai select
        u8g2.drawFrame(frameX, frameY, frameW, frameH);

        //frame bawah
        u8g2.drawFrame(0, 50, 127, 2);
        u8g2.sendBuffer();
        delay(50);
    }

    // Fungsi untuk membaca input tombol
    bool btnUpPressed = false;
    bool btnDownPressed = false;
    bool btnSelectPressed = false;


    //tampilan untuk milih frekuensi
    void freq_select() {
    ///navigasi
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_profont22_tr);

    char digitBuffer[2];  // Buffer buat simpan angka

    sprintf(digitBuffer, "%d", freqDigits[0]);
    u8g2.drawStr(22, 25, digitBuffer);

    sprintf(digitBuffer, "%d", freqDigits[1]);
    u8g2.drawStr(33, 25, digitBuffer);

    sprintf(digitBuffer, "%d", freqDigits[2]);
    u8g2.drawStr(50, 25, digitBuffer);

    sprintf(digitBuffer, "%d", freqDigits[3]);
    u8g2.drawStr(61, 25, digitBuffer);

    sprintf(digitBuffer, "%d", freqDigits[4]);
    u8g2.drawStr(72, 25, digitBuffer);

    u8g2.drawStr(84, 25, "Hz");

    if (freq_cursor == 0) {u8g2.drawXBMP(24, 1, 5, 7, image_arrow_curved_right_down_bits);}
    if (freq_cursor == 1) {u8g2.drawXBMP(35, 1, 5, 7, image_arrow_curved_right_down_bits);}
    if (freq_cursor == 2) {u8g2.drawXBMP(52, 1, 5, 7, image_arrow_curved_right_down_bits);}
    if (freq_cursor == 3) {u8g2.drawXBMP(63, 1, 5, 7, image_arrow_curved_right_down_bits);}
    if (freq_cursor == 4) {u8g2.drawXBMP(74, 1, 5, 7, image_arrow_curved_right_down_bits);}
    if (freq_cursor == 5) {u8g2.drawFrame(6, 34, 38, 12);}
    if (freq_cursor == 6) {u8g2.drawFrame(76, 34, 46, 11);}
    
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(78, 43, "Confirm");

    u8g2.drawStr(8, 43, "Cancel");

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(38, 62, "by 1802039");

    u8g2.drawFrame(0, 53, 128, 2);

    u8g2.sendBuffer();
    delay(50);

    }

    void volt_select (){
    static const unsigned char image_arrow_curved_right_down_bits[] U8X8_PROGMEM = {0x01,0x02,0x06,0x04,0x1f,0x0e,0x04};

    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_profont22_tr);
    

    char digitbBuffer[2];  // Buffer buat simpan angka

    sprintf(digitbBuffer, "%d", voltDigits[0]);
    u8g2.drawStr(29, 25, digitbBuffer);

    sprintf(digitbBuffer, "%d", voltDigits[1]);
    u8g2.drawStr(46, 25, digitbBuffer);
    
    u8g2.drawStr(60, 25, "Vpp");

    u8g2.drawStr(37, 25, ",");

    if (volt_cursor == 0) {u8g2.drawXBMP(31, 1, 5, 7, image_arrow_curved_right_down_bits);}
    if (volt_cursor == 1) {u8g2.drawXBMP(48, 1, 5, 7, image_arrow_curved_right_down_bits);}

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(78, 43, "Confirm");

    u8g2.drawStr(8, 43, "Cancel");

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(38, 62, "by 1802039");

    if (volt_cursor == 2) {u8g2.drawFrame(6, 34, 40, 11);}
    if (volt_cursor == 3) {u8g2.drawFrame(76, 34, 46, 11);}

    u8g2.drawFrame(0, 53, 128, 2);


    u8g2.sendBuffer();
    delay(50);
    }
    void readButton() {
        // Tombol UP
        if (digitalRead(BTN_UP) == LOW) {
          Serial.println("UP SELECTED");
            if (!btnUpPressed && menuselector==0) {
                if (selecturr > 0) selecturr--; // Navigasi ke atas
                drawUI();
                btnUpPressed = true; // Set flag
            }
            if (!btnUpPressed && menuselector==1) {
            if (freq_cursor>0){freq_cursor = (freq_cursor - 1) ;}
            else if (freq_cursor==0){freq_cursor=6;}
                // Navigasi ke atas
                btnUpPressed = true; // Set flag
                freq_select();
            }
            if (!btnUpPressed && menuselector==2) {
            if (volt_cursor>0){volt_cursor = (volt_cursor - 1) ;}
            else if (volt_cursor==0){volt_cursor=3;}
                // Navigasi ke atas
                btnUpPressed = true; // Set flag
                volt_select();
            }
        } else {
            btnUpPressed = false; // Reset flag saat tombol dilepas
        }

        // Tombol DOWN
        if (digitalRead(BTN_DOWN) == LOW) {
            Serial.println("DOWN SELECTED");
            if (!btnDownPressed && menuselector==0) {
                if (selecturr < 3) selecturr++; // Navigasi ke bawah
                btnDownPressed = true; // Set flag
                drawUI();
            }
            if (!btnDownPressed && menuselector==1) {
                freq_cursor = (freq_cursor + 1) % 7; // Navigasi ke atas
                btnDownPressed = true; // Set flag
                freq_select();
            }
            if (!btnDownPressed && menuselector==2) {
                volt_cursor = (volt_cursor + 1) % 4; // Navigasi ke atas
                btnDownPressed = true; // Set flag
                volt_select();
            }
        } else {
            btnDownPressed = false; // Reset flag saat tombol dilepas
        }

        // Tombol SELECT
        if (digitalRead(BTN_SELECT) == LOW) { 
            Serial.println("SELECT SELECTED");
            if (!btnSelectPressed && selecturr == 0) { // Cek apakah di menu jenis wave
                jeniswave = (jeniswave + 1) % 4; // Increment & wrap ke 0 setelah 3
                channelData[channelselector].jeniswave = jeniswave;
                update_scaled_wave();
                drawUI();
                btnSelectPressed = true; // Set flag supaya nggak spam
            }

            if (!btnSelectPressed && selecturr == 1 && menuselector == 0) { // masuk adjustmen frequency
                // Backup angka sebelum masuk ke menu
                memcpy(backupFreq, freqDigits, sizeof(freqDigits));
                menuselector = 1; // 
                freq_select();
                btnSelectPressed = true; // Set flag supaya nggak spam
            }
            if (!btnSelectPressed && menuselector == 1 && freq_cursor == 5) { 
                memcpy(freqDigits, backupFreq, sizeof(freqDigits)); // Restore angka lama
                menuselector =0; // balik ke menu utama
                drawUI();
                btnSelectPressed = true; // Set flag supaya nggak spam
            }
            if (!btnSelectPressed && menuselector == 1 && freq_cursor == 6) { 
                channelData[channelselector].frequency = getFrequency();
                update_resolusi();
                menuselector =0; // 
                Serial.println("Going back to drawUI()");
                drawUI();
                Serial.println("Arrived at drawUI()");
                btnSelectPressed = true; // Set flag supaya nggak spam
            }
            if (!btnSelectPressed && menuselector == 1 && freq_cursor < 5) { // Cek kalau cursor di angka (0-6)
            freqDigits[freq_cursor] = (freqDigits[freq_cursor] + 1) % 10; // Naik +1, balik ke 0 setelah 9
            freq_select();
            btnSelectPressed = true; 
            Serial.println("SOMETHING IN THE WAY... HMMMM MMMM");
            }
            

            ////VOLT DOMAIN
            if (!btnSelectPressed && selecturr == 2 && menuselector == 0) { // masuk adjustmen volt
                // Backup angka sebelum masuk ke menu
                memcpy(backupVolt, voltDigits, sizeof(voltDigits));
                menuselector = 2; // 
                volt_select();
                btnSelectPressed = true; // Set flag supaya nggak spam
            }
            if (!btnSelectPressed && menuselector == 2 && volt_cursor == 2) { 
                memcpy(voltDigits, backupVolt, sizeof(voltDigits)); // Restore angka lama
                menuselector =0; // balik ke menu utama
                drawUI();
                btnSelectPressed = true; // Set flag supaya nggak spam
            }
            if (!btnSelectPressed && menuselector == 2 && volt_cursor == 3) { 
                channelData[channelselector].voltage = getVoltage();
                update_scaled_wave();
                menuselector =0; // 
                drawUI();
                btnSelectPressed = true; // Set flag supaya nggak spam
            }
            if (!btnSelectPressed && menuselector == 2 && volt_cursor == 0) { // Cek kalau cursor di angka (0-6)
            voltDigits[volt_cursor] = (voltDigits[volt_cursor] + 1) % 9; // Naik +1, balik ke 0 setelah 9
            volt_select();
            btnSelectPressed = true; 
            }
            if (!btnSelectPressed && menuselector == 2 && volt_cursor == 1) { // Cek kalau cursor di angka (0-6)
            voltDigits[volt_cursor] = (voltDigits[volt_cursor] + 1) % 10; // Naik +1, balik ke 0 setelah 9
            volt_select();
            btnSelectPressed = true; 
            }
            if (!btnSelectPressed && channelselector==1) { 
            channelselector = 0;
            btnSelectPressed = true; 
            }
            if (!btnSelectPressed && menuselector==0 && selecturr==3) { ///start producing wave
                producingwave=1;
                u8g2.clearBuffer();
                u8g2.setFontMode(1);
                u8g2.setBitmapMode(1);
                u8g2.setFont(u8g2_font_profont12_tr);
                u8g2.drawStr(27, 28, "is running...");
                u8g2.drawStr(9, 16, "Function Generator");
                u8g2.drawStr(11, 48, "Press 'OK' to STOP");
                u8g2.sendBuffer();
                btnSelectPressed = true; 
            }
            }
        else {
            btnSelectPressed = false; // Reset flag saat tombol dilepas
        }
    }


    /// HARUS DI RUBAH, SESUAIKAN SAMA YANG BARU
    void stopTimer() {
        if(timer != NULL) {
        timerAlarmDisable(timer);    // 1. Matikan alarm
        timerDetachInterrupt(timer); // 2. Lepaskan interrupt
        timerEnd(timer);             // 3. Hancurkan timer
        timer = NULL; 
        }
    }

    /// HARUS DI RUBAH, SESUAIKAN SAMA YANG BARU
    void startTimer() {
        // Konfigurasi timer
        current_frequency=(channelData[0].frequency)*2;
        timer = timerBegin(0, 1, true);
        timerAttachInterrupt(timer, &onTimer, true);
        float timer_freq = current_frequency * 256.0;
        uint32_t divider = (uint32_t)((80000000.0 / timer_freq + 0.5)*pembagi_resolusi);
        timerAlarmWrite(timer, (divider > 0) ? divider : 1, true);
        timerAlarmEnable(timer);
    }

    void startupanimation(){
    
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_11_tr);
    u8g2.drawStr(25, 34, "Booting up...");

    u8g2.sendBuffer();
    delay(500);
    /////
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_13_tr);
    u8g2.drawStr(10, 16, "P O R T A B L E");
    u8g2.sendBuffer();
    delay(250);
    u8g2.drawStr(10, 32, "F U N C T I O N");
    u8g2.sendBuffer();
    delay(250);
    u8g2.setFont(u8g2_font_t0_22b_tr);
    u8g2.drawStr(12, 53, "GENERATOR");
    u8g2.sendBuffer();
    delay(500);
    u8g2.sendBuffer();
    delay(1000);
    
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_t0_11_tr);
    u8g2.drawStr(36, 32, "18023039");

    u8g2.setFont(u8g2_font_profont17_tr);
    u8g2.drawStr(9, 17, "Presented by");

    u8g2.setFont(u8g2_font_t0_11_tr);
    u8g2.drawStr(36, 44, "18023054");

    u8g2.drawStr(36, 56, "18023030");

    u8g2.sendBuffer();
    ///
    delay(1500);}
    

    ///funsgi tombol selesai
    void setup() {
        u8g2.begin();
        Serial.begin(9600);
        // Konfigurasi pin tombol sebagai input pull-up
        pinMode(BTN_UP, INPUT_PULLUP);
        pinMode(BTN_DOWN, INPUT_PULLUP);
        pinMode(BTN_SELECT, INPUT_PULLUP);

        //wavegen setup
        generate_wave_tables();
        update_scaled_wave();
        startupanimation();
        drawUI();
        
    }

    void loop() {
        if (producingwave==0){
            readButton(); // Baca input tombol (bagian display harus dimasukkan ke read button)
            ///Serial.println("Reading button");
            
        }
        else if (producingwave==1) {
            if(timer == NULL) {startTimer();}////hanya start timer saat timer NULL
            ///fungsi untuk cancel
            ///Serial.println("PRINTING SIGNAL");
            ///Serial.println(scaled_wave[phase]);
            delay(200);
            if (digitalRead(BTN_SELECT) == LOW) { 
                if (!btnSelectPressed) { // Cek kalau tombol select di tekan
                    stopTimer();             // Hentikan timer
                    dacWrite(26, 126);
                    btnSelectPressed = true; } //flag
                    producingwave=0;
                    drawUI();
                    

            }
            else {
                btnSelectPressed = false; // Reset flag saat tombol dilepas
            }

        }
        delay(50);
            
    }