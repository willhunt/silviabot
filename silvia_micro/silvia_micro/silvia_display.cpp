#include "silvia_display.h"
#include "silvia_modes.h"

SilviaDisplay::SilviaDisplay(TwoWire* twi)
    : Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, twi, -1)
    , mass_subscriber_("brew_mass", &SilviaDisplay::massCallback, this)
    , t_power_on_(0)
    , power_status_last_update_(false)
    , mass_(0)
    , t_mass_update_(0)
    , t_last_update_(0)
{}

void SilviaDisplay::setup(NodeHandle* nh) {
    nh_ = nh;
    nh->subscribe(mass_subscriber_);
}

void SilviaDisplay::massCallback(const django_interface::SilviaMass& msg) {
    mass_ = msg.mass;
    t_mass_update_ = millis();
}

void SilviaDisplay::showData() {
    clearDisplay();

    setTextColor(SSD1306_WHITE);
    char buffer [6];  // Buffer for printing strings
    bool brew_status = silvia_status.getBrew();
    int mode = silvia_status.getMode();

    // TOP
    setTextSize(3);
    if (!brew_status) {
        // 0.5 used for rounding correctly to nearest integer
        sprintf(buffer, "%d", (int)(temperature_sensor.getLatestReading() + 0.5));
        drawCentreString(buffer, 35, 6);
        setTextSize(1);
        cp437(true); write(167); print("C");  // Units
    } else {
        drawPressureString(buffer, pressure_sensor.getLatestReading(), 35, 6, 3);
        // setTextSize(1); setCursor(2,2); print(pressure_sensor.getLatestReading());
        setTextSize(1);
        setCursor(getCursorX(), getCursorY() + 14);
        print("bar");  // Units
    }
    
    drawRect(84, 7, 41, 20, WHITE);
    setTextSize(2);
    if (millis() - t_mass_update_ < ACCEPTABLE_DATA_AGE) {
        sprintf(buffer, "%dg", (int)(mass_ + 0.5));
        drawCentreString(buffer, 106, 10);
    } else if (brew_status) {
        // formatPressureString(buffer, pump.getSetpoint());
        // drawCentreString(buffer, 106, 10);
        drawPressureString(buffer, pump.getSetpoint(), 106, 10, 2);
    } else if (mode == MODE_PID) {
        sprintf(buffer, "%d", (int)(heater.getSetpoint() + 0.5));
        drawCentreString(buffer, 106, 10);
    } else if (mode == MODE_CLEAN) {
        setCursor(87, 10);
        print("(C)");
    } else {
        setCursor(87, 10);
        print("(M)");
    }

    // BOTTOM
    drawLine(0, 33, width()-1, 33, WHITE);
    if (mode == MODE_CLEAN) {  // Show cleaner time remaining
        setTextSize(3);
        sprintf(buffer, "%02d:%02d", cleaner.getTimeRemainingMins(), cleaner.getTimeRemainingSecs());
        setCursor(21, 40);
        print(buffer);
    } else { // Show brew time
        setTextSize(3);
        sprintf(buffer, "%02d:%02d", brew_timer.getDurationMins(), brew_timer.getDurationSecs());
        setCursor(21, 40);
        print(buffer);
    }
    display();
};

void SilviaDisplay::showLogo() {
  clearDisplay();
  drawBitmap(
    (width()  - LOGO_WIDTH ) / 2, (height() - LOGO_HEIGHT) / 2,
    bitmap_logo,
    LOGO_WIDTH, LOGO_HEIGHT,
    WHITE
  );
  display();
};

void SilviaDisplay::showBlank() {
  clearDisplay();
  display();
}

void SilviaDisplay::showDebug(char* text) {
    clearDisplay();
    setTextSize(2);
    setTextColor(SSD1306_WHITE);
    setCursor(2, 2);
    print(text);
    display();
}

void SilviaDisplay::update() {
    if (silvia_status.getMode() != MODE_OFF) {  // If machine on
        unsigned long t_now = millis();
        if (power_status_last_update_ == false) {
            t_power_on_ = t_now;
            showLogo();
        }
        if (t_now - t_last_update_ >= UPDATE_INTERVAL) {
            // Only show temperature after interval, to leave welcome up
            if (t_now - t_power_on_ > WELCOME_INTERVAL) {
                showData();
            }
            t_last_update_ = t_now;
        }
        power_status_last_update_ = true;
    } else if (power_status_last_update_ == true) {
        power_status_last_update_ = false;
        showBlank();
    }
}

void SilviaDisplay::drawCentreString(const char* buffer, int x, int y) {
    int16_t x1, y1;
    uint16_t w, h;
    int16_t x0, y0 = 0;  // Without this it sometimes does something weird....
    getTextBounds(buffer, x0, y0, &x1, &y1, &w, &h); // Calc width of new string
    setCursor(x - w / 2, y);
    print(buffer);
}

void SilviaDisplay::formatPressureString(char* buffer, double pressure) {
    // Arduino sprintf cannot handle floating point numbers - do manually
    pressure = pressure * 1e-5;  // Convert from Pa to bar
    int first_decimal = (int)((pressure - floor(pressure)) * 10 + 0.5);
    sprintf(buffer, "%d.%d", (int)(pressure), first_decimal);
}

/*
Contains hack to reduce spacing prior to decimal place
*/
void SilviaDisplay::drawPressureString(char* buffer, double pressure, int x, int y, int size) {
    setTextSize(size);
    size += 1;
    // Arduino sprintf cannot handle floating point numbers - do manually
    pressure = pressure * 1e-5;  // Convert from Pa to bar
    int first_decimal = (int)((pressure - floor(pressure)) * 10 + 0.5);
    sprintf(buffer, "%d.%d", (int)(pressure), first_decimal);

    int16_t x1, y1;
    uint16_t w, h;
    int16_t x0, y0 = 0;  // Without this it sometimes does something weird....
    getTextBounds(buffer, x0, y0, &x1, &y1, &w, &h); // Calc width of new string
    setCursor(x - (w - size) / 2, y);
    print(int(pressure)); 
    setCursor(getCursorX() - size, getCursorY());
    print(".");
    print(first_decimal);
}
