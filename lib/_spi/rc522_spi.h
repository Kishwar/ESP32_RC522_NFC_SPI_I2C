/**
 * @file rc522_spi.cc
 *
 * @brief Class File (header) response to handle read / write operations to RC522 using SPI
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 * 
 * @date 01/09/2024
 *
 */

class Rc522Spi {
    public:
        Rc522Spi();
        ~Rc522Spi();

    private:
        const int PIN_SS   = 05;
        const int PIN_MOSI = 23;
        const int PIN_MISO = 19;
        const int PIN_SCK  = 18;
        const int PIN_IRQ  = 04;      // need to check this interrupt..
};