#ifndef WIRES_MANAGER_HPP
#define WIRES_MANAGER_HPP

#include <Arduino.h>
#include <Tachometer.h>

class WiresManager{
public:
    WiresManager(uint8_t const &a_ccw, uint8_t const &a_cw, uint8_t const &a_speed, uint8_t const &a_encoder);
    uint8_t back() const;
    uint8_t forth() const;
    uint8_t speed() const;
    uint8_t encoder() const;

private:
    uint8_t m_direction_ccw;
    uint8_t m_direction_cw;
    uint8_t m_speed;
    uint8_t m_encoder;

};


#endif //WIRES_MANAGER_HPP