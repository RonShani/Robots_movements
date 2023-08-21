#include "wires_manager.hpp"

WiresManager::WiresManager(uint8_t const &a_ccw, uint8_t const &a_cw, uint8_t const &a_speed, uint8_t const &a_encoder)
: m_direction_ccw(a_ccw)
, m_direction_cw(a_ccw)
, m_speed(a_speed)
, m_encoder(a_encoder)
{
    pinMode(m_encoder,INPUT_PULLUP);
    pinMode(m_direction_ccw,OUTPUT);
    pinMode(m_direction_cw,OUTPUT);
    pinMode(m_speed,OUTPUT);
}

uint8_t WiresManager::back() const
{
    return m_direction_ccw;
}

uint8_t WiresManager::forth() const
{
    return m_direction_cw;
}

uint8_t WiresManager::speed() const
{
    return m_speed;
}

uint8_t WiresManager::encoder() const
{
    return m_encoder;
}
