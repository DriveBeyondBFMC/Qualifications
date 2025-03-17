#include "periodics/uwb.hpp"
namespace periodics
{
    CUwb::CUwb(
        UnbufferedSerial& f_serial
        , drivers::ISpeedingMonitor&  f_position 
    )

        : utils::CTask(std::chrono::milliseconds(0))
        , m_serialPort(f_serial)
        , m_RxBuffer()
        , m_parseIt(m_parseBuffer.begin())
        , m_parseBuffer()
        , m_position(f_position)
    {
        m_serialPort.attach(mbed::callback(this, &CUwb::serialRxCallback), SerialBase::RxIrq);
    }

    CUwb::~CUwb()
    {
        // Clean up resources if needed
    }

    void CUwb::serialRxCallback()
    {
        __disable_irq();
        while ((m_serialPort.readable()) && (!m_RxBuffer.isFull())) {
            char buf;
            m_serialPort.read(&buf, 1);
            m_RxBuffer.push(buf);
        }
        __enable_irq();
    }

    void CUwb::printBuffer(const std::array<char, 256>& buffer, size_t length)
    {
        for (size_t i = 0; i < length; ++i) {
            m_serialPort.write(&buffer[i], 1); // Print each character to the serial port
        }
        m_serialPort.write("\r\n", 2); // Add a newline for readability
    }

    void CUwb::printRxBuffer()
    {
        while (!m_RxBuffer.isEmpty()) {
            char currentChar = m_RxBuffer.pop();
            m_serialPort.write(&currentChar, 1); // Print each character to the serial port
        }
        m_serialPort.write("\r\n", 2); // Add a newline for readability
    }

    void CUwb::_run()
    {
        if (!m_RxBuffer.isEmpty()) {
            char currentChar = m_RxBuffer.pop(); 
            m_serialPort.write(&currentChar, 1);
    
            if (currentChar != '\n') {
                buffer += currentChar; 
            } else {
                if (!buffer.empty()) {
                    printf("%s\n", buffer.c_str()); 
                    buffer.clear();
                }
                m_serialPort.write("\r\n", 2); 
            }
        }


    }

    std::string CUwb::uwb_coordinate_value()
    {
        return "hi";
    }
    
} // namespace periodics