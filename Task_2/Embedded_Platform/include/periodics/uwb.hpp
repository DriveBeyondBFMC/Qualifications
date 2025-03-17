#ifndef UWB_HPP
#define UWB_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <utils/queue.hpp>
#include <map>
#include <array>
#include <string>
#include <functional>
#include <chrono>
#include <drivers/speedingmonitor.hpp>

namespace periodics
{
   /**
    * @brief Class uwb
    *
    */
    class ICUwb {
        virtual std::string uwb_coordinate_value();
    };
    class CUwb: public utils::CTask, periodics::ICUwb
    {
        public:
            /* Constructor */
            CUwb(
                UnbufferedSerial&          f_serial,
                drivers::ISpeedingMonitor&  f_position
            );
            /* Destructor */
            ~CUwb();
            void serialRxCallback();
            void printBuffer(const std::array<char, 256>& buffer, size_t length); // Add this line
            void processMessage();
            void printRxBuffer();
            std::string uwb_coordinate_value();
        private:
            /* private variables & method member */
            /* Run method */
            virtual void        _run();
            UnbufferedSerial& m_serialPort;
            utils::CQueue<char,255> m_RxBuffer;
            array<char,256>::iterator m_parseIt;
            std::array<char, 256> m_parseBuffer;
            char received_data[100];
            /** @brief Active flag  */
            bool m_isActive;
            std::string buffer; // Static buffer to accumulate characters
            std::string uwb_buffer; 
            drivers::ISpeedingMonitor&  m_position;



    }; // class CUwb
}; // namespace periodics

#endif // UWB_HPP
