
#ifndef __DDMS_DIFF_DDMS_SERIAL_PORT_H__
#define __DDMS_DIFF_DDMS_SERIAL_PORT_H__

#include <string>
#include <vector>
#include <ros/ros.h>

#define DDMS_SERIAL_BUFFER_MAX_SIZE       200
#define DDSM_SERIAL_FRAME_SIZE     10

namespace ddms_diff
{
    enum class return_type : std::int8_t
    {
        SUCCESS = 0,
        ERROR = -1,
        NON_FATAL_READ_ERROR = -2,
        MOTOR_ERROR = 1,
    };
    enum class Mode : uint8_t {
        Current = 0x01,
        Velocity = 0x02,
        Position = 0x03,
    }; 
    class DDMSSerial
    {
    public:
        DDMSSerial();
        ~DDMSSerial();
        
        return_type open(const std::string & port_name);
        return_type close();
        return_type motor_command(uint8_t ID, double commanded_value);
        ddms_diff::return_type get_wheel_state(uint8_t ID,double velocity,std::vector<double> & current_wheel_state);
        return_type set_motor_control(uint8_t ID, Mode mode);
        bool is_open() const;

    protected:
        uint8_t crc_update(uint8_t * data, uint8_t len);

    private:
        return_type read_frame(uint8_t * frame);
        return_type write_frame(uint8_t* data);
        uint8_t crc_update(const uint8_t* data);
        
        int serial_port_;
        uint8_t rx_frame_buffer_[DDSM_SERIAL_FRAME_SIZE];
        size_t rx_frame_length_;
        uint16_t rx_frame_crc_;
        bool rx_frame_escape_;
        uint8_t tx_frame_buffer_[DDSM_SERIAL_FRAME_SIZE];
        size_t tx_frame_length_;
        uint16_t tx_frame_crc_;
        //Motor serial command structures and values

        //******** not implemented, set before use **********
        //set motor id (only one connected at a time here)
        typedef struct set_motor_id {
            uint8_t cmd1 = 0xAA;
            uint8_t cmd2 = 0x55;
            uint8_t cmd3 = 0x53;
            uint8_t ID; //this is the new id of the motor
            uint8_t reserved[6] = {0};
        } __attribute__((packed)) set_motor_id;
        // get motor info (only one at a time)
        typedef struct interrogate_motor {
            uint8_t cmd1 = 0xC8;
            uint8_t cmd2 = 0x64;
            uint8_t reserved[7] = {0};
            uint8_t CRC;
        } __attribute__((packed)) interrogate_motor;
        //**************************************************
        //query motor state
        typedef struct get_motor_state {
            uint8_t ID;
            uint8_t cmd2 = 0x74;
            uint8_t reserved[7] = {0};
            uint8_t CRC;
        } __attribute__((packed)) get_motor_state;
        //motor state return
        typedef struct motor_state {
            uint8_t ID;
            uint8_t mode;
            uint8_t torque[2];//high bit first
            uint8_t velocity[2];//high bit first, -330 to 330, RPM
            uint8_t winding_temp; //deg C
            uint8_t position;//0-255 ~= 0-360
            uint8_t error_code;
            uint8_t CRC;
        } __attribute__((packed)) motor_state;
        //set operating value for current mode
        typedef struct command_motor {
            uint8_t ID;
            uint8_t command = 0x64;
            uint8_t commanded_state[2];//high bit first, this is velocity, current or position (-330 to 330, velocity, -32767~32767 - current, +/- 8A, 0-32767, position, 0-360)
            uint8_t res[2] = {0};
            uint8_t acceleration_time = 0; 
            uint8_t brake = 0;
            uint8_t res1 = 0;
            uint8_t CRC;
        } __attribute__((packed)) command_motor;  
        //reply to command
        typedef struct command_motor_reply {
            uint8_t ID;
            uint8_t Mode;
            uint8_t torque_current[2];//high bit first, -/+8A
            uint8_t velocity[2];// +/- 330 RPM
            uint8_t position[2];// 0-32767, 0-360 
            uint8_t error_code;// BIT: 0 - sensor, 1 - over current, 2 - phase current, 3 - stall, 4 - trouble
            uint8_t CRC;
        } __attribute__((packed)) command_motor_reply; 
        //set motor control mode
        typedef struct set_mode {
            uint8_t ID;
            uint8_t set_state = 0xA0;
            uint8_t res[7]={0};//high bit first
            uint8_t mode;//default mode power on = 2
        } __attribute__((packed)) set_mode;

    };
    
}


#endif // __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_SERIAL_PORT_H__
