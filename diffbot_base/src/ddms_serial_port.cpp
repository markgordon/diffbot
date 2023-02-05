
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include <chrono>
#include <diffbot_base/ddms_serial_port.hpp>

using namespace ddms_diff;

DDMSSerial::DDMSSerial()
    : serial_port_(-1)
    , rx_frame_length_(0)
    , rx_frame_escape_(false)
    , tx_frame_length_(0)
{

}

DDMSSerial::~DDMSSerial()
{
    close();
}

int DDMSSerial::open(const std::string & port_name)
{
    serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port_ < 0) {
        fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno), errno);
        return -1;
    }
    //make it exclusive
    ioctl(serial_port_, TIOCEXCL);
    //make it low latency so it returns much faster
    struct serial_struct serial;
    ioctl(serial_port_, TIOCGSERIAL, &serial); 
    serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
    ioctl(serial_port_, TIOCSSERIAL, &serial);
    struct termios tty_config{};
    if (::tcgetattr(serial_port_, &tty_config) != 0) {
        fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        -1;
    }

    memset(&tty_config, 0, sizeof(termios));
    tty_config.c_cflag = B115200 ;//| CRTSCTS | CS8 | CLOCAL | CREAD;
    tty_config.c_cflag &= ~PARENB;//clear parity bit (no parity)
    tty_config.c_cflag &= ~CSTOPB;//Stop bits = 1
    tty_config.c_cflag &= ~CSIZE;//clears the mask
    tty_config.c_cflag |= CS8; //set data bits = 8
    tty_config.c_cflag &= ~CRTSCTS; //turn off hardwar based flow ctrl
    tty_config.c_cflag |= CREAD | CLOCAL;//Turn on the reciever
    tty_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    tty_config.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty_config.c_cc[VMIN] = 0;
    tcflush(serial_port_, TCIFLUSH);

    if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
        fprintf(stderr, "Failed to set serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return -1;
    }
    /** read motor IDS to make sure we know what we are doing*/
    interrogate_motor int_mot;
    uint8_t ID;
    sleep(1);
    ROS_INFO("req ID");
    for (int i=0;i<5;i++){
        write_frame((uint8_t *)& int_mot);
        usleep(5000);
    }
    if(write_frame((uint8_t *)& int_mot)== return_type::SUCCESS){
        usleep(5000);
        uint8_t frame[10];
        ROS_INFO("read ID");
        if(read_frame(frame)!=return_type::SUCCESS) return -1;
        //motor_state reply
        ID = frame[0];
    }
    
    return (int)ID;
}

ddms_diff::return_type DDMSSerial::close()
{
    if (is_open()) {
        ::close(serial_port_);
        serial_port_ = -1;
    }
    return return_type::SUCCESS;
}


ddms_diff::return_type DDMSSerial::get_wheel_state(uint8_t ID,double velocity,std::vector<double> & states)
{
    //write velocity to motor
    return_type retval = motor_command(ID,velocity);
    if(retval == return_type::SUCCESS){
        command_motor_reply reply;
        if((retval = read_frame((uint8_t * )(&reply))) == return_type::SUCCESS){
            if(reply.ID < 3 && reply.Mode == 2){
                //rotation only goes to 32767, no negatives so conversion isn't an issue, just convert to radians
                //veclocities are +/- so some ugliness
                int16_t velocity = (reply.velocity[0]<<8) | reply.velocity[1];            
                if(ID==2){
                    states.push_back(-velocity);
                    //the opposite rotation
                    states.push_back((((reply.position[0] << 8) + reply.position[1])/(double)32767 )* 2* M_PI);

                }else{
                    states.push_back(velocity);
                    int pos = 32767 - ((uint32_t)(reply.position[0] << 8) + reply.position[1]);
                    states.push_back((pos/(double)32767 )* 2* M_PI);
                }
            }else {
                //flush any bytes, bad read
                tcflush(serial_port_, TCIFLUSH);

                //this is an error on read, but we can ignore as next read will recover
                return ddms_diff::return_type::SUCCESS;
            }
        }else{
            if(retval == return_type::NON_FATAL_READ_ERROR) {
                //flush any bytes, bad read
                tcflush(serial_port_, TCIFLUSH);

                return ddms_diff::return_type::SUCCESS;
            }
            ROS_FATAL( "DDSM Serial Failed to read state ID:%i, error %i",ID,(uint)retval);
            return retval;
        }
    }else{

        ROS_FATAL( " DDSM Serial Failed to send command ID:%i, error %i",ID,(uint)retval);
        return retval;
    }

    return ddms_diff::return_type::SUCCESS;
}
return_type DDMSSerial::motor_command(uint8_t ID, double commanded_value)
{
    //write velocity to motor
    command_motor command;
    command.ID = ID;
    // only supports integer RPMs
    uint16_t integer_velocity = (int16_t)round(commanded_value);
    command.commanded_state[0] = integer_velocity >>8;
    command.commanded_state[1] = integer_velocity;

    return write_frame((uint8_t *)(&command));
}
return_type DDMSSerial::read_frame(uint8_t * frame)
{
    // Read data from the serial port
    ssize_t num_bytes=0,retval=0;
    uint8_t rx_buffer[11];
    uint tries=0;
    auto start =  std::chrono::system_clock().now().time_since_epoch();
    do{
        retval = ::read(serial_port_, rx_buffer, 10);
        if(retval > 0) memcpy(frame + num_bytes,rx_buffer,retval);
        num_bytes+= retval;
        tries++;
      //if we get nine bytes, we get nine bytes. 10 is never showing. If the first two bytes are good the rest are good as well
      // as far as possible to tell.
        if(num_bytes == 9){
            //so if we wait just a few milliseconds the next call won't fail.......
            if(tries > 9 || frame[0] > 2)
            {
                uint time =   std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock().now().time_since_epoch() - start).count();
               // RCLCPP_INFO(rclcpp::get_logger("DDMSSerialPort"),"fail time %d, %d, %d",time, tries,frame[0]);

               // RCLCPP_ERROR(rclcpp::get_logger("DDMSSerialPort"),"rec %ld",num_bytes);
                if(num_bytes > 0) return return_type::NON_FATAL_READ_ERROR;
                if (retval == -1 ) {
                    ROS_ERROR(" DDSM Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
                }
                return return_type::ERROR;
            }
        }
    }while(retval > -1 && num_bytes < 10 && tries <11);
                ROS_INFO("answer %02x%02x%02x%02x%02x%02x%02x%02x%02x %02x, bytes %d, tries %d",
                frame[0],frame[1],frame[2],frame[3],frame[4],frame[5],frame[6],frame[7],frame[8], frame[9],num_bytes,tries);
    if (retval == -1 ) {
        ROS_FATAL("DDSM Failed to read serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }
   //uint time =   std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock().now().time_since_epoch() - start).count();
    return return_type::SUCCESS;
}


ddms_diff::return_type DDMSSerial::write_frame(uint8_t* data)
{
    if (!is_open()) {
        return return_type::ERROR;
    }
    
    // Generate the frame
    uint8_t crc = crc_update(data);
    data[DDSM_SERIAL_FRAME_SIZE -1] = crc;
    if (::write(serial_port_, data, DDSM_SERIAL_FRAME_SIZE) == -1) {
        ROS_FATAL("Failed to write serial port data: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }
    return return_type::SUCCESS;
}

bool DDMSSerial::is_open() const
{
    return serial_port_ >= 0;
}

// CRC-8/MAXIM
uint8_t DDMSSerial::crc_update(const uint8_t* data)
{
    unsigned crc = 0;
    for (size_t i = 0; i < DDSM_SERIAL_FRAME_SIZE-1; i++) {
        crc ^= data[i];
        for (unsigned k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0x8c : crc >> 1;
    }
    return crc;
}