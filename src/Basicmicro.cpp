#include "Arduino.h"
#include "Basicmicro.h"

#define MAXRETRY 3
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

/**
 * @brief Constructor for hardware serial communication
 * 
 * Initializes the controller interface using a hardware serial port
 * 
 * @param serial Pointer to the HardwareSerial object to use for communication
 * @param tout Communication timeout in microseconds
 */
Basicmicro::Basicmicro(HardwareSerial *serial, uint32_t tout)
{
	ST_Power=ST_Turn=-1;

	timeout = tout;
	hserial = serial;
#ifdef __AVR__
	sserial = 0;
#endif
}

#ifdef __AVR__
/**
 * @brief Constructor for software serial communication
 * 
 * Initializes the controller interface using a software serial port
 * (Available only on AVR-based Arduino boards)
 * 
 * @param serial Pointer to the SoftwareSerial object to use for communication
 * @param tout Communication timeout in microseconds
 */
Basicmicro::Basicmicro(SoftwareSerial *serial, uint32_t tout)
{
	ST_Power=ST_Turn=-1;

	timeout = tout;
	sserial = serial;
	hserial = 0;
}
#endif

//
// Destructor
//
Basicmicro::~Basicmicro()
{
}

/**
 * @brief Initialize the serial connection
 * 
 * Starts the communication at the specified baud rate
 * 
 * @param speed Baud rate for serial communication
 */
void Basicmicro::begin(long speed)
{
	if(hserial){
		hserial->begin(speed);
	}
#ifdef __AVR__
	else{
		sserial->begin(speed);
	}
#endif
}

bool Basicmicro::listen()
{
#ifdef __AVR__
	if(sserial){
		return sserial->listen();
	}
#endif
	return false;
}

bool Basicmicro::isListening()
{
#ifdef __AVR__
	if(sserial)
		return sserial->isListening();
#endif
	return false;
}

bool Basicmicro::overflow()
{
#ifdef __AVR__
	if(sserial)
		return sserial->overflow();
#endif
	return false;
}

int Basicmicro::peek()
{
	if(hserial)
		return hserial->peek();
#ifdef __AVR__
	else
		return sserial->peek();
#else	
	return 0;
#endif
}

size_t Basicmicro::write(uint8_t byte)
{
	if(hserial)
		return hserial->write(byte);
#ifdef __AVR__
	else
		return sserial->write(byte);
#else
	return 0;
#endif
}

int Basicmicro::read()
{
	if(hserial)
		return hserial->read();
#ifdef __AVR__
	else
		return sserial->read();
#else
	return -1;
#endif
}

int Basicmicro::available()
{
	if(hserial)
		return hserial->available();
#ifdef __AVR__
	else
		return sserial->available();
#else
	return 0;
#endif
}

void Basicmicro::flush()
{
	if(hserial)
		hserial->flush();
#ifdef __AVR__
    else if(sserial)
        sserial->flush();
#endif
}

/**
 * @brief Read data from serial with timeout
 * 
 * Waits for data to be available and returns it, or returns -1 if timeout occurs
 * 
 * @param timeout Maximum time to wait in microseconds
 * @return Byte read or -1 if timeout occurs
 */
int Basicmicro::read(uint32_t timeout)
{
	if(hserial){
		uint32_t start = micros();
		// Empty buffer?
		while(!hserial->available()){
		   if((micros()-start)>=timeout)
		      return -1;
		}
		return hserial->read();
	}
#ifdef __AVR__
	else{
		if(sserial->isListening()){
			uint32_t start = micros();
			// Empty buffer?
			while(!sserial->available()){
			   if((micros()-start)>=timeout)
				  return -1;
			}
			return sserial->read();
		}
	}
#else
	return -1;
#endif
}

/**
 * @brief Read a single byte with CRC update
 * 
 * Reads one byte from the serial port and updates the CRC.
 * 
 * @param value Reference to store the byte read
 * @return true if read succeeded, false if timeout or error occurred
 */
bool Basicmicro::ReadByte(uint8_t &value)
{
    int16_t data;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    value = (uint8_t)data;
    
    return true;
}
/**
 * @brief Read a 16-bit word with CRC update
 * 
 * Reads two bytes from the serial port and combines them into a 16-bit word.
 * Updates the CRC with each byte.
 * 
 * @return The 16-bit word read, or 0 if read failed
 */
bool Basicmicro::ReadWord(uint16_t &value)
{
    int16_t data;
    uint16_t val = 0;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    val = (uint16_t)data << 8;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    val |= (uint16_t)data;
    
    value = val;
    return true;
}

/**
 * @brief Read a 32-bit long with CRC update
 * 
 * Reads four bytes from the serial port and combines them into a 32-bit long.
 * Updates the CRC with each byte.
 * 
 * @return The 32-bit long read, or 0 if read failed
 */
bool Basicmicro::ReadLong(uint32_t &value)
{
    int16_t data;
    uint32_t val = 0;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    val = (uint32_t)data << 24;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    val |= (uint32_t)data << 16;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    val |= (uint32_t)data << 8;
    
    data = read(timeout);
    if(data == -1) return false;
    crc_update(data);
    val |= (uint32_t)data;
    
    value = val;
    return true;
}

/**
 * @brief Clear the serial buffer
 * 
 * Reads and discards all data in the input buffer
 */
void Basicmicro::clear()
{
	if(hserial){
		while(hserial->available())
			hserial->read();
	}
#ifdef __AVR__
	else{
		while(sserial->available())
			sserial->read();
	}
#endif
}

/**
 * @brief Clear the internal CRC calculation register
 * 
 * Resets the CRC value to start a new CRC calculation
 */
void Basicmicro::crc_clear()
{
	crc = 0;
}

/**
 * @brief Calculates CRC value for outgoing communication
 * 
 * Implements standard XMODEM CRC calculation for validating data integrity
 * 
 * @param data Byte to include in CRC calculation
 */
void Basicmicro::crc_update (uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;	//0x1021 is the standard XMODEM polynomial.
		else
			crc <<= 1;
	}
}

/**
 * @brief Get the current CRC value
 * 
 * Returns the current value of the CRC calculation register.
 * This value represents the CRC-16 checksum of all bytes processed
 * since the last call to crc_clear().
 * 
 * The CRC is used to validate data integrity in communications with the controller.
 * After sending or receiving data, the CRC value is compared to the transmitted CRC
 * to verify that no transmission errors occurred.
 * 
 * @return Current CRC-16 value
 */
uint16_t Basicmicro::crc_get()
{
	return crc;
}

/**
 * @brief Writes a command with data and CRC to the motor controller
 * 
 * Handles communication protocol including CRC calculation and retry mechanism
 * 
 * @param cnt Number of bytes to send
 * @param ... Variable arguments for each byte to send
 * @return true if acknowledgment received, false after max retries
 */
bool Basicmicro::write_n(uint8_t cnt, ... )
{
	uint8_t trys=MAXRETRY;
	do{
		clear();		
		crc_clear();
		//send data with crc
		va_list marker;
		va_start( marker, cnt );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint8_t data = va_arg(marker, int);
			crc_update(data);
			write(data);
		}
		va_end( marker );              /* Reset variable arguments.      */
		uint16_t crc = crc_get();
		write(crc>>8);
		write(crc);
		if(read(timeout)==0xFF)
			return true;
	}while(trys--);
	return false;
}

/**
 * @brief Prepare the address and command bytes for communication
 * 
 * Helper function to send the address and command bytes and update the CRC
 * 
 * @param address Controller address
 * @param cmd Command byte
 */
void Basicmicro::write_address_cmd(uint8_t address,uint8_t cmd)
{
	clear();	
	crc_clear();
	write(address);
	crc_update(address);
	write(cmd);
	crc_update(cmd);
}

/**
 * @brief Read multiple 32-bit values from the controller
 * 
 * Reads a sequence of 32-bit values with CRC validation and retry mechanism
 * 
 * @param cnt Number of 32-bit values to read
 * @param address Controller address
 * @param cmd Command to send
 * @param ... Variable arguments to store the read values (pointers to uint32_t)
 * @return true if successful, false otherwise
 */
bool Basicmicro::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		write_address_cmd(address,cmd);

		data=0;
		//send data with crc
		va_list marker;
		va_start( marker, cmd );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value=(uint32_t)data<<24;
			}
			else{
				break;
			}
			
			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data<<16;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data<<8;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data;
			}
			else{
				break;
			}

			*ptr = value;
		}
		va_end( marker );              /* Reset variable arguments.      */

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					return crc_get()==ccrc;
				}
			}
		}
	}while(trys--);

	return false;
}

/**
 * @brief Read multiple 16-bit values from the controller
 * 
 * Reads a sequence of 16-bit values with CRC validation and retry mechanism.
 * Similar to read_n but optimized for 16-bit word reading.
 * 
 * @param cnt Number of 16-bit values to read
 * @param address Controller address
 * @param cmd Command to send
 * @param ... Variable arguments to store the read values (pointers to uint32_t, but only lower 16 bits are used)
 * @return true if successful, false otherwise
 */
bool Basicmicro::read_n_words(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		write_address_cmd(address,cmd);

		data=0;
		//send data with crc
		va_list marker;
		va_start( marker, cmd );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);
			uint32_t value=0;

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint16_t)data<<8;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint16_t)data;
			}
			else{
				break;
			}

			*ptr = value;
		}
		va_end( marker );              /* Reset variable arguments.      */

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					return crc_get()==ccrc;
				}
			}
		}
	}while(trys--);

	return false;
}

/**
 * @brief Read multiple 8-bit values from the controller
 * 
 * Reads a sequence of 8-bit values with CRC validation and retry mechanism.
 * Similar to read_n but optimized for 8-bit byte reading.
 * 
 * @param cnt Number of 8-bit values to read
 * @param address Controller address
 * @param cmd Command to send
 * @param ... Variable arguments to store the read values (pointers to uint8_t)
 * @return true if successful, false otherwise
 */
bool Basicmicro::read_n_bytes(uint8_t cnt, uint8_t address, uint8_t cmd, ...)
{
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        write_address_cmd(address, cmd);

        data = 0;
        // Process variable arguments
        va_list marker;
        va_start(marker, cmd);     /* Initialize variable arguments. */
        for (uint8_t index = 0; index < cnt; index++) {
            uint8_t *ptr = va_arg(marker, uint8_t *);
            
            // Read a single byte
            if (data != -1) {
                data = read(timeout);
                if (data != -1) {
                    crc_update(data);
                    *ptr = data;
                } else {
                    break;
                }
            } else {
                break;
            }
        }
        va_end(marker);              /* Reset variable arguments. */

        if (data != -1) {
            uint16_t ccrc;
            data = read(timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(timeout);
                if (data != -1) {
                    ccrc |= data;
                    return crc_get() == ccrc;
                }
            }
        }
    } while (trys--);

    return false;
}

/**
 * @brief Read 8-bit value from controller
 * 
 * Retrieves an 8-bit value from the controller with the specified command.
 * 
 * @param address Controller address
 * @param cmd Command to send
 * @param valid Optional pointer to store communication validity
 * @return 8-bit value from controller or 0 if communication failed
 */
uint8_t Basicmicro::Read1(uint8_t address,uint8_t cmd,bool *valid)
{
	uint8_t crc;

	if(valid)
		*valid = false;
	
	uint8_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		write_address_cmd(address,cmd);
	
		data = read(timeout);
		crc_update(data);
		value=data;

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						if(valid)
							*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
	
	return 0;
}

/**
 * @brief Read 16-bit value from controller
 * 
 * Retrieves a 16-bit value from the controller with the specified command.
 * Handles communication protocol including CRC validation and retries.
 * 
 * @param address Controller address
 * @param cmd Command to send
 * @param valid Optional pointer to store communication validity
 * @return 16-bit value from controller or 0 if communication failed
 */
uint16_t Basicmicro::Read2(uint8_t address,uint8_t cmd,bool *valid)
{
	uint8_t crc;

	if(valid)
		*valid = false;
	
	uint16_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		write_address_cmd(address,cmd);
	
		data = read(timeout);
		crc_update(data);
		value=(uint16_t)data<<8;
		
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint16_t)data;
		}
		
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						if(valid)
							*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
		
	return 0;
}

/**
 * @brief Read 32-bit value from controller
 * 
 * Retrieves a 32-bit value from the controller with the specified command.
 * Handles communication protocol including CRC validation and retry mechanism.
 * Reads 4 bytes (MSB first) and combines them into a 32-bit value.
 * 
 * @param address Controller address
 * @param cmd Command to send
 * @param valid Optional pointer to store communication validity flag
 * @return 32-bit value from controller or 0 if communication failed
 */
uint32_t Basicmicro::Read4(uint8_t address, uint8_t cmd, bool *valid)
{
	uint8_t crc;
	
	if(valid)
		*valid = false;
	
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		write_address_cmd(address,cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}
		
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}
		
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						if(valid)
							*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);
	
	return 0;
}

/**
 * @brief Reads 4 bytes (32-bit value) plus a status byte from the controller
 * 
 * Handles protocol, retries, and CRC validation
 * 
 * @param address Controller address
 * @param cmd Command to send
 * @param status Pointer to store status byte
 * @param valid Pointer to store communication validity flag
 * @return 32-bit value read from controller
 */
uint32_t Basicmicro::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
	uint8_t crc;

	if(valid)
		*valid = false;
	
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		write_address_cmd(address,cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}
	
		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			if(status)
				*status = data;
		}
				
		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						if(valid)
							*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return 0;
}

/***************************************
Compatibility Command functions
***************************************/

/**
 * @brief Set PWM value for a single motor
 * 
 * Converts a user-friendly motor command into the appropriate duty cycle value
 * based on the provided range. Used for compatibility with other motor control libraries.
 * 
 * @param address Controller address
 * @param motor Motor selection (0 for Motor 1, 1 for Motor 2)
 * @param value Speed value, positive for forward, negative for reverse (-range to +range)
 * @param accel Acceleration value (controls how quickly the motor reaches the target speed)
 * @param range Maximum range of the input value (normalizes to internal -32767 to +32767 scale)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPWM(uint8_t address, uint8_t motor, int16_t value, int32_t accel, uint16_t range)
{
	int16_t duty = (int32_t)value*0x7FFF/range;
	if(!motor)
		return DutyAccelM1(address,duty,accel);
	else
		return DutyAccelM2(address,duty,accel);
}

/**
 * @brief Execute single motor command in Sabertooth-compatible mode
 * 
 * Processes commands in the format used by Sabertooth motor controllers.
 * Translates these commands to the equivalent Basicmicro motor control commands.
 * 
 * @param cmd Command type (M1FORWARD, M1BACKWARD, etc.)
 * @param address Controller address
 * @param speed Speed value (0-127 for standard mode, 0-127 with 64 as stop for 7-bit mode)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ST_Single(uint8_t cmd,uint8_t address, uint8_t speed)
{
	ST_Power=ST_Turn=-1;
	if(cmd==M17BIT && speed==0) speed=1;	//Special case
	uint8_t power = speed&0x7F;
	return SetPWM(address, (cmd<M2FORWARD || cmd==M17BIT) ? 0 : 1,cmd==M17BIT ? power-64 : cmd&0x1 ? -power : power,0,cmd>=M17BIT ? 63 : 127);
}

bool Basicmicro::ForwardM1(uint8_t address, uint8_t speed)
{
	return ST_Single(M1FORWARD,address,speed);
}
bool Basicmicro::BackwardM1(uint8_t address, uint8_t speed)
{
	return ST_Single(M1BACKWARD,address,speed);
}
bool Basicmicro::ForwardM2(uint8_t address, uint8_t speed)
{
	return ST_Single(M2FORWARD,address,speed);
}
bool Basicmicro::BackwardM2(uint8_t address, uint8_t speed)
{
	return ST_Single(M2BACKWARD,address,speed);
}
bool Basicmicro::ForwardBackwardM1(uint8_t address, uint8_t speed)
{
	return ST_Single(M17BIT,address,speed);
}
bool Basicmicro::ForwardBackwardM2(uint8_t address, uint8_t speed)
{
	return ST_Single(M27BIT,address,speed);
}

/**
 * @brief Set PWM values for both motors
 * 
 * Converts user-friendly motor commands into the appropriate duty cycle values
 * for both motors based on the provided range.
 * 
 * @param address Controller address
 * @param value1 Speed value for Motor 1, positive for forward, negative for reverse (-range to +range)
 * @param accel1 Acceleration value for Motor 1
 * @param value2 Speed value for Motor 2, positive for forward, negative for reverse (-range to +range)
 * @param accel2 Acceleration value for Motor 2
 * @param range Maximum range of the input values (normalizes to internal -32767 to +32767 scale)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPWM2(uint8_t address, int16_t value1, int32_t accel1, int16_t value2, int32_t accel2, uint16_t range)
{
	int16_t duty1 = (int32_t)value1*0x7FFF/range;
	int16_t duty2 = (int32_t)value2*0x7FFF/range;
	return DutyAccelM1M2(address,duty1,accel1,duty2,accel2);
}

/**
 * @brief Run Motor 1 in mixed steering mode
 * 
 * Special implementation for mixed control modes using ST-style command.
 * This calculates appropriate motor outputs for differential steering.
 * 
 * @param cmd Command type (forward, backward, right, left, etc.)
 * @param address Controller address
 * @param speed Speed value (0-127)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ST_Mixed(uint8_t cmd, uint8_t address, uint8_t speed)
{
	uint16_t power = speed&0x7F;
	int16_t temp = !power ? 2 : power*2;
	if(cmd<MIXEDFB) temp = cmd&0x1 ? 128-power : 128+power;	//calc single direction commands
	
	if(cmd==MIXEDRIGHT || cmd==MIXEDLEFT || cmd==MIXEDLR) 
		ST_Turn=temp; 
	else 
		ST_Power=temp;
	
	if(ST_Power>=0 && ST_Turn>=0){
		int16_t pwr0=ST_Power-128;
		int16_t pwr1=ST_Turn-128;
		int8_t out0,out1;
		int16_t tM2pwr = pwr1;
		if((tM2pwr^pwr0)&0x80){	//-+ or +-
			if(abs(tM2pwr)>abs(pwr0))
				out1 = -tM2pwr;
			else
				out1 = pwr0;
			out0 = pwr0+tM2pwr;
		}
		else{						//-- or ++
			if(abs(pwr0)>abs(tM2pwr))
				out0 = pwr0;
			else
				out0 = tM2pwr;
			out1 = pwr0-tM2pwr;
		}
		return SetPWM2(address,0,out0,0,out1,127);
	}
	
	//If both ST_Power and ST_Turn are NOT non-zero this command should return true.
	return true;
}

bool Basicmicro::ForwardMixed(uint8_t address, uint8_t speed)
{
	ST_Mixed(MIXEDFORWARD,address,speed);
	return true;
}
bool Basicmicro::BackwardMixed(uint8_t address, uint8_t speed)
{
	ST_Mixed(MIXEDBACKWARD,address,speed);
	return true;
}
bool Basicmicro::TurnRightMixed(uint8_t address, uint8_t speed)
{
	ST_Mixed(MIXEDRIGHT,address,speed);
	return true;
}
bool Basicmicro::TurnLeftMixed(uint8_t address, uint8_t speed)
{
	ST_Mixed(MIXEDLEFT,address,speed);
	return true;
}
bool Basicmicro::ForwardBackwardMixed(uint8_t address, uint8_t speed)
{
	ST_Mixed(MIXEDFB,address,speed);
	return true;
}
bool Basicmicro::LeftRightMixed(uint8_t address, uint8_t speed)
{
	ST_Mixed(MIXEDLR,address,speed);
	return true;
}

/***************************************
Basicmicro Commands
***************************************/

/**
 * @brief Read encoder values for both motors
 * 
 * Retrieves the current encoder count values for both motors in a single command.
 * 
 * @param address Controller address
 * @param enc1 Variable to store Motor 1 encoder value
 * @param enc2 Variable to store Motor 2 encoder value
 * @return true if successful, false otherwise
 */
uint32_t Basicmicro::ReadEncM1(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM1ENC,status,valid);
}

/**
 * @brief Read encoder count for Motor 2
 * 
 * Retrieves the current encoder count value for Motor 2.
 * The encoder count is a 32-bit signed value that increments or decrements
 * based on the motor's rotation direction.
 * 
 * @param address Controller address
 * @param status Optional pointer to store status byte:
 *        0 = OK
 *        1 = Underflow (count minimum reached)
 *        2 = Overflow (count maximum reached)
 *        3 = Direction bit set (encoder moving in reverse direction)
 * @param valid Optional pointer to store communication validity flag
 * @return 32-bit encoder count or 0 if communication failed
 */
uint32_t Basicmicro::ReadEncM2(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM2ENC,status,valid);
}

/**
 * @brief Read speed for Motor 1
 * 
 * Retrieves the current speed of Motor 1 in encoder counts per second.
 * This value can be used to monitor actual motor speed for closed-loop control.
 * 
 * @param address Controller address
 * @param status Optional pointer to store status byte:
 *        0 = Forward motion
 *        1 = Reverse motion
 * @param valid Optional pointer to store communication validity flag
 * @return 32-bit speed value in encoder counts per second or 0 if communication failed
 */
uint32_t Basicmicro::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM1SPEED,status,valid);
}

/**
 * @brief Read speed for Motor 2
 * 
 * Retrieves the current speed of Motor 2 in encoder counts per second.
 * This value can be used to monitor actual motor speed for closed-loop control.
 * 
 * @param address Controller address
 * @param status Optional pointer to store status byte:
 *        0 = Forward motion
 *        1 = Reverse motion
 * @param valid Optional pointer to store communication validity flag
 * @return 32-bit speed value in encoder counts per second or 0 if communication failed
 */
uint32_t Basicmicro::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM2SPEED,status,valid);
}

/**
 * @brief Reset both encoder counters
 * 
 * Sets both encoder counters to zero. This is useful for establishing
 * a reference position or when initializing position control.
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::ResetEncoders(uint8_t address)
{
	return write_n(2,address,RESETENC);
}

/**
 * @brief Read controller firmware version
 * 
 * Retrieves the firmware version string from the controller
 * 
 * @param address Controller address
 * @param version Buffer to store the version string (min 48 bytes)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadVersion(uint8_t address,char *version)
{
	uint8_t data;
	uint8_t trys=MAXRETRY;
	
	if(!version) //handle NULL pointer
		return false;
	
	do{
		clear();
		crc_clear();
		write(address);
		crc_update(address);
		write(GETVERSION);
		crc_update(GETVERSION);
	
		uint8_t i;
		for(i=0;i<48;i++){	//maximum version string length is 48 bytes including the null.
			data=read(timeout);
			if(data!=-1){
				version[i] = data;
				crc_update(version[i]);
				if(version[i]==0){
					uint16_t ccrc;
					data = read(timeout);
					if(data!=-1){
						ccrc = data << 8;
						data = read(timeout);
						if(data!=-1){
							ccrc |= data;
							return crc_get()==ccrc;
						}
					}
					break;
				}
			}
			else{
				break;
			}
		}
	}while(trys--);
	
	version[0] = 0;	//empty string on error
	return false;
}

/**
 * @brief Set encoder count for Motor 1
 * 
 * Sets the encoder counter for Motor 1 to a specific value.
 * This is useful for establishing a known position or resetting
 * the position to a specific offset value.
 * 
 * @param address Controller address
 * @param val The 32-bit signed value to set the encoder counter to
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetEncM1(uint8_t address, int32_t val)
{
	return write_n(6,address,SETM1ENCCOUNT,SetDWORDval(val));
}

/**
 * @brief Set encoder count for Motor 1
 * 
 * Sets the encoder counter for Motor 1 to a specific value.
 * This is useful for establishing a known position or resetting
 * the position to a specific offset value.
 * 
 * @param address Controller address
 * @param val The 32-bit signed value to set the encoder counter to
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetEncM2(uint8_t address, int32_t val)
{
	return write_n(6,address,SETM2ENCCOUNT,SetDWORDval(val));
}

/**
 * @brief Read main battery voltage
 * 
 * Retrieves the current voltage of the main battery (motor power supply).
 * The voltage is returned in tenths of a volt (e.g., 124 = 12.4V).
 * 
 * @param address Controller address
 * @param valid Optional pointer to store communication validity flag
 * @return 16-bit voltage value in tenths of a volt or 0 if communication failed
 */
uint16_t Basicmicro::ReadMainBatteryVoltage(uint8_t address,bool *valid)
{
	return Read2(address,GETMBATT,valid);
}

/**
 * @brief Read logic battery voltage
 * 
 * Retrieves the current voltage of the logic battery (controller power supply).
 * The voltage is returned in tenths of a volt (e.g., 49 = 4.9V).
 * Some controllers use a single power supply and this may return the same value
 * as ReadMainBatteryVoltage.
 * 
 * @param address Controller address
 * @param valid Optional pointer to store communication validity flag
 * @return 16-bit voltage value in tenths of a volt or 0 if communication failed
 */
uint16_t Basicmicro::ReadLogicBatteryVoltage(uint8_t address,bool *valid)
{
	return Read2(address,GETLBATT,valid);
}

/**
 * @brief Sets M1 velocity PID parameters
 * 
 * Converts floating point PID values to fixed-point format used by the controller.
 * The controller uses 16.16 fixed-point format (multiplier of 65536).
 * 
 * @param address Controller address
 * @param kp_fp Proportional constant (floating point)
 * @param ki_fp Integral constant (floating point)
 * @param kd_fp Derivative constant (floating point)
 * @param qpps Maximum speed in quadrature pulses per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps)
{
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

/**
 * @brief Set Motor 2 velocity PID parameters
 * 
 * Configures the PID control parameters for Motor 2 velocity control.
 * The controller uses 16.16 fixed-point format for PID values (multiplier of 65536).
 * 
 * @param address Controller address
 * @param kp_fp Proportional constant (floating point)
 * @param ki_fp Integral constant (floating point)
 * @param kd_fp Derivative constant (floating point)
 * @param qpps Maximum speed in quadrature pulses per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps)
{
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

/**
 * @brief Read internal speed values for both motors
 * 
 * Retrieves the current internal speed values for both motors in a single command.
 * These are the actual speed measurements used for PID calculations.
 * 
 * @param address Controller address
 * @param ispeed1 Variable to store Motor 1 internal speed
 * @param ispeed2 Variable to store Motor 2 internal speed
 * @return true if successful, false otherwise
 */
uint32_t Basicmicro::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM1ISPEED,status,valid);
}

/**
 * @brief Read instantaneous speed for Motor 2
 * 
 * Retrieves the current instantaneous speed of Motor 2 in encoder counts per second.
 * This value is the raw speed reading before any filtering is applied.
 * 
 * @param address Controller address
 * @param status Optional pointer to store status byte:
 *        0 = Forward motion
 *        1 = Reverse motion
 * @param valid Optional pointer to store communication validity flag
 * @return 32-bit speed value in encoder counts per second or 0 if communication failed
 */
uint32_t Basicmicro::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM2ISPEED,status,valid);
}

/**
 * @brief Set duty cycle for Motor 1
 * 
 * Sets the PWM duty cycle for Motor 1 directly.
 * The duty cycle is a 16-bit signed value from -32768 to +32767,
 * where positive values drive forward and negative values drive backward.
 * 
 * @param address Controller address
 * @param duty Duty cycle value (-32768 to +32767)
 * @return true if successful, false otherwise
 */
bool Basicmicro::DutyM1(uint8_t address, uint16_t duty)
{
	return write_n(4,address,M1DUTY,SetWORDval(duty));
}

/**
 * @brief Set duty cycle for Motor 2
 * 
 * Sets the PWM duty cycle for Motor 2 directly.
 * The duty cycle is a 16-bit signed value from -32768 to +32767,
 * where positive values drive forward and negative values drive backward.
 * 
 * @param address Controller address
 * @param duty Duty cycle value (-32768 to +32767)
 * @return true if successful, false otherwise
 */
bool Basicmicro::DutyM2(uint8_t address, uint16_t duty)
{
	return write_n(4,address,M2DUTY,SetWORDval(duty));
}

/**
 * @brief Set duty cycles for both motors
 * 
 * Sets the PWM duty cycles for both Motor 1 and Motor 2 with a single command.
 * The duty cycles are 16-bit signed values from -32768 to +32767,
 * where positive values drive forward and negative values drive backward.
 * 
 * @param address Controller address
 * @param duty1 Duty cycle value for Motor 1 (-32768 to +32767)
 * @param duty2 Duty cycle value for Motor 2 (-32768 to +32767)
 * @return true if successful, false otherwise
 */
bool Basicmicro::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2)
{
	return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

/**
 * @brief Set speed for Motor 1
 * 
 * Sets the target speed for Motor 1 in encoder counts per second.
 * This command uses the PID controller for closed-loop speed control.
 * 
 * @param address Controller address
 * @param speed Desired speed in encoder counts per second (signed value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedM1(uint8_t address, uint32_t speed)
{
	return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

/**
 * @brief Set speed for Motor 2
 * 
 * Sets the target speed for Motor 2 in encoder counts per second.
 * This command uses the PID controller for closed-loop speed control.
 * 
 * @param address Controller address
 * @param speed Desired speed in encoder counts per second (signed value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedM2(uint8_t address, uint32_t speed)
{
	return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

/**
 * @brief Set speeds for both motors
 * 
 * Sets the target speeds for both Motor 1 and Motor 2 with a single command.
 * This command uses the PID controllers for closed-loop speed control.
 * 
 * @param address Controller address
 * @param speed1 Desired speed for Motor 1 in encoder counts per second (signed value)
 * @param speed2 Desired speed for Motor 2 in encoder counts per second (signed value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2)
{
	return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

/**
 * @brief Set speed for Motor 1 with acceleration control
 * 
 * Sets the target speed for Motor 1 with controlled acceleration.
 * The motor will accelerate at the specified rate until it reaches the target speed.
 * 
 * @param address Controller address
 * @param accel Acceleration rate in encoder counts per second per second
 * @param speed Target speed in encoder counts per second (signed value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed)
{
	return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

/**
 * @brief Set speed for Motor 2 with acceleration control
 * 
 * Sets the target speed for Motor 2 with controlled acceleration.
 * The motor will accelerate at the specified rate until it reaches the target speed.
 * 
 * @param address Controller address
 * @param accel Acceleration rate in encoder counts per second per second
 * @param speed Target speed in encoder counts per second (signed value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed)
{
	return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

/**
 * @brief Set speeds for both motors with acceleration control
 * 
 * Sets the target speeds for both Motor 1 and Motor 2 with controlled acceleration.
 * Both motors will accelerate at the same specified rate until they reach their target speeds.
 * 
 * @param address Controller address
 * @param accel Acceleration rate for both motors in encoder counts per second per second
 * @param speed1 Target speed for Motor 1 in encoder counts per second (signed value)
 * @param speed2 Target speed for Motor 2 in encoder counts per second (signed value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2)
{
	return write_n(14,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

/**
 * @brief Run Motor 1 at specified speed with distance limit
 * 
 * Commands Motor 1 to run at the specified speed until it has moved
 * the specified distance, then stop.
 * 
 * @param address Controller address
 * @param speed Speed in encoder counts per second
 * @param distance Distance to travel in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward
 *             bit 1: 0 = Absolute position, 1 = Relative position
 *             bit 3: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

/**
 * @brief Run Motor 2 at specified speed with distance limit
 * 
 * Commands Motor 2 to run at the specified speed until it has moved
 * the specified distance, then stop.
 * 
 * @param address Controller address
 * @param speed Speed in encoder counts per second
 * @param distance Distance to travel in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward
 *             bit 1: 0 = Absolute position, 1 = Relative position
 *             bit 3: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

/**
 * @brief Run both motors at specified speeds with distance limits
 * 
 * Commands both motors to run at specified speeds until they have moved
 * their specified distances, then stop.
 * 
 * @param address Controller address
 * @param speed1 Speed for Motor 1 in encoder counts per second
 * @param distance1 Distance for Motor 1 in encoder counts
 * @param speed2 Speed for Motor 2 in encoder counts per second
 * @param distance2 Distance for Motor 2 in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward (Motor 1)
 *             bit 1: 0 = Absolute position, 1 = Relative position (Motor 1)
 *             bit 2: 0 = Forward, 1 = Backward (Motor 2)
 *             bit 3: 0 = Absolute position, 1 = Relative position (Motor 2)
 *             bit 4: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
	return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

/**
 * @brief Run Motor 1 at specified speed with controlled acceleration and distance limit
 * 
 * Commands Motor 1 to accelerate to the specified speed at the specified rate,
 * travel the specified distance, then stop.
 * 
 * @param address Controller address
 * @param accel Acceleration rate in counts per second per second
 * @param speed Target speed in encoder counts per second
 * @param distance Distance to travel in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward
 *             bit 1: 0 = Absolute position, 1 = Relative position
 *             bit 3: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

/**
 * @brief Run Motor 2 at specified speed with controlled acceleration and distance limit
 * 
 * Commands Motor 2 to accelerate to the specified speed at the specified rate,
 * travel the specified distance, then stop.
 * 
 * @param address Controller address
 * @param accel Acceleration rate in counts per second per second
 * @param speed Target speed in encoder counts per second
 * @param distance Distance to travel in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward
 *             bit 1: 0 = Absolute position, 1 = Relative position
 *             bit 3: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

/**
 * @brief Run both motors with controlled acceleration, speed, and distance
 * 
 * Commands both motors to run with the specified acceleration, speed, and distance parameters.
 * 
 * @param address Controller address
 * @param accel Acceleration rate for both motors in counts per second per second
 * @param speed1 Target speed for Motor 1 in encoder counts per second
 * @param distance1 Distance for Motor 1 in encoder counts
 * @param speed2 Target speed for Motor 2 in encoder counts per second
 * @param distance2 Distance for Motor 2 in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward (Motor 1)
 *             bit 1: 0 = Absolute position, 1 = Relative position (Motor 1)
 *             bit 2: 0 = Forward, 1 = Backward (Motor 2)
 *             bit 3: 0 = Absolute position, 1 = Relative position (Motor 2)
 *             bit 4: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
	return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

/**
 * @brief Read buffer depths for command queues
 * 
 * Retrieves the number of commands queued in each motor's buffer
 * 
 * @param address Controller address
 * @param depth1 Variable to store Motor 1 buffer depth
 * @param depth2 Variable to store Motor 2 buffer depth
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2)
{
	bool valid = false;
	uint16_t value = Read2(address,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}

/**
 * @brief Read PWM values for both motors
 * 
 * Retrieves the current PWM duty cycle values being applied to each motor.
 * The returned values are signed 16-bit integers (-32768 to 32767).
 * 
 * @param address Controller address
 * @param pwm1 Variable to store Motor 1 PWM value
 * @param pwm2 Variable to store Motor 2 PWM value
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2)
{
	bool valid = false;
	uint32_t value = Read4(address,GETPWMS,&valid);
	if(valid){
		pwm1 = value>>16;
		pwm2 = value&0xFFFF;
	}
	return valid;
}

/**
 * @brief Read motor current draw
 * 
 * Retrieves the current draw in milliamps for both motors
 * 
 * @param address Controller address
 * @param current1 Variable to store Motor 1 current in milliamps
 * @param current2 Variable to store Motor 2 current in milliamps
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2)
{
	bool valid = false;
	uint32_t value = Read4(address,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>16;
		current2 = value&0xFFFF;
	}
	return valid;
}

/**
 * @brief Set speed for both motors with individual acceleration rates
 * 
 * Controls both motors with independent acceleration parameters
 * 
 * @param address Controller address
 * @param accel1 Acceleration rate for Motor 1
 * @param speed1 Target speed for Motor 1
 * @param accel2 Acceleration rate for Motor 2
 * @param speed2 Target speed for Motor 2
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2)
{
	return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

/**
 * @brief Run both motors with individual acceleration, speed, and distance parameters
 * 
 * Commands both motors to run with independent acceleration, speed, and distance settings.
 * This allows for complex, coordinated movements with different parameters for each motor.
 * 
 * @param address Controller address
 * @param accel1 Acceleration rate for Motor 1 in counts per second per second
 * @param speed1 Target speed for Motor 1 in encoder counts per second
 * @param distance1 Distance for Motor 1 in encoder counts
 * @param accel2 Acceleration rate for Motor 2 in counts per second per second
 * @param speed2 Target speed for Motor 2 in encoder counts per second
 * @param distance2 Distance for Motor 2 in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward (Motor 1)
 *             bit 1: 0 = Absolute position, 1 = Relative position (Motor 1)
 *             bit 2: 0 = Forward, 1 = Backward (Motor 2)
 *             bit 3: 0 = Absolute position, 1 = Relative position (Motor 2)
 *             bit 4: 0 = Immediate execution, 1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
	return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

/**
 * @brief Set duty cycle for Motor 1 with acceleration control
 * 
 * Sets the duty cycle with controlled acceleration rate
 * 
 * @param address Controller address
 * @param duty Target duty cycle (signed 16-bit value)
 * @param accel Acceleration rate (32-bit value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel)
{
	return write_n(8,address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

/**
 * @brief Set duty cycle for Motor 2 with acceleration control
 * 
 * Sets the PWM duty cycle for Motor 2 with controlled acceleration rate.
 * The duty cycle will ramp up/down at the specified acceleration rate
 * until it reaches the target value.
 * 
 * @param address Controller address
 * @param duty Target duty cycle (signed 16-bit value, -32768 to +32767)
 * @param accel Acceleration rate (32-bit value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel)
{
	return write_n(8,address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

/**
 * @brief Set duty cycles for both motors with individual acceleration control
 * 
 * Sets the PWM duty cycles for both motors with independent acceleration rates.
 * Each motor's duty cycle will ramp up/down at its specified rate until
 * it reaches its target value.
 * 
 * @param address Controller address
 * @param duty1 Target duty cycle for Motor 1 (signed 16-bit value, -32768 to +32767)
 * @param accel1 Acceleration rate for Motor 1 (32-bit value)
 * @param duty2 Target duty cycle for Motor 2 (signed 16-bit value, -32768 to +32767)
 * @param accel2 Acceleration rate for Motor 2 (32-bit value)
 * @return true if successful, false otherwise
 */
bool Basicmicro::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2)
{
	return write_n(14,address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2));
}

/**
 * @brief Read M1 velocity PID parameters
 * 
 * Retrieves the current PID parameters for Motor 1 velocity control
 * 
 * @param address Controller address
 * @param Kp_fp Variable to store proportional constant (converted to floating point)
 * @param Ki_fp Variable to store integral constant (converted to floating point)
 * @param Kd_fp Variable to store derivative constant (converted to floating point)
 * @param qpps Variable to store maximum speed in quadrature pulses per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

/**
 * @brief Read Motor 2 velocity PID parameters
 * 
 * Retrieves the current PID parameters for Motor 2 velocity control.
 * The controller stores PID values in 16.16 fixed-point format,
 * but this function converts them to floating point for ease of use.
 * 
 * @param address Controller address
 * @param Kp_fp Variable to store proportional constant (converted to floating point)
 * @param Ki_fp Variable to store integral constant (converted to floating point)
 * @param Kd_fp Variable to store derivative constant (converted to floating point)
 * @param qpps Variable to store maximum speed in quadrature pulses per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

/**
 * @brief Set main battery voltage limits
 * 
 * Sets the minimum and maximum voltage limits for the main battery
 * 
 * @param address Controller address
 * @param min Minimum allowed voltage in tenths of a volt (e.g., 100 = 10.0V)
 * @param max Maximum allowed voltage in tenths of a volt
 * @param autoMax Flag to set the auto max setting (0 or 1)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max,uint8_t autoMax)
{
	return write_n(7,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max),autoMax);
}

/**
 * @brief Set logic battery voltage limits
 * 
 * Sets the minimum and maximum voltage limits for the logic battery.
 * The controller will generate warnings or errors if the voltage goes
 * outside these limits.
 * 
 * @param address Controller address
 * @param min Minimum allowed voltage in tenths of a volt (e.g., 45 = 4.5V)
 * @param max Maximum allowed voltage in tenths of a volt (e.g., 60 = 6.0V)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

/**
 * @brief Read minimum and maximum main battery voltages
 * 
 * Retrieves the current voltage limit settings for the main battery.
 * 
 * @param address Controller address
 * @param min Variable to store minimum voltage setting (in tenths of a volt)
 * @param max Variable to store maximum voltage setting (in tenths of a volt)
 * @param autoMax Variable to store auto max setting (0 or 1)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max,uint8_t& autoMax)
{
	bool valid = false;
	uint8_t temp;
	uint32_t value =  Read4_1(address,GETMINMAXMAINVOLTAGES,&temp,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
		autoMax = temp;
	}
	return valid;
}
			
/**
 * @brief Read logic battery voltage limits
 * 
 * Retrieves the current minimum and maximum voltage limits for the logic battery.
 * These limits determine when the controller generates warnings or errors.
 * 
 * @param address Controller address
 * @param min Variable to store minimum voltage limit in tenths of a volt
 * @param max Variable to store maximum voltage limit in tenths of a volt
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max)
{
	bool valid = false;
	uint32_t value = Read4(address,GETMINMAXLOGICVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}

/**
 * @brief Sets Motor 1 position PID parameters
 * 
 * Position PID uses 10.14 fixed-point format (multiplier of 1024)
 * 
 * @param address Controller address
 * @param kp_fp Proportional constant (floating point)
 * @param ki_fp Integral constant (floating point)
 * @param kd_fp Derivative constant (floating point)
 * @param kiMax Maximum integral windup limit
 * @param deadzone Error deadzone where no correction is applied
 * @param min Minimum allowed position
 * @param max Maximum allowed position
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)
{
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

/**
 * @brief Set Motor 2 position PID parameters
 * 
 * Configures the PID control parameters for Motor 2 position control.
 * The controller uses 10.14 fixed-point format for PID values (multiplier of 1024).
 * 
 * @param address Controller address
 * @param kp_fp Proportional constant (floating point)
 * @param ki_fp Integral constant (floating point)
 * @param kd_fp Derivative constant (floating point)
 * @param kiMax Maximum integral windup limit
 * @param deadzone Error deadzone where no correction is applied
 * @param min Minimum allowed position
 * @param max Maximum allowed position
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)
{
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

/**
 * @brief Read Motor 1 position PID parameters
 * 
 * Retrieves the current PID parameters for Motor 1 position control.
 * The controller stores PID values in 10.14 fixed-point format,
 * but this function converts them to floating point for ease of use.
 * 
 * @param address Controller address
 * @param Kp_fp Variable to store proportional constant (converted to floating point)
 * @param Ki_fp Variable to store integral constant (converted to floating point)
 * @param Kd_fp Variable to store derivative constant (converted to floating point)
 * @param KiMax Variable to store maximum integral windup limit
 * @param DeadZone Variable to store error deadzone value
 * @param Min Variable to store minimum allowed position
 * @param Max Variable to store maximum allowed position
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

/**
 * @brief Read Motor 2 position PID parameters
 * 
 * Retrieves the current PID parameters for Motor 2 position control.
 * The controller stores PID values in 10.14 fixed-point format,
 * but this function converts them to floating point for ease of use.
 * 
 * @param address Controller address
 * @param Kp_fp Variable to store proportional constant (converted to floating point)
 * @param Ki_fp Variable to store integral constant (converted to floating point)
 * @param Kd_fp Variable to store derivative constant (converted to floating point)
 * @param KiMax Variable to store maximum integral windup limit
 * @param DeadZone Variable to store error deadzone value
 * @param Min Variable to store minimum allowed position
 * @param Max Variable to store maximum allowed position
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

/**
 * @brief Controls motor speed, acceleration, deceleration and position in one command
 * 
 * This function allows for complex motion profiles with controlled acceleration,
 * speed, and deceleration to a specific position.
 * 
 * @param address Controller address
 * @param accel Acceleration in counts per second per second
 * @param speed Maximum speed in counts per second
 * @param deccel Deceleration in counts per second per second
 * @param position Target position in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward
 *             bit 1: 0 = Absolute position, 1 = Relative position
 *             bit 2: 0 = Accel not limited, 1 = Accel limited
 *             bit 3: 0 = No buffer command, 1 = Buffer command
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag)
{
	return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

/**
 * @brief Control Motor 2 with advanced motion profile
 * 
 * Commands Motor 2 to follow a trapezoidal velocity profile with
 * controlled acceleration, cruise speed, and deceleration to a target position.
 * This provides smooth motion with precise positioning.
 * 
 * @param address Controller address
 * @param accel Acceleration in counts per second per second
 * @param speed Maximum cruise speed in counts per second
 * @param deccel Deceleration in counts per second per second
 * @param position Target position in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward
 *             bit 1: 0 = Absolute position, 1 = Relative position
 *             bit 2: 0 = Accel not limited, 1 = Accel limited
 *             bit 3: 0 = No buffer command, 1 = Buffer command
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag)
{
	return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

/**
 * @brief Control both motors with advanced motion profiles
 * 
 * Commands both motors to follow trapezoidal velocity profiles with
 * independent acceleration, cruise speed, deceleration, and target position settings.
 * This allows for complex, coordinated motion with precise positioning of both motors.
 * 
 * @param address Controller address
 * @param accel1 Acceleration for Motor 1 in counts per second per second
 * @param speed1 Maximum cruise speed for Motor 1 in counts per second
 * @param deccel1 Deceleration for Motor 1 in counts per second per second
 * @param position1 Target position for Motor 1 in encoder counts
 * @param accel2 Acceleration for Motor 2 in counts per second per second
 * @param speed2 Maximum cruise speed for Motor 2 in counts per second
 * @param deccel2 Deceleration for Motor 2 in counts per second per second
 * @param position2 Target position for Motor 2 in encoder counts
 * @param flag Control flags:
 *             bit 0: 0 = Forward, 1 = Backward (Motor 1)
 *             bit 1: 0 = Absolute position, 1 = Relative position (Motor 1)
 *             bit 2: 0 = Accel not limited, 1 = Accel limited (Motor 1)
 *             bit 3: 0 = No buffer command, 1 = Buffer command
 *             bit 4: 0 = Forward, 1 = Backward (Motor 2)
 *             bit 5: 0 = Absolute position, 1 = Relative position (Motor 2)
 *             bit 6: 0 = Accel not limited, 1 = Accel limited (Motor 2)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag)
{
	return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

/**
 * @brief Set Motor 1 default acceleration and deceleration
 * 
 * Sets the default acceleration and deceleration rates used when
 * not explicitly specified in a command
 * 
 * @param address Controller address
 * @param accel Default acceleration in counts per second per second
 * @param decel Default deceleration in counts per second per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1DefaultAccel(uint8_t address, uint32_t accel, uint32_t decel)
{
	return write_n(10,address,SETM1DEFAULTACCEL,SetDWORDval(accel),SetDWORDval(decel));
}

/**
 * @brief Set Motor 2 default acceleration and deceleration
 * 
 * Sets the default acceleration and deceleration rates used for Motor 2
 * when not explicitly specified in a command. These values are used
 * by commands that don't include acceleration or deceleration parameters.
 * 
 * @param address Controller address
 * @param accel Default acceleration in counts per second per second
 * @param decel Default deceleration in counts per second per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2DefaultAccel(uint8_t address, uint32_t accel, uint32_t decel)
{
	return write_n(10,address,SETM2DEFAULTACCEL,SetDWORDval(accel),SetDWORDval(decel));
}

/**
 * @brief Read default acceleration and deceleration values for both motors
 * 
 * Retrieves the current default acceleration and deceleration values
 * for both motors in a single command. These are the values used when
 * acceleration/deceleration are not explicitly specified.
 * 
 * @param address Controller address
 * @param accelM1 Variable to store Motor 1 default acceleration
 * @param decelM1 Variable to store Motor 1 default deceleration
 * @param accelM2 Variable to store Motor 2 default acceleration
 * @param decelM2 Variable to store Motor 2 default deceleration
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetDefaultAccels(uint8_t address, uint32_t &accelM1, uint32_t &decelM1, uint32_t &accelM2, uint32_t &decelM2)
{
	return read_n(4,address,GETDEFAULTACCELS,&accelM1,&decelM1,&accelM2,&decelM2);
}

/**
 * @brief Set pin functions for configurable I/O pins
 * 
 * Configures the mode of operation for the programmable I/O pins
 * 
 * @param address Controller address
 * @param S3mode Mode for S3 pin
 * @param S4mode Mode for S4 pin
 * @param S5mode Mode for S5 pin
 * @param D1mode Mode for D1 pin
 * @param D2mode Mode for D2 pin
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode, uint8_t D1mode, uint8_t D2mode)
{
	return write_n(7,address,SETPINFUNCTIONS,S3mode,S4mode,S5mode,D1mode,D2mode);
}

/**
 * @brief Get pin function settings
 * 
 * Retrieves the current mode settings for all configurable I/O pins.
 * Each pin has multiple possible modes that determine its function.
 * 
 * @param address Controller address
 * @param S3mode Variable to store S3 pin mode
 * @param S4mode Variable to store S4 pin mode
 * @param S5mode Variable to store S5 pin mode
 * @param D1mode Variable to store D1 pin mode
 * @param D2mode Variable to store D2 pin mode
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode, uint8_t &D1mode, uint8_t &D2mode)
{
    return read_n_bytes(5, address, GETPINFUNCTIONS, &S3mode, &S4mode, &S5mode, &D1mode, &D2mode);
}

/**
 * @brief Set controller settings for both motors
 * 
 * Sets detailed control parameters for both motors including deadband,
 * limits, and center positions.
 * 
 * @param address Controller address
 * @param minDBM1 Minimum deadband value for Motor 1 (8-bit)
 * @param maxDBM1 Maximum deadband value for Motor 1 (8-bit)
 * @param minLimitsM1 Minimum limits for Motor 1 (16-bit)
 * @param maxLimitsM1 Maximum limits for Motor 1 (16-bit)
 * @param centerM1 Center position for Motor 1 (16-bit)
 * @param minM1 Minimum value for Motor 1 (16-bit)
 * @param maxM1 Maximum value for Motor 1 (16-bit)
 * @param minDBM2 Minimum deadband value for Motor 2 (8-bit)
 * @param maxDBM2 Maximum deadband value for Motor 2 (8-bit)
 * @param minLimitsM2 Minimum limits for Motor 2 (16-bit) 
 * @param maxLimitsM2 Maximum limits for Motor 2 (16-bit)
 * @param centerM2 Center position for Motor 2 (16-bit)
 * @param minM2 Minimum value for Motor 2 (16-bit)
 * @param maxM2 Maximum value for Motor 2 (16-bit)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetCtrlSettings(uint8_t address, 
                              uint8_t minDBM1, uint8_t maxDBM1, 
                              uint16_t minLimitsM1, uint16_t maxLimitsM1, 
                              uint16_t centerM1, uint16_t minM1, uint16_t maxM1,
                              uint8_t minDBM2, uint8_t maxDBM2, 
                              uint16_t minLimitsM2, uint16_t maxLimitsM2, 
                              uint16_t centerM2, uint16_t minM2, uint16_t maxM2)
{
    return write_n(26, address, SETCTRLSETTINGS, 
                 minDBM1, maxDBM1,
                 SetWORDval(minLimitsM1), SetWORDval(maxLimitsM1), 
                 SetWORDval(centerM1), SetWORDval(minM1), SetWORDval(maxM1),
                 minDBM2, maxDBM2,
                 SetWORDval(minLimitsM2), SetWORDval(maxLimitsM2), 
                 SetWORDval(centerM2), SetWORDval(minM2), SetWORDval(maxM2));
}

/**
 * @brief Get controller settings for both motors
 * 
 * Retrieves detailed control parameters for both motors including deadband,
 * limits, and center positions.
 * 
 * @param address Controller address
 * @param minDBM1 Variable to store minimum deadband value for Motor 1 (8-bit)
 * @param maxDBM1 Variable to store maximum deadband value for Motor 1 (8-bit)
 * @param minLimitsM1 Variable to store minimum limits for Motor 1 (16-bit)
 * @param maxLimitsM1 Variable to store maximum limits for Motor 1 (16-bit)
 * @param centerM1 Variable to store center position for Motor 1 (16-bit)
 * @param minM1 Variable to store minimum value for Motor 1 (16-bit)
 * @param maxM1 Variable to store maximum value for Motor 1 (16-bit)
 * @param minDBM2 Variable to store minimum deadband value for Motor 2 (8-bit)
 * @param maxDBM2 Variable to store maximum deadband value for Motor 2 (8-bit)
 * @param minLimitsM2 Variable to store minimum limits for Motor 2 (16-bit)
 * @param maxLimitsM2 Variable to store maximum limits for Motor 2 (16-bit)
 * @param centerM2 Variable to store center position for Motor 2 (16-bit)
 * @param minM2 Variable to store minimum value for Motor 2 (16-bit)
 * @param maxM2 Variable to store maximum value for Motor 2 (16-bit)
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetCtrlSettings(uint8_t address, 
                              uint8_t &minDBM1, uint8_t &maxDBM1, 
                              uint16_t &minLimitsM1, uint16_t &maxLimitsM1, 
                              uint16_t &centerM1, uint16_t &minM1, uint16_t &maxM1,
                              uint8_t &minDBM2, uint8_t &maxDBM2,
                              uint16_t &minLimitsM2, uint16_t &maxLimitsM2, 
                              uint16_t &centerM2, uint16_t &minM2, uint16_t &maxM2)
{
    uint8_t trys = MAXRETRY;
    
    do {
        write_address_cmd(address, GETCTRLSETTINGS);
        
        // Read Motor 1 deadband settings (8-bit values)
        int16_t data = read(timeout);
        if (data == -1) continue;
        crc_update(data);
        minDBM1 = data;
        
        data = read(timeout);
        if (data == -1) continue;
        crc_update(data);
        maxDBM1 = data;
        
        // Read all 16-bit values using ReadWord
        if (!ReadWord(minLimitsM1)) continue;
        if (!ReadWord(maxLimitsM1)) continue;
        if (!ReadWord(centerM1)) continue;
        if (!ReadWord(minM1)) continue;
        if (!ReadWord(maxM1)) continue;
        
        // Read Motor 2 deadband settings (8-bit values)
        data = read(timeout);
        if (data == -1) continue;
        crc_update(data);
        minDBM2 = data;
        
        data = read(timeout);
        if (data == -1) continue;
        crc_update(data);
        maxDBM2 = data;
        
        // Read Motor 2 16-bit values using ReadWord
        if (!ReadWord(minLimitsM2)) continue;
        if (!ReadWord(maxLimitsM2)) continue;
        if (!ReadWord(centerM2)) continue;
        if (!ReadWord(minM2)) continue;
        if (!ReadWord(maxM2)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if (!ReadWord(ccrc)) continue;
        
        if (crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read encoder values for both motors
 * 
 * Retrieves the current encoder count values for both motors in a single command.
 * These values represent the current position of each motor.
 * 
 * @param address Controller address
 * @param enc1 Variable to store Motor 1 encoder value
 * @param enc2 Variable to store Motor 2 encoder value
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2)
{
	return read_n(2,address,GETENCODERS,&enc1,&enc2);
}

/**
 * @brief Read instantaneous speed values for both motors
 * 
 * Retrieves the current instantaneous speed values for both motors in a single command.
 * These are the raw speed readings before any filtering is applied.
 * 
 * @param address Controller address
 * @param ispeed1 Variable to store Motor 1 instantaneous speed in counts per second
 * @param ispeed2 Variable to store Motor 2 instantaneous speed in counts per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2)
{
	return read_n(2,address,GETISPEEDS,&ispeed1,&ispeed2);
}

/**
 * @brief Restore default settings
 * 
 * Resets all controller settings to factory defaults
 * Uses a magic number to prevent accidental resets
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::RestoreDefaults(uint8_t address)
{
	return write_n(2,address,RESTOREDEFAULTS,0xE22EAB7A);
}

/**
 * @brief Read temperature sensor 1
 * 
 * Retrieves the value from temperature sensor 1.
 * The returned value is in tenths of a degree Celsius.
 * 
 * @param address Controller address
 * @param temp Variable to store temperature (e.g., 245 = 24.5°C)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadTemp(uint8_t address, uint16_t &temp)
{
	bool valid = false;
	temp = Read2(address,GETTEMP,&valid);
	return valid;
}

/**
 * @brief Read temperature sensor 2
 * 
 * Retrieves the value from temperature sensor 2 (if available).
 * The returned value is in tenths of a degree Celsius.
 * Note: Not all controller models have a second temperature sensor.
 * 
 * @param address Controller address
 * @param temp Variable to store temperature (e.g., 245 = 24.5°C)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadTemp2(uint8_t address, uint16_t &temp)
{
	bool valid = false;
	temp = Read2(address,GETTEMP2,&valid);
	return valid;
}

/**
 * @brief Read controller error state
 * 
 * Returns a 32-bit value with error and warning flags
 * Each bit represents a specific error or warning condition
 * 
 * @param address Controller address
 * @param valid Optional pointer to store communication validity
 * @return 32-bit error flags, see error code definitions
 */
uint32_t Basicmicro::ReadError(uint8_t address,bool *valid)
{
	return Read4(address,GETERROR,valid);
}

/**
 * @brief Read encoder modes
 * 
 * Retrieves the current encoder mode settings for both motors.
 * Mode values:
 * 0 = Quadrature encoder
 * 1 = Single-ended (pulse/direction)
 * 
 * @param address Controller address
 * @param M1mode Variable to store Motor 1 encoder mode
 * @param M2mode Variable to store Motor 2 encoder mode
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode)
{
	bool valid = false;
	uint16_t value = Read2(address,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

/**
 * @brief Set encoder mode for Motor 1
 * 
 * Sets the encoder interface mode for Motor 1.
 * Mode values:
 * 0 = Quadrature encoder
 * 1 = Single-ended (pulse/direction)
 * 
 * @param address Controller address
 * @param mode Encoder mode (0 or 1)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1EncoderMode(uint8_t address,uint8_t mode)
{
	return write_n(3,address,SETM1ENCODERMODE,mode);
}

/**
 * @brief Set encoder mode for Motor 2
 * 
 * Sets the encoder interface mode for Motor 2.
 * This determines how the controller interprets encoder signals.
 * 
 * Mode values:
 * 0 = Quadrature encoder (standard mode, uses both channels)
 * 1 = Single-ended (pulse/direction mode)
 * 
 * @param address Controller address
 * @param mode Encoder mode (0 or 1)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2EncoderMode(uint8_t address,uint8_t mode)
{
	return write_n(3,address,SETM2ENCODERMODE,mode);
}

/**
 * @brief Write settings to non-volatile memory
 * 
 * Saves the current settings to non-volatile memory so they persist across power cycles
 * Uses a magic number to prevent accidental writes
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::WriteNVM(uint8_t address)
{
	return write_n(6,address,WRITENVM, SetDWORDval(0xE22EAB7A) );
}

/**
 * @brief Read settings from non-volatile memory
 * 
 * Reloads settings from non-volatile memory, discarding any unsaved changes
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadNVM(uint8_t address)
{
	return write_n(2,address,READNVM);
}

/**
 * @brief Set configuration value
 * 
 * Sets the controller configuration value.
 * The configuration value is a bit field with various settings.
 * 
 * @param address Controller address
 * @param config Configuration value
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetConfig(uint8_t address, uint16_t config)
{
	return write_n(4,address,SETCONFIG,SetWORDval(config));
}

/**
 * @brief Get configuration value
 * 
 * Retrieves the current controller configuration value.
 * 
 * @param address Controller address
 * @param config Variable to store configuration value
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetConfig(uint8_t address, uint16_t &config)
{
	bool valid = false;
	uint16_t value = Read2(address,GETCONFIG,&valid);
	if(valid){
		config = value;
	}
	return valid;
}

/**
 * @brief Set maximum current limits for Motor 1
 * 
 * Sets the maximum current draw limits in milliamps.
 * The controller will limit power to keep current below these values.
 * 
 * @param address Controller address
 * @param max Maximum current limit in milliamps
 * @param min Return current limit in milliamps (regen braking)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1MaxCurrent(uint8_t address,uint32_t max,uint32_t min)
{
	return write_n(10,address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(min));
}

/**
 * @brief Set maximum current limits for Motor 2
 * 
 * Sets the maximum current draw limits in milliamps for Motor 2.
 * The controller will limit power to keep current below these values,
 * protecting the motor and driver circuitry from excessive current draw.
 * 
 * @param address Controller address
 * @param max Maximum current limit in milliamps for driving (forward/reverse)
 * @param min Return current limit in milliamps for regenerative braking
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2MaxCurrent(uint8_t address,uint32_t max,uint32_t min)
{
	return write_n(10,address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(min));
}

/**
 * @brief Read maximum current limits for Motor 1
 * 
 * Retrieves the maximum current draw limits in milliamps.
 * 
 * @param address Controller address
 * @param max Variable to store maximum current limit in milliamps
 * @param min Variable to store return current limit in milliamps (regen braking)
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadM1MaxCurrent(uint8_t address,uint32_t &max,uint32_t&min)
{
	uint32_t tmax,tmin;
	bool valid = read_n(2,address,GETM1MAXCURRENT,&tmax,&tmin);
	if(valid){
		max = tmax;
		min = tmin;
	}
	return valid;
}

/**
 * @brief Read maximum current limits for Motor 2
 * 
 * Retrieves the current maximum current draw limits in milliamps for Motor 2.
 * These limits determine when the controller will restrict power to the motor.
 * 
 * @param address Controller address
 * @param max Variable to store maximum current limit in milliamps for driving
 * @param min Variable to store return current limit in milliamps for braking
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadM2MaxCurrent(uint8_t address,uint32_t &max,uint32_t&min)
{
	uint32_t tmax,tmin;
	bool valid = read_n(2,address,GETM2MAXCURRENT,&tmax,&tmin);
	if(valid){
		max = tmax;
		min = tmin;
	}
	return valid;
}

/**
 * @brief Set PWM mode
 * 
 * Sets the PWM mode for each motor
 * Valid modes include:
 * 0 = Standard PWM, 1 = Complementary PWM (locked anti-phase)
 * 
 * @param address Controller address
 * @param modeM1 PWM mode for Motor 1
 * @param modeM2 PWM mode for Motor 2
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPWMMode(uint8_t address, uint8_t modeM1, uint8_t modeM2)
{
	return write_n(4,address,SETPWMMODE,modeM1,modeM2);
}

/**
 * @brief Get PWM mode
 * 
 * Retrieves the current PWM mode for each motor
 * 
 * @param address Controller address
 * @param modeM1 Variable to store Motor 1 PWM mode
 * @param modeM2 Variable to store Motor 2 PWM mode
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetPWMMode(uint8_t address, uint8_t &modeM1, uint8_t &modeM2)
{
	bool valid = false;
	uint8_t value = Read2(address,GETPWMMODE,&valid);
	if(valid){
		modeM1 = value>>8;
		modeM2 = value;
	}
	return valid;
}

/**
 * @brief Set auxiliary pin duty cycles
 * 
 * Sets the PWM duty cycles for auxiliary output pins
 * 
 * @param address Controller address
 * @param S3duty Duty cycle for S3 pin (0-32767)
 * @param S4duty Duty cycle for S4 pin (0-32767)
 * @param S5duty Duty cycle for S5 pin (0-32767)
 * @param CTRL1duty Duty cycle for CTRL1 pin (0-32767)
 * @param CTRL2duty Duty cycle for CTRL2 pin (0-32767)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetAUXDutys(uint8_t address, uint16_t S3duty, uint16_t S4duty, uint16_t S5duty, uint16_t CTRL1duty, uint16_t CTRL2duty)
{
	return write_n(12,address,SETAUXDUTYS,SetWORDval(S3duty),SetWORDval(S4duty),SetWORDval(S5duty),SetWORDval(CTRL1duty),SetWORDval(CTRL2duty));
}

/**
 * @brief Read auxiliary pin duty cycles
 * 
 * Retrieves the current duty cycle settings for the auxiliary output pins.
 * 
 * @param address Controller address
 * @param S3duty Variable to store S3 pin duty cycle
 * @param S4duty Variable to store S4 pin duty cycle
 * @param S5duty Variable to store S5 pin duty cycle
 * @param CTRL1duty Variable to store CTRL1 pin duty cycle
 * @param CTRL2duty Variable to store CTRL2 pin duty cycle
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetAUXDutys(uint8_t address, uint16_t &S3duty, uint16_t &S4duty, uint16_t &S5duty, uint16_t &CTRL1duty, uint16_t &CTRL2duty)
{
    return read_n_words(5, address, GETAUXDUTYS, &S3duty, &S4duty, &S5duty, &CTRL1duty, &CTRL2duty);
}

/**
 * @brief Set communication timeout
 * 
 * Sets the communication timeout value for the controller.
 * This timeout determines how long the controller will wait for
 * communication before triggering a failsafe action.
 * 
 * @param address Controller address
 * @param timeout Timeout value in seconds (0.0 to 25.5 seconds)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetTimeout(uint8_t address, float timeout)
{
    // Timeout is stored in tenths of a second (0-255)
    uint8_t value = (uint8_t)(timeout * 10);
    return write_n(3, address, SETTIMEOUT, value);
}

/**
 * @brief Get communication timeout
 * 
 * Retrieves the current communication timeout value from the controller.
 * 
 * @param address Controller address
 * @param timeout Variable to store the timeout value in seconds
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetTimeout(uint8_t address, float &timeout)
{
    bool valid = false;
    uint8_t value = Read1(address, GETTIMEOUT, &valid);
    if(valid) {
        // Convert from tenths of a second to seconds
        timeout = value / 10.0f;
        return true;
    }
    return false;
}

/**
 * @brief Set default speed for Motor 1
 * 
 * Sets the default speed value for Motor 1. This value is used when
 * a speed is not explicitly specified in a command.
 * 
 * @param address Controller address
 * @param speed Default speed in encoder counts per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1DefaultSpeed(uint8_t address, uint32_t speed)
{
    return write_n(6, address, SETM1DEFAULTSPEED, SetDWORDval(speed));
}

/**
 * @brief Set default speed for Motor 2
 * 
 * Sets the default speed value for Motor 2. This value is used when
 * a speed is not explicitly specified in a command.
 * 
 * @param address Controller address
 * @param speed Default speed in encoder counts per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2DefaultSpeed(uint8_t address, uint32_t speed)
{
    return write_n(6, address, SETM2DEFAULTSPEED, SetDWORDval(speed));
}

/**
 * @brief Read default speeds for both motors
 * 
 * Retrieves the current default speed values for both motors.
 * These values are used when speeds are not explicitly specified in commands.
 * 
 * @param address Controller address
 * @param speed1 Variable to store Motor 1 default speed in encoder counts per second
 * @param speed2 Variable to store Motor 2 default speed in encoder counts per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetDefaultSpeeds(uint8_t address, uint32_t &speed1, uint32_t &speed2)
{
    return read_n(2, address, GETDEFAULTSPEEDS, &speed1, &speed2);
}

/**
 * @brief Read comprehensive status from controller
 * 
 * Retrieves a comprehensive set of status information from the controller
 * including controller state, voltages, temperatures, encoder counts,
 * speed values, PWM values, and error metrics.
 * 
 * @param address Controller address
 * @param tick Variable to store tick counter value
 * @param state Variable to store controller state flags
 * @param temp1 Variable to store temperature sensor 1 value (in tenths of °C)
 * @param temp2 Variable to store temperature sensor 2 value (in tenths of °C)
 * @param mainBattVoltage Variable to store main battery voltage (in tenths of a volt)
 * @param logicBattVoltage Variable to store logic battery voltage (in tenths of a volt)
 * @param pwm1 Variable to store Motor 1 PWM value (-32768 to +32767)
 * @param pwm2 Variable to store Motor 2 PWM value (-32768 to +32767)
 * @param current1 Variable to store Motor 1 current in milliamps
 * @param current2 Variable to store Motor 2 current in milliamps
 * @param enc1 Variable to store Motor 1 encoder count
 * @param enc2 Variable to store Motor 2 encoder count
 * @param speedSetpoint1 Variable to store Motor 1 target speed
 * @param speedSetpoint2 Variable to store Motor 2 target speed
 * @param speed1 Variable to store Motor 1 actual speed
 * @param speed2 Variable to store Motor 2 actual speed
 * @param speedError1 Variable to store Motor 1 speed error
 * @param speedError2 Variable to store Motor 2 speed error
 * @param posError1 Variable to store Motor 1 position error
 * @param posError2 Variable to store Motor 2 position error
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetStatus(uint8_t address, uint32_t &tick, uint32_t &state, 
                          uint16_t &temp1, uint16_t &temp2,
                          uint16_t &mainBattVoltage, uint16_t &logicBattVoltage, 
                          int16_t &pwm1, int16_t &pwm2, int16_t &current1, int16_t &current2,
                          uint32_t &enc1, uint32_t &enc2, uint32_t &speedSetpoint1, uint32_t &speedSetpoint2,
                          uint32_t &speed1, uint32_t &speed2, uint16_t &speedError1, uint16_t &speedError2,
                          uint16_t &posError1, uint16_t &posError2)
{
    uint8_t trys = MAXRETRY;
    uint16_t rawPwm1, rawPwm2, rawCurrent1, rawCurrent2;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETSTATUS);
        crc_update(GETSTATUS);
        
        // Read all values using our utility functions
        if(!ReadLong(tick)) continue;
        if(!ReadLong(state)) continue;
        
        if(!ReadWord(temp1)) continue;
        if(!ReadWord(temp2)) continue;
        if(!ReadWord(mainBattVoltage)) continue;
        if(!ReadWord(logicBattVoltage)) continue;
        
        if(!ReadWord(rawPwm1)) continue;
        if(!ReadWord(rawPwm2)) continue;
        if(!ReadWord(rawCurrent1)) continue;
        if(!ReadWord(rawCurrent2)) continue;
        
        if(!ReadLong(enc1)) continue;
        if(!ReadLong(enc2)) continue;
        if(!ReadLong(speedSetpoint1)) continue;
        if(!ReadLong(speedSetpoint2)) continue;
        if(!ReadLong(speed1)) continue;
        if(!ReadLong(speed2)) continue;
        
        if(!ReadWord(speedError1)) continue;
        if(!ReadWord(speedError2)) continue;
        if(!ReadWord(posError1)) continue;
        if(!ReadWord(posError2)) continue;
        
        // Convert raw unsigned PWM and current values to signed values
        pwm1 = (int16_t)rawPwm1;
        pwm2 = (int16_t)rawPwm2;
        current1 = (int16_t)rawCurrent1;
        current2 = (int16_t)rawCurrent2;
        
        // Verify CRC using the ReadWord utility function
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set the controller serial number
 * 
 * Sets the device serial number (up to 36 characters).
 * Serial number will be padded with nulls if less than 36 bytes.
 * 
 * @param address Controller address
 * @param serialNumber String containing the serial number to set (max 36 chars)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetSerialNumber(uint8_t address, const char* serialNumber)
{
    uint8_t trys = MAXRETRY;
    uint8_t length = strlen(serialNumber);
    
    // Limit length to 36 characters
    if(length > 36)
        length = 36;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(SETSERIALNUMBER);
        crc_update(SETSERIALNUMBER);
        
        // Write length byte
        write(length);
        crc_update(length);
        
        // Write serial number bytes
        for(uint8_t i = 0; i < 36; i++) {
            uint8_t c = (i < length) ? serialNumber[i] : 0; // Pad with zeros if shorter than 36
            write(c);
            crc_update(c);
        }
        
        // Write CRC and check for acknowledgment
        if(_writechecksum()) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Get the controller serial number
 * 
 * Reads the device serial number (36 bytes).
 * 
 * @param address Controller address
 * @param serialNumber Buffer to store the serial number string (should be at least 37 bytes for null terminator)
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetSerialNumber(uint8_t address, char* serialNumber)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETSERIALNUMBER);
        crc_update(GETSERIALNUMBER);
        
        // Read length byte
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        uint8_t length = data;
        
        // Read 36 bytes for the serial number
        uint8_t buffer[36];
        bool readError = false;
        
        for(uint8_t i = 0; i < 36; i++) {
            data = read(timeout);
            if(data == -1) {
                readError = true;
                break;
            }
            crc_update(data);
            buffer[i] = data;
        }
        
        if(readError) continue;
        
        // Verify CRC using the ReadWord utility function
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            // Copy the active part of the serial number to the output buffer
            // and null-terminate it
            for(uint8_t i = 0; i < length; i++) {
                serialNumber[i] = buffer[i];
            }
            serialNumber[length] = 0;
            
            return true;
        }
    } while(trys--);
    
    // Ensure null termination even on failure
    if(serialNumber)
        serialNumber[0] = 0;
    
    return false;
}

/**
 * @brief Read battery voltage levels
 * 
 * Retrieves the main battery and logic battery voltage levels 
 * in a single command.
 * 
 * @param address Controller address
 * @param mainBattVoltage Variable to store main battery voltage in tenths of a volt
 * @param logicBattVoltage Variable to store logic battery voltage in tenths of a volt
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetVolts(uint8_t address, uint16_t &mainBattVoltage, uint16_t &logicBattVoltage)
{
    uint8_t trys = MAXRETRY;
    do {
        write_address_cmd(address, GETVOLTS);
        
        // Read voltage values
        if (!ReadWord(mainBattVoltage)) continue;
        if (!ReadWord(logicBattVoltage)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if (!ReadWord(ccrc)) continue;
        
        if (crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read temperature sensor values
 * 
 * Retrieves the values from both temperature sensors in a single command.
 * 
 * @param address Controller address
 * @param temp1 Variable to store temperature sensor 1 value in tenths of a degree Celsius
 * @param temp2 Variable to store temperature sensor 2 value in tenths of a degree Celsius
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetTemps(uint8_t address, uint16_t &temp1, uint16_t &temp2)
{
    uint8_t trys = MAXRETRY;
    do {
        write_address_cmd(address, GETTEMPS);
        
        // Read temperature values
        if (!ReadWord(temp1)) continue;
        if (!ReadWord(temp2)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if (!ReadWord(ccrc)) continue;
        
        if (crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read encoder error statuses
 * 
 * Retrieves the error status flags for both encoders.
 * 
 * @param address Controller address
 * @param enc1Status Variable to store encoder 1 status flags
 * @param enc2Status Variable to store encoder 2 status flags
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetEncStatus(uint8_t address, uint8_t &enc1Status, uint8_t &enc2Status)
{
    return read_n_bytes(2, address, GETENCSTATUS, &enc1Status, &enc2Status);
}

/**
 * @brief Set Auto Mode 1 value
 * 
 * Sets the auto mode 1 configuration value.
 * 
 * @param address Controller address
 * @param value Auto mode 1 configuration value
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetAuto1(uint8_t address, uint32_t value)
{
    return write_n(6, address, SETAUTO1, SetDWORDval(value));
}

/**
 * @brief Set Auto Mode 2 value
 * 
 * Sets the auto mode 2 configuration value.
 * 
 * @param address Controller address
 * @param value Auto mode 2 configuration value
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetAuto2(uint8_t address, uint32_t value)
{
    return write_n(6, address, SETAUTO2, SetDWORDval(value));
}

/**
 * @brief Read Auto Mode values
 * 
 * Retrieves the auto mode configuration values for both channels.
 * 
 * @param address Controller address
 * @param auto1 Variable to store auto mode 1 configuration value
 * @param auto2 Variable to store auto mode 2 configuration value
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetAutos(uint8_t address, uint32_t &auto1, uint32_t &auto2)
{
    return read_n(2, address, GETAUTOS, &auto1, &auto2);
}

/**
 * @brief Read current speeds for both motors
 * 
 * Retrieves the current speed values for both motors in a single command.
 * 
 * @param address Controller address
 * @param speed1 Variable to store Motor 1 speed in encoder counts per second
 * @param speed2 Variable to store Motor 2 speed in encoder counts per second
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetSpeeds(uint8_t address, uint32_t &speed1, uint32_t &speed2)
{
    return read_n(2, address, GETSPEEDS, &speed1, &speed2);
}

/**
 * @brief Set speed error limits
 * 
 * Sets the maximum allowable speed error for both motors.
 * If the error exceeds this limit, an error flag will be set.
 * 
 * @param address Controller address
 * @param limit1 Speed error limit for Motor 1
 * @param limit2 Speed error limit for Motor 2
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetSpeedErrorLimit(uint8_t address, uint16_t limit1, uint16_t limit2)
{
    return write_n(6, address, SETSPEEDERRORLIMIT, SetWORDval(limit1), SetWORDval(limit2));
}

/**
 * @brief Read speed error limits
 * 
 * Retrieves the current speed error limit settings for both motors.
 * 
 * @param address Controller address
 * @param limit1 Variable to store Motor 1 speed error limit
 * @param limit2 Variable to store Motor 2 speed error limit
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetSpeedErrorLimit(uint8_t address, uint16_t &limit1, uint16_t &limit2)
{
    return read_n_words(2, address, GETSPEEDERRORLIMIT, &limit1, &limit2);
}

/**
 * @brief Read current speed errors
 * 
 * Retrieves the current speed error values for both motors.
 * 
 * @param address Controller address
 * @param error1 Variable to store Motor 1 speed error
 * @param error2 Variable to store Motor 2 speed error
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetSpeedErrors(uint8_t address, uint16_t &error1, uint16_t &error2)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETSPEEDERRORS);
        crc_update(GETSPEEDERRORS);
        
        // Read error1 (16-bit)
        if(!ReadWord(error1)) continue;
        
        // Read error2 (16-bit)
        if(!ReadWord(error2)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set position error limits
 * 
 * Sets the maximum allowable position error for both motors.
 * If the error exceeds this limit, an error flag will be set.
 * 
 * @param address Controller address
 * @param limit1 Position error limit for Motor 1
 * @param limit2 Position error limit for Motor 2
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPosErrorLimit(uint8_t address, uint16_t limit1, uint16_t limit2)
{
    return write_n(6, address, SETPOSERRORLIMIT, SetWORDval(limit1), SetWORDval(limit2));
}

/**
 * @brief Read position error limits
 * 
 * Retrieves the current position error limit settings for both motors.
 * 
 * @param address Controller address
 * @param limit1 Variable to store Motor 1 position error limit
 * @param limit2 Variable to store Motor 2 position error limit
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetPosErrorLimit(uint8_t address, uint16_t &limit1, uint16_t &limit2)
{
    return read_n_words(2, address, GETPOSERRORLIMIT, &limit1, &limit2);
}

/**
 * @brief Read current position errors
 * 
 * Retrieves the current position error values for both motors.
 * 
 * @param address Controller address
 * @param error1 Variable to store Motor 1 position error
 * @param error2 Variable to store Motor 2 position error
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetPosErrors(uint8_t address, uint16_t &error1, uint16_t &error2)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETPOSERRORS);
        crc_update(GETPOSERRORS);
        
        // Read error1 (16-bit)
        if(!ReadWord(error1)) continue;
        
        // Read error2 (16-bit)
        if(!ReadWord(error2)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set encoder offsets
 * 
 * Sets offset values for both encoders. These offsets are applied
 * to the encoder signals to fine-tune positioning.
 * 
 * @param address Controller address
 * @param offset1 Offset value for encoder 1 (0-255)
 * @param offset2 Offset value for encoder 2 (0-255)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetOffsets(uint8_t address, uint8_t offset1, uint8_t offset2)
{
    return write_n(4, address, SETOFFSETS, offset1, offset2);
}

/**
 * @brief Read encoder offsets
 * 
 * Retrieves the current offset values for both encoders.
 * 
 * @param address Controller address
 * @param offset1 Variable to store encoder 1 offset value
 * @param offset2 Variable to store encoder 2 offset value
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetOffsets(uint8_t address, uint8_t &offset1, uint8_t &offset2)
{
    return read_n_bytes(2, address, GETOFFSETS, &offset1, &offset2);
}

/**
 * @brief Move Motor 1 to absolute position
 * 
 * Commands Motor 1 to move to an absolute encoder position using
 * the default speed and acceleration values.
 * 
 * @param address Controller address
 * @param position Target encoder count position
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::M1Position(uint8_t address, uint32_t position, uint8_t buffer)
{
    return write_n(7, address, M1POS, SetDWORDval(position), buffer);
}

/**
 * @brief Move Motor 2 to absolute position
 * 
 * Commands Motor 2 to move to an absolute encoder position using
 * the default speed and acceleration values.
 * 
 * @param address Controller address
 * @param position Target encoder count position
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::M2Position(uint8_t address, uint32_t position, uint8_t buffer)
{
    return write_n(7, address, M2POS, SetDWORDval(position), buffer);
}

/**
 * @brief Move both motors to absolute positions
 * 
 * Commands both motors to move to absolute encoder positions using
 * the default speed and acceleration values.
 * 
 * @param address Controller address
 * @param position1 Target encoder count position for Motor 1
 * @param position2 Target encoder count position for Motor 2
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::MixedPosition(uint8_t address, uint32_t position1, uint32_t position2, uint8_t buffer)
{
    return write_n(11, address, MIXEDPOS, SetDWORDval(position1), SetDWORDval(position2), buffer);
}

/**
 * @brief Move Motor 1 to position at specified speed
 * 
 * Commands Motor 1 to move to an absolute encoder position
 * with the specified maximum speed.
 * 
 * @param address Controller address
 * @param speed Maximum speed in encoder counts per second
 * @param position Target encoder count position
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::M1SpeedPosition(uint8_t address, uint32_t speed, uint32_t position, uint8_t buffer)
{
    return write_n(11, address, M1SPEEDPOS, SetDWORDval(speed), SetDWORDval(position), buffer);
}

/**
 * @brief Move Motor 2 to position at specified speed
 * 
 * Commands Motor 2 to move to an absolute encoder position
 * with the specified maximum speed.
 * 
 * @param address Controller address
 * @param speed Maximum speed in encoder counts per second
 * @param position Target encoder count position
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::M2SpeedPosition(uint8_t address, uint32_t speed, uint32_t position, uint8_t buffer)
{
    return write_n(11, address, M2SPEEDPOS, SetDWORDval(speed), SetDWORDval(position), buffer);
}

/**
 * @brief Move both motors to positions at specified speeds
 * 
 * Commands both motors to move to absolute encoder positions
 * with independent maximum speeds for each motor.
 * 
 * @param address Controller address
 * @param speed1 Maximum speed for Motor 1 in encoder counts per second
 * @param position1 Target encoder count position for Motor 1
 * @param speed2 Maximum speed for Motor 2 in encoder counts per second
 * @param position2 Target encoder count position for Motor 2
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::MixedSpeedPosition(uint8_t address, uint32_t speed1, uint32_t position1, uint32_t speed2, uint32_t position2, uint8_t buffer)
{
    return write_n(19, address, MIXEDSPEEDPOS, SetDWORDval(speed1), SetDWORDval(position1), SetDWORDval(speed2), SetDWORDval(position2), buffer);
}

/**
 * @brief Move Motor 1 to a percentage position
 * 
 * Commands Motor 1 to move to a position specified as a percentage
 * of the configured position range.
 * 
 * @param address Controller address
 * @param position Target position as a percentage (-32768 to +32767)
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::M1PercentPosition(uint8_t address, int16_t position, uint8_t buffer)
{
    return write_n(5, address, M1PPOS, SetWORDval(position), buffer);
}

/**
 * @brief Move Motor 2 to a percentage position
 * 
 * Commands Motor 2 to move to a position specified as a percentage
 * of the configured position range.
 * 
 * @param address Controller address
 * @param position Target position as a percentage (-32768 to +32767)
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::M2PercentPosition(uint8_t address, int16_t position, uint8_t buffer)
{
    return write_n(5, address, M2PPOS, SetWORDval(position), buffer);
}

/**
 * @brief Move both motors to percentage positions
 * 
 * Commands both motors to move to positions specified as percentages
 * of their configured position ranges.
 * 
 * @param address Controller address
 * @param position1 Target position for Motor 1 as a percentage (-32768 to +32767)
 * @param position2 Target position for Motor 2 as a percentage (-32768 to +32767)
 * @param buffer Command buffer control:
 *        0 = Execute immediately
 *        1 = Add to buffer
 * @return true if successful, false otherwise
 */
bool Basicmicro::MixedPercentPosition(uint8_t address, int16_t position1, int16_t position2, uint8_t buffer)
{
    return write_n(7, address, MIXEDPPOS, SetWORDval(position1), SetWORDval(position2), buffer);
}

/**
 * @brief Set Motor 1 inductance and resistance values
 * 
 * Sets the motor inductance and resistance parameters for Motor 1.
 * These values are used for advanced motor control algorithms.
 * 
 * @param address Controller address
 * @param L Inductance value in Henries
 * @param R Resistance value in Ohms
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM1LR(uint8_t address, float L, float R)
{
    // Convert floating point values to fixed-point format (24.8)
    uint32_t lval = (uint32_t)(L * 0x1000000);
    uint32_t rval = (uint32_t)(R * 0x1000000);
    
    return write_n(10, address, SETM1LR, SetDWORDval(lval), SetDWORDval(rval));
}

/**
 * @brief Set Motor 2 inductance and resistance values
 * 
 * Sets the motor inductance and resistance parameters for Motor 2.
 * These values are used for advanced motor control algorithms.
 * 
 * @param address Controller address
 * @param L Inductance value in Henries
 * @param R Resistance value in Ohms
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetM2LR(uint8_t address, float L, float R)
{
    // Convert floating point values to fixed-point format (24.8)
    uint32_t lval = (uint32_t)(L * 0x1000000);
    uint32_t rval = (uint32_t)(R * 0x1000000);
    
    return write_n(10, address, SETM2LR, SetDWORDval(lval), SetDWORDval(rval));
}

/**
 * @brief Read Motor 1 inductance and resistance values
 * 
 * Retrieves the current inductance and resistance parameters for Motor 1.
 * 
 * @param address Controller address
 * @param L Variable to store the inductance value in Henries
 * @param R Variable to store the resistance value in Ohms
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetM1LR(uint8_t address, float &L, float &R)
{
    uint32_t lval, rval;
    
    if(read_n(2, address, GETM1LR, &lval, &rval)) {
        // Convert from fixed-point format back to floating point
        L = (float)lval / 0x1000000;
        R = (float)rval / 0x1000000;
        return true;
    }
    
    return false;
}

/**
 * @brief Read Motor 2 inductance and resistance values
 * 
 * Retrieves the current inductance and resistance parameters for Motor 2.
 * 
 * @param address Controller address
 * @param L Variable to store the inductance value in Henries
 * @param R Variable to store the resistance value in Ohms
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetM2LR(uint8_t address, float &L, float &R)
{
    uint32_t lval, rval;
    
    if(read_n(2, address, GETM2LR, &lval, &rval)) {
        // Convert from fixed-point format back to floating point
        L = (float)lval / 0x1000000;
        R = (float)rval / 0x1000000;
        return true;
    }
    
    return false;
}

/**
 * @brief Set digital output
 * 
 * Sets the action for a digital output pin.
 * 
 * @param address Controller address
 * @param index Output index
 * @param action Action to perform
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetDOUT(uint8_t address, uint8_t index, uint8_t action)
{
    return write_n(4, address, SETDOUT, index, action);
}

/**
 * @brief Read digital outputs configuration
 * 
 * Retrieves the current digital output actions.
 * 
 * @param address Controller address
 * @param count Variable to store the number of actions
 * @param actions Array to store the action values
 * @param maxActions Maximum number of actions that can be stored in the array
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetDOUTS(uint8_t address, uint8_t &count, uint8_t *actions, uint8_t maxActions)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETDOUTS);
        crc_update(GETDOUTS);
        
        // Read count
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        count = data;
        
        // Check if we have enough space in the provided array
        uint8_t readCount = (count <= maxActions) ? count : maxActions;
        
        // Read action values
        for(uint8_t i = 0; i < count; i++) {
            data = read(timeout);
            if(data == -1) {
                break;
            }
            crc_update(data);
            
            if(i < readCount) {
                actions[i] = data;
            }
        }
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set priority levels
 * 
 * Sets the priority levels for different control modes.
 * 
 * @param address Controller address
 * @param priority1 Priority level 1
 * @param priority2 Priority level 2
 * @param priority3 Priority level 3
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPriority(uint8_t address, uint8_t priority1, uint8_t priority2, uint8_t priority3)
{
    return write_n(5, address, SETPRIORITY, priority1, priority2, priority3);
}

/**
 * @brief Read priority levels
 * 
 * Retrieves the current priority level settings.
 * 
 * @param address Controller address
 * @param priority1 Variable to store priority level 1
 * @param priority2 Variable to store priority level 2
 * @param priority3 Variable to store priority level 3
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetPriority(uint8_t address, uint8_t &priority1, uint8_t &priority2, uint8_t &priority3)
{
    return read_n_bytes(3, address, GETPRIORITY, &priority1, &priority2, &priority3);
}

/**
 * @brief Set address and mixing mode
 * 
 * Sets the controller address and mixing mode.
 * 
 * @param address Controller address
 * @param newAddress New controller address
 * @param enableMixing Mixing enable flag (0 or 1)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetAddressMixed(uint8_t address, uint8_t newAddress, uint8_t enableMixing)
{
    return write_n(4, address, SETADDRESSEDMIXED, newAddress, enableMixing);
}

/**
 * @brief Read address and mixing mode
 * 
 * Retrieves the current controller address and mixing mode.
 * 
 * @param address Controller address
 * @param newAddress Variable to store the current controller address
 * @param mixingEnabled Variable to store the mixing enabled flag
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetAddressMixed(uint8_t address, uint8_t &newAddress, uint8_t &mixingEnabled)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETADDRESSEDMIXED);
        crc_update(GETADDRESSEDMIXED);
        
        // Read values
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        newAddress = data;
        
        data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        mixingEnabled = data;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set signal parameters
 * 
 * Configures a signal channel with detailed parameters.
 * 
 * @param address Controller address
 * @param index Signal index
 * @param signalType Signal type
 * @param mode Mode setting
 * @param target Target setting
 * @param minAction Minimum action value
 * @param maxAction Maximum action value
 * @param lowpass Lowpass filter setting
 * @param timeout Timeout value
 * @param loadhome Load home position
 * @param minVal Minimum value
 * @param maxVal Maximum value
 * @param center Center value
 * @param deadband Deadband value
 * @param powerexp Power exponent
 * @param minout Minimum output
 * @param maxout Maximum output
 * @param powermin Minimum power
 * @param potentiometer Potentiometer value
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetSignal(uint8_t address, uint8_t index, uint8_t signalType, uint8_t mode, 
                          uint8_t target, uint16_t minAction, uint16_t maxAction, uint8_t lowpass, 
                          uint32_t timeout, int32_t loadhome, int32_t minVal, int32_t maxVal, 
                          int32_t center, uint32_t deadband, uint32_t powerexp, uint32_t minout, 
                          uint32_t maxout, uint32_t powermin, uint32_t potentiometer)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(SETSIGNAL);
        crc_update(SETSIGNAL);
        
        // Write index and type
        write(index);
        crc_update(index);
        write(signalType);
        crc_update(signalType);
        
        // Write mode and target
        write(mode);
        crc_update(mode);
        write(target);
        crc_update(target);
        
        // Write min/max action (16-bit)
        _writeword(minAction);
        _writeword(maxAction);
        
        // Write lowpass
        write(lowpass);
        crc_update(lowpass);
        
        // Write timeout (32-bit)
        _writelong(timeout);
        
        // Write loadhome, min, max, center (32-bit signed)
        _writelong(loadhome);
        _writelong(minVal);
        _writelong(maxVal);
        _writelong(center);
        
        // Write remaining 32-bit values
        _writelong(deadband);
        _writelong(powerexp);
        _writelong(minout);
        _writelong(maxout);
        _writelong(powermin);
        _writelong(potentiometer);
        
        // Write CRC and check for acknowledgment
        if(_writechecksum()) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read signal configurations
 * 
 * Retrieves the configurations for all signal channels.
 * 
 * @param address Controller address
 * @param count Variable to store the number of signal configurations
 * @param signals Array to store the signal configurations
 * @param maxSignals Maximum number of signal configurations that can be stored in the array
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetSignals(uint8_t address, uint8_t &count, SignalConfig *signals, uint8_t maxSignals)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETSIGNALS);
        crc_update(GETSIGNALS);
        
        // Read count
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        count = data;
        
        // Check if we have enough space in the provided array
        uint8_t readCount = (count <= maxSignals) ? count : maxSignals;
        bool readError = false;
        
        // Read signal configurations
        for(uint8_t i = 0; i < count; i++) {
            SignalConfig config;
            
            // Read type
            data = read(timeout);
            if(data == -1) {
                readError = true;
                break;
            }
            crc_update(data);
            config.type = data;
            
            // Read mode
            data = read(timeout);
            if(data == -1) {
                readError = true;
                break;
            }
            crc_update(data);
            config.mode = data;
            
            // Read target
            data = read(timeout);
            if(data == -1) {
                readError = true;
                break;
            }
            crc_update(data);
            config.target = data;
            
            // Read minAction (16-bit)
            if(!ReadWord(config.minAction)) {
                readError = true;
                break;
            }
            
            // Read maxAction (16-bit)
            if(!ReadWord(config.maxAction)) {
                readError = true;
                break;
            }
            
            // Read lowpass
            data = read(timeout);
            if(data == -1) {
                readError = true;
                break;
            }
            crc_update(data);
            config.lowpass = data;
            
            // Read timeout (32-bit)
            if(!ReadLong(config.timeout)) {
                readError = true;
                break;
            }
            
            // Read loadhome (32-bit signed)
            uint32_t tempVal;
            if(!ReadLong(tempVal)) {
                readError = true;
                break;
            }
            config.loadhome = (int32_t)tempVal;
            
            // Read minVal (32-bit signed)
            if(!ReadLong(tempVal)) {
                readError = true;
                break;
            }
            config.minVal = (int32_t)tempVal;
            
            // Read maxVal (32-bit signed)
            if(!ReadLong(tempVal)) {
                readError = true;
                break;
            }
            config.maxVal = (int32_t)tempVal;
            
            // Read center (32-bit signed)
            if(!ReadLong(tempVal)) {
                readError = true;
                break;
            }
            config.center = (int32_t)tempVal;
            
            // Read remaining 32-bit values
            if(!ReadLong(config.deadband) || !ReadLong(config.powerexp) || 
               !ReadLong(config.minout) || !ReadLong(config.maxout) || 
               !ReadLong(config.powermin) || !ReadLong(config.potentiometer)) {
                readError = true;
                break;
            }
            
            // Store the configuration if we have space
            if(i < readCount) {
                signals[i] = config;
            }
        }
        
        if(readError) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set stream parameters
 * 
 * Configures a stream channel with communication parameters.
 * 
 * @param address Controller address
 * @param index Stream index
 * @param streamType Stream type
 * @param baudrate Baud rate for the stream
 * @param timeout Timeout value
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetStream(uint8_t address, uint8_t index, uint8_t streamType, uint32_t baudrate, uint32_t timeout)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(SETSTREAM);
        crc_update(SETSTREAM);
        
        // Write index and type
        write(index);
        crc_update(index);
        write(streamType);
        crc_update(streamType);
        
        // Write baudrate and timeout (32-bit)
        _writelong(baudrate);
        _writelong(timeout);
        
        // Write CRC and check for acknowledgment
        if(_writechecksum()) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read stream configurations
 * 
 * Retrieves the configurations for all stream channels.
 * 
 * @param address Controller address
 * @param count Variable to store the number of stream configurations
 * @param streams Array to store the stream configurations
 * @param maxStreams Maximum number of stream configurations that can be stored in the array
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetStreams(uint8_t address, uint8_t &count, StreamConfig *streams, uint8_t maxStreams)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETSTREAMS);
        crc_update(GETSTREAMS);
        
        // Read count
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        count = data;
        
        // Check if we have enough space in the provided array
        uint8_t readCount = (count <= maxStreams) ? count : maxStreams;
        bool readError = false;
        
        // Read stream configurations
        for(uint8_t i = 0; i < count; i++) {
            StreamConfig config;
            
            // Read type
            data = read(timeout);
            if(data == -1) {
                readError = true;
                break;
            }
            crc_update(data);
            config.type = data;
            
            // Read baudrate and timeout (32-bit)
            if(!ReadLong(config.baudrate) || !ReadLong(config.timeout)) {
                readError = true;
                break;
            }
            
            // Store the configuration if we have space
            if(i < readCount) {
                streams[i] = config;
            }
        }
        
        if(readError) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read signals data
 * 
 * Retrieves the current data for all signal channels.
 * 
 * @param address Controller address
 * @param count Variable to store the number of signal data entries
 * @param signalsData Array to store the signal data
 * @param maxSignals Maximum number of signal data entries that can be stored in the array
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetSignalsData(uint8_t address, uint8_t &count, SignalData *signalsData, uint8_t maxSignals)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETSIGNALSDATA);
        crc_update(GETSIGNALSDATA);
        
        // Read count
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        count = data;
        
        // Check if we have enough space in the provided array
        uint8_t readCount = (count <= maxSignals) ? count : maxSignals;
        bool readError = false;
        
        // Read signal data
        for(uint8_t i = 0; i < count; i++) {
            SignalData sData;
            
            // Read command, position, percent, speed, speeds (all 32-bit)
            if(!ReadLong(sData.command) || !ReadLong(sData.position) || 
               !ReadLong(sData.percent) || !ReadLong(sData.speed) || 
               !ReadLong(sData.speeds)) {
                readError = true;
                break;
            }
            
            // Store the data if we have space
            if(i < readCount) {
                signalsData[i] = sData;
            }
        }
        
        if(readError) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set CAN node ID
 * 
 * Sets the node ID used for CAN communications.
 * 
 * @param address Controller address
 * @param nodeID CAN node ID (0-255)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetNodeID(uint8_t address, uint8_t nodeID)
{
    return write_n(3, address, SETNODEID, nodeID);
}

/**
 * @brief Read CAN node ID
 * 
 * Retrieves the current CAN node ID.
 * 
 * @param address Controller address
 * @param nodeID Variable to store the CAN node ID
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetNodeID(uint8_t address, uint8_t &nodeID)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETNODEID);
        crc_update(GETNODEID);
        
        // Read node ID
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        nodeID = data;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set PWM idle parameters
 * 
 * Configures the PWM idle behavior for both motors.
 * 
 * @param address Controller address
 * @param idleDelay1 Idle delay for Motor 1 in seconds (0-12.7s)
 * @param idleMode1 Idle mode for Motor 1 (false = Coast, true = Brake)
 * @param idleDelay2 Idle delay for Motor 2 in seconds (0-12.7s)
 * @param idleMode2 Idle mode for Motor 2 (false = Coast, true = Brake)
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetPWMIdle(uint8_t address, float idleDelay1, bool idleMode1, float idleDelay2, bool idleMode2)
{
    // Convert delays to tenths of a second (0-127) and combine with mode bit
    uint8_t byte1 = (uint8_t)(idleDelay1 * 10.0f) & 0x7F;
    if(idleMode1) byte1 |= 0x80;  // Set high bit for Brake mode
    
    uint8_t byte2 = (uint8_t)(idleDelay2 * 10.0f) & 0x7F;
    if(idleMode2) byte2 |= 0x80;  // Set high bit for Brake mode
    
    return write_n(4, address, SETPWMIDLE, byte1, byte2);
}

/**
 * @brief Read PWM idle parameters
 * 
 * Retrieves the current PWM idle settings for both motors.
 * 
 * @param address Controller address
 * @param idleDelay1 Variable to store idle delay for Motor 1 in seconds
 * @param idleMode1 Variable to store idle mode for Motor 1 (false = Coast, true = Brake)
 * @param idleDelay2 Variable to store idle delay for Motor 2 in seconds
 * @param idleMode2 Variable to store idle mode for Motor 2 (false = Coast, true = Brake)
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetPWMIdle(uint8_t address, float &idleDelay1, bool &idleMode1, float &idleDelay2, bool &idleMode2)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETPWMIDLE);
        crc_update(GETPWMIDLE);
        
        // Read byte 1
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        uint8_t byte1 = data;
        
        // Read byte 2
        data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        uint8_t byte2 = data;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            // Extract delay values (lower 7 bits) and convert from tenths to seconds
            idleDelay1 = (byte1 & 0x7F) / 10.0f;
            idleDelay2 = (byte2 & 0x7F) / 10.0f;
            
            // Extract mode flags (high bit)
            idleMode1 = (byte1 & 0x80) != 0;
            idleMode2 = (byte2 & 0x80) != 0;
            
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Get CAN buffer state
 * 
 * Retrieves the number of pending CAN packets in the buffer.
 * 
 * @param address Controller address
 * @param count Variable to store the number of pending packets
 * @return true if successful, false otherwise
 */
bool Basicmicro::CANBufferState(uint8_t address, uint8_t &count)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(CANBUFFERSTATE);
        crc_update(CANBUFFERSTATE);
        
        // Read packet count
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        count = data;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Send a CAN packet
 * 
 * Transmits a CAN packet through the controller's CAN interface.
 * 
 * @param address Controller address
 * @param cobID CAN Object Identifier (11-bit standard or 29-bit extended)
 * @param rtr Remote Transmission Request flag (0 or 1)
 * @param length Number of data bytes (0-8)
 * @param data Array containing the data bytes
 * @return true if successful, false otherwise
 */
bool Basicmicro::CANPutPacket(uint8_t address, uint16_t cobID, uint8_t rtr, uint8_t length, const uint8_t *data)
{
    uint8_t trys = MAXRETRY;
    
    // Validate length
    if(length > 8) length = 8;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(CANPUTPACKET);
        crc_update(CANPUTPACKET);
        
        // Write CAN ID (16-bit)
        _writeword(cobID);
        
        // Write RTR flag and length
        write(rtr);
        crc_update(rtr);
        write(length);
        crc_update(length);
        
        // Write data bytes (up to 8)
        for(uint8_t i = 0; i < 8; i++) {
            uint8_t byteVal = (i < length) ? data[i] : 0; // Use 0 for padding if less than 8 bytes
            write(byteVal);
            crc_update(byteVal);
        }
        
        // Write CRC and check for acknowledgment
        if(_writechecksum()) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Receive a CAN packet
 * 
 * Retrieves a CAN packet from the controller's CAN interface.
 * 
 * @param address Controller address
 * @param cobID Variable to store the CAN Object Identifier
 * @param rtr Variable to store the Remote Transmission Request flag
 * @param length Variable to store the number of data bytes
 * @param data Array to store the data bytes (must be at least 8 bytes)
 * @return true if successful, false otherwise
 */
bool Basicmicro::CANGetPacket(uint8_t address, uint16_t &cobID, uint8_t &rtr, uint8_t &length, uint8_t *data)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(CANGETPACKET);
        crc_update(CANGETPACKET);
        
        // Read packet status (should be 0xFF for valid packet)
        int16_t val = read(timeout);
        if(val == -1) continue;
        crc_update(val);
        
        if(val != 0xFF) {
            // No valid packet available
            
            // Still need to read CRC to clear communication
            uint16_t ccrc;
            if(!ReadWord(ccrc)) continue;
            
            // Return false to indicate no packet
            return false;
        }
        
        // Read CAN ID (16-bit)
        if(!ReadWord(cobID)) continue;
        
        // Read RTR flag and length
        val = read(timeout);
        if(val == -1) continue;
        crc_update(val);
        rtr = val;
        
        val = read(timeout);
        if(val == -1) continue;
        crc_update(val);
        length = val;
        
        // Validate length
        if(length > 8) length = 8;
        
        // Read data bytes (always 8 bytes, padded with zeros if needed)
        for(uint8_t i = 0; i < 8; i++) {
            val = read(timeout);
            if(val == -1) break;
            crc_update(val);
            data[i] = val;
        }
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Write to CANopen local dictionary
 * 
 * Writes a value to the CANopen object dictionary entry.
 * 
 * @param address Controller address
 * @param index Object dictionary index
 * @param subindex Object dictionary subindex
 * @param value Value to write
 * @param size Size of the value (1, 2 or 4 bytes)
 * @param result Variable to store the operation result code
 * @return true if successful, false otherwise
 */
bool Basicmicro::CANOpenWriteLocalDict(uint8_t address, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size, uint32_t &result)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(CANOPENWRITELOCALDICT);
        crc_update(CANOPENWRITELOCALDICT);
        
        // Write index (16-bit)
        _writeword(index);
        
        // Write subindex
        write(subindex);
        crc_update(subindex);
        
        // Write value (32-bit)
        _writelong(value);
        
        // Write size
        write(size);
        crc_update(size);
        
        // Write CRC
        _writeword(crc_get());
        
        // Read the result (32-bit)
        crc_clear();
        uint32_t tempResult = 0;
        
        // Read result (32-bit)
        if(!ReadLong(tempResult)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            result = tempResult;
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Read from CANopen local dictionary
 * 
 * Reads a value from the CANopen object dictionary entry.
 * 
 * @param address Controller address
 * @param index Object dictionary index
 * @param subindex Object dictionary subindex
 * @param value Variable to store the read value
 * @param size Variable to store the size of the value (1, 2 or 4 bytes)
 * @param type Variable to store the data type
 * @param result Variable to store the operation result code
 * @return true if successful, false otherwise
 */
bool Basicmicro::CANOpenReadLocalDict(uint8_t address, uint16_t index, uint8_t subindex, uint32_t &value, uint8_t &size, uint8_t &type, uint32_t &result)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(CANOPENREADLOCALDICT);
        crc_update(CANOPENREADLOCALDICT);
        
        // Write index (16-bit)
        _writeword(index);
        
        // Write subindex
        write(subindex);
        crc_update(subindex);
        
        // Write CRC
        _writeword(crc_get());
        
        // Reset CRC for response
        crc_clear();
        
        // Read the value, size, type and result
        uint32_t tempValue, tempResult;
        uint8_t tempSize, tempType;
        
        // Read value (32-bit)
        if(!ReadLong(tempValue)) continue;
        
        // Read size (8-bit)
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        tempSize = data;
        
        // Read type (8-bit)
        data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        tempType = data;
        
        // Read result (32-bit)
        if(!ReadLong(tempResult)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            value = tempValue;
            size = tempSize;
            type = tempType;
            result = tempResult;
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Reset emergency stop condition
 * 
 * Clears an emergency stop condition if the controller is configured
 * to allow software reset of E-Stop.
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::ResetEStop(uint8_t address)
{
    return write_n(2, address, RESETESTOP);
}

/**
 * @brief Set emergency stop lock state
 * 
 * Configures how the emergency stop state is reset.
 * 
 * @param address Controller address
 * @param lockState Lock state:
 *        0x00 = Hardware reset required
 *        0x55 = Automatic reset
 *        0xAA = Software reset allowed
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetEStopLock(uint8_t address, uint8_t lockState)
{
    return write_n(3, address, SETESTOPLOCK, lockState);
}

/**
 * @brief Read emergency stop lock state
 * 
 * Retrieves the current emergency stop lock configuration.
 * 
 * @param address Controller address
 * @param lockState Variable to store the lock state:
 *        0x00 = Hardware reset required
 *        0x55 = Automatic reset
 *        0xAA = Software reset allowed
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetEStopLock(uint8_t address, uint8_t &lockState)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(GETESTOPLOCK);
        crc_update(GETESTOPLOCK);
        
        // Read lock state
        int16_t data = read(timeout);
        if(data == -1) continue;
        crc_update(data);
        lockState = data;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Set script auto-run time
 * 
 * Configures when an onboard script will automatically start after power-up.
 * 
 * @param address Controller address
 * @param scriptAutorunTime Time in milliseconds after startup to run script
 *        0 = Script does not autorun
 *        Values less than 100 are treated as 0
 * @return true if successful, false otherwise
 */
bool Basicmicro::SetScriptAutorun(uint8_t address, uint32_t scriptAutorunTime)
{
    return write_n(6, address, SETSCRIPTAUTORUN, SetDWORDval(scriptAutorunTime));
}

/**
 * @brief Read script auto-run time
 * 
 * Retrieves the current script auto-run time setting.
 * 
 * @param address Controller address
 * @param scriptAutorunTime Variable to store the time in milliseconds after startup
 * @return true if successful, false otherwise
 */
bool Basicmicro::GetScriptAutorun(uint8_t address, uint32_t &scriptAutorunTime)
{
    return read_n(1, address, GETSCRIPTAUTORUN, &scriptAutorunTime);
}

/**
 * @brief Start script execution
 * 
 * Begins execution of the onboard script.
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::StartScript(uint8_t address)
{
    return write_n(2, address, STARTSCRIPT);
}

/**
 * @brief Stop script execution
 * 
 * Stops execution of the onboard script.
 * 
 * @param address Controller address
 * @return true if successful, false otherwise
 */
bool Basicmicro::StopScript(uint8_t address)
{
    return write_n(2, address, STOPSCRIPT);
}

/**
 * @brief Read value from EEPROM
 * 
 * Reads a 16-bit value from a specified EEPROM address.
 * 
 * @param address Controller address
 * @param eeAddress EEPROM address to read from (0-255)
 * @param value Variable to store the read value
 * @return true if successful, false otherwise
 */
bool Basicmicro::ReadEEPROM(uint8_t address, uint8_t eeAddress, uint16_t &value)
{
    uint8_t trys = MAXRETRY;
    
    do {
        clear();
        crc_clear();
        write(address);
        crc_update(address);
        write(READEEPROM);
        crc_update(READEEPROM);
        
        // Write EEPROM address
        write(eeAddress);
        crc_update(eeAddress);
        
        // Read value (16-bit)
        if(!ReadWord(value)) continue;
        
        // Verify CRC
        uint16_t ccrc;
        if(!ReadWord(ccrc)) continue;
        
        if(crc_get() == ccrc) {
            return true;
        }
    } while(trys--);
    
    return false;
}

/**
 * @brief Write value to EEPROM
 * 
 * Writes a 16-bit value to a specified EEPROM address.
 * 
 * @param address Controller address
 * @param eeAddress EEPROM address to write to (0-255)
 * @param value Value to write
 * @return true if successful, false otherwise
 */
bool Basicmicro::WriteEEPROM(uint8_t address, uint8_t eeAddress, uint16_t value)
{
    return write_n(5, address, WRITEEEPROM, eeAddress, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF));
}