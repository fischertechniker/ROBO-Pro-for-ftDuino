/*
 * ROBO Pro für den ftDuino
 * Implementierung des Fish.X1-Protokolls für den TX Controller
 * (TX Firmware 1.30)
 * 
 * Dirk Fox
 * based on Fish.X1-parser fx1sample 0.3 by ft-ninja
 * 
 * History: 
 * 
 * - 07.09.2022: Publication of v1.1
 * - 02.09.2022: Publication of v1.0
 * - 27.07.2022: Fork of fx1sample 0.3 by ft-ninja
 */

UINT8   rxBuffer[BUF_MAXLEN];  // receive buffer
UINT8   txBuffer[BUF_MAXLEN];  // transmit buffer
UINT8   tmpData[BUF_MAXLEN];

UINT8   *pBuf;  // pointer

UINT16  x1FSMState = FSM_WAIT_STX;    // X1 protocol finite state machine
UINT16  x1SHFSMState = GET_0D_START;  // remote shell finite state machine

UINT16  x1DataLen;    // length of X1 data packet 
UINT16  rxBufLen;     // length of buffer used
UINT16  x1DataCnt;    // number of received data
UINT16  bufCheckSum;  // calculated checksum
UINT16  x1CheckSum;   // transmitted checksum
 
UINT16  x1TransactionID = 0;  // protocol transaction ID (TID)
UINT16  x1SessionID = 0;      // protocol session ID (SID)

UINT16  cnt_reset_req[N_CNT];   // counter: reset requests
BOOL8   counter_config[N_CNT];  // HIGH: count if signal is rising, LOW: count if signal is falling

BOOL8   line_sensor[N_UNI];     // flag indicating that line sensor is attached

UINT16  cnt_ext_motor_cmd[N_MOTOR];  // counter: extended motor commands
BOOL8   motor_config[N_MOTOR];       // TRUE: m[i] pair of corresponding outputs, FALSE: separate PWM outputs
BOOL8   motor_distance[N_MOTOR];     // TRUE: extended motor commands activated
BOOL8   motor_active[N_MOTOR];       // TRUE: extended motor command active, position not yet reached
UINT8   position_reached[N_MOTOR];   // position reached counter

BOOL8   i2c_ch_open;      // flag: (no) open I2C connection
UINT16  i2c_speed = 100;  // current I2C bus transmission speed (default: 100 kHz)

// parse symbol

void fx1Parse(unsigned char symbol) {

  // parse Fish.X1 command packet
  
  switch (x1FSMState) {
    case FSM_WAIT_STX:  // waiting for Fish.X1 start of transmission (STX) trigger
      if (symbol == STX) {
        rxBufLen = 0;     // count number of received bytes
        pBuf = rxBuffer;  // pointer to next free position in receive buffer
        *pBuf = symbol;   // receive buffer: start with 0x02 0x55 ...
        pBuf++;
        x1FSMState = FSM_WAIT_55;
        return;
      }
      break;

    case FSM_WAIT_DATA:  // waiting for Fish.X1 data (payload)
      bufCheckSum += symbol;
      *pBuf = symbol;
      pBuf++;
      x1DataCnt++;
      if (x1DataCnt == x1DataLen)  // end of data packet reached
        x1FSMState = FSM_WAIT_CS_1;
      return;

    case FSM_WAIT_55:  // waiting for Fish.X1 first symbol
      if (symbol != U) {
        x1FSMState = FSM_WAIT_STX;
        break;
      }
      *pBuf = symbol;
      pBuf++;
      x1DataLen = 0;
      bufCheckSum = 0;
      x1FSMState = FSM_WAIT_L_1;
      return;
      
    case FSM_WAIT_L_1:  // waiting for Fish.X1 length field
      x1DataLen = ((symbol << 8) & 0xFF00);
      bufCheckSum += symbol;
      *pBuf = symbol;
      pBuf++;
      x1FSMState = FSM_WAIT_L_2;
      return;
    
    case FSM_WAIT_L_2:  // waiting for Fish.X1 length field (part 2)
      x1DataLen += symbol;
      bufCheckSum += symbol;
      *pBuf = symbol;
      pBuf++;
      x1DataCnt = 0;
      x1FSMState = FSM_WAIT_DATA;
      return;

    case FSM_WAIT_CS_1:  // waiting for Fish.X1 checksunm
      x1CheckSum = ((symbol << 8) & 0xFF00);
      x1FSMState = FSM_WAIT_CS_2;
      return;
  
    case FSM_WAIT_CS_2:  // waiting for Fish.X1 checksunm (part 2)
      x1CheckSum += symbol;
      x1FSMState = FSM_WAIT_ETX;
      return;
  
    case FSM_WAIT_ETX:  // waiting for Fish.X1 end of transmission (ETX) trigger
      if (symbol != 0x03) {
        x1FSMState = FSM_WAIT_STX;
        break;
      }
      bufCheckSum = (~bufCheckSum) + 1;
      if (bufCheckSum != x1CheckSum) {  // compare X1 data packet checksum with calculated checksum
        x1FSMState = FSM_WAIT_STX;
        return;  // checksum error 
      }
      fx1ParseProcPacket(rxBuffer);
      x1FSMState = FSM_WAIT_STX;
      return;
                
    default:
      break;      
  }

  // parse shell command
  
  switch (x1SHFSMState) {
    case GET_0D_END: // if shell command: send predefined reply packet...
      if (symbol != 0x0D) break;
      sendShellData();
      x1SHFSMState = GET_0D_START;      
      break; 
    
    case GET_0D_START:
      if (symbol != 0x0D) break;
      x1SHFSMState = GET_67;
      break;
      
    case GET_67:
      if (symbol != 0x67) {
        x1SHFSMState = GET_0D_START;
        break;
      }
      x1SHFSMState = GET_0D_END;      
      break; 
    
    default:
      break;      
  }
  return;
}

// send function for Fish.X1 data packet

void sendX1Data(int replyCMD, short items, unsigned char *pData, short dataLength) {
  UINT16  CheckSum = 0;
  INT16   i, packetLength = 24;
    
  // Fill the packet structure with data according Fish.X1 protocol structure

  *(txBuffer) = STX;  // write 0x03 (STX) and 0x55 (U) in transmit buffer
  *(txBuffer+1) = U;
  set16BitBE(txBuffer+2, dataLength+20);  // set total length of Fish.X1 packet (data + header)

  set32BitLE(txBuffer+4, 2);  // set FROM Address (2: Master)
  set32BitLE(txBuffer+8, 2);  // set TO Address (2: Master)

  set16BitLE(txBuffer+12, x1TransactionID);  // set TID (= received Transaction ID = Packet Number)
  set16BitLE(txBuffer+14, x1SessionID);      // set SID (= current Session ID)

  set32BitLE(txBuffer+16, replyCMD);  // set Command Code
  set32BitLE(txBuffer+20, items);     // set Number of following Fish.X1 data structures (0/1)

  memcpy((void*)(txBuffer+packetLength), (void*)pData, dataLength);  // copy TA_ID and Fish.X1 data
  packetLength += dataLength;

  CheckSum = calcCheckSum(txBuffer+2, packetLength-2);  // calculate checkSum
  set16BitBE(txBuffer+packetLength, CheckSum);
  *(txBuffer+packetLength+2) = ETX;  // Set ETX
  
  packetLength += 3;
  for (i = 0; i < packetLength; i++)
    Serial.write(txBuffer[i]);  // transmit packet (bytewise)
}

// send function for reply to Shell request "get_ser_num"

void sendShellData(void) {
  short i;
  
  sprintf((char*)txBuffer, "\r\nROBO TX-000/USB>get_ser_num\r\n0000000000\r\nROBO TX-000/USB>");
  for (i = 0; i < 0x3B; i++)
    Serial.write(txBuffer[i]);  // transfer predefined replay data (remote console)
}

// execute callback functions according to command code (CMD) in received packet

void fx1ParseProcPacket(unsigned char *packetPtr) {
  x1TransactionID = get16BitLE(packetPtr+12);                     // get packet number (TID)
  if ((get16BitLE(packetPtr+14) == 0) && (x1TransactionID == 1))  // trigger: if SID 0 and TID 1:
    x1SessionID +=1;                                              // increment current SID

  switch(get32BitLE(packetPtr+16)) {  // select and call reply function for received CMD code
    case CMD_001: CMD_101_Reply();              // reply to connect request
      break;
    case CMD_002: CMD_102_Reply(packetPtr+24);  // reply to input/output request
      break;
    case CMD_005: CMD_105_Reply(packetPtr+24);  // reply to configuration request
      break;
    case CMD_006: CMD_106_Reply(packetPtr+24);  // reply to information request
      break;
    case CMD_007: CMD_107_Reply();              // reply to status request
      break;
    case CMD_019: CMD_119_Reply(packetPtr+24);  // reply to i2c read request
      break;
    case CMD_020: CMD_120_Reply(packetPtr+24);  // reply to i2c write request
      break;
  }
}

// set I2C speed

void set_i2c_speed(BOOL8 fast) {
  if (fast) {  // change to fast mode
    if (i2c_speed == 1) {
      i2c_speed = 4;
      Wire.setClock(400000);
    }
    else {  // change to normal mode
      if (i2c_speed == 4) {
        i2c_speed = 1;
        Wire.setClock(100000);
      }
    }
  }
}

// CMD 001: Connect Request, CMD 101: Connect Reply

void CMD_101_Reply(void) {  // initialize TX: all counters to zero, flags to default, motors and outputs off
  short i;
  
  for (i = 0; i < N_CNT; i++)
    cnt_reset_req[i] = 0;       // reset counter for counter reset requests
    
  for (i = 0; i < N_MOTOR; i++) { 
    cnt_ext_motor_cmd[i] = 0;   // initialize counter for extended motor commands
    motor_distance[i] = false;  // reset flag for extended motor commands (distance)
    motor_active[i] = false;    // reset flag for active extendet motor commands
    position_reached[i] = 0;    // reset position reached counter
  }

  for (i = 0; i < N_PWM_CHAN; i++)
    ftduino.output_set(i, Ftduino::OFF, Ftduino::OFF); 

  i2c_ch_open = false;   // all I2C connections closed
  sendX1Data(CMD_101, 0, (unsigned char*)NULL, 0);  // send empty packet with incremented SID
}

// CMD 002: Input-Output Request, CMD 102: Input-Output Reply

void CMD_102_Reply(unsigned char *data) {
  short i;
  UINT8 pwm, mode; 
  
  CMD_002_Data *request = (CMD_002_Data *) data;           // received data packet; only the first data packet (Master) is considered
  CMD_102_Data *reply = (CMD_102_Data *) tmpData;          // reply data packet
  memset((unsigned char*)reply, 0, sizeof(CMD_102_Data));  // initialise data packet with zeros

  // check first if any active distance command reached position

  for (i = 0; i < N_MOTOR; i++) {  
    if (motor_active[i] && !ftduino.motor_counter_active(i)) {
        motor_active[i] = false;
        position_reached[i]++;
    }
  }

  // counter resets

  for (i = 0; i < N_CNT; i++) {
    if (request->X1Data.cnt_reset_cmd_id[i] > cnt_reset_req[i]) {  // reset counters if requested
      cnt_reset_req[i] = request->X1Data.cnt_reset_cmd_id[i];
      ftduino.counter_clear(i);
      reply->X1Data.cnt_resetted[i] = 1;  // indicate fulfillment of counter reset in reply packet
    }
  }

  // motor commands

  // motor synchronisation requests are not supported by ftDuino, yet
  // all synchronized motors are merely set to the pwm value of their "master" motors

  for (i = 0; i < N_MOTOR; i++) {  

    // output port settings (motor_config[i] == false)

    if (!motor_config[i]) {  
      if (request->X1Data.duty[2*i] > 0) {
        pwm = map(request->X1Data.duty[2*i], 0, 512, Ftduino::OFF, Ftduino::ON);
        ftduino.output_set(2*i, Ftduino::HI, pwm);
      } else ftduino.output_set(2*i, Ftduino::OFF, Ftduino::OFF);

      if (request->X1Data.duty[2*i+1] > 0) {
        pwm = map(request->X1Data.duty[2*i+1], 0, 512, Ftduino::OFF, Ftduino::ON);
        ftduino.output_set(2*i+1, Ftduino::HI, pwm);
      } else ftduino.output_set(2*i+1, Ftduino::OFF, Ftduino::OFF);
    }

    // motor port settings (motor_config[i] == true)

    else {

      // start extended motor command (cnt_ext_motor_cmd_id was increased)
      // (set_brake is active by default)

      if ((request->X1Data.cnt_ext_motor_cmd_id[i] > cnt_ext_motor_cmd[i]) && !motor_active[i]) {
        cnt_ext_motor_cmd[i] = request->X1Data.cnt_ext_motor_cmd_id[i];
        motor_distance[i] = true;  // status flag: accecpt only motor distance commands
        if (request->X1Data.duty[2*i] > 0) {
          pwm = map(request->X1Data.duty[2*i], 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::LEFT;
        } else if (request->X1Data.duty[2*i] < 0) {  // negative pwm value: invert direction
          pwm = map(abs(request->X1Data.duty[2*i]), 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::RIGHT;
        } else if (request->X1Data.duty[2*i+1] > 0) {
          pwm = map(request->X1Data.duty[2*i+1], 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::RIGHT;
        } else if (request->X1Data.duty[2*i+1] < 0) {  // negative pwm value: invert direction
          pwm = map(abs(request->X1Data.duty[2*i+1]), 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::LEFT;
        } else {  // duty = 0: brake
          pwm = Ftduino::OFF;
          mode = Ftduino::BRAKE;
        }
        if (request->X1Data.distance[i] == 0) {  // if distance = 0: 
          position_reached[i]++;                 // increment position reached counter directly 
          motor_distance[i] = false;             // status flag: accept standard motor commands
          ftduino.counter_clear(i);              // counter reset
          reply->X1Data.cnt_resetted[i] = 1;     // indicate counter reset in reply packet
        } else {
          ftduino.motor_counter(i, mode, pwm, request->X1Data.distance[i]);  // extended motor distance command
          motor_active[i] = true;
        }
      } 

      // 'normal' motor commands, if no distance command active
    
      else if (!motor_distance[i] && !motor_active[i]) {
        if (request->X1Data.duty[2*i] > 0) {
          pwm = map(request->X1Data.duty[2*i], 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::LEFT;
        } else if (request->X1Data.duty[2*i] < 0) {  // negative pwm value: invert direction
          pwm = map(abs(request->X1Data.duty[2*i]), 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::RIGHT;
        } else if (request->X1Data.duty[2*i+1] > 0) {
          pwm = map(request->X1Data.duty[2*i+1], 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::RIGHT;
        } else if (request->X1Data.duty[2*i+1] < 0) {  // negative pwm value: invert direction
          pwm = map(abs(request->X1Data.duty[2*i+1]), 0, 512, Ftduino::OFF, Ftduino::MAX);
          mode = Ftduino::LEFT;
        } else {  // duty = 0, brake
          pwm = Ftduino::OFF;
          mode = Ftduino::BRAKE;
        }
        ftduino.motor_set(i, mode, pwm);
      }
    }    
  }

  // fill reply packet
  
  reply->TAId = TA_LOCAL;  // set TA_ID

  // input ports
  
  for (i = 0; i < N_UNI; i++) {  // read input values
    if (line_sensor[i]) {
      if (ftduino.input_get(i) < LINE_SENSOR_THRESHOLD)
        reply->X1Data.uni[i] = 0;
      else
        reply->X1Data.uni[i] = 1;
    } else reply->X1Data.uni[i] = ftduino.input_get(i);
  }

  // fast counters

  for (i = 0; i < N_CNT; i++) {  // read counter values
    reply->X1Data.cnt_in[i] = (ftduino.counter_get_state(i) == counter_config[i]);  // get counter states with respect to their configuration
    reply->X1Data.counter[i] = ftduino.counter_get(i);                              // get current counter value
  }

  // position reached counter

  for (i = 0; i < N_MOTOR; i++) {  // set position reached counter in reply packet
    reply->X1Data.motor_pos_reached[i] = position_reached[i];  
  }

  sendX1Data(CMD_102, 1, (unsigned char*)reply, sizeof(CMD_102_Data));  // send Fish.X1 reply packet
}

// CMD 005: Configuration Request, CMD 105: Configuration Reply

void CMD_105_Reply(unsigned char *data) {
  short i;
  
  CMD_005_Data *request = (CMD_005_Data *) data;  // execute configuration selection
  
  for (i = 0; i < N_MOTOR; i++) {  // memorize motor configuration
    motor_config[i] = request->X1Data.motor[i];
  }

  for (i = 0; i < N_UNI; i++) {  // set selected input mode
    switch (request->X1Data.uni[i]) {
      case 0x00:
        ftduino.input_set_mode(i, Ftduino::VOLTAGE);
        line_sensor[i] = false;
        break;
      case 0x01:
        ftduino.input_set_mode(i, Ftduino::RESISTANCE);
        line_sensor[i] = false;
        break;
      case 0x80:
        ftduino.input_set_mode(i, Ftduino::VOLTAGE);
        line_sensor[i] = true;
        break;
      case 0x81:
        ftduino.input_set_mode(i, Ftduino::SWITCH);
        line_sensor[i] = false;
        break;
    }
  }

  for (i = 0; i < N_CNT; i++) {  // set selected counter mode
    if (request->X1Data.cnt[i]) {
      counter_config[i] = LOW;
      ftduino.counter_set_mode(i, Ftduino::C_EDGE_FALLING);
    } else {
      counter_config[i] = HIGH;
      ftduino.counter_set_mode(i, Ftduino::C_EDGE_RISING);
    }
  }

  CMD_105_Data *reply = (CMD_105_Data *) tmpData;  // reply data packet
  reply->TAId = TA_LOCAL;                          // set TA_ID (TA_LOCAL = Master TX)
  
  sendX1Data(CMD_105, 1, (unsigned char*)reply, sizeof(CMD_105_Data));  // send Fish.X1 reply packet
}

// CMD 006: Information Request, CMD 106: Information Reply

void CMD_106_Reply(unsigned char *data) {
  CMD_006_Data *request = (CMD_006_Data *) data;
  
  if (request->TAId != TA_LOCAL)  // check if information request is for TA_LOCAL (Master)
    return;                       // return otherwise
  
  CMD_106_Data *reply = (CMD_106_Data *) tmpData;          // reply data packet
  memset((unsigned char*)reply, 0, sizeof(CMD_106_Data));  // initialise data packet with zeros

  reply->TAId = TA_LOCAL;  // set TA_ID := TA_LOCAL (Master)

  sprintf(reply->X1Data.device_name, "ROBO TX-000");    // fake identifier
  sprintf(reply->X1Data.bt_addr, "00:00:00:00:00:00");  // invalid bluetooth mac address
  
  reply->X1Data.ta_array_start_addr = 0;  // no array start address
  reply->X1Data.pgm_area_start_addr = 0;  // no program area array start address
  reply->X1Data.pgm_area_size = 0;        // no area size
  
  reply->X1Data.version.hardware.part.a = 0x43;  // = 'C' (fischertechnik TX hardware version)
  reply->X1Data.version.hardware.part.b = 0x00;
  reply->X1Data.version.hardware.part.c = 0x00;
  reply->X1Data.version.hardware.part.d = 0x00; 
  
  reply->X1Data.version.firmware.part.a = 0x00; 
  reply->X1Data.version.firmware.part.b = 0x06;  
  reply->X1Data.version.firmware.part.c = 0x01; 
  reply->X1Data.version.firmware.part.d = 0x1E;  // = 1.30.0.6 - latest TX firmware

  reply->X1Data.version.ta.part.a = 0x01;
  reply->X1Data.version.ta.part.b = 0x01;
  reply->X1Data.version.ta.part.c = 0x01;
  reply->X1Data.version.ta.part.d = 0x08;  // = 1.8.1.1
  
  sendX1Data(CMD_106, 1, (unsigned char*)reply, sizeof(CMD_106_Data));  // send Fish.X1 reply packet
}

// CMD 007: Status Request, CMD 107: Status Reply

void CMD_107_Reply(void) {
  CMD_107_Data *reply = (CMD_107_Data *) tmpData;  // reply data packet

  // set extension status and bluetooth status = 0, TA_ID = TA_LOCAL
  memset((unsigned char*)reply, 0, sizeof(CMD_107_Data));
  
  sendX1Data(CMD_107, 1, (unsigned char*)reply, sizeof(CMD_107_Data));  // send Fish.X1 reply packet
}

// CMD 019: I2C Read Request, CMD 119: I2C Read Reply

void CMD_119_Reply(unsigned char *data) {
  TA_I2C_CMD *request = (TA_I2C_CMD *) data;
  TA_I2C_DATA *reply = (TA_I2C_DATA *) tmpData;  // reply data packet
  memset((unsigned char*)reply, 0, sizeof(TA_I2C_DATA));  // initialise data packet with zeros

  if (!i2c_ch_open)  // if no open I2C connection
    set_i2c_speed(request->i2c_config & 0x80);

  if (request->i2c_config & 0x03) {
    Wire.beginTransmission(request->i2c_address);
    if (request->i2c_config & 0x02)
      Wire.write(request->i2c_subaddress[1]);
    Wire.write(request->i2c_subaddress[0]);
    Wire.endTransmission();
  }
  if (request->i2c_config & 0x08) {
    Wire.requestFrom(request->i2c_address, 2);
    while (Wire.available() < 2);
    reply->i2c_data[1] = Wire.read();
    reply->i2c_data[0] = Wire.read();
  } 
  else if (request->i2c_config & 0x04) {
    Wire.requestFrom(request->i2c_address, 1);
    while (Wire.available() < 0);
    reply->i2c_data[0] = Wire.read();
  }
    
  sendX1Data(CMD_119, 1, (unsigned char*)reply, sizeof(TA_I2C_DATA));  // send Fish.X1 reply packet
}

// CMD 020: I2C Write Request, CMD 120: I2C Write Reply

void CMD_120_Reply(unsigned char *data) {
  TA_I2C_CMD *request = (TA_I2C_CMD *) data;
  TA_I2C_DATA *reply = (TA_I2C_DATA *) tmpData;  // reply data packet
  memset((unsigned char*)reply, 0, sizeof(TA_I2C_DATA));  // initialise data packet with zeros
  
  if (!i2c_ch_open)  // if no open I2C connection
    set_i2c_speed(request->i2c_config & 0x80);

  Wire.beginTransmission(request->i2c_address);
  if (request->i2c_config & 0x03) {
    if (request->i2c_config & 0x02)
      Wire.write(request->i2c_subaddress[1]);
    Wire.write(request->i2c_subaddress[0]);
  }
  if (request->i2c_config & 0x0C) {
    if (request->i2c_config & 0x08)
      Wire.write(request->i2c_data[1]);
    Wire.write(request->i2c_data[0]);  // write at least one byte (8 bit data)
  }
  Wire.endTransmission();

  memcpy((void*)(request->i2c_data), (void*)reply, 2);  // copy data to reply packet
  sendX1Data(CMD_120, 1, (unsigned char*)reply, 4);     // send Fish.X1 reply packet
}
