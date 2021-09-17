#include <epickRTU.h>
#include <unistd.h>
#include <modbus/modbus.h>
#include <string.h>


epick::epick(std::string serial_filepath,int device_id,int baud_rate,char parity,int bits,int stop_bits){

   
    char* serial_chr = new char[serial_filepath.length() + 1];
    std::copy(serial_filepath.begin(), serial_filepath.end(), serial_chr);

    serial  = modbus_new_rtu(serial_chr, baud_rate, parity, bits, stop_bits);

    if (serial == NULL) {
       fprintf(stderr, "Unable to create the libmodbus context\n");       
    }

    timeval t;
    t.tv_sec = 6;

    modbus_set_slave(serial, device_id);
    modbus_set_response_timeout(serial, &t);
    modbus_rtu_set_serial_mode(serial, MODBUS_RTU_RS485);
    modbus_set_debug(serial,FALSE);
    modbus_flush(serial);    

    running = false;

    command.rATR = 0;
    command.rGTO = 0;
    command.rMOD = 0;
    command.rACT = 0;
    command.rPR  = 0;
    command.rSP  = 0;
    command.rFR  = 0;
    
}


int epick::connect(){

    if (modbus_connect(serial) == -1) {
    fprintf(stderr, "Connection epick::connect() failed: %s\n", modbus_strerror(errno));
    modbus_free(serial);
    return -1;
    }
    running = true;
    return 0;

}


void epick::exit(){

    int rc;
    uint16_t tab_reg[64];

    running = false;
    drive_thread.join();

    modbus_flush(serial);
    modbus_close(serial);
    modbus_free(serial);

}

void epick::configure(int MOD,int PR,int SR,int FR){

    command.rMOD = MOD;    
    command.rPR = PR;
    command.rSP = SR;
    command.rFR = FR;
    target_vacuum = PR;   

}

void epick::grip(){

    command.rACT = 1;
    command.rGTO = 1;
    command.rPR = target_vacuum;
}

void epick::release(){

    command.rACT = 1;
    command.rGTO = 1;
    command.rPR = 255;
}

void epick::setCommand(int ATR,int MOD,int GTO,int ATC,int PR,int SP,int FR){

    command.rATR = ATR;
    command.rGTO = GTO;
    command.rMOD = MOD;
    command.rACT = ATC;
    command.rPR  = PR;
    command.rSP  = SP;
    command.rFR  = FR;
}


void epick::updateDrive(){
    
    uint16_t tab_reg[3];
    uint16_t status_reg[3];
    int rc;

    while(running){   

        tab_reg[0] = ( ( command.rACT + (command.rMOD << 1) + (command.rGTO << 3) + (command.rATR << 4) ) << 8 ) + 0;        
        tab_reg[1] = ( 0 << 8 ) +  command.rPR;       
        tab_reg[2] = command.rSP << 8 + command.rFR;

        rc =  modbus_write_registers(serial,INPUT_REGISTERS,3,tab_reg);
        if(rc == -1)  fprintf(stderr, "Write registers failed: %s\n", modbus_strerror(errno));

        rc =  modbus_read_input_registers(serial, OUTPUT_REGISTERS, 3, status_reg);
        if(rc == -1)  fprintf(stderr, "Write registers failed: %s\n", modbus_strerror(errno)); 

        status.gACT = ( ((status_reg[0] & 0xFF00) >> 8) >> 0) & 0x01;
        status.gMOD = ( ((status_reg[0] & 0xFF00) >> 8) >> 1) & 0x03;
        status.gGTO = ( ((status_reg[0] & 0xFF00) >> 8) >> 3) & 0x01;
        status.gSTA = ( ((status_reg[0] & 0xFF00) >> 8) >> 4) & 0x03;
        status.gOBJ = ( ((status_reg[0] & 0xFF00) >> 8) >> 6) & 0x03;

        status.gVAS = (  (status_reg[0] & 0x00FF ) >> 0) & 0x03;
        status.gFLT = ( ((status_reg[1] & 0xFF00 ) >> 8) >> 0) & 0x0F;
        status.gPR =  (   status_reg[1] & 0x00FF );
        status.gPO =  (  (status_reg[2] & 0xFF00 ) >> 8);

        usleep(2e5);
    }

}






