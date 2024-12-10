#include "serial_commands.h"

// Static variables for internal use


static UART_HandleTypeDef *serial_huart;     // UART handle
static DMA_HandleTypeDef *hdma_huartrx;
uint8_t rx_rec_buf[RX_BUFFER_SIZE];
uint8_t rx_buffer[CMD_BUF_SIZE][RX_BUFFER_SIZE];       // Buffer for receiving packets

SerialBuffer_t buf;

static SerialCommand_t nullCmd = {
		.invalid = true,
		.newCommand = false
};

// Initialize the serial commands system
void SerialCommands_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma) {
    serial_huart = huart;                     // Save the UART handle
    hdma_huartrx = hdma;
    buf.recieveIdx = 0;
    buf.sendIdx = 0;
    buf.processIdx = 0;
    // Start UART reception in interrupt mode
    SerialCommands_SetupRecieve();
}

static uint8_t SerialCommands_IncrementIdx(uint8_t idx) {
	idx++;
	if(idx >= CMD_BUF_SIZE) idx = 0;

	return idx;
}

// Process a complete packet
static SerialCommand_t SerialCommands_ProcessPacket(uint8_t *packet) {

	SerialCommand_t cmd = {
		.invalid = true
	};
	uint8_t length = 0;

	cmd.invalid = false;

    if (packet[0] != START_BYTE) {
    	cmd.invalid = true; // Invalid start byte, discard packet
    }

    cmd.length = packet[1];
    if (packet[1] > MAX_DATA_SIZE) {
		cmd.invalid = true;
		cmd.length = 0;
	}

    for(uint8_t i = (RX_BUFFER_SIZE - 1); i > 1 ; i--) {
    	if(packet[i] != 0) {
    		length = i;
    		break;
    	}
    }

    uint8_t checksum = packet[0];
    for(uint8_t i = 1; i < length; i++) {
		checksum ^= packet[i];
	}

	if(checksum != packet[length]) {
		cmd.invalid = true;
	}

    // Populate the command structure
    cmd.command = (packet[2] << 8) | packet[3];

    if(cmd.length != 0) {
    	memcpy(cmd.data, &packet[4], cmd.length);
    }
    cmd.newCommand = true;

    return cmd;

}

// Handle the UART interrupt callback
void SerialCommands_HandleUARTInterrupt(void) {

	memcpy(rx_buffer[buf.recieveIdx], rx_rec_buf, RX_BUFFER_SIZE);
	buf.recieveIdx = SerialCommands_IncrementIdx(buf.recieveIdx);
	memset(rx_rec_buf, 0, RX_BUFFER_SIZE);
	// Restart UART reception

	SerialCommands_SetupRecieve();
}

void SerialCommands_SetupRecieve(void) {
	HAL_UARTEx_ReceiveToIdle_DMA(serial_huart, rx_rec_buf, RX_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_huartrx, DMA_IT_HT);

}

void SerialCommands_Send(uint16_t command, int16_t value) {
	uint8_t tx_buffer[PACKET_SIZE];

	tx_buffer[0] = START_BYTE;
	tx_buffer[1] = (command & 0xFF00) >> 8;
	tx_buffer[2] = (command & 0x00FF);
	tx_buffer[3] = (value & 0xFF00) >> 8;
	tx_buffer[4] = (value & 0x00FF);
	tx_buffer[5] = tx_buffer[0] ^ tx_buffer[1] ^ tx_buffer[2] ^ tx_buffer[3] ^ tx_buffer[4];

	HAL_UART_Transmit(serial_huart, tx_buffer, PACKET_SIZE, 0xFFFF);
}

void SerialCommands_BigSend(uint8_t *input, uint8_t length) {
	HAL_UART_Transmit(serial_huart, input, length, 0xFFFF);
}

SerialCommand_t * SerialCommands_GetCommand() {
	uint8_t idx = buf.sendIdx;

	if(buf.sendIdx == buf.recieveIdx) return &nullCmd;

	buf.sendIdx = SerialCommands_IncrementIdx(buf.sendIdx);

	return &buf.command[idx];
}

void SerialCommands_DoEvents() {

	if(buf.recieveIdx == buf.processIdx) return;

	memset(buf.command[buf.processIdx].data, 0, MAX_DATA_SIZE);
	buf.command[buf.processIdx] = SerialCommands_ProcessPacket(rx_buffer[buf.processIdx]);

	buf.processIdx = SerialCommands_IncrementIdx(buf.processIdx);


}

