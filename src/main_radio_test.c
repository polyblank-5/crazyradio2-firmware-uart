#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
//#include "zephyr/drivers/gpio.h"
#include "hal/nrf_gpio.h"
//#include "nrf_gpio.h"
#include <zephyr/sys/printk.h>

/* Public API */

// S1 is used for compatibility with NRF24L0+. These three bits are used
// to store the PID and NO_ACK.
#define PACKET0_S1_SIZE                  (3UL)
// S0 is not used
#define PACKET0_S0_SIZE                  (0UL)
// The size of the packet length field is 6 bits
#define PACKET0_PAYLOAD_SIZE             (6UL)
// The size of the base address field is 4 bytes
#define PACKET1_BASE_ADDRESS_LENGTH      (4UL)
// Don't use any extra added length besides the length field when sending
#define PACKET1_STATIC_LENGTH            (0UL)
// Max payload allowed in a packet
#define PACKET1_PAYLOAD_SIZE             (63UL)

static uint64_t address = 0xE7E7E7E7E7ULL;

struct esbPacket_s {
    uint8_t length;
    uint8_t s1;
    char data[32];
} __attribute__((packed));

static uint32_t swap_bits(uint32_t inp)
{
  uint32_t i;
  uint32_t retval = 0;

  inp = (inp & 0x000000FFUL);

  for(i = 0; i < 8; i++)
  {
    retval |= ((inp >> i) & 0x01) << (7 - i);
  }

  return retval;
}

static uint32_t bytewise_bitswap(uint32_t inp)
{
  return (swap_bits(inp >> 24) << 24)
       | (swap_bits(inp >> 16) << 16)
       | (swap_bits(inp >> 8) << 8)
       | (swap_bits(inp));
}


#define RADIO_CHANNEL 47  // Should match the transmitter's channel

void radio_init(void) {
    NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm;
    NRF_RADIO->FREQUENCY = RADIO_CHANNEL;
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit;

    NRF_RADIO->SHORTS = 0;

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET0_S1_SIZE << RADIO_PCNF0_S1LEN_Pos) |
                      (PACKET0_S0_SIZE << RADIO_PCNF0_S0LEN_Pos) |
                      (PACKET0_PAYLOAD_SIZE << RADIO_PCNF0_LFLEN_Pos);

    // Packet configuration
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos)    |
                        (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)           |
                        (PACKET1_BASE_ADDRESS_LENGTH << RADIO_PCNF1_BALEN_Pos)       |
                        (PACKET1_STATIC_LENGTH << RADIO_PCNF1_STATLEN_Pos)           |
                        (PACKET1_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos);
                        
    /*NRF_RADIO->PREFIX0 = 0xC4C3FF00UL | (bytewise_bitswap(address >> 32) & 0xFF);  // Prefix byte of addresses 3 to 0
    NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;  // Prefix byte of addresses 7 to 4
    NRF_RADIO->BASE0   = bytewise_bitswap((uint32_t)address);  // Base address for prefix 0
    NRF_RADIO->BASE1   = 0xE7E7E7E7UL;  // Base address for prefix 1-7
    NRF_RADIO->TXADDRESS = 0x00UL;      // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = (1<<0) | (1<<1);    // Enable device address 0 and 1 to use which rec
    */
    NRF_RADIO->PREFIX0 = 0x000000e7;  // Prefix byte of addresses 3 to 0
    //NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;  // Prefix byte of addresses 7 to 4
    NRF_RADIO->BASE0   = 0xe7e7e7e7;  // Base address for prefix 0
    //NRF_RADIO->BASE1   = 0xE7E7E7E7UL;  // Base address for prefix 1-7
    NRF_RADIO->TXADDRESS = 0x00UL;      // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = (1<<0) | (1<<1);    // Enable device address 0 and 1 to use which rec
    
    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
    //NRF_RADIO->TASKS_RXEN = 1U;
}


void radio_receive(struct esbPacket_s *data, uint8_t *length) {
    NRF_RADIO->PACKETPTR = (uint32_t)&data;
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->TASKS_START = 1;

    while (NRF_RADIO->EVENTS_END == 0);
    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);

    *length = NRF_RADIO->RXMATCH;
}

int main(void) {
    nrf_gpio_cfg_output(20); // LED on pin 20

    radio_init();

    while (1) {
        struct esbPacket_s message; // Adjust the buffer size as needed
        uint8_t message_len = 0;

        // Receive a message
        radio_receive(&message, &message_len);

        // Toggle an LED to indicate a reception
        nrf_gpio_pin_toggle(20);

        // Process the received message (e.g., print it)
        if (message_len > 0) {
            //message[message_len] = '\0'; // Null-terminate the received data
            printk("Received: %s\n", message.data);
        }
    }
}