/*
 * port.c
 *
 * Created: 9/10/2021 1:20:05 PM
 *  Author: Emim Eminof
 */

#include "dw3000_port.h"
#include "esp_timer.h"
// #include "driver/mcpwm.h"
// #include "soc/mcpwm_periph.h"
// #include "driver/mcpwm.h"
#include "esp_private/esp_clk.h" // for esp_clk_apb_freq()
// #include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_cap.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"

#define TAG "DW3000_PORT"

//! debug
// #define INSTRUMENT_FUNCTIONS
#ifdef INSTRUMENT_FUNCTIONS

#define MAX_EVENTS 1024 // Adjust the size as needed.

typedef struct
{
    void *func;         // Address of the called function.
    void *call_site;    // Address of the caller.
    uint64_t timestamp; // Timestamp in microseconds.
    char event_type;    // 'E' for enter, 'X' for exit.
} InstrumentEvent;

// Global ring-buffer for events.
static InstrumentEvent event_buffer[MAX_EVENTS];
// A global index that increments on each event; wraps around via modulo.
static volatile size_t event_index = 0;

// Global flag to indicate whether to record events.
volatile int in_critical = 0;

// A helper to obtain a timestamp in microseconds.
static inline uint64_t get_timestamp(void) __attribute__((no_instrument_function));
static inline uint64_t get_timestamp(void)
{
    return (uint64_t)xthal_get_ccount();
}

// Instrumentation hook: called at function entry.
void __attribute__((no_instrument_function))
__cyg_profile_func_enter(void *this_fn, void *call_site)
{
    if (in_critical)
    {
        size_t idx = event_index;
        // Record the event in a ring-buffer.
        event_buffer[idx % MAX_EVENTS].func = this_fn;
        event_buffer[idx % MAX_EVENTS].call_site = call_site;
        event_buffer[idx % MAX_EVENTS].timestamp = get_timestamp();
        event_buffer[idx % MAX_EVENTS].event_type = 'E';
        event_index++;
    }
}

// Instrumentation hook: called at function exit.
void __attribute__((no_instrument_function))
__cyg_profile_func_exit(void *this_fn, void *call_site)
{
    if (in_critical)
    {
        size_t idx = event_index;
        event_buffer[idx % MAX_EVENTS].func = this_fn;
        event_buffer[idx % MAX_EVENTS].call_site = call_site;
        event_buffer[idx % MAX_EVENTS].timestamp = get_timestamp();
        event_buffer[idx % MAX_EVENTS].event_type = 'X';
        event_index++;
    }
}

void __attribute__((no_instrument_function))
dump_instrument_events(void)
{
    // Determine how many events have been recorded.
    size_t count = event_index < MAX_EVENTS ? event_index : MAX_EVENTS;
    printf("Dumping %zu events:\n", count);
    for (size_t i = 0; i < count; i++)
    {
        InstrumentEvent *ev = &event_buffer[i];
        printf("%llu: %c  func=%p  call_site=%p\n",
               (unsigned long long)ev->timestamp,
               ev->event_type,
               ev->func,
               ev->call_site);
    }
}

#endif // __INSTRUMENT_FUNCTIONS__

//! end of debug

spi_device_handle_t _spi_handle;
spi_host_device_t _spi_host;
uint8_t _irq;
uint8_t _rst;
uint8_t _ss;
uint8_t _mosi;
uint8_t _miso;
uint8_t _sclk;

// SPI device configurations
spi_device_interface_config_t _fastSPI;
spi_device_interface_config_t _slowSPI;
spi_device_interface_config_t *_currentSPI;

bool _debounceClockEnabled = false;

/* SPI configs. */
/*const SPISettings _fastSPI;
const SPISettings _slowSPI;
const SPISettings* _currentSPI;*/

/* register caches. */
uint8_t _syscfg[LEN_SYS_CFG];
uint8_t _sysctrl[LEN_SYS_CTRL];
uint8_t _sysstatus[LEN_SYS_STATUS];
uint8_t _txfctrl[LEN_TX_FCTRL];
uint8_t _sysmask[LEN_SYS_MASK];
uint8_t _chanctrl[LEN_CHAN_CTRL];

uint8_t _deviceMode;

/* device status monitoring */
uint8_t _vmeas3v3;
uint8_t _tmeas23C;

/* PAN and short address. */
uint8_t _networkAndAddress[LEN_PANADR];

/* Mutex from dw3000_mutex.cpp */
// extern portMUX_TYPE deca_mutex;
volatile bool irq_enabled = false;
// static intr_handle_t dw3000_isr_handle = NULL;

/* Global handler for DW3000 ISR */
static port_dwic_isr_t port_dwic_isr = NULL;

// capture
mcpwm_capture_timer_config_t capture_config = {
    .clk_src = MCPWM_CAPTURE_CLK_SRC_APB,
    .resolution_hz = 80 * 000000, // 80 MHz
};
mcpwm_cap_timer_handle_t capture_timer = NULL;
mcpwm_cap_channel_handle_t capture_channel = NULL;

static volatile uint64_t cap_start_ts = 0;
static volatile uint64_t cap_roll_over_period = 0;
static volatile uint64_t last_cap_ts = 0;
static volatile uint64_t capture_time = 2;

TaskHandle_t xTaskGT;

// generator
// mcpwm_sync_handle_t sync;
// mcpwm_soft_sync_config_t sync_config;
// mcpwm_timer_sync_phase_config_t phase_config = {
//     .sync_src    = sync,
//     .count_value = 250,  // desired phase offset
//     .direction   = MCPWM_TIMER_DIRECTION_UP,
// };
// mcpwm_timer_handle_t gen_timer;

static volatile bool FakeEvent = false;

void setFakeEvent(bool value)
{
    FakeEvent = value;
}

bool getFakeEvent()
{
    return FakeEvent;
}

void enableDebounceClock()
{
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[0] |= (1 << GPDCE_BIT);
    pmscctrl0[0] |= (1 << KHZCLKEN_BIT);
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    _debounceClockEnabled = true;
}

void sleepms(uint32_t x)
{
    //_delay_ms(x); // delay by milliseconds
}

int sleepus(uint32_t x)
{
    //_delay_us(x); // delay by microseconds
    return 0;
}

void deca_sleep(uint8_t time_ms) // wrapper for decawave sleep function
{
    // This is a CPU-busy wait and not recommended for large delays
    while (time_ms--)
    {
        esp_rom_delay_us(1000);
    }
}

void deca_usleep(uint8_t time_us) // wrapper for decawave sleep function
{
    esp_rom_delay_us(time_us);
}

spi_device_handle_t getSPIHandle()
{
    return _spi_handle;
}

// spiBegin to initialize SPI bus and attach device
void spiBegin(spi_host_device_t host, uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t cs, uint8_t irq, uint8_t rst)
{
    esp_err_t ret;

    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE,
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO); // SPI_DMA_CH_AUTO
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return;
    }

    // Fast SPI device config (8 MHz)
    _fastSPI = (spi_device_interface_config_t){
        .mode = 0,                          // 0
        .clock_speed_hz = 30 * 1000 * 1000, // 20 MHz
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
    };

    // Slow SPI device config (2 MHz)
    _slowSPI = (spi_device_interface_config_t){
        .mode = 0,
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
    };

    // Set the default SPI speed to fast
    _currentSPI = &_fastSPI;

    // Attach the DW3000 device to the SPI bus with fast SPI initially
    ret = spi_bus_add_device(host, _currentSPI, &_spi_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add DW3000 device to SPI bus");
        return;
    }

    _irq = irq;
    _rst = rst;
    _mosi = mosi;
    _miso = miso;
    _sclk = sclk;
    _spi_host = host;

    // Configure IRQ and reset pins
    gpio_set_direction(static_cast<gpio_num_t>(_rst), GPIO_MODE_OUTPUT);
    gpio_set_direction(static_cast<gpio_num_t>(_irq), GPIO_MODE_INPUT);
    gpio_set_level((gpio_num_t)rst, 1);

    deca_sleep(5);

    ESP_LOGI(TAG, "SPI initialized successfully for DW3000 (fast mode)");
}
// Switch between fast and slow SPI configurations
void setSPIClockSpeed(bool fast)
{
    esp_err_t ret;

    // Remove the current device
    spi_bus_remove_device(_spi_handle);

    // Attach device with the selected SPI speed
    _currentSPI = fast ? &_fastSPI : &_slowSPI;
    ret = spi_bus_add_device(SPI2_HOST, _currentSPI, &_spi_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to switch SPI speed");
    }
    else
    {
        ESP_LOGI(TAG, "Switched to %s SPI speed", fast ? "FAST" : "SLOW");
    }
}

void reselect(uint8_t ss)
{
    _ss = ss;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ss),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    esp_err_t err = gpio_set_level((gpio_num_t)ss, 1); // Set CS HIGH (deselect)

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set level for SS pin");
    }
}

void readBytes(uint8_t cmd, uint16_t offset, uint8_t data[], uint16_t n)
{
    uint8_t header[3];
    uint8_t headerLen = 1;

    // Build SPI header
    if (offset == NO_SUB)
    {
        header[0] = READ | cmd;
    }
    else
    {
        header[0] = READ_SUB | cmd;
        if (offset < 128)
        {
            header[1] = (uint8_t)offset;
            headerLen++;
        }
        else
        {
            header[1] = RW_SUB_EXT | (uint8_t)offset;
            header[2] = (uint8_t)(offset >> 7);
            headerLen += 2;
        }
    }
    // Perform SPI read transaction
    readfromspi(headerLen, header, n, data);
}

void readBytesOTP(uint16_t address, uint8_t data[])
{
    uint8_t addressBytes[LEN_OTP_ADDR];

    // Prepare address bytes (little-endian)
    addressBytes[0] = (uint8_t)(address & 0xFF);
    addressBytes[1] = (uint8_t)((address >> 8) & 0xFF);

    // Step 1: Write OTP address
    writeBytes(OTP_IF, OTP_ADDR_SUB, addressBytes, LEN_OTP_ADDR);

    // Step 2: Switch to read mode
    writeByte(OTP_IF, OTP_CTRL_SUB, 0x03); // OTPRDEN | OTPREAD
    writeByte(OTP_IF, OTP_CTRL_SUB, 0x01); // Keep OTPRDEN

    // Step 3: Read 4 bytes from the OTP memory
    readBytes(OTP_IF, OTP_RDAT_SUB, data, LEN_OTP_RDAT);

    // Step 4: End read mode
    writeByte(OTP_IF, OTP_CTRL_SUB, 0x00);
}

// Helper to set a single register
void writeByte(uint8_t cmd, uint16_t offset, uint8_t data)
{
    writeBytes(cmd, offset, &data, 1);
}

/*
 * Write bytes to the DW1000. Single bytes can be written to registers via sub-addressing.
 * @param cmd
 *    The register address (see Chapter 7 in the DW1000 user manual).
 * @param offset
 *    The offset to select register sub-parts for writing, or 0x00 to disable
 *    sub-adressing.
 * @param data
 *    The data array to be written.
 * @param data_size
 *    The number of bytes to be written (take care not to go out of bounds of
 *    the register).
 */
void writeBytes(uint8_t cmd, uint16_t offset, uint8_t data[], uint16_t data_size)
{
    uint8_t header[3];
    uint8_t headerLen = 1; // At least one header byte is always present

    // Build the header:
    // If sub-addressing is not used, simply OR the cmd with WRITE flag.
    if (offset == NO_SUB)
    {
        header[0] = WRITE | cmd;
    }
    else
    {
        // With sub-addressing, set the sub-address flag.
        header[0] = WRITE_SUB | cmd;
        if (offset < 128)
        {
            // A single extra header byte is sufficient.
            header[1] = (uint8_t)offset;
            headerLen++;
        }
        else
        {
            // For offsets >= 128, use an extended two-byte sub-address.
            header[1] = RW_SUB_EXT | (uint8_t)offset;
            header[2] = (uint8_t)(offset >> 7);
            headerLen += 2;
        }
    }

    // Create a contiguous buffer for header + data.
    uint16_t total_len = headerLen + data_size;
    uint8_t tx_buffer[total_len];
    // if (tx_buffer == NULL)
    // {
    //     ESP_LOGE("writeBytes", "Failed to allocate memory for SPI transaction");
    //     return;
    // }

    // Copy the header and data into the transmit buffer.
    memcpy(tx_buffer, header, headerLen);
    memcpy(tx_buffer + headerLen, data, data_size);

    // Prepare the SPI transaction.
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = total_len * 8; // Length in bits
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = NULL; // No data is read back in a write

    decaIrqStatus_t status = decamutexon();

    // Manually control the chip-select (CS) line.
    gpio_set_level((gpio_num_t)_ss, 0); // Pull CS low

    // Transmit the entire transaction.
    esp_err_t ret = spi_device_polling_transmit(_spi_handle, &trans);

    gpio_set_level((gpio_num_t)_ss, 1); // Pull CS high
    decamutexoff(status);

    // (Optional) Release the mutex here: decamutexoff(status);

    if (ret != ESP_OK)
    {
        ESP_LOGE("writeBytes", "SPI transaction failed: %d", ret);
    }
}

void enableClock(uint8_t clock)
{
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    if (clock == AUTO_CLOCK)
    {
        _currentSPI = &_fastSPI;
        pmscctrl0[0] = AUTO_CLOCK;
        pmscctrl0[1] &= 0xFE;
    }
    else if (clock == XTI_CLOCK)
    {
        _currentSPI = &_slowSPI;
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= XTI_CLOCK;
    }
    else if (clock == PLL_CLOCK)
    {
        _currentSPI = &_fastSPI;
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= PLL_CLOCK;
    }
    else
    {
        // TODO deliver proper warning
        ESP_LOGE(TAG, "Invalid clock setting");
    }
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
}

void reset()
{
    if (_rst == 0xFF)
    {
        softReset();
    }
    else
    {
        // DW1000/DW3000 datasheet: RSTn pin should be driven low but left floating after
        gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)_rst, 0); // Drive RST low

        deca_sleep(2); // Delay 2 ms (safe margin for 50ns)

        // Set RST pin to input (float)
        gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_INPUT);

        // Delay 10 ms for safe boot time
        deca_sleep(10);

        // Force idle mode (optional but aligns with original logic)
        idle();
    }
}

void softReset()
{
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[0] = 0x01;
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[3] = 0x00;
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    deca_sleep(10);
    pmscctrl0[0] = 0x00;
    pmscctrl0[3] = 0xF0;
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    // force into idle mode
    idle();
}

void setBit(uint8_t data[], uint16_t n, uint16_t bit, bool val)
{
    uint16_t idx;
    uint8_t shift;

    idx = bit / 8;
    if (idx >= n)
    {
        // Out-of-bounds, handle error (log or assert)
        ESP_LOGW(TAG, "setBit: Index out of bounds (idx=%d, size=%d)", idx, n);
        return;
    }

    uint8_t *targetByte = &data[idx];
    shift = bit % 8;

    if (val)
    {
        *targetByte |= (1 << shift); // Set the bit
    }
    else
    {
        *targetByte &= ~(1 << shift); // Clear the bit
    }
}

/*
 * Check the value of a bit in an array of bytes that are considered
 * consecutive and stored from MSB to LSB.
 * @param data
 *    The number as byte array.
 * @param n
 *    The number of bytes in the array.
 * @param bit
 *    The position of the bit to be checked.
 */
bool getBit(uint8_t data[], uint16_t n, uint16_t bit)
{
    uint16_t idx;
    uint8_t shift;

    idx = bit / 8;
    if (idx >= n)
    {
        ESP_LOGE(TAG, "getBit: Index out of bounds (idx=%d, size=%d)", idx, n);
        return false; // TODO proper error handling: out of bounds
    }
    uint8_t targetByte = data[idx];
    shift = bit % 8;

    return (targetByte & (1 << shift)) != 0; // Return boolean value
}

void writeValuesToBytes(uint8_t data[], int32_t val, uint16_t n)
{
    uint16_t i;
    for (i = 0; i < n; i++)
    {
        data[i] = ((val >> (i * 8)) & 0xFF); // TODO bad types - signed unsigned problem
    }
}

void readSystemConfigurationRegister()
{
    readBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void writeSystemConfigurationRegister()
{
    writeBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void readSystemEventStatusRegister()
{
    readBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void readNetworkIdAndDeviceAddress()
{
    readBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void writeNetworkIdAndDeviceAddress()
{
    writeBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void readSystemEventMaskRegister()
{
    readBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void writeSystemEventMaskRegister()
{
    writeBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void readChannelControlRegister()
{
    readBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void writeChannelControlRegister()
{
    writeBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void readTransmitFrameControlRegister()
{
    readBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

void writeTransmitFrameControlRegister()
{
    writeBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

void idle()
{
    memset(_sysctrl, 0, LEN_SYS_CTRL);
    setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, true);
    _deviceMode = IDLE_MODE;
    writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void setDoubleBuffering(bool val)
{
    setBit(_syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, !val);
}

void setInterruptPolarity(bool val)
{
    setBit(_syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, val);
}

void clearInterrupts()
{
    memset(_sysmask, 0, LEN_SYS_MASK);
}

void manageLDE()
{
    // transfer any ldo tune values
    uint8_t ldoTune[LEN_OTP_RDAT];
    readBytesOTP(0x04, ldoTune); // TODO #define
    if (ldoTune[0] != 0)
    {
        // TODO tuning available, copy over to RAM: use OTP_LDO bit
        ESP_LOGE(TAG, "LDO tune values available in OTP");
    }
    // tell the chip to load the LDE microcode
    // TODO remove clock-related code (PMSC_CTRL) as handled separately
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    uint8_t otpctrl[LEN_OTP_CTRL];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    memset(otpctrl, 0, LEN_OTP_CTRL);
    readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    readBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
    pmscctrl0[0] = 0x01;
    pmscctrl0[1] = 0x03;
    otpctrl[0] = 0x00;
    otpctrl[1] = 0x80;
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
    writeBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, 2);
    deca_sleep(5);
    pmscctrl0[0] = 0x00;
    pmscctrl0[1] &= 0x02;
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
}

void Sleep(uint32_t d)
{
    deca_sleep(d);
}

void spiSelect(uint8_t ss)
{
    // Reconfigure CS pin
    reselect(ss);

    // Lock clock to PLL speed (or AUTO)
    enableClock(AUTO_CLOCK);
    deca_sleep(5); // 5 ms sleep (vTaskDelay wrapper)

    // Reset chip if necessary
    if (_rst != 0xFF)
    {
        // DW3000 datasheet: Float RSTn pin by setting it to input
        gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_INPUT);
    }
    reset(); // Perform a soft/hard reset

    // Set default network ID and node address (PANADR)
    writeValuesToBytes(_networkAndAddress, 0xFF, LEN_PANADR);
    writeNetworkIdAndDeviceAddress();

    // Configure default system settings
    memset(_syscfg, 0, LEN_SYS_CFG);
    setDoubleBuffering(false);
    setInterruptPolarity(true);
    writeSystemConfigurationRegister();

    // Disable interrupts by default (clear mask)
    clearInterrupts();
    writeSystemEventMaskRegister();

    // Load LDE micro-code
    enableClock(XTI_CLOCK);
    deca_sleep(5); // 5 ms sleep (vTaskDelay wrapper)
    manageLDE();
    deca_sleep(5); // 5 ms sleep (vTaskDelay wrapper)
    enableClock(AUTO_CLOCK);
    deca_sleep(5); // 5 ms sleep (vTaskDelay wrapper)

    // Read factory-calibrated temperature and voltage values from OTP
    uint8_t buf_otp[4];
    readBytesOTP(0x008, buf_otp); // Read 3.3V calibration value
    _vmeas3v3 = buf_otp[0];

    readBytesOTP(0x009, buf_otp); // Read 23C temperature calibration value
    _tmeas23C = buf_otp[0];
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer)
{
    esp_err_t ret;
    spi_transaction_t t;
    uint16_t totalLength = headerLength + readLength;

    // Create combined transmit and receive buffers.
    uint8_t combinedTx[totalLength];
    uint8_t combinedRx[totalLength];

    // Prepare combinedTx: first send the header bytes,
    // then send dummy bytes (JUNK) to clock out the read data.
    memcpy(combinedTx, headerBuffer, headerLength);
    memset(combinedTx + headerLength, JUNK, readLength);

    // Set up the SPI transaction for the complete message.
    memset(&t, 0, sizeof(t));
    t.length = totalLength * 8; // total transaction length in bits
    t.tx_buffer = combinedTx;
    t.rx_buffer = combinedRx;
    t.rxlength = totalLength * 8; // total bits to be received
    t.flags = 0;

    // Acquire the mutex, assert CS, and perform the transaction.
    decaIrqStatus_t s = decamutexon();
    gpio_set_level((gpio_num_t)_ss, 0); // Assert chip-select (CS low)

    ret = spi_device_polling_transmit(_spi_handle, &t);
    if (ret != ESP_OK)
    {
        gpio_set_level((gpio_num_t)_ss, 1); // Release CS on error
        decamutexoff(s);
        assert(0);
        return -1;
    }

    gpio_set_level((gpio_num_t)_ss, 1); // Deassert chip-select (CS high)

    // Copy the received data (ignoring the header portion)
    memcpy(readBuffer, combinedRx + headerLength, readLength);

    decamutexoff(s);
    return 0;
}

int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t bodyLength, uint8_t *bodyBuffer)
{
    esp_err_t ret;
    spi_transaction_t t;
    uint16_t totalLength = headerLength + bodyLength;

    // Create a combined buffer for header and body. VLA so better have a small header
    uint8_t combinedBuffer[totalLength];
    memcpy(combinedBuffer, headerBuffer, headerLength);
    memcpy(combinedBuffer + headerLength, bodyBuffer, bodyLength);

    // Set up a single SPI transaction for the entire message.
    memset(&t, 0, sizeof(t));
    t.length = totalLength * 8; // total length in bits
    t.tx_buffer = combinedBuffer;
    t.flags = 0; // No special flags

    // Acquire the mutex, pull CS low, and perform the transaction.
    decaIrqStatus_t s = decamutexon();
    gpio_set_level((gpio_num_t)_ss, 0); // Assert chip-select (CS low)

    ret = spi_device_polling_transmit(_spi_handle, &t);
    if (ret != ESP_OK)
    {
        gpio_set_level((gpio_num_t)_ss, 1); // Release CS on error
        decamutexoff(s);
        // assert(0);
        return -1;
    }

    gpio_set_level((gpio_num_t)_ss, 1); // Deassert chip-select (CS high)

    decamutexoff(s);
    return 0;
}

esp_err_t acquire_bus()
{
    esp_err_t ret;
    ret = spi_device_acquire_bus(_spi_handle, portMAX_DELAY);
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ret;
}

void release_bus()
{
    // Release the SPI bus
    spi_device_release_bus(_spi_handle);
}

void wakeup_device_with_io()
{
    gpio_set_level((gpio_num_t)_ss, 0);
    deca_sleep(2);
    gpio_set_level((gpio_num_t)_ss, 1);
    if (_debounceClockEnabled)
    {
        enableDebounceClock();
    }
}

void port_set_dw_ic_spi_fastrate(uint8_t irq, uint8_t rst, uint8_t ss)
{
    // spiBegin(irq, rst);
    spiBegin(_spi_host, _mosi, _miso, _sclk, ss, irq, rst);
    spiSelect(ss);
}

/* Get IRQ Status */
bool port_GetEXT_IRQStatus(void)
{
    return irq_enabled;
}

/* Check IRQ Pin Level */
uint32_t port_CheckEXT_IRQ(void)
{
    return gpio_get_level(static_cast<gpio_num_t>(_irq)); // Return current state of the IRQ pin
}

/* Disable External IRQ */
void port_DisableEXT_IRQ(void)
{
    // gpio_intr_disable(static_cast<gpio_num_t>(_irq));
    mcpwm_capture_channel_disable(capture_channel);
    irq_enabled = false;
}

/* Enable External IRQ */
void port_EnableEXT_IRQ(void)
{
    // gpio_intr_enable(static_cast<gpio_num_t>(_irq));
    mcpwm_capture_channel_enable(capture_channel);
    irq_enabled = true;
}

void actual_interrupt()
{

    acquire_bus();
#ifdef INSTRUMENT_FUNCTIONS
    event_index = 0;
    in_critical = 1;
#endif
    port_dwic_isr();
#ifdef INSTRUMENT_FUNCTIONS
    in_critical = 0;
#endif
    release_bus();
}

void handle_dw3000_interrupt_task(void *param)
{
    uint32_t ulNotificationValue;
    xTaskISR = xTaskGetCurrentTaskHandle();
    while (true)
    {
        bool is_gpio_set = gpio_get_level(static_cast<gpio_num_t>(_irq));
        bool is_notification_or_gpio = is_gpio_set || ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
        if (is_notification_or_gpio == 0)
        {
            ESP_LOGE(TAG, "Failed to take notification");
            continue;
        }
        // if(is_gpio_set) { port_DisableEXT_IRQ();}
        do
        {
            actual_interrupt();
        } while (gpio_get_level(static_cast<gpio_num_t>(_irq)));
        // port_EnableEXT_IRQ();
    }
    vTaskDelete(NULL);
}

// uint64_t get_capture_time()
// {
//     uint64_t elapsed_time = last_cap_ts - cap_start_ts;  // us
//     // translate the elapsed time to the timer resolution
//     double timer_resolution = 1000000.0 / 80.0; // 80 MHz
//     uint64_t elapsed_time_in_timer_ticks = elapsed_time / timer_resolution; // timer ticks
//     uint64_t unwrapped_capture_time = capture_time + (elapsed_time_in_timer_ticks / cap_roll_over_period) * cap_roll_over_period;
//     return unwrapped_capture_time;
//     // return capture_time;
// }
uint64_t get_capture_time()
{
    // 1) Compute how many microseconds have elapsed between cap_start_ts and last_cap_ts.
    //    Both of these are from esp_timer_get_time(), which returns µs.
    uint64_t elapsed_time_us = last_cap_ts - cap_start_ts; // in microseconds

    // 2) Convert that elapsed time in microseconds to elapsed ticks of the 80 MHz timer.
    //    80 MHz = 80,000,000 ticks per second = 80 ticks per microsecond.
    uint64_t elapsed_ticks = elapsed_time_us * 80ULL; // in 80 MHz ticks

    // 3) Determine how many times the 32-bit timer (which rolls over at 2^32) has wrapped.
    //    roll_over_ticks = 2^32 is the number of ticks at which the hardware timer resets to 0.
    const uint64_t roll_over_ticks = 1ULL << 32;
    uint64_t num_rollovers = elapsed_ticks / roll_over_ticks;

    // 4) "unwrap" the raw 32-bit capture_time by adding the rollover offset.
    //    capture_time is the 32-bit reading from the hardware (in ticks), so we convert it to 64 bits
    //    and add the number of full rollovers (each rollover is 2^32 ticks).
    uint64_t unwrapped_capture_time = (uint64_t)capture_time + (num_rollovers * roll_over_ticks);

    // 5) Return the unwrapped capture time in ticks (the original 80 MHz resolution).
    return unwrapped_capture_time;
}

uint64_t get_raw_cap()
{
    return capture_time;
}

/* ISR Handler */
static bool IRAM_ATTR dw3000_isr_handler(mcpwm_cap_channel_handle_t cap_channel, const mcpwm_capture_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (edata->cap_edge == MCPWM_CAP_EDGE_NEG)
    {
        return false;
    }
    capture_time = edata->cap_value;
    last_cap_ts = esp_timer_get_time();
    if (FakeEvent)
    {
        vTaskNotifyGiveIndexedFromISR(xTaskGT, 0, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return false;
    }
    vTaskNotifyGiveIndexedFromISR(xTaskISR, 0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken;
}


/* GPIO Initialization for DW3000 IRQ */
void dw3000_gpio_irq_init(void)
{
    esp_err_t ret;
    cap_start_ts = esp_timer_get_time();
    ret = mcpwm_new_capture_timer(&capture_config, &capture_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create capture timer");
        return;
    }

    mcpwm_capture_channel_config_t capture_channel_config = {
        .gpio_num = static_cast<gpio_num_t>(_irq),
        .intr_priority = 3,
        .prescale = 1,
        .flags = {
            .pos_edge = 1,
            .neg_edge = 0,
            .pull_up = 0,
            .pull_down = 0,
            .invert_cap_signal = 0,
            .io_loop_back = 0,
        },
    };

    ret = mcpwm_new_capture_channel(capture_timer, &capture_channel_config, &capture_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create capture channel");
        return;
    }
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = dw3000_isr_handler,
    };
    ret = mcpwm_capture_channel_register_event_callbacks(capture_channel, &cbs, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register capture event callbacks");
        return;
    }

    ret = mcpwm_capture_channel_enable(capture_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable capture channel");
        return;
    }

    ret = mcpwm_capture_timer_enable(capture_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable capture timer");
        return;
    }

    ret = mcpwm_capture_timer_start(capture_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start capture timer");
        return;
    }
    uint32_t res;
    ret = mcpwm_capture_timer_get_resolution(capture_timer, &res);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get capture timer resolution");
        return;
    }

    // gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);

    // // 1. Configure the timer with high resolution for low frequency pulse
    // mcpwm_timer_config_t timer_config = {
    //     .group_id       = 0,
    //     .clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT ,  // or use APB clock if preferred
    //     .resolution_hz  = 80*1000000,                  // 80 MHz
    //     .count_mode     = MCPWM_TIMER_COUNT_MODE_UP,
    //     .period_ticks   = 50000,
    //     .intr_priority  = 3,
    // };
    
    // ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &gen_timer));

    // 2. Set up the operator and connect it to the timer
    // mcpwm_operator_config_t oper_config = {
    //     .group_id       = 0,
    //     .intr_priority  = 3,
    //     // additional flags if needed
    // };
    // mcpwm_oper_handle_t oper;
    // ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));
    // ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, gen_timer));


    // gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    // 3. Set up the generator for the output pulse
    // mcpwm_generator_config_t gen_config = {
    //     .gen_gpio_num = GPIO_NUM_3,  // Replace with your actual GPIO
    //     // other configurations such as pull-up/down as needed
    //     .flags = {
    //         .invert_pwm=0,   /*!< Whether to invert the PWM signal (done by GPIO matrix) */
    //         .io_loop_back=0, /*!< For debug/test, the signal output from the GPIO will be fed to the input path as well */
    //         .io_od_mode=0,   /*!< Configure the GPIO as open-drain mode */
    //         .pull_up=0,      /*!< Whether to pull up internally */
    //         .pull_down=0,    /*!< Whether to pull down internally */ 
    //     },
    // };
    // mcpwm_gen_handle_t gen;
    // ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen));

    // Configure generator actions:
    // At the start of the period (timer empty), force output HIGH.
    // ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
    //     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

    // Use a comparator to set when the pulse should go LOW (i.e., pulse width)
    // mcpwm_cmpr_handle_t cmp;
    // mcpwm_comparator_config_t cmp_config = {
    //     .intr_priority = 3,
    //     .flags = {
    //         .update_cmp_on_tez = 1,  // Update comparator on timer event
    //         .update_cmp_on_tep = 1,  // update on timer event
    //         .update_cmp_on_sync = 1, // update on sync event
    //     },
    // };
    // (Allocate and configure the comparator; set its compare value as pulse width in ticks)
    // ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_config, &cmp));
    // ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
    //     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));


    // 4. Enable the timer
    // ESP_ERROR_CHECK(mcpwm_timer_enable(gen_timer));
    // ESP_ERROR_CHECK(mcpwm_timer_start_stop(gen_timer, MCPWM_TIMER_START_NO_STOP));
    
    // 5. Set up phase control using sync
    // ESP_ERROR_CHECK(mcpwm_new_soft_sync_src(&sync_config, &sync));
    // phase_config.sync_src = sync;
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp, 5000));
    // ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(gen_timer, &phase_config));
    // ret = mcpwm_soft_sync_activate(sync);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to activate soft sync: %s", esp_err_to_name(ret));
    //     ESP_ERROR_CHECK(ret);
    //     return;
    // }


    ESP_LOGI(TAG, "Capture IRQ initialized successfully");
    xTaskCreatePinnedToCore(handle_dw3000_interrupt_task, "dw3000_interrupt_task", 4096, NULL, 24, NULL, 0);
    // port_EnableEXT_IRQ();  // Enable IRQ after initialization
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_dwic_isr()
 *
 * @brief This function is used to install the handling function for DW IC IRQ.
 *
 * NOTE:
 *   - The user application shall ensure that a proper handler is set by calling this function before any DW IC IRQ occurs.
 *   - This function deactivates the DW IC IRQ line while the handler is installed.
 *
 * @param deca_isr function pointer to DW IC interrupt handler to install
 *
 * @return none
 */
/* ISR registration function */
void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
    decaIrqStatus_t s = decamutexon();
    port_dwic_isr = dwic_isr;
    decamutexoff(s);
}


/**
 * @brief Sets the global task handle for the GT task.
 *
 * This function assigns the provided task handle to the global variable
 * `xTaskGT`, which represents the GT task in the system.
 *
 * @param xTask The task handle to be assigned to the GT task.
 */
void setGTtask(TaskHandle_t xTask)
{
    xTaskGT = xTask;
}

#if 0
void open_spi(void)
{
  //PORTB &= ~_BV(PORTB2); // set SS pin to LOW to enable SPI
}

void close_spi(void)
{
  //PORTB |= _BV(PORTB2); // set SS pin to HIGH to disable SPI
}

int spi_tranceiver (uint8_t *data) // send single byte
{
  /*SPDR0 = data; // Load data into the buffer
  sleepus(1);
  while(!(SPSR0 & _BV(SPIF) )); // Wait until transmission complete
  
  // Return received data
  return(SPDR0);*/
  return (0);
}



void port_set_dw_ic_spi_slowrate(void)
{
  //SPSR0 &= ~_BV(SPI2X); // turn off fast speed
}

void port_set_dw_ic_spi_fastrate(void)
{
  //SPSR0 |= _BV(SPI2X); // set fast speed by changing oscillator speed to FOSC / 2
}

void reset_DWIC(void) // currently not used as we are using softreset()
{
  /*DDR_PORTD |= _BV(DD_RESET_PIN); // set reset PIN as output
  PORTD &= ~_BV(PORTD7); // set reset pin to low for brief amount of time

  sleepus(1);

  DDR_PORTD &= ~_BV(DD_RESET_PIN); // set reset pin to input again

  sleepms(2); // allow for chip to turn back on*/

}

#endif
