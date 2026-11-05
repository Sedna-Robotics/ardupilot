#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>
#include <AP_BattMonitor/AP_BattMonitor_VEDirect.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_BATTERY_VEDIRECT_ENABLED

#include <cctype>
#include <string>

static AP_BattMonitor battery{0, nullptr, nullptr};

// a text block from an MPPT, up to and including the checksum label
static std::string text_block(int32_t battery_mv, const std::string &serial = "HQ2042ABCDE")
{
    return "\r\nPID\t0xA053\r\nSER#\t" + serial + "\r\nV\t" + std::to_string(battery_mv) +
           "\r\nI\t1500\r\nVPV\t45000\r\nPPV\t40\r\nCS\t3\r\nChecksum\t";
}

static uint8_t checksum_byte(const std::string &block)
{
    uint8_t sum = 0;
    for (const char c : block) {
        sum += uint8_t(c);
    }
    return uint8_t(256 - sum);
}

static std::string with_checksum(const std::string &block)
{
    return block + char(checksum_byte(block));
}

// a valid block whose checksum byte is ':', made by padding the serial number
static std::string block_with_colon_checksum(int32_t battery_mv)
{
    std::string serial = "HQ";
    while (true) {
        // the serial character that would make the checksum byte ':'
        const char c = char(checksum_byte(text_block(battery_mv, serial)) - ':');
        if (isalnum(c)) {
            return with_checksum(text_block(battery_mv, serial + c));
        }
        serial += 'Z';
    }
}

// an asynchronous HEX frame, as sent on a charger state change
static const std::string hex_frame = ":A0102000543\n";

class VEDirectParserTest : public ::testing::Test {
protected:
    AP_BattMonitor::BattMonitor_State state {};
    AP_BattMonitor_Params params {};
    AP_BattMonitor_VEDirect vedirect {battery, state, params};

    void feed(const std::string &bytes)
    {
        for (const char c : bytes) {
            vedirect.process_byte(uint8_t(c));
        }
    }
    float voltage() const { return state.voltage; }
    float current() const { return state.current_amps; }
};

TEST_F(VEDirectParserTest, accepts_text_block)
{
    feed(with_checksum(text_block(26700)));
    EXPECT_FLOAT_EQ(26.7f, voltage());
    EXPECT_FLOAT_EQ(-1.5f, current());
}

TEST_F(VEDirectParserTest, rejects_bad_checksum)
{
    std::string block = with_checksum(text_block(26700));
    block.back() = char(block.back() + 1);
    feed(block);
    EXPECT_FLOAT_EQ(0.0f, voltage());
}

TEST_F(VEDirectParserTest, skips_hex_frame_inside_text_block)
{
    const std::string block = with_checksum(text_block(26700));
    const size_t vpv = block.find("\r\nVPV");
    feed(block.substr(0, vpv) + hex_frame + block.substr(vpv));
    EXPECT_FLOAT_EQ(26.7f, voltage());
}

TEST_F(VEDirectParserTest, skips_hex_frames_between_blocks)
{
    feed(with_checksum(text_block(26700)) + hex_frame + hex_frame);
    feed(with_checksum(text_block(26800)));
    EXPECT_FLOAT_EQ(26.8f, voltage());
}

TEST_F(VEDirectParserTest, accepts_colon_checksum_byte)
{
    const std::string block = block_with_colon_checksum(26600);
    ASSERT_EQ(':', block.back());
    feed(block);
    EXPECT_FLOAT_EQ(26.6f, voltage());

    // and the block after it is not lost
    feed(with_checksum(text_block(26700)));
    EXPECT_FLOAT_EQ(26.7f, voltage());
}

#endif // AP_BATTERY_VEDIRECT_ENABLED

AP_GTEST_MAIN()
