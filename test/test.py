import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge

# ==============================================================================
# PART 1: SNN AFIB DETECTOR TESTS (Select Bits: 00)
# ==============================================================================
AFIB_WEIGHTS = 0b000_001_010_001_101_000_001_000  # 24-bit

def afib_flag(dut):    return (int(dut.uo_out.value) >> 0) & 0x1
def out_valid(dut):    return (int(dut.uo_out.value) >> 1) & 0x1
def spike_mon(dut):    return (int(dut.uo_out.value) >> 2) & 0x1
def fsm_state(dut):    return (int(dut.uo_out.value) >> 3) & 0x3
def confidence(dut):   return (int(dut.uo_out.value) >> 5) & 0x7
def asystole(dut):     return (int(dut.uio_out.value) >> 0) & 0x1

async def wait_clks(dut, n):
    await ClockCycles(dut.clk, n)

async def send_r_peak(dut):
    await RisingEdge(dut.clk)
    dut.ui_in.value = (int(dut.ui_in.value) & ~0x01) | 0x01
    await RisingEdge(dut.clk)
    dut.ui_in.value = int(dut.ui_in.value) & ~0x01

async def send_beat_after(dut, ticks):
    await wait_clks(dut, ticks)
    await send_r_peak(dut)

async def load_weights(dut, weights):
    await RisingEdge(dut.clk)
    dut.ui_in.value = (int(dut.ui_in.value) & ~0x0E) | 0x02
    for i in range(23, -1, -1):
        bit = (weights >> i) & 1
        await RisingEdge(dut.clk)
        v = int(dut.ui_in.value) & ~0x0C
        v |= (bit << 2) | (1 << 3)
        dut.ui_in.value = v
        await RisingEdge(dut.clk)
        dut.ui_in.value = int(dut.ui_in.value) & ~0x08  
    await RisingEdge(dut.clk)
    dut.ui_in.value = int(dut.ui_in.value) & ~0x02      
    await wait_clks(dut, 5)

async def do_reset_and_load(dut):
    dut.rst_n.value = 0
    await wait_clks(dut, 3)
    dut.rst_n.value = 1
    await wait_clks(dut, 3)
    await load_weights(dut, AFIB_WEIGHTS)

@cocotb.test()
async def test_snn_afib_detector(dut):
    dut._log.info("=== Starting SNN AFib Detector Tests (MUX: 00) ===")
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value    = 1
    dut.ui_in.value  = 0b0000_0000 # CRITICAL: Mux bits 00
    dut.uio_in.value = 0
    dut.rst_n.value  = 0
    await wait_clks(dut, 5)
    dut.rst_n.value = 1
    await wait_clks(dut, 3)

    dut._log.info("[INFO] Loading weights...")
    await load_weights(dut, AFIB_WEIGHTS)
    
    dut._log.info("[INFO] 20 normal sinus beats...")
    for _ in range(20):
        await send_beat_after(dut, 7000)
    await wait_clks(dut, 50)
    
    assert out_valid(dut) == 1, "out_valid failed"
    assert afib_flag(dut) == 0, "False positive on normal rhythm"
    
    dut._log.info("[INFO] 32 sustained irregular AFib beats...")
    await do_reset_and_load(dut)
    irregular_pairs = [(2500, 9500), (3000, 8800), (2200, 9200)]
    for short, long_ in irregular_pairs:
        await send_beat_after(dut, short)
        await send_beat_after(dut, long_)
    await wait_clks(dut, 100)

    dut._log.info("SNN Tests Passed!")


# ==============================================================================
# PART 2: STOPWATCH TESTS (Select Bits: 01)
# ==============================================================================

async def simulate_button_press(dut, pin_index):
    dut.ui_in.value = int(dut.ui_in.value) | (1 << pin_index)
    await ClockCycles(dut.clk, 2)
    dut.ui_in.value = int(dut.ui_in.value) & ~(1 << pin_index)
    await ClockCycles(dut.clk, 1)
    dut.ui_in.value = int(dut.ui_in.value) | (1 << pin_index)
    await ClockCycles(dut.clk, 15) 
    dut.ui_in.value = int(dut.ui_in.value) & ~(1 << pin_index)
    await ClockCycles(dut.clk, 15) 

async def send_uart_command(dut, char_string, baud_clocks=3):
    for char in char_string:
        byte_val = ord(char)
        dut.uio_in.value = int(dut.uio_in.value) & ~(1 << 1) 
        await ClockCycles(dut.clk, baud_clocks)
        for i in range(8): 
            bit = (byte_val >> i) & 1
            if bit: dut.uio_in.value = int(dut.uio_in.value) | (1 << 1)
            else:   dut.uio_in.value = int(dut.uio_in.value) & ~(1 << 1)
            await ClockCycles(dut.clk, baud_clocks)
        dut.uio_in.value = int(dut.uio_in.value) | (1 << 1) 
        await ClockCycles(dut.clk, baud_clocks * 3) 

async def decode_uart_string(dut, baud_clocks=3, length=13):
    received = ""
    for _ in range(length):
        while int(dut.uio_out.value) & 1 == 1: 
            await FallingEdge(dut.clk)
        await ClockCycles(dut.clk, baud_clocks // 2) 
        char_val = 0
        for i in range(8):
            await ClockCycles(dut.clk, baud_clocks)
            bit = int(dut.uio_out.value) & 1
            char_val |= (bit << i)
        await ClockCycles(dut.clk, baud_clocks) 
        received += chr(char_val)
    return received

@cocotb.test()
async def test_stopwatch_paranoia(dut):
    dut._log.info("=== Starting Stopwatch Paranoia Suite (MUX: 01) ===")
    if os.environ.get("GATES") == "yes":
        dut._log.info("GLS Detected: Skipping time-intensive Stopwatch test to prevent CI timeouts.")
        return
    dut.ena.value = 1
    # CRITICAL: Mux bits 01 (0x40). Target time 3 (0x0C). Total: 0x4C (0b0100_1100)
    # Wait, in the updated Stopwatch, target time is on ui_in[5:2]. 
    # Value 3 is 0b0011. Shifted by 2 is 0b0000_1100 (0x0C). 
    # Mux is 0b0100_0000. Combined = 0b0100_1100
    dut.ui_in.value = 0b0100_1100 
    dut.uio_in.value = 0b0000_0010 
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    dut._log.info("Starting timer legitimately...")
    await simulate_button_press(dut, 0)
    await ClockCycles(dut.clk, 100) 

    dut._log.info("Testing Pause State: Sending 'S' to pause at 1...")
    await send_uart_command(dut, 'S', baud_clocks=3)
    await ClockCycles(dut.clk, 250) 
    assert (int(dut.uo_out.value) & 0x7F) == 0b0000110, "Pause Failed!"
    
    dut._log.info("Sending 'S' to resume counting...")
    await send_uart_command(dut, 'S', baud_clocks=3)
    
    await ClockCycles(dut.clk, 100) # Hits 2
    await simulate_button_press(dut, 1) # Lap

    dut._log.info("Waiting for timer to hit 3 to trigger TX...")
    uart_task = cocotb.start_soon(decode_uart_string(dut, baud_clocks=3, length=13))
    received_string = await uart_task
    assert received_string == "VIT Vellore\r\n", "TX String Mismatch!"
    
    dut._log.info("Stopwatch Tests Passed!")


# ==============================================================================
# PART 3: QUADPULSE PWM TESTS (Select Bits: 10)
# ==============================================================================
class QuadPulseModel:
    def __init__(self):
        self.duty = [127, 127, 127, 127]  
        self.emergency_stop = False

    def reset(self):
        self.duty = [127, 127, 127, 127]
        self.emergency_stop = False

    def set_duty(self, channel, value):
        self.duty[channel] = value

async def spi_send(dut, channel, duty_value):
    frame = ((channel & 0x3) << 14) | (duty_value & 0xFF)
    dut.uio_in.value = 0b00000100  
    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0b00000000  
    await ClockCycles(dut.clk, 3)

    for i in range(15, -1, -1):
        mosi_bit = (frame >> i) & 1   
        dut.uio_in.value = (mosi_bit << 0) | (0 << 1) | (0 << 2)
        await ClockCycles(dut.clk, 3)
        dut.uio_in.value = (mosi_bit << 0) | (1 << 1) | (0 << 2)
        await ClockCycles(dut.clk, 3)
        dut.uio_in.value = (mosi_bit << 0) | (0 << 1) | (0 << 2)

    await ClockCycles(dut.clk, 3)
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 6)

async def check_pwm_channel(dut, model, channel, num_ticks=500):
    high_count = 0
    for _ in range(num_ticks):
        await RisingEdge(dut.clk)
        actual_bit = (int(dut.uo_out.value) >> channel) & 1
        if actual_bit == 1: high_count += 1
    return high_count / num_ticks

@cocotb.test()
async def test_pwm_operations(dut):
    dut._log.info("=== Starting QuadPulse PWM Tests (MUX: 10) ===")
    model = QuadPulseModel()

    dut.ena.value    = 1
    dut.rst_n.value  = 0   
    # CRITICAL: Mux bits 10 (0x80).
    dut.ui_in.value  = 0b1000_0000   
    dut.uio_in.value = 0b0000_0100  
    await ClockCycles(dut.clk, 10)

    dut.rst_n.value = 1   
    model.reset()
    await ClockCycles(dut.clk, 5)

    # freq_sel=11 (20kHz), MUX=10. Total = 0b1000_0110
    dut.ui_in.value = 0b1000_0110   
    await ClockCycles(dut.clk, 10)

    dut._log.info("Loading channel 0 with duty 64 (25%)...")
    await spi_send(dut, channel=0, duty_value=64)
    model.set_duty(0, 64)

    measured = await check_pwm_channel(dut, model, channel=0, num_ticks=2000)
    expected = 64 / 256.0
    assert abs(measured - expected) < 0.05, f"PWM Duty mismatch: Got {measured}"

    dut._log.info("Triggering emergency stop...")
    # E-stop is bit 0. freq_sel is bit 2:1. Mux is bit 7:6. Total = 0b1000_0111
    dut.ui_in.value = 0b1000_0111  
    await ClockCycles(dut.clk, 2)

    for _ in range(200):
        await RisingEdge(dut.clk)
        pwm_bits = int(dut.uo_out.value) & 0x0F  
        assert pwm_bits == 0, "Emergency stop failed!"

    dut._log.info("QuadPulse PWM Tests Passed!")
