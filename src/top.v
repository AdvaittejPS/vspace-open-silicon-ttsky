`default_nettype none

module tt_um_multiplexer (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high)
    input  wire       ena,      // Goes high when design is enabled
    input  wire       clk,      // Clock
    input  wire       rst_n     // Reset_n (low to reset)
);

    // --------------------------------------------------------
    // The Bouncer: Design selection using the top two pins
    // 00 = SpO2 Engine
    // 01 = Stopwatch
    // 10 = QuadPulse PWM
    // --------------------------------------------------------
    wire [1:0] sel = ui_in[7:6];
    
    // The remaining 6 pins are passed to the sub-modules
    wire [5:0] shared_ui_in = ui_in[5:0];

    // --------------------------------------------------------
    // 00: Shailesh's SpO2 Engine
    // --------------------------------------------------------
    wire [7:0] spo2_uo_out, spo2_uio_out, spo2_uio_oe;
    
    tt_um_shailesh_spo2_engine spo2_inst (
        .ui_in   ({2'b00, shared_ui_in}), 
        .uo_out  (spo2_uo_out),
        .uio_in  (uio_in),
        .uio_out (spo2_uio_out),
        .uio_oe  (spo2_uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // --------------------------------------------------------
    // 01: Advait Tej's Stopwatch
    // --------------------------------------------------------
    wire [7:0] sw_uo_out, sw_uio_out, sw_uio_oe;
    
    tt_um_advaittej_stopwatch sw_inst (
        // WARNING: ui_in[7:6] are stripped. 
        // target_time[3:2] will always be 00 unless the module is rewritten.
        .ui_in   ({2'b00, shared_ui_in}), 
        .uo_out  (sw_uo_out),
        .uio_in  (uio_in),
        .uio_out (sw_uio_out),
        .uio_oe  (sw_uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // --------------------------------------------------------
    // 10: Sarvesh's QuadPulse PWM
    // --------------------------------------------------------
    wire [7:0] pwm_uo_out, pwm_uio_out, pwm_uio_oe;
    
    tt_um_quadpulse_pwm pwm_inst (
        .ui_in   ({2'b00, shared_ui_in}), 
        .uo_out  (pwm_uo_out),
        .uio_in  (uio_in),
        .uio_out (pwm_uio_out),
        .uio_oe  (pwm_uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // --------------------------------------------------------
    // Output Multiplexing
    // --------------------------------------------------------
    assign uo_out  = (sel == 2'b00) ? spo2_uo_out :
                     (sel == 2'b01) ? sw_uo_out :
                     (sel == 2'b10) ? pwm_uo_out :
                     8'b0;

    assign uio_out = (sel == 2'b00) ? spo2_uio_out :
                     (sel == 2'b01) ? sw_uio_out :
                     (sel == 2'b10) ? pwm_uio_out :
                     8'b0;

    assign uio_oe  = (sel == 2'b00) ? spo2_uio_oe :
                     (sel == 2'b01) ? sw_uio_oe :
                     (sel == 2'b10) ? pwm_uio_oe :
                     8'b0;

endmodule
