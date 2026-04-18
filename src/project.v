`default_nettype none

// ============================================================================
// 1. TOP LEVEL MULTIPLEXER
// ============================================================================
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
    // 00 = SNN AFib Detector
    // 01 = Stopwatch
    // 10 = QuadPulse PWM
    // --------------------------------------------------------
    wire [1:0] sel = ui_in[7:6];
    
    // The remaining 6 pins are passed to the sub-modules
    wire [5:0] shared_ui_in = ui_in[5:0];

    // --------------------------------------------------------
    // 00: SNN AFib Detector
    // --------------------------------------------------------
    wire [7:0] snn_uo_out, snn_uio_out, snn_uio_oe;
    
    tt_um_snn_afib_detector snn_inst (
        .ui_in   ({2'b00, shared_ui_in}), 
        .uo_out  (snn_uo_out),
        .uio_in  (uio_in),
        .uio_out (snn_uio_out),
        .uio_oe  (snn_uio_oe),
        .ena     (ena),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // --------------------------------------------------------
    // 01: Advait Tej's Stopwatch
    // --------------------------------------------------------
    wire [7:0] sw_uo_out, sw_uio_out, sw_uio_oe;
    
    tt_um_advaittej sw_inst (
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
    assign uo_out  = (sel == 2'b00) ? snn_uo_out :
                     (sel == 2'b01) ? sw_uo_out :
                     (sel == 2'b10) ? pwm_uo_out :
                     8'b0;

    assign uio_out = (sel == 2'b00) ? snn_uio_out :
                     (sel == 2'b01) ? sw_uio_out :
                     (sel == 2'b10) ? pwm_uio_out :
                     8'b0;

    assign uio_oe  = (sel == 2'b00) ? snn_uio_oe :
                     (sel == 2'b01) ? sw_uio_oe :
                     (sel == 2'b10) ? pwm_uio_oe :
                     8'b0;

endmodule

// ============================================================================
// 2. SNN AFIB DETECTOR & SUB-MODULES
// ============================================================================

module tt_um_snn_afib_detector (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // uio[0] is an output (asystole_flag)
    assign uio_out[7:1] = 7'b0;
    assign uio_oe       = 8'b0000_0001;   

    wire r_peak = ui_in[0];
    wire w_load = ui_in[1];
    wire w_data = ui_in[2];
    wire w_clk  = ui_in[3];

    wire [5:0]  rr_interval;
    wire [5:0]  rr_delta;
    wire        rr_valid;
    wire        asystole_flag;
    wire [3:0]  spike_interval;
    wire [3:0]  spike_delta;
    wire        spike_valid;
    wire [7:0]  neuron_spikes;
    wire        any_spike;
    wire        afib_flag;
    wire        out_valid;
    wire [1:0]  fsm_state;
    wire [2:0]  confidence;
    wire [2:0]  confidence_latch;

    rr_features u_rr_features (
        .clk          (clk),
        .rst_n        (rst_n),
        .ena          (ena),
        .r_peak       (r_peak),
        .rr_interval  (rr_interval),
        .rr_delta     (rr_delta),
        .rr_valid     (rr_valid),
        .asystole_flag(asystole_flag)
    );

    spike_encoder u_spike_enc (
        .clk           (clk),
        .rst_n         (rst_n),
        .ena           (ena),
        .rr_interval   (rr_interval),
        .rr_delta      (rr_delta),
        .rr_valid      (rr_valid),
        .spike_interval(spike_interval),
        .spike_delta   (spike_delta),
        .spike_valid   (spike_valid)
    );

    reservoir u_reservoir (
        .clk           (clk),
        .rst_n         (rst_n),
        .ena           (ena),
        .spike_interval(spike_interval),
        .spike_delta   (spike_delta),
        .spike_valid   (spike_valid),
        .neuron_spikes (neuron_spikes),
        .any_spike     (any_spike)
    );

    readout u_readout (
        .clk             (clk),
        .rst_n           (rst_n),
        .ena             (ena),
        .w_load          (w_load),
        .w_data          (w_data),
        .w_clk           (w_clk),
        .neuron_spikes   (neuron_spikes),
        .spike_valid     (spike_valid),
        .afib_flag       (afib_flag),
        .out_valid       (out_valid),
        .fsm_state       (fsm_state),
        .confidence      (confidence),
        .confidence_latch(confidence_latch)
    );

    assign uo_out[0]   = afib_flag;
    assign uo_out[1]   = out_valid;
    assign uo_out[2]   = any_spike;
    assign uo_out[3]   = fsm_state[0];
    assign uo_out[4]   = fsm_state[1];
    assign uo_out[7:5] = confidence_latch;
    assign uio_out[0]  = asystole_flag;   // bradycardia / asystole output
endmodule

module spike_encoder (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ena,
    input  wire [5:0] rr_interval,
    input  wire [5:0] rr_delta,
    input  wire       rr_valid,
    output reg  [3:0] spike_interval,
    output reg  [3:0] spike_delta,
    output reg        spike_valid
);
    wire [3:0] enc_interval = 4'd15 - rr_interval[5:2];
    wire [3:0] enc_delta    = (rr_delta > 6'd15) ? 4'd15 : rr_delta[3:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spike_interval <= 4'd0;
            spike_delta    <= 4'd0;
            spike_valid    <= 1'b0;
        end else if (ena) begin
            spike_valid <= 1'b0;
            if (rr_valid) begin
                spike_interval <= enc_interval;
                spike_delta    <= enc_delta;
                spike_valid    <= 1'b1;
            end
        end
    end
endmodule

module rr_features (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ena,
    input  wire       r_peak,
    output reg  [5:0] rr_interval,
    output reg  [5:0] rr_delta,
    output reg        rr_valid,
    output reg        asystole_flag   
);
    reg [15:0] tick_count;
    reg  [5:0] rr_prev;
    reg        r_peak_prev;
    wire       r_peak_rise = r_peak & ~r_peak_prev;
    wire brd_thresh = tick_count[15] | tick_count[14];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_count    <= 16'd0;
            rr_interval   <= 6'd32;
            rr_delta      <= 6'd0;
            rr_prev       <= 6'd32;
            rr_valid      <= 1'b0;
            r_peak_prev   <= 1'b0;
            asystole_flag <= 1'b0;
        end else if (ena) begin
            r_peak_prev <= r_peak;
            rr_valid    <= 1'b0;

            if (r_peak_rise) begin
                rr_interval   <= (tick_count[15:9] > 7'd63) ? 6'd63 : tick_count[15:9];
                rr_delta      <= (tick_count[15:9] > rr_prev) ? tick_count[15:9] - rr_prev : rr_prev - tick_count[15:9];
                rr_prev       <= (tick_count[15:9] > 7'd63) ? 6'd63 : tick_count[15:9];
                rr_valid      <= 1'b1;
                tick_count    <= 16'd0;
                asystole_flag <= 1'b0;
            end else begin
                if (tick_count < 16'hFFFF)
                    tick_count <= tick_count + 16'd1;
                if (brd_thresh)
                    asystole_flag <= 1'b1;
            end
        end
    end
endmodule

module reservoir (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ena,
    input  wire [3:0]  spike_interval,
    input  wire [3:0]  spike_delta,
    input  wire        spike_valid,
    output wire [7:0]  neuron_spikes,  
    output wire        any_spike
);
    wire [3:0] gi = spike_valid ? spike_interval : 4'b0;
    wire [3:0] gd = spike_valid ? spike_delta    : 4'b0;

    wire [7:0] s;
    assign neuron_spikes = s;
    assign any_spike     = |s;

    lif_neuron #(.THRESHOLD(8'd5), .WEIGHT(3'd4)) n0  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gi[0]), .spike_out(s[0]));
    lif_neuron #(.THRESHOLD(8'd4), .WEIGHT(3'd5)) n1  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gi[1]), .spike_out(s[1]));
    lif_neuron #(.THRESHOLD(8'd6), .WEIGHT(3'd3)) n2  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gi[2]), .spike_out(s[2]));
    lif_neuron #(.THRESHOLD(8'd4), .WEIGHT(3'd6)) n3  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gi[3]), .spike_out(s[3]));

    reg spike_reg1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            spike_reg1 <= 1'b0;
        else if (ena && spike_valid)
            spike_reg1 <= s[0];
    end

    lif_neuron #(.THRESHOLD(8'd3), .WEIGHT(3'd5)) n4  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gd[0]),              .spike_out(s[4]));
    lif_neuron #(.THRESHOLD(8'd3), .WEIGHT(3'd4)) n5  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gd[1]),              .spike_out(s[5]));
    lif_neuron #(.THRESHOLD(8'd4), .WEIGHT(3'd6)) n6  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gd[2]),              .spike_out(s[6]));
    lif_neuron #(.THRESHOLD(8'd2), .WEIGHT(3'd3)) n7  (.clk(clk),.rst_n(rst_n),.ena(ena),.spike_valid(spike_valid),.spike_in(gd[3] | spike_reg1), .spike_out(s[7]));
endmodule

module readout (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ena,
    input  wire        w_load,
    input  wire        w_data,
    input  wire        w_clk,
    input  wire [7:0]  neuron_spikes,   
    input  wire        spike_valid,
    output reg         afib_flag,
    output reg         out_valid,
    output reg  [1:0]  fsm_state,
    output wire [2:0]  confidence,
    output reg  [2:0]  confidence_latch
);
    localparam FAST_WINDOW  = 4'd8;
    localparam SLOW_WINDOW  = 5'd16;

    localparam signed [8:0]  FAST_THRESH  = -9'sd1;
    localparam signed [9:0]  SLOW_THRESH  = -10'sd2;

    localparam LOAD   = 2'b00;
    localparam RUN    = 2'b01;
    localparam OUTPUT = 2'b10;

    reg [23:0] weight_sr;
    reg        w_clk_prev;
    reg        w_load_seen;

    wire [2:0] w0  = weight_sr[ 2: 0];  wire [2:0] w1  = weight_sr[ 5: 3];
    wire [2:0] w2  = weight_sr[ 8: 6];  wire [2:0] w3  = weight_sr[11: 9];
    wire [2:0] w4  = weight_sr[14:12];  wire [2:0] w5  = weight_sr[17:15];
    wire [2:0] w6  = weight_sr[20:18];  wire [2:0] w7  = weight_sr[23:21];

    wire signed [8:0] ws0  = {{6{w0[2]}},  w0};
    wire signed [8:0] ws1  = {{6{w1[2]}},  w1};
    wire signed [8:0] ws2  = {{6{w2[2]}},  w2};
    wire signed [8:0] ws3  = {{6{w3[2]}},  w3};
    wire signed [8:0] ws4  = {{6{w4[2]}},  w4};
    wire signed [8:0] ws5  = {{6{w5[2]}},  w5};
    wire signed [8:0] ws6  = {{6{w6[2]}},  w6};
    wire signed [8:0] ws7  = {{6{w7[2]}},  w7};

    wire signed [8:0] c0  = neuron_spikes[0] ? ws0 : 9'sd0;
    wire signed [8:0] c1  = neuron_spikes[1] ? ws1 : 9'sd0;
    wire signed [8:0] c2  = neuron_spikes[2] ? ws2 : 9'sd0;
    wire signed [8:0] c3  = neuron_spikes[3] ? ws3 : 9'sd0;
    wire signed [8:0] c4  = neuron_spikes[4] ? ws4 : 9'sd0;
    wire signed [8:0] c5  = neuron_spikes[5] ? ws5 : 9'sd0;
    wire signed [8:0] c6  = neuron_spikes[6] ? ws6 : 9'sd0;
    wire signed [8:0] c7  = neuron_spikes[7] ? ws7 : 9'sd0;

    wire signed [12:0] cycle_sum =
        $signed(c0) + $signed(c1) + $signed(c2) + $signed(c3) +
        $signed(c4) + $signed(c5) + $signed(c6) + $signed(c7);

    wire signed [8:0]  cycle_fast = cycle_sum[8:0];   
    wire signed [9:0]  cycle_slow = cycle_sum[9:0];

    reg signed [8:0]  accum_fast;
    reg [3:0]         beat_fast;
    reg               afib_fast;
    reg signed [8:0]  accum_fast_snap;

    reg signed [9:0]  accum_slow;
    reg [4:0]         beat_slow;
    reg               afib_slow;

    assign confidence =
        (accum_fast_snap >= FAST_THRESH + 9'sd8) ? 3'b111 :
        (accum_fast_snap >= FAST_THRESH + 9'sd4) ? 3'b110 :
        (accum_fast_snap >= FAST_THRESH)          ? 3'b101 :
        (accum_fast_snap <= FAST_THRESH - 9'sd8) ? 3'b000 :
        (accum_fast_snap <= FAST_THRESH - 9'sd4) ? 3'b001 : 3'b010;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_sr        <= 24'b0;
            w_clk_prev       <= 1'b0;
            w_load_seen      <= 1'b0;
            accum_fast       <= 9'sd0;
            accum_slow       <= 10'sd0;
            accum_fast_snap  <= 9'sd0;
            beat_fast        <= 4'd0;
            beat_slow        <= 5'd0;
            afib_fast        <= 1'b0;
            afib_slow        <= 1'b0;
            afib_flag        <= 1'b0;
            out_valid        <= 1'b0;
            confidence_latch <= 3'b010;
            fsm_state        <= LOAD;
        end else if (ena) begin
            w_clk_prev  <= w_clk;
            case (fsm_state)
                LOAD: begin
                    out_valid <= 1'b0;
                    if (w_load) w_load_seen <= 1'b1;
                    if (w_clk & ~w_clk_prev) weight_sr <= {weight_sr[22:0], w_data};
                    if (w_load_seen && !w_load) begin
                        fsm_state  <= RUN;
                        accum_fast <= 9'sd0;
                        accum_slow <= 10'sd0;
                        beat_fast  <= 4'd0;
                        beat_slow  <= 5'd0;
                        w_load_seen <= 1'b0;
                    end
                end
                RUN: begin
                    if (w_load) begin
                        fsm_state <= LOAD;
                    end else if (spike_valid) begin
                        accum_fast <= accum_fast + cycle_fast;
                        accum_slow <= accum_slow + cycle_slow;
                        beat_fast  <= beat_fast  + 4'd1;
                        beat_slow  <= beat_slow  + 5'd1;

                        if (beat_fast == FAST_WINDOW - 1) begin
                            afib_fast       <= (accum_fast > FAST_THRESH);
                            accum_fast_snap <= accum_fast;
                            accum_fast      <= 9'sd0;
                            beat_fast       <= 4'd0;
                        end

                        if (beat_slow == SLOW_WINDOW - 1) begin
                            afib_slow  <= (accum_slow > SLOW_THRESH);
                            accum_slow <= 10'sd0;
                            beat_slow  <= 5'd0;
                            fsm_state  <= OUTPUT;
                        end
                    end
                end
                OUTPUT: begin
                    confidence_latch <= confidence;
                    afib_flag  <= afib_fast & afib_slow;
                    out_valid  <= 1'b1;
                    fsm_state  <= RUN;
                end
                default: fsm_state <= LOAD;
            endcase
        end
    end
endmodule

module lif_neuron #(
    parameter THRESHOLD = 8'd5,
    parameter WEIGHT    = 3'd4
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire spike_valid,   
    input  wire spike_in,
    output reg  spike_out
);
    reg [3:0] potential;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            potential <= 4'd0;
            spike_out <= 1'b0;
        end else if (ena && spike_valid) begin
            if (potential >= THRESHOLD) begin
                spike_out <= 1'b1;
                potential <= 4'd0;
            end else begin
                spike_out <= 1'b0;
                potential <= (potential >> 1) + (spike_in ? {1'b0, WEIGHT} : 4'd0);
            end
        end
    end
endmodule

// ============================================================================
// 3. STOPWATCH (WITH TARGET_TIME FIX)
// ============================================================================

module tt_um_advaittej #(
    parameter CLOCKS_PER_SECOND = 24'd9_999_999, // 10 MHz = 1 sec
    parameter CLOCKS_PER_BAUD   = 12'd1041       // 10 MHz / 9600 baud = ~1042
)(
    input  wire [7:0] ui_in,    
    output wire [7:0] uo_out,   
    input  wire [7:0] uio_in,   
    output wire [7:0] uio_out,  
    output wire [7:0] uio_oe,   
    input  wire       ena,      
    input  wire       clk,      
    input  wire       rst_n     
);
    wire reset_active = !rst_n;
    
    assign uio_oe = 8'b1111_0001; 
    assign uio_out[3:1] = 3'b0;
    wire [6:0] led_segments;
    assign uo_out[6:0] = led_segments;

    reg rx_s1, rx_s2;
    always @(posedge clk or posedge reset_active) begin
        if (reset_active) begin
            rx_s1 <= 1'b1; rx_s2 <= 1'b1;
        end else begin
            rx_s1 <= uio_in[1]; rx_s2 <= rx_s1;
        end
    end

    reg [1:0]  rx_state;
    reg [3:0]  rx_bit_idx;
    reg [11:0] rx_baud_counter;
    reg [7:0]  rx_shift_reg;
    reg [7:0]  rx_data;
    reg        rx_valid;

    always @(posedge clk or posedge reset_active) begin
        if (reset_active) begin
            rx_state <= 0; rx_valid <= 0;
            rx_baud_counter <= 0; rx_bit_idx <= 0;
            rx_shift_reg <= 0; rx_data <= 0;
        end else begin
            rx_valid <= 0;
            case (rx_state)
                0: begin 
                    if (rx_s2 == 1'b0) begin 
                        rx_state <= 1; rx_baud_counter <= 0;
                    end
                end
                1: begin 
                    if (rx_baud_counter == CLOCKS_PER_BAUD/2) begin
                        if (rx_s2 == 1'b0) begin
                            rx_state <= 2; rx_baud_counter <= 0; rx_bit_idx <= 0;
                        end else rx_state <= 0;
                    end else rx_baud_counter <= rx_baud_counter + 1;
                end
                2: begin 
                    if (rx_baud_counter == CLOCKS_PER_BAUD) begin
                        rx_baud_counter <= 0;
                        rx_shift_reg <= {rx_s2, rx_shift_reg[7:1]};
                        if (rx_bit_idx == 7) rx_state <= 3;
                        else rx_bit_idx <= rx_bit_idx + 1;
                    end else rx_baud_counter <= rx_baud_counter + 1;
                end
                3: begin 
                    if (rx_baud_counter == CLOCKS_PER_BAUD) begin
                        rx_baud_counter <= 0;
                        if (rx_s2 == 1'b1) begin 
                            rx_data <= rx_shift_reg; rx_valid <= 1'b1;
                        end
                        rx_state <= 0;
                    end else rx_baud_counter <= rx_baud_counter + 1;
                end
            endcase
        end
    end

    wire cmd_start = rx_valid && (rx_data == 8'h53 || rx_data == 8'h73);
    wire cmd_lap   = rx_valid && (rx_data == 8'h4C || rx_data == 8'h6C);
    wire cmd_reset = rx_valid && (rx_data == 8'h52 || rx_data == 8'h72);

    reg [7:0] btn_start_shift; reg [7:0] btn_lap_shift;
    reg       start_btn;       reg       start_btn_prev;
    reg       lap_btn;         reg       lap_btn_prev;

    always @(posedge clk or posedge reset_active) begin
        if (reset_active) begin
            btn_start_shift <= 8'b0; btn_lap_shift <= 8'b0;
            start_btn <= 0; start_btn_prev <= 0;
            lap_btn <= 0; lap_btn_prev <= 0;
        end else begin
            btn_start_shift <= {btn_start_shift[6:0], ui_in[0]};
            btn_lap_shift   <= {btn_lap_shift[6:0],   ui_in[1]};

            if (btn_start_shift == 8'hFF) start_btn <= 1'b1;
            else if (btn_start_shift == 8'h00) start_btn <= 1'b0;

            if (btn_lap_shift == 8'hFF) lap_btn <= 1'b1;
            else if (btn_lap_shift == 8'h00) lap_btn <= 1'b0;
            
            start_btn_prev <= start_btn;
            lap_btn_prev <= lap_btn;
        end
    end

    wire start_trigger = (start_btn && !start_btn_prev) || cmd_start;
    wire lap_trigger   = (lap_btn && !lap_btn_prev)     || cmd_lap;
    wire soft_reset    = cmd_reset;

    reg is_running;
    reg [23:0] clock_counter;
    wire one_second_pulse = (clock_counter == CLOCKS_PER_SECOND);
    
    reg [3:0] current_digit;
    reg [3:0] lap_1, lap_2, lap_3;
    assign uio_out[7:4] = lap_1; 

    always @(posedge clk or posedge reset_active) begin
        if (reset_active) begin
            is_running <= 0; clock_counter <= 0; current_digit <= 0;
            lap_1 <= 0; lap_2 <= 0; lap_3 <= 0;
        end else if (soft_reset) begin
            is_running <= 0; clock_counter <= 0; current_digit <= 0;
            lap_1 <= 0; lap_2 <= 0; lap_3 <= 0;
        end else begin
            if (start_trigger) is_running <= ~is_running;

            if (is_running) begin
                if (one_second_pulse) begin
                    clock_counter <= 0;
                    if (current_digit == 9) current_digit <= 0;
                    else current_digit <= current_digit + 1;
                end else begin
                    clock_counter <= clock_counter + 1;
                end
            end
            
            if (lap_trigger) begin
                lap_3 <= lap_2; lap_2 <= lap_1; lap_1 <= current_digit;
            end
        end
    end

    reg [6:0] decoded_leds;
    assign led_segments = decoded_leds;
    always @(*) begin
        case (current_digit)
            4'd0: decoded_leds = 7'b0111111; 4'd1: decoded_leds = 7'b0000110;
            4'd2: decoded_leds = 7'b1011011; 4'd3: decoded_leds = 7'b1001111;
            4'd4: decoded_leds = 7'b1100110; 4'd5: decoded_leds = 7'b1101101;
            4'd6: decoded_leds = 7'b1111101; 4'd7: decoded_leds = 7'b0000111;
            4'd8: decoded_leds = 7'b1111111; 4'd9: decoded_leds = 7'b1101111;
            default: decoded_leds = 7'b0000000;
        endcase
    end

    // ALARM READS FROM PINS 5:2
    wire [3:0] target_time = ui_in[5:2]; 
    wire alarm_match = (current_digit == target_time);
    assign uo_out[7] = alarm_match;

    reg alarm_match_prev;
    always @(posedge clk or posedge reset_active) begin
        if (reset_active) alarm_match_prev <= 1'b0;
        else alarm_match_prev <= alarm_match;
    end
    wire alarm_trigger_pulse = (alarm_match && !alarm_match_prev);

    reg [11:0] tx_baud_counter;
    reg [3:0]  tx_bit_idx;     
    reg [3:0]  char_idx;       
    reg [7:0]  tx_shift_reg;
    reg        tx_active;
    reg        tx_pin_reg;
    assign uio_out[0] = tx_pin_reg;
    wire tx_baud_tick = (tx_baud_counter == CLOCKS_PER_BAUD);

    reg [7:0] char_to_send;
    always @(*) begin
        case(char_idx)
            4'd0:  char_to_send = 8'h56; 4'd1:  char_to_send = 8'h49;
            4'd2:  char_to_send = 8'h54; 4'd3:  char_to_send = 8'h20;
            4'd4:  char_to_send = 8'h56; 4'd5:  char_to_send = 8'h65;
            4'd6:  char_to_send = 8'h6C; 4'd7:  char_to_send = 8'h6C;
            4'd8:  char_to_send = 8'h6F; 4'd9:  char_to_send = 8'h72;
            4'd10: char_to_send = 8'h65; 4'd11: char_to_send = 8'h0D;
            4'd12: char_to_send = 8'h0A; default: char_to_send = 8'h00;
        endcase
    end

    always @(posedge clk or posedge reset_active) begin
        if (reset_active) begin
            tx_baud_counter <= 0; tx_bit_idx <= 0; char_idx <= 0;
            tx_active <= 0; tx_pin_reg <= 1'b1;
        end else begin
            if (alarm_trigger_pulse && !tx_active) begin
                tx_active <= 1'b1; char_idx <= 0; tx_bit_idx <= 0;
                tx_baud_counter <= 0; tx_shift_reg <= 8'h56;
            end

            if (tx_active) begin
                if (tx_baud_tick) begin
                    tx_baud_counter <= 0;
                    if (tx_bit_idx == 0) begin
                        tx_pin_reg <= 1'b0; tx_bit_idx <= tx_bit_idx + 1;
                    end else if (tx_bit_idx <= 8) begin
                        tx_pin_reg <= tx_shift_reg[0];
                        tx_shift_reg <= {1'b0, tx_shift_reg[7:1]};
                        tx_bit_idx <= tx_bit_idx + 1;
                    end else if (tx_bit_idx == 9) begin
                        tx_pin_reg <= 1'b1;
                        if (char_idx == 4'd12) tx_active <= 1'b0; 
                        else begin
                            char_idx <= char_idx + 1; tx_bit_idx <= 0; 
                        end
                    end
                end else begin
                    tx_baud_counter <= tx_baud_counter + 1;
                    if (tx_bit_idx == 0 && tx_baud_counter == 1) tx_shift_reg <= char_to_send;
                end
            end
        end
    end
endmodule

// ============================================================================
// 4. QUADPULSE PWM
// ============================================================================

module tt_um_quadpulse_pwm (
    input  wire [7:0] ui_in,    
    output wire [7:0] uo_out,   
    input  wire [7:0] uio_in,   
    output wire [7:0] uio_out,  
    output wire [7:0] uio_oe,   
    input  wire       ena,      
    input  wire       clk,      
    input  wire       rst_n     
);
    wire emergency_stop  = ui_in[0];
    wire freq_sel_0_raw  = ui_in[1];
    wire freq_sel_1_raw  = ui_in[2];

    wire spi_mosi_raw    = uio_in[0];
    wire spi_sclk_raw    = uio_in[1];
    wire spi_cs_n_raw    = uio_in[2];

    assign uio_oe  = 8'b0000_0000;
    assign uio_out = 8'b0000_0000;

    wire reset = ~rst_n;

    reg spi_sclk_s1, spi_sclk_s2; 
    reg spi_mosi_s1, spi_mosi_s2;
    reg spi_cs_n_s1, spi_cs_n_s2;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            spi_sclk_s1 <= 1'b0;  spi_sclk_s2 <= 1'b0;
            spi_mosi_s1 <= 1'b0;  spi_mosi_s2 <= 1'b0;
            spi_cs_n_s1 <= 1'b1;  spi_cs_n_s2 <= 1'b1;
        end else begin
            spi_sclk_s1 <= spi_sclk_raw;  spi_sclk_s2 <= spi_sclk_s1;
            spi_mosi_s1 <= spi_mosi_raw;  spi_mosi_s2 <= spi_mosi_s1;
            spi_cs_n_s1 <= spi_cs_n_raw;  spi_cs_n_s2 <= spi_cs_n_s1;
        end
    end

    wire spi_sclk = spi_sclk_s2;
    wire spi_mosi = spi_mosi_s2;
    wire spi_cs_n = spi_cs_n_s2;

    reg [1:0] freq_sel_sync; 
    reg [1:0] freq_sel_prev;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            freq_sel_sync <= 2'b00; freq_sel_prev <= 2'b00;
        end else begin
            freq_sel_sync <= {freq_sel_1_raw, freq_sel_0_raw};
            freq_sel_prev <= freq_sel_sync;
        end
    end

    wire freq_sel_changed = (freq_sel_sync != freq_sel_prev);
    reg [17:0] period_counter;

    wire [17:0] pwm_period =
        (freq_sel_sync == 2'b00) ? 18'd199935 :
        (freq_sel_sync == 2'b01) ? 18'd9983   :
        (freq_sel_sync == 2'b10) ? 18'd767    : 18'd255;

    wire period_done = (period_counter == pwm_period);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            period_counter <= 18'd0;
        end else if (freq_sel_changed || period_done) begin
            period_counter <= 18'd0;
        end else begin
            period_counter <= period_counter + 18'd1;
        end
    end

    wire [17:0] sub_period =
        (freq_sel_sync == 2'b00) ? 18'd781 :
        (freq_sel_sync == 2'b01) ? 18'd39  :
        (freq_sel_sync == 2'b10) ? 18'd3   : 18'd1;

    reg [17:0] sub_counter;
    reg  [7:0] ramp;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            sub_counter <= 18'd0; ramp <= 8'd0;
        end else if (freq_sel_changed || period_done) begin
            sub_counter <= 18'd0; ramp <= 8'd0;
        end else if (sub_counter == sub_period - 18'd1) begin
            sub_counter <= 18'd0;
            if (ramp < 8'd255) ramp <= ramp + 8'd1;
        end else begin
            sub_counter <= sub_counter + 18'd1;
        end
    end

    reg [7:0] duty_ch0; reg [7:0] duty_ch1; 
    reg [7:0] duty_ch2; reg [7:0] duty_ch3; 

    wire pwm_ch0 = (ramp < duty_ch0) && !emergency_stop;
    wire pwm_ch1 = (ramp < duty_ch1) && !emergency_stop;
    wire pwm_ch2 = (ramp < duty_ch2) && !emergency_stop;
    wire pwm_ch3 = (ramp < duty_ch3) && !emergency_stop;

    reg [14:0] spi_shift_reg;
    reg  [3:0] spi_bit_count;
    reg        spi_sclk_prev;

    wire spi_sclk_rising = (spi_sclk == 1'b1) && (spi_sclk_prev == 1'b0);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            spi_shift_reg <= 15'd0; spi_bit_count <= 4'd0;
            spi_sclk_prev <= 1'b0;
            duty_ch0 <= 8'd127; duty_ch1 <= 8'd127;
            duty_ch2 <= 8'd127; duty_ch3 <= 8'd127;
        end else begin
            spi_sclk_prev <= spi_sclk;

            if (spi_cs_n == 1'b1) begin
                spi_bit_count <= 4'd0; spi_shift_reg <= 15'd0;
            end else if (spi_sclk_rising) begin
                spi_shift_reg <= {spi_shift_reg[13:0], spi_mosi};
                spi_bit_count <= spi_bit_count + 4'd1;

                if (spi_bit_count == 4'd15) begin
                    case (spi_shift_reg[14:13])  
                        2'b00: duty_ch0 <= {spi_shift_reg[6:0], spi_mosi};
                        2'b01: duty_ch1 <= {spi_shift_reg[6:0], spi_mosi};
                        2'b10: duty_ch2 <= {spi_shift_reg[6:0], spi_mosi};
                        2'b11: duty_ch3 <= {spi_shift_reg[6:0], spi_mosi};
                    endcase
                    spi_bit_count <= 4'd0;
                end
            end
        end
    end

    assign uo_out[0] = pwm_ch0; assign uo_out[1] = pwm_ch1;        
    assign uo_out[2] = pwm_ch2; assign uo_out[3] = pwm_ch3;        
    assign uo_out[4] = emergency_stop;
    assign uo_out[5] = 1'b0; assign uo_out[6] = 1'b0; assign uo_out[7] = 1'b0;           
endmodule
