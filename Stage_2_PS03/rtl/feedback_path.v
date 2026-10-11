// ============================================================================
// Module: feedback_path
// Description: Synchronizes raw async feedback inputs (tach/encoder), filters
//              glitches, measures pulse frequencies over a window, and flags
//              feedback loss using a dedicated watchdog timer.
// ============================================================================

module feedback_path (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        feedback_raw,     // Async tachometer/encoder pulse input
    input  wire [15:0] window_period,    // Speed measurement window (clock cycles)
    input  wire [15:0] watchdog_limit,   // Max cycles allowed without edge before timeout
    output reg  [15:0] measured_speed,   // Pulses counted per window
    output reg         feedback_loss     // High if watchdog expires
);

    // 2-Stage Synchronizer for Clock Domain Crossing (CDC)
    reg [1:0] sync_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sync_reg <= 2'b00;
        else
            sync_reg <= {sync_reg[0], feedback_raw};
    end

    // Digital Debounce Filter (4-cycle stability requirement)
    reg [3:0] filter_shift;
    reg       clean_fb;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filter_shift <= 4'b0000;
            clean_fb     <= 1'b0;
        end else begin
            filter_shift <= {filter_shift[2:0], sync_reg[1]};
            if (&filter_shift)
                clean_fb <= 1'b1;
            else if (~|filter_shift)
                clean_fb <= 1'b0;
        end
    end

    // Edge Detection
    reg clean_fb_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) clean_fb_q <= 1'b0;
        else        clean_fb_q <= clean_fb;
    end
    wire edge_detected = clean_fb && !clean_fb_q;

    // Measurement Window and Pulse Accumulator
    reg [15:0] window_cnt;
    reg [15:0] pulse_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_cnt     <= 16'd0;
            pulse_cnt      <= 16'd0;
            measured_speed <= 16'd0;
        end else if (enable) begin
            if (edge_detected)
                pulse_cnt <= pulse_cnt + 16'd1;

            if (window_cnt >= window_period) begin
                window_cnt     <= 16'd0;
                measured_speed <= pulse_cnt;
                pulse_cnt      <= 16'd0;
            end else begin
                window_cnt <= window_cnt + 16'd1;
            end
        end else begin
            window_cnt     <= 16'd0;
            pulse_cnt      <= 16'd0;
            measured_speed <= 16'd0;
        end
    end

    // Watchdog Timer for Feedback Loss Detection
    reg [15:0] wd_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wd_cnt        <= 16'd0;
            feedback_loss <= 1'b0;
        end else if (enable) begin
            if (edge_detected) begin
                wd_cnt        <= 16'd0;
                feedback_loss <= 1'b0;
            end else if (wd_cnt >= watchdog_limit) begin
                feedback_loss <= 1'b1;
            end else begin
                wd_cnt <= wd_cnt + 16'd1;
            end
        end else begin
            wd_cnt        <= 16'd0;
            feedback_loss <= 1'b0;
        end
    end

endmodule
