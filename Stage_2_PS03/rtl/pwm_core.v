// ============================================================================
// Module: pwm_core
// Description: 16-bit PWM generator with double-buffered duty cycle reloading
//              at period boundaries (glitch-free) and complementary output 
//              generation with programmable dead-time insertion.
// ============================================================================

module pwm_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [15:0] prescaler,
    input  wire [15:0] period,
    input  wire [15:0] duty_in,        // Next duty cycle from PI/Protection
    input  wire [15:0] dead_time,      // Dead-time duration in clock cycles
    output reg  [15:0] active_duty,    // Shadow duty cycle currently in use
    output reg         period_tick,    // High for 1 cycle at period boundary
    output reg         pwm_h,          // High-side complementary output
    output reg         pwm_l           // Low-side complementary output
);

    reg [15:0] prescale_cnt;
    reg [15:0] pwm_cnt;
    reg        raw_pwm;
    
    // Dead-time state counters
    reg [15:0] dt_h_cnt;
    reg [15:0] dt_l_cnt;

    // Prescaler and Primary PWM Counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prescale_cnt <= 16'd0;
            pwm_cnt      <= 16'd0;
            active_duty  <= 16'd0;
            period_tick  <= 1'b0;
        end else if (enable) begin
            period_tick <= 1'b0;
            if (prescale_cnt >= prescaler) begin
                prescale_cnt <= 16'd0;
                if (pwm_cnt >= period) begin
                    pwm_cnt     <= 16'd0;
                    active_duty <= duty_in;  // Double-buffered glitch-free update
                    period_tick <= 1'b1;
                end else begin
                    pwm_cnt <= pwm_cnt + 16'd1;
                end
            end else begin
                prescale_cnt <= prescale_cnt + 16'd1;
            end
        end else begin
            prescale_cnt <= 16'd0;
            pwm_cnt      <= 16'd0;
            active_duty  <= 16'd0;
            period_tick  <= 1'b0;
        end
    end

    // Raw PWM Comparison
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_pwm <= 1'b0;
        end else if (enable) begin
            raw_pwm <= (pwm_cnt < active_duty);
        end else begin
            raw_pwm <= 1'b0;
        end
    end

    // Dead-time Logic for Complementary Outputs (Shoot-Through Prevention)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dt_h_cnt <= 16'd0;
            dt_l_cnt <= 16'd0;
            pwm_h    <= 1'b0;
            pwm_l    <= 1'b0;
        end else if (enable) begin
            // High-side dead-time insertion
            if (raw_pwm) begin
                dt_l_cnt <= 16'd0;
                if (dt_h_cnt < dead_time) begin
                    dt_h_cnt <= dt_h_cnt + 16'd1;
                    pwm_h    <= 1'b0;
                end else begin
                    pwm_h    <= 1'b1;
                end
            end else begin
                dt_h_cnt <= 16'd0;
                pwm_h    <= 1'b0;
            end

            // Low-side dead-time insertion (active high complementary)
            if (!raw_pwm) begin
                dt_h_cnt <= 16'd0;
                if (dt_l_cnt < dead_time) begin
                    dt_l_cnt <= dt_l_cnt + 16'd1;
                    pwm_l    <= 1'b0;
                end else begin
                    pwm_l    <= 1'b1;
                end
            end else begin
                dt_l_cnt <= 16'd0;
                pwm_l    <= 1'b0;
            end
        end else begin
            dt_h_cnt <= 16'd0;
            dt_l_cnt <= 16'd0;
            pwm_h    <= 1'b0;
            pwm_l    <= 1'b0;
        end
    end

endmodule
