// ============================================================================
// Module: protection_stage
// Description: Handles fault latching, soft-start ramp, duty cycle slew-rate
//              limiting, and guarantees bounded latency (<2 cycles) shutoff.
// ============================================================================

module protection_stage (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        soft_start_en,   // Enables soft-start ramp-up
    input  wire        fault_oc_raw,    // Over-current fault input pin
    input  wire        fault_fbloss,    // Watchdog feedback loss fault
    input  wire        fault_clear,     // SW clear pulse
    input  wire [15:0] target_duty,     // Duty from PI controller
    input  wire [15:0] slew_rate,       // Max duty step per period
    input  wire [15:0] soft_start_step, // Duty increment per step during startup
    input  wire        period_tick,     // Boundary update pulse
    input  wire        pwm_h_in,
    input  wire        pwm_l_in,
    output reg         pwm_h_safe,      // Final gated safe PWM output H
    output reg         pwm_l_safe,      // Final gated safe PWM output L
    output reg  [15:0] limited_duty,    // Slew/Soft-start rate-limited duty
    output reg  [1:0]  fault_status     // [1]: FB Loss, [0]: Over-current
);

    // 2-Flop CDC Synchronizer for Over-Current Pin
    reg [1:0] oc_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) oc_sync <= 2'b00;
        else        oc_sync <= {oc_sync[0], fault_oc_raw};
    end

    // Fault Latching Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_status <= 2'b00;
        end else begin
            if (fault_clear) begin
                fault_status <= 2'b00;
            end else begin
                if (oc_sync[1])     fault_status[0] <= 1'b1;
                if (fault_fbloss)   fault_status[1] <= 1'b1;
            end
        end
    end

    wire is_faulted = (|fault_status) || oc_sync[1] || fault_fbloss;

    // Soft-Start Ramp Logic
    reg [15:0] soft_ramp;
    wire [15:0] current_max = (soft_start_en && (soft_ramp < target_duty)) ? soft_ramp : target_duty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            soft_ramp    <= 16'd0;
            limited_duty <= 16'd0;
        end else if (is_faulted) begin
            soft_ramp    <= 16'd0;
            limited_duty <= 16'd0;
        end else if (period_tick) begin
            // 1. Soft-Start Ramp Update
            if (soft_start_en && (soft_ramp < target_duty)) begin
                soft_ramp <= soft_ramp + soft_start_step;
            end

            // 2. Slew Rate Limiting
            if (target_duty > limited_duty) begin
                if ((target_duty - limited_duty) > slew_rate) begin
                    if ((limited_duty + slew_rate) > current_max)
                        limited_duty <= current_max;
                    else
                        limited_duty <= limited_duty + slew_rate;
                end else begin
                    if (target_duty > current_max)
                        limited_duty <= current_max;
                    else
                        limited_duty <= target_duty;
                end
            end else if (limited_duty > target_duty) begin
                if ((limited_duty - target_duty) > slew_rate)
                    limited_duty <= limited_duty - slew_rate;
                else
                    limited_duty <= target_duty;
            end
        end
    end

    // Output Safe Gating: Synchronous bounded 1-cycle latency shut-off
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_h_safe <= 1'b0;
            pwm_l_safe <= 1'b0;
        end else if (is_faulted) begin
            pwm_h_safe <= 1'b0;
            pwm_l_safe <= 1'b0;
        end else begin
            pwm_h_safe <= pwm_h_in;
            pwm_l_safe <= pwm_l_in;
        end
    end

endmodule
