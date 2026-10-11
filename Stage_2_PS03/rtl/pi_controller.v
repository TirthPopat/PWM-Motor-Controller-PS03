// ============================================================================
// Module: pi_controller
// Description: Fixed-point Q16.8 PI controller with output clamping and 
//              integrator anti-windup to prevent accumulation overflow.
// ============================================================================

module pi_controller (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        calc_trigger,      // Triggered right before period boundary
    input  wire [15:0] setpoint,          // Target throttle/speed command
    input  wire [15:0] feedback,          // Measured speed from feedback path
    input  wire [15:0] kp,                // Proportional gain (Q8.8 format)
    input  wire [15:0] ki,                // Integral gain (Q8.8 format)
    input  wire [15:0] duty_min,          // Minimum clamp limit
    input  wire [15:0] duty_max,          // Maximum clamp limit
    output reg  [15:0] pi_duty_out        // Computed raw duty cycle
);

    // Signed Error Computation
    wire signed [16:0] error = $signed({1'b0, setpoint}) - $signed({1'b0, feedback});

    // 32-bit Accumulator for Integrator State (Q16.16)
    reg signed [31:0] integrator;

    // Intermediate Term Products
    wire signed [32:0] p_term = error * $signed({1'b0, kp});
    wire signed [32:0] i_term = error * $signed({1'b0, ki});

    // Max Integrator Clamp Limit
    localparam signed [31:0] INT_MAX = 32'sd16777215;
    localparam signed [31:0] INT_MIN = -32'sd16777215;

    // Combinational Intermediate Calculation
    wire signed [31:0] next_int = integrator + i_term[31:0];
    wire signed [32:0] raw_sum  = (p_term + {integrator[31], integrator[31:1]}) >>> 8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integrator  <= 32'sd0;
            pi_duty_out <= 16'd0;
        end else if (enable) begin
            if (calc_trigger) begin
                // Update Integrator with Anti-Windup Clamping
                if (next_int > INT_MAX)
                    integrator <= INT_MAX;
                else if (next_int < INT_MIN)
                    integrator <= INT_MIN;
                else
                    integrator <= next_int;

                // Duty Clamping
                if (raw_sum < $signed({1'b0, duty_min}))
                    pi_duty_out <= duty_min;
                else if (raw_sum > $signed({1'b0, duty_max}))
                    pi_duty_out <= duty_max;
                else
                    pi_duty_out <= raw_sum[15:0];
            end
        end else begin
            integrator  <= 32'sd0;
            pi_duty_out <= 16'd0;
        end
    end

endmodule
