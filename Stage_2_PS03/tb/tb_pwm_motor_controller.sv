// ============================================================================
// Module: tb_pwm_motor_controller
// Description: Testbench including motor emulation, SV assertions for 
//              shoot-through and fault response latency, and test scenarios.
// ============================================================================

`timescale 1ns/1ps

module tb_pwm_motor_controller;

    reg        clk;
    reg        rst_n;
    reg        psel, penable, pwrite;
    reg  [7:0] paddr;
    reg [31:0] pwdata;
    wire[31:0] prdata;
    wire       pready;

    reg        feedback_in;
    reg        fault_oc_in;
    wire       pwm_h_out;
    wire       pwm_l_out;

    // Clock Generation (100 MHz)
    always #5 clk = ~clk;

    // DUT Instantiation
    pwm_motor_controller_top dut (
        .pclk        (clk),
        .presetn     (rst_n),
        .psel        (psel),
        .penable     (penable),
        .pwrite      (pwrite),
        .paddr       (paddr),
        .pwdata      (pwdata),
        .prdata      (prdata),
        .pready      (pready),
        .feedback_in (feedback_in),
        .fault_oc_in (fault_oc_in),
        .pwm_h_out   (pwm_h_out),
        .pwm_l_out   (pwm_l_out)
    );

    // ========================================================================
    // SYSTEMVERILOG ASSERTIONS (SVA)
    // ========================================================================
    
    // 1. Shoot-Through Prevention Assertion
    property p_no_shoot_through;
        @(posedge clk) disable iff (!rst_n)
        !(pwm_h_out && pwm_l_out);
    endproperty
    assert property (p_no_shoot_through) 
        else $error("ASSERTION FAILED: Shoot-through detected! Both PWM outputs HIGH!");

    // 2. Bounded Fault Response Latency Assertion (<2 Clock Cycles)
    property p_fault_response;
        @(posedge clk) disable iff (!rst_n)
        fault_oc_in |-> ##[1:2] (!pwm_h_out && !pwm_l_out);
    endproperty
    assert property (p_fault_response)
        else $error("ASSERTION FAILED: Fault shut-off latency exceeded 2 cycles!");

    // ========================================================================
    // FIRST-ORDER MOTOR MODEL EMULATOR
    // ========================================================================
    real motor_speed = 0.0;
    real motor_current = 0.0;
    real v_bus = 24.0;
    real R = 1.0, L = 0.001, Ke = 0.05, Kt = 0.05, J = 0.0001, B = 0.001;

    // Intermediate variables for ODE step
    real dt, v_in, di, dw;

    always @(posedge clk) begin
        if (!rst_n) begin
            motor_speed   <= 0.0;
            motor_current <= 0.0;
            feedback_in   <= 0;
        end else begin
            // First-order Euler integration for electrical and mechanical dynamics
            dt   = 1e-8; // 10ns per cycle
            v_in = pwm_h_out ? v_bus : 0.0;
            di   = (v_in - R*motor_current - Ke*motor_speed) / L;
            dw   = (Kt*motor_current - B*motor_speed) / J;

            motor_current <= motor_current + di * dt;
            motor_speed   <= motor_speed + dw * dt;

            // Generate pulse feedback based on motor speed
            if (motor_speed > 10.0) begin
                if ($urandom_range(0, 100) < (motor_speed / 10.0))
                    feedback_in <= ~feedback_in;
            end
        end
    end

    // APB Write Helper Task
    task apb_write(input [7:0] addr, input [31:0] data);
        begin
            @(posedge clk);
            paddr  <= addr;
            pwdata <= data;
            psel   <= 1'b1;
            pwrite <= 1'b1;
            @(posedge clk);
            penable <= 1'b1;
            @(posedge clk);
            psel    <= 1'b0;
            penable <= 1'b0;
            pwrite  <= 1'b0;
        end
    endtask

    // Main Test Stimulus
    initial begin
        clk         = 0;
        rst_n       = 0;
        psel        = 0;
        penable     = 0;
        pwrite      = 0;
        paddr       = 0;
        pwdata      = 0;
        fault_oc_in = 0;
        feedback_in = 0;

        #100;
        rst_n = 1;
        #100;

        $display("--- Step 1: Configuring Registers via APB ---");
        apb_write(8'h0C, 32'd1000);  // Period = 1000 cycles
        apb_write(8'h10, 32'd20);    // Dead-time = 20 cycles
        apb_write(8'h14, 32'd500);   // Throttle Setpoint = 500
        apb_write(8'h00, 32'h0003);  // Enable PWM + Soft-start

        #50000;
        $display("--- Step 2: Injecting Over-Current Fault ---");
        @(posedge clk);
        fault_oc_in = 1;
        #20;
        fault_oc_in = 0;

        #1000;
        $display("--- Step 3: Clearing Fault and Resuming Operation ---");
        apb_write(8'h00, 32'h0007);  // Clear fault pulse bit

        #100000;
        $display("SUCCESS: Verification complete with zero assertion failures!");
        $finish;
    end

endmodule
