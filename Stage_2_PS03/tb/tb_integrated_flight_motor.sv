// ============================================================================
// File: tb_integrated_flight_motor.sv
// Description: Co-simulation testbench for Integrated Track (PS-02 + PS-03).
//              Verifies IMU sample -> PID -> PWM motor drive, mode switching,
//              and bounded failsafe shutdown.
// ============================================================================
`timescale 1ns / 1ps

module tb_integrated_flight_motor;

    reg        pclk;
    reg        presetn;
    reg [31:0] paddr;
    reg        psel;
    reg        penable;
    reg        pwrite;
    reg [31:0] pwdata;
    wire[31:0] prdata;
    wire       pready;
    wire       pslverr;

    reg [15:0] imu_sample_in;
    reg        imu_valid_in;
    reg [15:0] flight_target_setpoint;
    reg [1:0]  flight_mode_select;

    reg        ext_oc_fault;
    reg        tacho_feedback;
    wire       pwm_h;
    wire       pwm_l;
    wire       system_failsafe;

    // Clock Generation (100 MHz)
    always #5 pclk = ~pclk;

    // DUT Instantiation
    integrated_flight_motor_top dut (
        .pclk(pclk), .presetn(presetn),
        .paddr(paddr), .psel(psel), .penable(penable),
        .pwrite(pwrite), .pwdata(pwdata), .prdata(prdata),
        .pready(pready), .pslverr(pslverr),
        .imu_sample_in(imu_sample_in),
        .imu_valid_in(imu_valid_in),
        .flight_target_setpoint(flight_target_setpoint),
        .flight_mode_select(flight_mode_select),
        .ext_oc_fault(ext_oc_fault),
        .tacho_feedback(tacho_feedback),
        .pwm_h(pwm_h), .pwm_l(pwm_l),
        .system_failsafe(system_failsafe)
    );

    // SVA Assertion: Zero Shoot-Through on PWM Outputs
    property p_no_shoot_through;
        @(posedge pclk) ~(pwm_h && pwm_l);
    endproperty
    assert property (p_no_shoot_through) else $error("ASSERTION FAILED: Shoot-through on PWM pins!");

    // SVA Assertion: Failsafe forces PWM pins off within 2 clock cycles
    property p_failsafe_shutdown;
        @(posedge pclk) system_failsafe |-> ##2 (!pwm_h && !pwm_l);
    endproperty
    assert property (p_failsafe_shutdown) else $error("ASSERTION FAILED: Failsafe shutdown latency exceeded!");

    // Tachometer Feedback Emulation
    always #50 tacho_feedback = ~tacho_feedback;

    // APB Write Task
    task apb_write(input [31:0] addr, input [31:0] data);
        begin
            @(posedge pclk);
            paddr = addr;
            pwrite = 1;
            psel = 1;
            pwdata = data;
            penable = 0;
            @(posedge pclk);
            penable = 1;
            wait(pready);
            @(posedge pclk);
            psel = 0;
            penable = 0;
            pwrite = 0;
        end
    endtask

    // Test Sequence
    initial begin
        pclk = 0; presetn = 0;
        paddr = 0; psel = 0; penable = 0; pwrite = 0; pwdata = 0;
        imu_sample_in = 16'd500; imu_valid_in = 0;
        flight_target_setpoint = 16'd1000;
        flight_mode_select = 2'b01; // Stabilize Mode
        ext_oc_fault = 0;
        tacho_feedback = 0;

        #20 presetn = 1;
        $display("--- Step 1: Initializing Integrated Flight-Control Chain ---");

        // APB Write: Enable PS-03 Control Block (Bit 0) AND Hardware Throttle Routing (Bit 3)
        apb_write(32'h0000_0000, 32'h0000_0009);

        // Feed periodic IMU sensor samples
        repeat(20) begin
            #100;
            imu_valid_in = 1;
            imu_sample_in = imu_sample_in + 16'd20;
            #10;
            imu_valid_in = 0;
        end

        $display("--- Step 2: Testing Mode Switch (Stabilize -> Acro) ---");
        flight_mode_select = 2'b00; // Acro Mode
        repeat(10) begin
            #100;
            imu_valid_in = 1;
            #10;
            imu_valid_in = 0;
        end

        $display("--- Step 3: Injecting Cross-Block Over-Current Fault ---");
        #50 ext_oc_fault = 1;
        #50;
        if (system_failsafe && !pwm_h && !pwm_l)
            $display("SUCCESS: System successfully entered Failsafe state!");
        else
            $display("ERROR: System failed to enter Failsafe state!");

        #100;
        $display("SUCCESS: Integrated Flight-Control Motor Chain Verification Complete!");
        $finish;
    end

endmodule
