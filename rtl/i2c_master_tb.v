// Language: Verilog 2001

`timescale 1ns/1ps

/*
 * Testbench for i2c_master
 */
module i2c_master_tb;

  // Parameters

  parameter CLK_PERIOD = 10;  // 10ns clock period (100MHz clock)
    parameter ENABLE_DEVICE_3 = 1;  // Set to 0 to disable device 3
  parameter ENABLE_DEVICE_4 = 1;  // Set to 0 to disable device 4

    reg clk = 0;
  reg rst = 0;
  reg [7:0] current_test = 0;

  // I2C master signals
  reg [6:0] s_axis_cmd_address = 0;
  reg s_axis_cmd_start = 0;
  reg s_axis_cmd_read = 0;
  reg s_axis_cmd_write = 0;
  reg s_axis_cmd_write_multiple = 0;
  reg s_axis_cmd_stop = 0;
  reg s_axis_cmd_valid = 0;
  reg [7:0] s_axis_data_tdata = 0;
  reg s_axis_data_tvalid = 0;
  reg s_axis_data_tlast = 0;
  reg m_axis_data_tready = 0;
  reg scl_i = 1;
  reg sda_i = 1;
  reg [15:0] prescale = 0;
  reg stop_on_idle = 0;

  wire s_axis_cmd_ready;
  wire s_axis_data_tready;
  wire [7:0] m_axis_data_tdata;
  wire m_axis_data_tvalid;
  wire m_axis_data_tlast;
  wire scl_o;
  wire scl_t;
  wire sda_o;
  wire sda_t;
  wire busy;
  wire bus_control;
  wire bus_active;
  wire missed_ack;

  // Wires for modeling pull-up resistors and open-drain outputs
  wire scl_wire;
  wire sda_wire;

  // Dummy registers
  reg sda2 = 1;
  reg scl2 = 1;

  // Generate block for Device 3
  generate
    if (ENABLE_DEVICE_3) begin : device_3
      wire sda_o_3;
      wire sda_t_3;
      wire scl_o_3;
      wire scl_t_3;
      reg [7:0] data_in_3;
      reg data_latch_3;
      wire [7:0] data_out_3;

      i2c_single_reg #(
          .FILTER_LEN(4),
          .DEV_ADDR  (7'h70)
      ) i2c_reg (
          .clk(clk),
          .rst(rst),
          .scl_i(scl_wire),
          .scl_o(scl_o_3),
          .scl_t(scl_t_3),
          .sda_i(sda_wire),
          .sda_o(sda_o_3),
          .sda_t(sda_t_3),
          .data_in(data_in_3),
          .data_latch(data_latch_3),
          .data_out(data_out_3)
      );
    end
  endgenerate

  // Generate block for Device 4
  generate
    if (ENABLE_DEVICE_4) begin : device_4
      wire sda_o_4;
      wire sda_t_4;
      wire scl_o_4;
      wire scl_t_4;
      wire [7:0] m_axis_data_tdata_4;
      wire m_axis_data_tvalid_4;
      wire m_axis_data_tready_4;
      wire m_axis_data_tlast_4;
      wire [7:0] s_axis_data_tdata_4;
      wire s_axis_data_tvalid_4;
      wire s_axis_data_tready_4;
      wire s_axis_data_tlast_4;
      wire busy_4;
      wire [6:0] bus_address_4;
      wire bus_addressed_4;
      wire bus_active_4;
      reg release_bus_4;
      reg enable_4;
      reg [6:0] device_address_4;
      reg [6:0] device_address_mask_4;

      i2c_slave #(
          .FILTER_LEN(4)
      ) i2c_slave_inst (
          .clk(clk),
          .rst(rst),
          .release_bus(release_bus_4),
          .s_axis_data_tdata(s_axis_data_tdata_4),
          .s_axis_data_tvalid(s_axis_data_tvalid_4),
          .s_axis_data_tready(s_axis_data_tready_4),
          .s_axis_data_tlast(s_axis_data_tlast_4),
          .m_axis_data_tdata(m_axis_data_tdata_4),
          .m_axis_data_tvalid(m_axis_data_tvalid_4),
          .m_axis_data_tready(m_axis_data_tready_4),
          .m_axis_data_tlast(m_axis_data_tlast_4),
          .scl_i(scl_wire),
          .scl_o(scl_o_4),
          .scl_t(scl_t_4),
          .sda_i(sda_wire),
          .sda_o(sda_o_4),
          .sda_t(sda_t_4),
          .busy(busy_4),
          .bus_address(bus_address_4),
          .bus_addressed(bus_addressed_4),
          .bus_active(bus_active_4),
          .enable(enable_4),
          .device_address(device_address_4),
          .device_address_mask(device_address_mask_4)
      );
    end
  endgenerate

  // Model pull-up resistors with weak pull-ups
  pullup (scl_wire);
  pullup (sda_wire);

  // Model open-drain outputs, including conditional devices
  assign scl_wire = (scl_o & scl2 & 
                     (ENABLE_DEVICE_3 ? device_3.scl_o_3 : 1'b1) & 
                     (ENABLE_DEVICE_4 ? device_4.scl_o_4 : 1'b1)) ? 1'bz : 1'b0;
  assign sda_wire = (sda_o & sda2 & 
                     (ENABLE_DEVICE_3 ? device_3.sda_o_3 : 1'b1) & 
                     (ENABLE_DEVICE_4 ? device_4.sda_o_4 : 1'b1)) ? 1'bz : 1'b0;


  always #(CLK_PERIOD / 2) clk <= ~clk;

  // Sample the bus with non-blocking assignments to avoid race conditions
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      scl_i <= 1'b1;
      sda_i <= 1'b1;
    end else begin
      scl_i <= scl_wire;
      sda_i <= sda_wire;

      // Assert that sda_i is not X
      if (sda_i === 1'bx) $fatal(1, "sda_i is X at time %t", $time);
    end
  end

  initial begin
    // dump file
    $dumpfile("i2c_master_tb.fst");
    $dumpvars(0, i2c_master_tb);

  end

  i2c_master UUT (
      .clk(clk),
      .rst(rst),
      .s_axis_cmd_address(s_axis_cmd_address),
      .s_axis_cmd_start(s_axis_cmd_start),
      .s_axis_cmd_read(s_axis_cmd_read),
      .s_axis_cmd_write(s_axis_cmd_write),
      .s_axis_cmd_write_multiple(s_axis_cmd_write_multiple),
      .s_axis_cmd_stop(s_axis_cmd_stop),
      .s_axis_cmd_valid(s_axis_cmd_valid),
      .s_axis_cmd_ready(s_axis_cmd_ready),
      .s_axis_data_tdata(s_axis_data_tdata),
      .s_axis_data_tvalid(s_axis_data_tvalid),
      .s_axis_data_tready(s_axis_data_tready),
      .s_axis_data_tlast(s_axis_data_tlast),
      .m_axis_data_tdata(m_axis_data_tdata),
      .m_axis_data_tvalid(m_axis_data_tvalid),
      .m_axis_data_tready(m_axis_data_tready),
      .m_axis_data_tlast(m_axis_data_tlast),
      .scl_i(scl_i),
      .scl_o(scl_o),
      .scl_t(scl_t),
      .sda_i(sda_i),
      .sda_o(sda_o),
      .sda_t(sda_t),
      .busy(busy),
      .bus_control(bus_control),
      .bus_active(bus_active),
      .missed_ack(missed_ack),
      .prescale(prescale),
      .stop_on_idle(stop_on_idle)
  );

  task initialize_testbench;
    begin
      clk = 0;
      rst = 1;
      s_axis_cmd_address = 0;
      s_axis_cmd_start = 0;
      s_axis_cmd_read = 0;
      s_axis_cmd_write = 0;
      s_axis_cmd_write_multiple = 0;
      s_axis_cmd_stop = 0;
      s_axis_cmd_valid = 0;
      s_axis_data_tdata = 0;
      s_axis_data_tvalid = 0;
      s_axis_data_tlast = 0;
      m_axis_data_tready = 0;
      scl_i = 1;
      sda_i = 1;
      prescale = 0;
      stop_on_idle = 0;
      sda2 = 1;
      scl2 = 1;

      #10 rst = 0;
    end
  endtask

  task i2c_start;
    input [6:0] address;
    input is_read;
    begin
      s_axis_cmd_address = address;
      s_axis_cmd_start = 1;
      s_axis_cmd_read = is_read;
      s_axis_cmd_write = !is_read;
      s_axis_cmd_valid = 1;

      @(negedge sda_wire);
      @(negedge scl_wire);  //start condition done! 

      s_axis_cmd_start = 0;
      s_axis_cmd_read  = 0;
      s_axis_cmd_write = 0;
      s_axis_cmd_valid = 0;
    end
  endtask

  task send_byte;
    input [7:0] byte_to_send;
    integer i;
    begin
      for (i = 7; i >= 0; i = i - 1) begin
        sda2 = byte_to_send[i];
        @(negedge scl_o);
      end
    end

  endtask

  task wait_for_ready;
    begin
      $display("Waiting for ready");
      @(posedge s_axis_cmd_ready);
      #CLK_PERIOD;
    end
  endtask

  task send_ack;
    begin
      @(negedge scl_o);
      sda2 = 0;
      #CLK_PERIOD;
      @(negedge scl_o);
      sda2 = 1;
    end
  endtask

  // Task to test NACK handling
task test_nack_handling;
    begin
        $display("Testing NACK handling.");
        i2c_start(7'b0000001, 0);
        s_axis_data_tdata = 8'b10100101;
        s_axis_data_tvalid = 1;
        @(posedge missed_ack);
        $display("NACK expected!");
        wait_for_ready();
    end
endtask

// Task to test writing
task test_writing;
    begin
        $display("Testing Writing.");
        i2c_start(7'b0000001, 0);
        s_axis_data_tdata = 8'b10101111;
        s_axis_data_tvalid = 1;
        repeat(7) @(negedge scl_o);
        send_ack();
        @(negedge scl_o);
         //send_byte(8'b01111111);
        wait_for_ready();
       end
endtask

// Task to test reading
task test_reading;
    begin
        $display("Testing reading.");
        s_axis_data_tdata = 8'b10101111;
        s_axis_data_tvalid = 1;
        i2c_start(7'b0000001, 1);
        repeat(7) @(negedge scl_o);
        s_axis_cmd_valid = 0;
        send_ack();
        send_byte(8'b01111111);
        s_axis_cmd_read = 1;
        s_axis_cmd_valid = 1;
        m_axis_data_tready = 1;
        @(negedge scl_o);
        if (sda_wire) $fatal(1, "Got NACK from master");
        $display("Received m_axis_data_tdata %d", m_axis_data_tdata);
        if (m_axis_data_tdata != 8'b01111111) $fatal(1, "We didn't get what we sent");
        #(CLK_PERIOD);
        stop_on_idle = 1;
        send_byte(8'd69);
        @(posedge m_axis_data_tvalid);
        if (m_axis_data_tdata != 8'd69) $fatal(1, "We didn't get what we sent");
        $display("Received m_axis_data_tdata %d", m_axis_data_tdata);
        s_axis_cmd_valid = 0;
        wait_for_ready();
    end
endtask

// Task to test i2c_single_reg writing
task test_i2c_single_reg_writing;
    begin
        $display("Testing i2c_single_reg writing");
        i2c_start(7'h70, 0);
        s_axis_data_tdata = 8'd55;
        s_axis_data_tvalid = 1;
        wait_for_ready();
        if (device_3.data_out_3 != 8'd55) $fatal(1, "We didn't get what we sent");
        $display("Received data %d", device_3.data_out_3);
    end
endtask

// Task to test i2c_single_reg reading
task test_i2c_single_reg_reading;
    begin
        $display("Testing i2c_single_reg reading");
        device_3.data_latch_3 = 1;
        device_3.data_in_3 = 8'd123;
        #(CLK_PERIOD);
        device_3.data_latch_3 = 0;
        i2c_start(7'h70, 1);
        @(posedge m_axis_data_tvalid);
        $display("Received m_axis_data_tdata %d", m_axis_data_tdata);
        if (m_axis_data_tdata != 8'd123) $fatal(1, "We didn't get what we sent");
    end
endtask

  initial begin
    $display("Starting I2C Master test");
    initialize_testbench;
    test_nack_handling();
    stop_on_idle = 1;
    test_writing();
    #1000;//stopping

    test_reading();
    wait_for_ready();
    test_i2c_single_reg_writing();
    test_i2c_single_reg_reading();
    #1000;
    $finish;
  end
endmodule
