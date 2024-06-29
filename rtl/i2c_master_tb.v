// Language: Verilog 2001

`timescale 1ns / 1ps

/*
 * Testbench for i2c_master
 */
module test_i2c_master;

// Parameters

  parameter CLK_PERIOD = 10;  // 10ns clock period (100MHz clock)
// Inputs
reg clk = 0;
reg rst = 0;
reg [7:0] current_test = 0;

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

// Outputs
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

// Additional wires for modeling pull-up resistors and open-drain outputs
wire scl_wire;
wire sda_wire;
  reg sda2=1;//dummy registers
  reg scl2=1;//dummy registers
 

// Model pull-up resistors with weak pull-ups
pullup(scl_wire);
pullup(sda_wire);

// Model open-drain outputs
assign scl_wire = (scl_o & scl2) ? 1'bz : 1'b0;
assign sda_wire = (sda_o & sda2) ? 1'bz : 1'b0;

  always #(CLK_PERIOD / 2) clk <= ~clk;
// Sample the bus with non-blocking assignments to avoid race conditions
always @(posedge clk or posedge rst) begin
    if (rst) begin
        scl_i <= 1'b1;
        sda_i <= 1'b1;
    end else begin
        scl_i <= scl_wire;
        sda_i <= sda_wire;
        
        // Assert that sda_i_reg_tb is not X
        if (sda_i === 1'bx) $fatal(1, "sda_i_reg_tb is X at time %t", $time);
    end
end


initial begin
    // dump file
    $dumpfile("i2c_master_tb.vcd");
    $dumpvars(0, test_i2c_master);

end

i2c_master
UUT (
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
  @(negedge scl_wire);//start condition done! 
      
      s_axis_cmd_start = 0;
      s_axis_cmd_read = 0;
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


initial begin
  $display("Starting I2C Master test");
  initialize_testbench;

  // Set address to 00000001, command to write
  i2c_start(7'b0000001, 0);
  s_axis_data_tdata = 8'b10100101;
  s_axis_data_tvalid = 1;

  // Wait for NACK
  @(posedge missed_ack);
  $display("NACK expected!");
  
  wait_for_ready();

  // Second write attempt
  i2c_start(7'b0000001, 0);
  s_axis_data_tdata = 8'b10101111;
  s_axis_data_tvalid = 1;
  
  // Hardcoding the acknowledgement
  repeat(7) @(negedge scl_o);

  // Send ACK
  send_ack();
  @(negedge scl_o); // let 1 pass through
  send_byte(8'b01111111);
  stop_on_idle = 1;

  wait_for_ready();
  #1000;

  // Start is implied
  s_axis_data_tdata = 8'b10101111;
  s_axis_data_tvalid = 1;
  i2c_start(7'b0000001, 1);

  $display("Now doing a read");

  // Hardcoding the acknowledgement
  repeat(7) @(negedge scl_o);

  // Send ACK
  s_axis_cmd_valid = 0;
  send_ack();
  send_byte(8'b01111111);

  // Continue with next read (does not require i2c_start condition)
  s_axis_cmd_read = 1;
  s_axis_cmd_valid = 1;
  m_axis_data_tready = 1; // prepare readiness

  @(negedge scl_o);
  if (sda_wire) begin
    $fatal(1, "Got NACK from master");
  end

  $display("Received m_axis_data_tdata %d", m_axis_data_tdata);
  #(CLK_PERIOD);
  stop_on_idle = 1;
  @(posedge m_axis_data_tvalid);
  $display("Received m_axis_data_tdata %d", m_axis_data_tdata);
  $display("Received readiness %d", s_axis_cmd_ready);
  s_axis_cmd_valid = 0; // now stop

  wait_for_ready();
  #1000;

  $display("Simulation ended due to timeout");
  $finish;
end
endmodule


