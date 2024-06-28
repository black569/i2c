`timescale 1ns / 1ps

module i2c_phy_tb;

  // Parameters
  parameter CLK_PERIOD = 10; // 10ns clock period (100MHz clock)

  // Signals
  reg clk = 0;
  reg rst = 0;
  reg phy_start_bit = 0;
  reg phy_stop_bit = 0;
  reg phy_write_bit = 0;
  reg phy_read_bit = 0;
  reg phy_tx_data = 0;
  reg phy_release_bus = 0;
  wire scl_i = 1;
  wire scl_o;
  wire sda_i = 1;
  wire sda_o;
  wire sda_t;
  wire scl_t;
  wire phy_busy;
  wire bus_control_reg;
  wire phy_rx_data_reg;
  wire [4:0] phy_state_reg;

  // Clock generation
  always #(CLK_PERIOD/2) clk <= ~clk;

  // Instantiate the i2c_phy module
  i2c_phy uut (
    .clk(clk),
    .rst(rst),
    .phy_start_bit(phy_start_bit),
    .phy_stop_bit(phy_stop_bit),
    .phy_write_bit(phy_write_bit),
    .phy_read_bit(phy_read_bit),
    .phy_tx_data(phy_tx_data),
    .phy_release_bus(phy_release_bus),
    .scl_i(scl_i),
    .scl_o(scl_o),
    .sda_i(sda_i),
    .sda_o(sda_o),
    .sda_t(sda_t),
    .scl_t(scl_t),
    .phy_busy(phy_busy),
    .bus_control_reg(bus_control_reg),
    .phy_rx_data_reg(phy_rx_data_reg),
    .phy_state_reg(phy_state_reg),
    .prescale(17'd3)
  );

  // Simulate I2C bus behavior
  //always @(posedge clk) begin
  //  if (!scl_t) scl_i <= scl_o;
  //  if (!sda_t) sda_i <= sda_o;
  //end
  wire sda_pin;
  wire scl_pin;
  assign sda_pin = sda_o ? 1'bz : 1'b0;
  assign scl_pin = scl_o ? 1'bz : 1'b0;
  assign sda_i = sda_pin;
  assign scl_i = scl_pin;

  // Test procedure
  initial begin
    // Initialize inputs
    //$display("Starting");
    rst = 1;
    phy_start_bit = 0;
    phy_stop_bit = 0;
    phy_write_bit = 0;
    phy_read_bit = 0;
    phy_tx_data = 0;
    phy_release_bus = 0;

    // Wait for a few clock cycles and release reset
    //$display("before wait");
    #(CLK_PERIOD * 5);
    //$display("before reset");
    rst = 0;
    #(CLK_PERIOD * 5);

    //$display("before set");
    // Assert phy_start_bit
    phy_start_bit = 1;
    #(CLK_PERIOD);
    phy_start_bit=0;

    // Wait until phy_state becomes active (PHY_STATE_ACTIVE)
    //$display("wiating when phy becomes active");
    wait(phy_state_reg == 5'd1); // Assuming PHY_STATE_ACTIVE is 5'd1
    $display("PHY_STATE_ACTIVE reached at time %t", $time);

    phy_write_bit = 1;
    phy_tx_data = 0;
    $display("Sending a 00000001");
    #(CLK_PERIOD* 3*4);
    phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 0;
    #(CLK_PERIOD* 3*4);
 phy_write_bit = 1;
    phy_tx_data = 1;
    #(CLK_PERIOD* 3*4);
    phy_write_bit = 1;
    phy_tx_data = 1;//READ =1
    $display("send write value");
    #(CLK_PERIOD* 3*4);
    phy_write_bit = 0;
    phy_read_bit = 1;
    //do not Send acknoledgement
    #(CLK_PERIOD* 3*4);
    $display("Reading value %d",phy_rx_data_reg);
    //rx should be 1 since we have not send anything this is a NACK
    // End simulation
    $finish;
  end

  // Optional: Dump waveforms
  initial begin
    $dumpfile("i2c_phy_tb.vcd");
    $dumpvars(0, i2c_phy_tb);
  end

endmodule
