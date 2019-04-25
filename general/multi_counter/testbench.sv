// Code your testbench here
// or browse Examples
`include "multi_counter_pkg.vh"

module tb;
    
  parameter int CNTRS_N = 256;
  parameter int CNTRS_W = 32;
  parameter int CNTRS_ID_W = $clog2(CNTRS_N);
  
  bit clk = 0 ;
  always #5 clk = ~clk ;
  
  logic   rst;

  // ------------------------------------------------------------------------ //
  // Command Interface                                                        //
  // ------------------------------------------------------------------------ //
  logic                              cntr_pass;
  logic [CNTRS_ID_W-1:0]             cntr_id;
  multi_counter_pkg::op_t     cntr_op;
  logic [CNTRS_W-1:0]                cntr_dat;

  // ------------------------------------------------------------------------ //
  // Status Interface                                                         //
  // ------------------------------------------------------------------------ //
  logic                             status_pass_r;
  logic                             status_qry_r;
  logic [CNTRS_ID_W-1:0]            status_id_r;
  logic [CNTRS_W-1:0]               status_dat_r;
  
  class op_transaction ;
    rand bit [CNTRS_ID_W-1:0] id ;
    rand multi_counter_pkg::op_t op ;
    rand bit [CNTRS_W-1:0] data ;
  endclass
  
  multi_counter dut (.*);
  
  initial begin
    int i;
    
    $dumpfile("dump.vcd");
    $dumpvars;
 
    initialize_inputs();
    reset();
    
    for (i=0;i<10;i=i+1) begin
      init_counter(i);
    end
    
    for (i=0;i<10;i=i+1) begin
      inc_counter(i);
    end
    
    for (i=0;i<10;i=i+1) begin
      read_counter(i);
    end
       
    for (i=0;i<10;i=i+1) begin
      dec_counter(i);
    end
    
    for (i=0;i<10;i=i+1) begin
      read_counter(i);
    end
    
    quiet();
    
    for (i=0;i<10;i=i+1) begin
      init_counter(i);
      inc_counter(i);
      read_counter(i);
      dec_counter(i);
      read_counter(i);
    end
    
    quiet();
    
    $finish;
    
  end
             
  task tick(int n=1);
    repeat(n) @(posedge clk);
  endtask
  
  task rand_delay();
    tick( $urandom_range(10,20));
  endtask
  
  task initialize_inputs();
    rst = 0;
    cntr_pass = 0;
    cntr_id = 0;
    cntr_dat = 0;
    cntr_op = multi_counter_pkg::OP_NOP;
  endtask
  
  task quiet();
    tick();
    cntr_pass <= 0;
  endtask
  
  task init_counter(int _id, int data = -1 ,bit b2b=1 );
    op_transaction cmd  ;    
    cmd = new ;
    
    tick();

    cmd.randomize() with { id == _id ; op == multi_counter_pkg::OP_INIT; } ;
    
    if (data == -1)
      cmd.data = _id+8'h10;
    else
      cmd.data = data;
    
    cntr_pass <= 1;
    cntr_id = cmd.id;
    cntr_op = cmd.op;
    cntr_dat = cmd.data;
    
    if (~b2b) begin
      tick();
      cntr_pass <= 0;
    end
    
  endtask
  
  task inc_counter(int _id,bit b2b=1);
    op_transaction cmd  ;    
    cmd = new ;
    
    tick();

    cmd.randomize() with { id == _id ; op == multi_counter_pkg::OP_INCR; } ;
    
    cntr_pass <= 1;
    cntr_id = cmd.id;
    cntr_op = cmd.op;
    if (~b2b) begin
      tick();
      cntr_pass <= 0;
    end
  endtask
  
  
  task dec_counter(int _id,bit b2b=1);
    op_transaction cmd  ;    
    cmd = new ;
    
    tick();

    cmd.randomize() with { id == _id ; op == multi_counter_pkg::OP_DECR; } ;
    cntr_pass <= 1;
    cntr_id = cmd.id;
    cntr_op = cmd.op;
    if (~b2b) begin
      tick();
      cntr_pass <= 0;
    end
  endtask
  
  task read_counter(int _id, bit b2b=1);
    op_transaction cmd  ;    
    cmd = new ;
    
    tick();

    cmd.randomize() with { id == _id ; op == multi_counter_pkg::OP_QRY; } ;
    cntr_pass <= 1;
    cntr_id = cmd.id;
    cntr_op = cmd.op;
    if (~b2b) begin
      tick();
      cntr_pass <= 0;
    end
  endtask
  
  task reset();
    tick();
    rst <= 1;
    tick(5);
    rst <= 0;
  endtask
  
endmodule



