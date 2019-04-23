// Code your testbench here
// or browse Examples
module tb;
  
  parameter int W = 32;
  
  bit clk = 0 ;
  always #5 clk = ~clk ;
  
  logic issue,clear,retire,rst;
  logic   [W-1:0]                  issue_cnt_r;
  logic   [W-1:0]                  aggregate_cnt_r;
  
  latency dut(.*);
  
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
 
    reset();
    init();
    clear_op();
    
    for(int i=0; i<50;i++) begin
      issue_op();
      rand_delay();
      retire_op();
    end
    
    $finish;
    
  end
  
  task tick(int n=1);
    repeat(n) @(posedge clk);
  endtask
  
  task rand_delay();
    tick( $urandom_range(10,100));
  endtask
  
  task clear_op();
    tick();
    clear <= 1;
    tick();
    clear <= 0;
  endtask
  
  task init();
    tick();
    clear <= 0;
    issue <= 0;
    retire <= 0;
  endtask
  
  task reset();
    tick();
    rst <= 1;
    tick(5);
    rst <= 0;
  endtask
  
  task issue_op();
    tick();
    issue <= 1;
    tick();
    issue <= 0;
  endtask
  
  task retire_op();
    tick();
    retire <= 1;
    tick();
    retire <= 0;
  endtask
  
endmodule


