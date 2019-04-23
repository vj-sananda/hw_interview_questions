// Code your testbench here
// or browse Examples
module tb;

  bit clk = 0 ;
  always #5 clk = ~clk ;
  
  logic a, b, fail;
  
  gates_from_MUX2X1 dut(.*);
  
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
 
    a=0;
    b=0;
    
    for(int i=0; i<50;i++) begin
      @(posedge clk);
      a = $urandom();
      b = $urandom();
    end
    
    $finish;
    
  end
  
endmodule

