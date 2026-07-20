module selftest_bram #(
  parameter DEPTH=2048, INIT="stim_X.hex"
)(
  input  wire        clk, rstn,
  input  wire        selftest_en,
  input  wire        start,
  input  wire        advance,          
  input  wire [15:0] radar_re, radar_im,
  input  wire        radar_valid,
  output wire [15:0] out_re, out_im,
  output wire        out_valid,       
  output reg         frame_done
);
  reg [15:0] mem [0:DEPTH-1];
  initial $readmemh(INIT, mem);
  reg [11:0] addr; reg run;
  always @(posedge clk) begin
    if (!rstn) begin addr<=0; run<=0; frame_done<=0; end
    else begin
      frame_done <= 0;
      if (start & selftest_en) begin run<=1; addr<=0; end
      else if (run & advance) begin
        if (addr == DEPTH-2) begin run<=0; frame_done<=1; end
        addr <= addr + 12'd2;
      end
    end
  end
  assign out_re    = selftest_en ? mem[addr]   : radar_re;
  assign out_im    = selftest_en ? mem[addr+1] : radar_im;
  assign out_valid = selftest_en ? run         : radar_valid;
endmodule