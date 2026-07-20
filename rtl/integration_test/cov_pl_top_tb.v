`timescale 1ns/1ps

module cov_pl_top_tb;
  reg clk = 0, rst = 1;
  always #5 clk = ~clk;

  reg  [7:0]  awaddr=0;  reg awvalid=0;  wire awready;
  reg  [31:0] wdata=0;   reg wvalid=0;   wire wready;
  wire [1:0]  bresp;     wire bvalid;    reg bready=1;
  reg  [7:0]  araddr=0;  reg arvalid=0;  wire arready;
  wire [31:0] rdata;     wire [1:0] rresp; wire rvalid; reg rready=1;

  cov_pl_top_live dut (
    .clk(clk), .rst(rst),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata),  .s_axi_wvalid(wvalid),  .s_axi_wready(wready),
    .s_axi_bresp(bresp),  .s_axi_bvalid(bvalid),  .s_axi_bready(bready),
    .s_axi_araddr(araddr),.s_axi_arvalid(arvalid),.s_axi_arready(arready),
    .s_axi_rdata(rdata),  .s_axi_rresp(rresp),
    .s_axi_rvalid(rvalid),.s_axi_rready(rready));

  task axwrite(input [7:0] a, input [31:0] d);
    begin
      @(posedge clk); awaddr<=a; awvalid<=1; wdata<=d; wvalid<=1;
      @(posedge clk);
      while (!bvalid) @(posedge clk);
      awvalid<=0; wvalid<=0;
      @(posedge clk);
    end
  endtask

  task axread(input [7:0] a, output [31:0] d);
    begin
      @(posedge clk); araddr<=a; arvalid<=1;
      @(posedge clk);
      while (!rvalid) @(posedge clk);
      d = rdata;
      arvalid<=0;
      @(posedge clk);
    end
  endtask

  reg [15:0] refmem [0:2047];
  real xr[0:3], xi[0:3];
  real Rre[0:3][0:3], Rim[0:3][0:3];
  real exp_re[0:9], exp_im[0:9];
  integer i, j, k, s, idx;
  real vr, vi;

  initial begin
    $readmemh("stim_X.hex", refmem);
    for (i=0;i<4;i=i+1) for (j=0;j<4;j=j+1) begin Rre[i][j]=0.0; Rim[i][j]=0.0; end
    for (s=0; s<256; s=s+1) begin
      for (k=0; k<4; k=k+1) begin
        vr = ($signed(refmem[(s*4+k)*2])   >>> 2) / 8192.0;
        vi = ($signed(refmem[(s*4+k)*2+1]) >>> 2) / 8192.0;
        xr[k]=vr; xi[k]=vi;
      end
      for (i=0;i<4;i=i+1) for (j=0;j<4;j=j+1) begin
        Rre[i][j] = Rre[i][j] + (xr[i]*xr[j] + xi[i]*xi[j]);
        Rim[i][j] = Rim[i][j] + (xi[i]*xr[j] - xr[i]*xi[j]);
      end
    end
    for (i=0;i<4;i=i+1) for (j=0;j<4;j=j+1) begin
      Rre[i][j] = Rre[i][j] / 256.0; Rim[i][j] = Rim[i][j] / 256.0;
    end
    idx = 0;
    for (i=0;i<4;i=i+1) for (j=i;j<4;j=j+1) begin
      exp_re[idx]=Rre[i][j]; exp_im[idx]=Rim[i][j]; idx=idx+1;
    end
    $display("reference model computed from stim_X.hex, %0d upper-tri entries", idx);
  end

  // ---- debug counters: localize exactly how far data gets ----
  integer n_valid=0, n_fs=0, n_covvalid=0, n_ceout=0, n_we=0;
  always @(posedge clk) begin
    if (dut.u_bram.out_valid) n_valid=n_valid+1;
    if (dut.frame_start_r)    n_fs=n_fs+1;
    if (dut.cov_valid)        n_covvalid=n_covvalid+1;
    if (dut.ce_out)           n_ceout=n_ceout+1;
    if (dut.coeff_we_r)       n_we=n_we+1;
  end

  reg [31:0] rd; integer errors; real got_re, got_im, err_re, err_im;
  localparam real TOL = 0.02;
  initial begin
    errors = 0;
    repeat(4) @(posedge clk); rst<=0; repeat(2) @(posedge clk);
    axwrite(8'h00, 32'h1);
    $display("after EN write t=%0t", $time);
    axwrite(8'h00, 32'h3);
    $display("after START write t=%0t", $time);
    repeat(200000) @(posedge clk);
    $display("after wait t=%0t | n_valid=%0d n_fs=%0d n_covvalid=%0d n_ceout=%0d n_we=%0d",
              $time, n_valid, n_fs, n_covvalid, n_ceout, n_we);
    for (i=0;i<10;i=i+1) begin
      axread(8'h10+4*i, rd);
      got_re = ($signed(rd[15:0]))  / 16384.0;
      got_im = ($signed(rd[31:16])) / 16384.0;
      err_re = got_re - exp_re[i]; if (err_re<0) err_re=-err_re;
      err_im = got_im - exp_im[i]; if (err_im<0) err_im=-err_im;
      if (err_re > TOL || err_im > TOL) begin
        $display("FAIL reg %0d: got=(%f,%f) expect=(%f,%f)", i, got_re, got_im, exp_re[i], exp_im[i]);
        errors = errors+1;
      end else begin
        $display("ok   reg %0d: got=(%f,%f) expect=(%f,%f)", i, got_re, got_im, exp_re[i], exp_im[i]);
      end
    end
    if (errors==0) $display("COV_PL_TOP_TB PASS");
    else $display("COV_PL_TOP_TB FAIL (%0d/10 registers wrong)", errors);
    $finish;
  end
  initial begin #3000000 $display("TIMEOUT"); $finish; end
endmodule