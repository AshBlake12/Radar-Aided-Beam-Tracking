
// REGISTER MAP 
//  0x00 CTRL    RW  bit0 SELFTEST_EN (1=BRAM stimulus, 0=radar ingest)
//                   bit1 START (pulse: run one EVD frame)
//                   bit2 SOFT_RST
//  0x04 STATUS  RO  bit0 DONE  bit1 BUSY
//  0x08 CAL     RW  signed Q3.13 phase offset (rad), added to CORDIC input
//  0x0C FRMCNT  RO  frames processed since reset
//  0x10-0x34    RO  RTRI[0..9]: R upper triangle row-major (R11 R12 R13
//                   R14 R22 R23 R24 R33 R34 R44), {im[31:16],re[15:0]} Q1.15

module axi_regs #(parameter NC=10)(
  input  wire        s_axi_aclk, s_axi_aresetn,
  // AXI4-Lite slave
  input  wire [7:0]  s_axi_awaddr, input wire s_axi_awvalid, output reg s_axi_awready,
  input  wire [31:0] s_axi_wdata,  input wire s_axi_wvalid,  output reg s_axi_wready,
  output reg  [1:0]  s_axi_bresp,  output reg s_axi_bvalid,  input wire s_axi_bready,
  input  wire [7:0]  s_axi_araddr, input wire s_axi_arvalid, output reg s_axi_arready,
  output reg  [31:0] s_axi_rdata,  output reg [1:0] s_axi_rresp,
  output reg         s_axi_rvalid, input wire s_axi_rready,
  // fabric side
  output wire        bpm_mode,
    output wire [15:0] samples_per_chirp,
    output wire [2:0]  sel0, sel1, sel2, sel3,
    output wire [7:0]  bin_sel,
    output wire signed [15:0] cal_re, cal_im,
    output wire [15:0] snapshots_k,
    output wire        stream_sync_req,
    
  output wire        selftest_en, output wire start_pulse, output wire soft_rst,
  output wire signed [15:0] cal_phase,          // Q3.13
  input  wire        core_done, core_busy,
  input  wire        coeff_we,                  // strobe: latch coeff_idx
  input  wire [3:0]  coeff_idx,
  input  wire [31:0] coeff_data                 // {im,re}
);
  reg [31:0] ctrl, cal; reg [31:0] frmcnt; reg [31:0] coeff [0:NC-1];
  reg [7:0] awaddr_l; reg start_r; reg aw_ok, w_ok; reg [31:0] wdata_l;
  assign selftest_en = ctrl[0];
  assign start_pulse = start_r;
  assign soft_rst    = ctrl[2];
  assign cal_phase   = cal[15:0];

  integer i;
  // w channel
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      ctrl<=0; cal<=0; s_axi_awready<=0; s_axi_wready<=0; s_axi_bvalid<=0;
      start_r<=0; aw_ok<=0; w_ok<=0;
    end else begin
      start_r <= 0;                              // START self-clears
      
     
      s_axi_awready <= s_axi_awvalid & ~aw_ok & ~s_axi_bvalid;
      s_axi_wready  <= s_axi_wvalid  & ~w_ok  & ~s_axi_bvalid;
      if (s_axi_awvalid & s_axi_awready) begin awaddr_l <= s_axi_awaddr; aw_ok <= 1; end
      if (s_axi_wvalid  & s_axi_wready)  begin wdata_l  <= s_axi_wdata;  w_ok  <= 1; end
      if (aw_ok & w_ok & ~s_axi_bvalid) begin
        case (awaddr_l)
          8'h00: begin ctrl <= wdata_l; start_r <= wdata_l[1]; end
          8'h08: cal <= wdata_l;
        endcase
        s_axi_bvalid <= 1; s_axi_bresp <= 0; aw_ok <= 0; w_ok <= 0;
      end else if (s_axi_bvalid & s_axi_bready) s_axi_bvalid <= 0;
    end
  end
  
  //frame count loop
    always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn | soft_rst) begin
      frmcnt<=0; for(i=0;i<NC;i=i+1) coeff[i]<=0;
    end else begin
      if (coeff_we) coeff[coeff_idx] <= coeff_data;
      if (core_done) frmcnt <= frmcnt+1;
    end
  end
  
  //r channel
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin s_axi_arready<=0; s_axi_rvalid<=0; end
    else begin
      s_axi_arready <= s_axi_arvalid & ~s_axi_rvalid;
      if (s_axi_arvalid & s_axi_arready) begin
        s_axi_rvalid <= 1; s_axi_rresp <= 0;
        case (s_axi_araddr)
          8'h00: s_axi_rdata <= ctrl;
          8'h04: s_axi_rdata <= {30'b0, core_busy, core_done};
          8'h08: s_axi_rdata <= cal;
          8'h0C: s_axi_rdata <= frmcnt;
          default: s_axi_rdata <= (s_axi_araddr>=8'h10 && s_axi_araddr<8'h10+4*NC)
                                  ? coeff[(s_axi_araddr-8'h10)>>2] : 32'hDEADBEEF;
        endcase
      end else if (s_axi_rvalid & s_axi_rready) s_axi_rvalid <= 0;
    end
  end
endmodule
