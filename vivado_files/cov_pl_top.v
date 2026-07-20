// cov_pl_top.v - integrates covariance_core_fixpt (MATLAB HDL Coder
// output, exact port list from the DUT report) with the existing
// selftest_bram.v and axi_regs.v (NC=10).
//
// FORMAT NOTE: samp_re/samp_im are sfix16_En13 (Q2.13, 3 int + 13 frac
// bits) per the DUT report. selftest_bram.v's stim_X.hex is dumped in
// Q1.15 by dump_hex() in run_golden.m. Converting the RAW CODE from
// Q1.15 to Q2.13 for the SAME physical value is an arithmetic >>2 (both
// are 16-bit signed; Q2.13 has 2 fewer fraction bits, so shifting the
// integer code right by 2 preserves the represented value exactly,
// with 2 bits of now-unneeded precision dropped). Done below.
//
// IMPORTANT - SEPARATE ISSUE, not fixed here: dump_hex() normalizes
// each frame by that frame's own max(abs(X)) before scaling by 2^15.
// That per-frame-adaptive factor is data-dependent, so even after the
// >>2 format fix the self-test values will NOT numerically match
// golden.csv/frames.csv (which were computed from true, unnormalized
// X). For the self-test path to be a real oracle, re-dump stim_X.hex
// with a FIXED scale matching Q2.13 directly:
//   q = int16(round(real_and_imag_of_X * 8192));   % 2^13, no per-frame max
// This file assumes that regeneration has been done; the >>2 here is
// necessary but not sufficient without it.
module cov_pl_top (
    input  wire        clk,
    input  wire        rst,                 // active-high system reset
    input  wire [7:0]  s_axi_awaddr, input wire s_axi_awvalid, output wire s_axi_awready,
    input  wire [31:0] s_axi_wdata,  input wire s_axi_wvalid,  output wire s_axi_wready,
    output wire [1:0]  s_axi_bresp,  output wire s_axi_bvalid, input wire s_axi_bready,
    input  wire [7:0]  s_axi_araddr, input wire s_axi_arvalid, output wire s_axi_arready,
    output wire [31:0] s_axi_rdata,  output wire [1:0] s_axi_rresp,
    output wire        s_axi_rvalid, input wire s_axi_rready
);
    wire rstn = ~rst;
    wire selftest_en, ax_start, soft_rst; wire signed [15:0] cal_phase;
    wire [15:0] bram_re, bram_im; wire bram_valid, bram_fdone;
    wire cov_valid, ce_out;
    wire signed [15:0] Rtri_re [0:9];
    wire signed [15:0] Rtri_im [0:9];
    reg [3:0] wr_idx; reg wr_active;
    reg coeff_we_r; reg [3:0] coeff_idx_r; reg [31:0] coeff_data_r;

    selftest_bram #(.DEPTH(2048), .INIT("stim_X.hex")) u_bram (
        .clk(clk), .rstn(rstn & ~soft_rst), .selftest_en(selftest_en), .start(ax_start),
        .radar_re(16'd0), .radar_im(16'd0), .radar_valid(1'b0),
        .out_re(bram_re), .out_im(bram_im), .out_valid(bram_valid), .frame_done(bram_fdone));

    wire signed [15:0] samp_re = $signed(bram_re) >>> 2;   // Q1.15 -> Q2.13
    wire signed [15:0] samp_im = $signed(bram_im) >>> 2;   // (see header note)

    reg armed;                                             // pulse frame_start on 1st sample
    always @(posedge clk) begin
        if (!rstn) armed <= 1'b0;
        else if (ax_start) armed <= 1'b1;
        else if (bram_valid) armed <= 1'b0;                // one-shot: clears once consumed
    end
    wire frame_start_r = armed && bram_valid;               // combinational: same cycle as
                                                              // the true first valid sample

    covariance_core_fixpt u_cov (
        .clk(clk), .reset(rst), .clk_enable(1'b1),
        .samp_re(samp_re), .samp_im(samp_im),
        .samp_valid(bram_valid), .frame_start(frame_start_r),
        .ce_out(ce_out),
        .Rtri_re_0(Rtri_re[0]), .Rtri_re_1(Rtri_re[1]), .Rtri_re_2(Rtri_re[2]),
        .Rtri_re_3(Rtri_re[3]), .Rtri_re_4(Rtri_re[4]), .Rtri_re_5(Rtri_re[5]),
        .Rtri_re_6(Rtri_re[6]), .Rtri_re_7(Rtri_re[7]), .Rtri_re_8(Rtri_re[8]),
        .Rtri_re_9(Rtri_re[9]),
        .Rtri_im_0(Rtri_im[0]), .Rtri_im_1(Rtri_im[1]), .Rtri_im_2(Rtri_im[2]),
        .Rtri_im_3(Rtri_im[3]), .Rtri_im_4(Rtri_im[4]), .Rtri_im_5(Rtri_im[5]),
        .Rtri_im_6(Rtri_im[6]), .Rtri_im_7(Rtri_im[7]), .Rtri_im_8(Rtri_im[8]),
        .Rtri_im_9(Rtri_im[9]),
        .valid(cov_valid));

    // 10-cycle write burst into axi_regs, triggered once per completed frame
    always @(posedge clk) begin
        if (!rstn) begin wr_active<=1'b0; wr_idx<=4'd0; coeff_we_r<=1'b0; end
        else begin
            coeff_we_r <= 1'b0;
            if (cov_valid && ce_out && !wr_active) begin
                wr_active <= 1'b1; wr_idx <= 4'd0;
            end else if (wr_active) begin
                coeff_we_r   <= 1'b1;
                coeff_idx_r  <= wr_idx;
                coeff_data_r <= {Rtri_im[wr_idx], Rtri_re[wr_idx]};
                if (wr_idx == 4'd9) wr_active <= 1'b0;
                wr_idx <= wr_idx + 4'd1;
            end
        end
    end

    axi_regs #(.NC(10)) u_regs (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .selftest_en(selftest_en), .start_pulse(ax_start), .soft_rst(soft_rst),
        .cal_phase(cal_phase), .core_done(cov_valid && ce_out), .core_busy(wr_active),
        .coeff_we(coeff_we_r), .coeff_idx(coeff_idx_r), .coeff_data(coeff_data_r));
endmodule