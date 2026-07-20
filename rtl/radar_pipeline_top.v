// -----------------------------------------------------------------------
// radar_pipeline_top.v
//
// THE parent that makes the front-end real. Before this file, range_fft_top,
// rdif_phy, rdif_cdc_sync, and tdm_bpm_decode were orphan modules --
// compiled, but instantiated by nothing, sitting at the same level as the
// testbench. This module instantiates all of them and wires the chain:
//
//   RDIF pins
//     -> rdif_cdc_sync   (reset + stream_sync into rdif_clk domain)
//     -> rdif_phy         (DDR deser -> {rx_idx, adc_sample, adc_valid})
//     -> tdm_bpm_decode   (TX1/TX2 separation -> 6 virtual channels)
//     -> vch_select4      (pick 4: {V0,V1,V2,V4} default ULA)
//     -> range_fft_top    (256-pt FFT/ch, Hann, bin select, ch3 cal)
//     -> selftest_bram.radar_* port  (live-data injection point)
//     -> covariance_core / axi_regs   (UNCHANGED existing path)
//
// The existing cov_pl_top is preserved verbatim as the inner block; this
// wraps it and drives its radar-ingest port instead of tying it to zero.
// selftest_en (AXI CTRL bit0) still selects: 1 = golden BRAM stimulus
// (bit-identical to today's verified path), 0 = live radar via this chain.
// So every existing test keeps passing; the new path is purely additive.
//
// NOTE ON range_fft_top OUTPUT vs selftest_bram INPUT:
// range_fft_top emits paced samples with cov_clk_enable/cov_frame_start
// intended to drive the covariance core's clk_enable directly. But the
// existing cov_pl_top drives clk_enable=1'b1 and paces via ce_out+advance
// on selftest_bram instead. To avoid forking the verified pacing logic,
// this integration feeds the FFT's complex output into the radar_re/im
// port of selftest_bram (which already muxes on selftest_en) and lets the
// EXISTING ce_out/advance mechanism pull samples at the base rate. The
// FFT's own cov_clk_enable/cov_frame_start/snapshots are therefore left
// for the standalone-FFT use case and NOT used here -- see the
// radar_valid handshake below. This keeps ONE pacing authority in the
// design (the covariance core's ce_out), which is what the timing
// constraints were closed against.
// -----------------------------------------------------------------------
module radar_pipeline_top #(
    parameter ADC_W = 12,
    parameter FFT_N = 256
)(
    input  wire        clk,            // clk_fpga_0, 50 MHz
    input  wire        rst,

    // ---- RDIF pins from the IWRL6432 --------------------------------
    input  wire        RDIF_CLK,
    input  wire [3:0]  RDIF_D,
    input  wire        RDIF_FRM_CLK,

    // ---- radar config (from AXI, see note in axi_regs section) ------
//    input  wire        bpm_mode,          // 0=TDM, 1=BPM
//    input  wire [15:0] samples_per_chirp,
//    input  wire [2:0]  sel0, sel1, sel2, sel3,   // vch_select4 indices
//    input  wire [7:0]  bin_sel,           // range gate
//    input  wire signed [15:0] cal_re, cal_im,    // ch3 calibration Q1.15
//    input  wire [15:0] snapshots_k,
//    input  wire        stream_sync_req,   // AXI self-clearing bit, clk domain

    // ---- AXI4-Lite passthrough to the inner cov_pl_top --------------
    input  wire [7:0]  s_axi_awaddr, input wire s_axi_awvalid, output wire s_axi_awready,
    input  wire [31:0] s_axi_wdata,  input wire s_axi_wvalid,  output wire s_axi_wready,
    output wire [1:0]  s_axi_bresp,  output wire s_axi_bvalid, input wire s_axi_bready,
    input  wire [7:0]  s_axi_araddr, input wire s_axi_arvalid, output wire s_axi_arready,
    output wire [31:0] s_axi_rdata,  output wire [1:0] s_axi_rresp,
    output wire        s_axi_rvalid, input wire s_axi_rready
);

    // =================================================================
    // 1. RDIF PHY (owns the single BUFG) + CDC clocked from its output.
    //    rdif_phy instantiates the ONE BUFG on RDIF_CLK and exports the
    //    buffered net as rdif_clk_out; the CDC's destination flops use
    //    that same net. No second BUFG here -> no multi-driver error.
    // =================================================================
    wire signed [ADC_W-1:0] adc_sample;
    wire [1:0]              rx_idx;
    wire                    adc_valid;
    wire                    rdif_clk_bufg;
    wire                    rst_rdif, stream_sync_rdif;
    

    rdif_phy u_rdif (
        .RDIF_CLK(RDIF_CLK), .RDIF_D(RDIF_D), .RDIF_FRM_CLK(RDIF_FRM_CLK),
        .clk_fpga_0(clk), .rst_fpga_0(rst_rdif), .stream_sync(stream_sync_rdif),
        .adc_sample(adc_sample), .rx_idx(rx_idx), .adc_valid(adc_valid),
        .rdif_clk_out(rdif_clk_bufg));

    rdif_cdc_sync u_cdc (
        .clk_fpga_0(clk), .rdif_clk(rdif_clk_bufg),
        .rst_fpga_0(rst), .stream_sync_req(stream_sync_req),
        .rst_rdif(rst_rdif), .stream_sync_rdif(stream_sync_rdif));

    // =================================================================
    // 3. TDM/BPM decode -> 6 virtual channels
    // =================================================================
    wire signed [ADC_W:0] t1r0,t1r1,t1r2,t2r0,t2r1,t2r2;
    wire t1r0v,t1r1v,t1r2v,t2r0v,t2r1v,t2r2v;

    tdm_bpm_decode #(.ADC_W(ADC_W)) u_decode (
        .clk(clk), .rst(rst),
        .bpm_mode(bpm_mode), .samples_per_chirp(samples_per_chirp),
        .stream_sync(stream_sync_req),
        .adc_sample(adc_sample), .rx_idx(rx_idx), .adc_valid(adc_valid),
        .tx1_rx0(t1r0), .tx1_rx0_valid(t1r0v),
        .tx1_rx1(t1r1), .tx1_rx1_valid(t1r1v),
        .tx1_rx2(t1r2), .tx1_rx2_valid(t1r2v),
        .tx2_rx0(t2r0), .tx2_rx0_valid(t2r0v),
        .tx2_rx1(t2r1), .tx2_rx1_valid(t2r1v),
        .tx2_rx2(t2r2), .tx2_rx2_valid(t2r2v));

    // =================================================================
    // 4. Select 4 of 6 virtual channels
    // =================================================================
    wire signed [ADC_W:0] ch0,ch1,ch2,ch3;
    wire ch0v,ch1v,ch2v,ch3v;

    vch_select4 #(.ADC_W(ADC_W)) u_sel (
        .clk(clk), .sel0(sel0), .sel1(sel1), .sel2(sel2), .sel3(sel3),
        .tx1_rx0(t1r0), .tx1_rx1(t1r1), .tx1_rx2(t1r2),
        .tx2_rx0(t2r0), .tx2_rx1(t2r1), .tx2_rx2(t2r2),
        .tx1_rx0_valid(t1r0v), .tx1_rx1_valid(t1r1v), .tx1_rx2_valid(t1r2v),
        .tx2_rx0_valid(t2r0v), .tx2_rx1_valid(t2r1v), .tx2_rx2_valid(t2r2v),
        .ch0(ch0), .ch1(ch1), .ch2(ch2), .ch3(ch3),
        .ch0_v(ch0v), .ch1_v(ch1v), .ch2_v(ch2v), .ch3_v(ch3v));

    // =================================================================
    // 5. Range FFT: 4 channels -> complex range-bin sample stream
    // =================================================================
    // xfft_0 AXI-Stream wiring (instantiate the IP alongside this module)
    wire [31:0] fft_s_tdata;  wire fft_s_tvalid, fft_s_tready, fft_s_tlast;
    wire [23:0] fft_cfg_tdata; wire fft_cfg_tvalid, fft_cfg_tready;
    wire [31:0] fft_m_tdata;  wire [15:0] fft_m_tuser;
    wire        fft_m_tvalid, fft_m_tlast, fft_m_tready, fft_aresetn;

    wire signed [15:0] fft_samp_re, fft_samp_im;
    wire               fft_ce, fft_fs;

    range_fft_top #(.N(FFT_N), .LOGN(8), .IW(ADC_W+1), .OW(16), .CE_DIV(80)) u_fft (
        .clk(clk), .rst(rst),
        .bin_sel(bin_sel), .cal_re(cal_re), .cal_im(cal_im),
        .snapshots_k(snapshots_k),
        .ch0(ch0), .ch1(ch1), .ch2(ch2), .ch3(ch3),
        .ch0_v(ch0v), .ch1_v(ch1v), .ch2_v(ch2v), .ch3_v(ch3v),
        .cov_samp_re(fft_samp_re), .cov_samp_im(fft_samp_im),
        .cov_clk_enable(fft_ce), .cov_frame_start(fft_fs),
        .s_axis_data_tdata(fft_s_tdata), .s_axis_data_tvalid(fft_s_tvalid),
        .s_axis_data_tready(fft_s_tready), .s_axis_data_tlast(fft_s_tlast),
        .s_axis_config_tdata(fft_cfg_tdata), .s_axis_config_tvalid(fft_cfg_tvalid),
        .s_axis_config_tready(fft_cfg_tready),
        .m_axis_data_tdata(fft_m_tdata), .m_axis_data_tuser(fft_m_tuser),
        .m_axis_data_tvalid(fft_m_tvalid), .m_axis_data_tlast(fft_m_tlast),
        .m_axis_data_tready(fft_m_tready), .fft_aresetn(fft_aresetn));

    // ---- Xilinx FFT IP instance (generate xfft_0 per range_fft_top hdr)
    xfft_0 u_xfft (
        .aclk(clk), .aresetn(fft_aresetn),
        .s_axis_config_tdata(fft_cfg_tdata),
        .s_axis_config_tvalid(fft_cfg_tvalid),
        .s_axis_config_tready(fft_cfg_tready),
        .s_axis_data_tdata(fft_s_tdata),
        .s_axis_data_tvalid(fft_s_tvalid),
        .s_axis_data_tready(fft_s_tready),
        .s_axis_data_tlast(fft_s_tlast),
        .m_axis_data_tdata(fft_m_tdata),
        .m_axis_data_tuser(fft_m_tuser),
        .m_axis_data_tvalid(fft_m_tvalid),
        .m_axis_data_tready(fft_m_tready),
        .m_axis_data_tlast(fft_m_tlast));

    // =================================================================
    // 6. Live-data handshake into the existing covariance path.
    //    range_fft_top produces one complex sample per channel when
    //    fft_ce is high. We convert that into the radar_valid/radar_re/im
    //    contract selftest_bram expects: present the sample and pulse
    //    valid; the covariance core's ce_out+advance consumes it.
    //    Because both are paced by the SAME CE_DIV=80 cadence, they stay
    //    lock-stepped without a FIFO. (If you later decouple the rates,
    //    drop a small AXIS FIFO here -- flagged, not needed now.)
    // =================================================================
    reg signed [15:0] radar_re_r, radar_im_r;
    reg               radar_valid_r;
    always @(posedge clk) begin
        if (rst) begin radar_valid_r <= 1'b0; end
        else begin
            radar_valid_r <= fft_ce;          // one strobe per paced sample
            if (fft_ce) begin
                radar_re_r <= fft_samp_re;
                radar_im_r <= fft_samp_im;
            end
        end
    end

    wire        bpm_mode;
    wire [15:0] samples_per_chirp;
    wire [2:0]  sel0, sel1, sel2, sel3;
    wire [7:0]  bin_sel;
    wire signed [15:0] cal_re, cal_im;
    wire [15:0] snapshots_k;
    wire        stream_sync_req;
    // =================================================================
    // 7. Inner covariance path -- the EXISTING, verified cov_pl_top,
    //    modified only to expose its radar-ingest port (see note).
    //    If you prefer zero edits to cov_pl_top.v, use the variant
    //    cov_pl_top_live below instead (provided as a separate file).
    // =================================================================
    cov_pl_top_live u_core (
        .clk(clk), .rst(rst),
        .radar_re(radar_re_r), .radar_im(radar_im_r), .radar_valid(radar_valid_r),
        
        .bpm_mode(bpm_mode),
        .samples_per_chirp(samples_per_chirp),
        .sel0(sel0), .sel1(sel1), .sel2(sel2), .sel3(sel3),
        .bin_sel(bin_sel),
        .cal_re(cal_re), .cal_im(cal_im),
        .snapshots_k(snapshots_k),
        .stream_sync_req(stream_sync_req),
        
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready));

endmodule
