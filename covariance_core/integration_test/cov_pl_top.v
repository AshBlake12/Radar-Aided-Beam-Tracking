module cov_pl_top_live (
    input  wire        clk,
    input  wire        rst,

    
    input  wire signed [15:0] radar_re,
    input  wire signed [15:0] radar_im,
    input  wire               radar_valid,
    
    output wire        bpm_mode,
    output wire [15:0] samples_per_chirp,
    output wire [2:0]  sel0, sel1, sel2, sel3,
    output wire [7:0]  bin_sel,
    output wire signed [15:0] cal_re, cal_im,
    output wire [15:0] snapshots_k,
    output wire        stream_sync_req,

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

    selftest_bram #(.DEPTH(2048), .INIT("stim_X.hex")) u_bram (
        .clk(clk), .rstn(rstn & ~soft_rst), .selftest_en(selftest_en), .start(ax_start),
        .advance(ce_out & bram_valid),          // one sample per base-rate strobe
        .radar_re(radar_re), .radar_im(radar_im), .radar_valid(radar_valid),  // <-- was 0,0,0
        .out_re(bram_re), .out_im(bram_im), .out_valid(bram_valid), .frame_done(bram_fdone));

    wire signed [15:0] samp_re = $signed(bram_re) >>> 2;   // Q1.15 -> Q2.13
    wire signed [15:0] samp_im = $signed(bram_im) >>> 2;

    reg first_pending;
    reg frame_start_r;
    always @(posedge clk) begin
        if (!rstn) begin first_pending<=1'b0; frame_start_r<=1'b0; end
        else begin
            frame_start_r <= 1'b0;
            if (ax_start) first_pending <= 1'b1;
            if (first_pending & ce_out & bram_valid) begin
                frame_start_r <= 1'b1;
                first_pending <= 1'b0;
            end
        end
    end

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

    reg cov_valid_d; always @(posedge clk) cov_valid_d <= rstn ? cov_valid : 1'b0;
    wire cov_done = cov_valid & ~cov_valid_d;

    reg [3:0] wr_idx; reg wr_active;
    reg coeff_we_r; reg [3:0] coeff_idx_r; reg [31:0] coeff_data_r;
    always @(posedge clk) begin
        if (!rstn) begin wr_active<=1'b0; wr_idx<=4'd0; coeff_we_r<=1'b0; end
        else begin
            coeff_we_r <= 1'b0;
            if (cov_done && !wr_active) begin
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
        .bpm_mode(bpm_mode),
        .samples_per_chirp(samples_per_chirp),
        .sel0(sel0), .sel1(sel1), .sel2(sel2), .sel3(sel3),
        .bin_sel(bin_sel),
        .cal_re(cal_re), .cal_im(cal_im),
        .snapshots_k(snapshots_k),
        .stream_sync_req(stream_sync_req),
        .selftest_en(selftest_en), .start_pulse(ax_start), .soft_rst(soft_rst),
        .cal_phase(cal_phase), .core_done(cov_done), .core_busy(wr_active),
        .coeff_we(coeff_we_r), .coeff_idx(coeff_idx_r), .coeff_data(coeff_data_r));
endmodule