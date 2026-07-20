// pending: verify the s_axis_config word layout. For Radix-2
// Lite @ 256 pts the config is {SCALE_SCH[15:0], FWD_INV} packed
// LSB-first then padded to 24 bits; FFT_CONFIG_WORD below assumes that
// and a shift-1-per-stage schedule (net /256). The IP customization
// GUI's "Implementation Details" tab prints the authoritative field
// layout for the exact configuration check it once against this
// localparam before trusting output scaling.
module range_fft_top #(
    parameter N       = 256,           
    parameter LOGN    = 8,
    parameter IW      = 13,            
    parameter OW      = 16,          
    parameter CE_DIV  = 80            
)(
    input  wire                  clk,          
    input  wire                  rst,

     input  wire [LOGN-1:0]       bin_sel,        
    input  wire signed [15:0]    cal_re,         
    input  wire signed [15:0]    cal_im,         
    input  wire [15:0]           snapshots_k,    

    
    input  wire signed [IW-1:0]  ch0, ch1, ch2, ch3,
    input  wire                  ch0_v, ch1_v, ch2_v, ch3_v,

 
    output reg  signed [OW-1:0]  cov_samp_re,
    output reg  signed [OW-1:0]  cov_samp_im,
    output reg                   cov_clk_enable,
    output reg                   cov_frame_start,

    
    output reg  [31:0]           s_axis_data_tdata,
    output reg                   s_axis_data_tvalid,
    input  wire                  s_axis_data_tready,
    output reg                   s_axis_data_tlast,
    output wire [23:0]           s_axis_config_tdata,
    output wire                  s_axis_config_tvalid,
    input  wire                  s_axis_config_tready,
    input  wire [31:0]           m_axis_data_tdata,    
    input  wire [15:0]           m_axis_data_tuser,   
    input  wire                  m_axis_data_tvalid,
    input  wire                  m_axis_data_tlast,
    output wire                  m_axis_data_tready,
    output wire                  fft_aresetn
);
    assign fft_aresetn = ~rst;
    assign m_axis_data_tready = 1'b1;   

    localparam [23:0] FFT_CONFIG_WORD = {7'b0, 16'b01_01_01_01_01_01_01_01, 1'b1};
    reg cfg_done;
    assign s_axis_config_tdata  = FFT_CONFIG_WORD;
    assign s_axis_config_tvalid = ~cfg_done & ~rst;
    always @(posedge clk) begin
        if (rst) cfg_done <= 1'b0;
        else if (s_axis_config_tvalid & s_axis_config_tready) cfg_done <= 1'b1;
    end

    // Hann Window
    reg [15:0] hann [0:N-1];
    initial $readmemh("hann256.mem", hann);

    
 //ping pong buffers per channel
    reg signed [IW-1:0] buf0 [0:2*N-1];
    reg signed [IW-1:0] buf1 [0:2*N-1];
    reg signed [IW-1:0] buf2 [0:2*N-1];
    reg signed [IW-1:0] buf3 [0:2*N-1];

    reg               bank;                        
    reg [LOGN:0]      wp0, wp1, wp2, wp3;          
    wire all_full = (wp0==N) && (wp1==N) && (wp2==N) && (wp3==N);

    always @(posedge clk) begin
        if (rst) begin
            bank<=1'b0; wp0<=0; wp1<=0; wp2<=0; wp3<=0;
        end else begin
            if (ch0_v && wp0<N) begin buf0[{bank,wp0[LOGN-1:0]}]<=ch0; wp0<=wp0+1; end
            if (ch1_v && wp1<N) begin buf1[{bank,wp1[LOGN-1:0]}]<=ch1; wp1<=wp1+1; end
            if (ch2_v && wp2<N) begin buf2[{bank,wp2[LOGN-1:0]}]<=ch2; wp2<=wp2+1; end
            if (ch3_v && wp3<N) begin buf3[{bank,wp3[LOGN-1:0]}]<=ch3; wp3<=wp3+1; end
            if (all_full) begin
                bank<=~bank; wp0<=0; wp1<=0; wp2<=0; wp3<=0;   // swap; FFT side told below
            end
        end
    end

    // rotate FSM seq
    localparam S_IDLE=0, S_LOAD=1, S_DRAIN=2, S_ROT=3, S_OUT=4;
    reg [2:0]        state;
    reg [1:0]        fch;                 // channel currently in the FFT
    reg [LOGN:0]     rp;                  // read pointer
    reg              fbank;              // bank being transformed
    reg              pend;               // a full frame waiting FFT
    reg signed [IW-1:0]  rd_s;
    reg [15:0]           rd_w;
    reg                  rd_stage;       

    reg signed [15:0] Xre [0:3];
    reg signed [15:0] Xim [0:3];

    wire signed [IW+16-1:0] wprod = rd_s * $signed({1'b0, rd_w});
    wire signed [15:0]      wsamp = wprod >>> 15;

    wire signed [31:0] rr = Xre[3]*cal_re - Xim[3]*cal_im;
    wire signed [31:0] ri = Xre[3]*cal_im + Xim[3]*cal_re;

    reg [15:0] snap_ctr;                 
    reg [1:0]  och;                        
    reg [6:0]  ce_ctr;                    

    always @(posedge clk) begin
        if (rst) begin
            state<=S_IDLE; pend<=1'b0; s_axis_data_tvalid<=1'b0; s_axis_data_tlast<=1'b0;
            cov_clk_enable<=1'b0; cov_frame_start<=1'b0; snap_ctr<=0;
            cov_samp_re<=0; cov_samp_im<=0; rd_stage<=1'b0;
        end else begin
            if (all_full) begin pend<=1'b1; fbank<=bank; end   // bank captured at swap

            case (state)
            S_IDLE: if (pend && cfg_done) begin
                        pend<=1'b0; fch<=2'd0; rp<=0; rd_stage<=1'b0; state<=S_LOAD;
                    end

            S_LOAD: begin
                if (!rd_stage) begin
                    case (fch)
                        2'd0: rd_s <= buf0[{fbank, rp[LOGN-1:0]}];
                        2'd1: rd_s <= buf1[{fbank, rp[LOGN-1:0]}];
                        2'd2: rd_s <= buf2[{fbank, rp[LOGN-1:0]}];
                        2'd3: rd_s <= buf3[{fbank, rp[LOGN-1:0]}];
                    endcase
                    rd_w <= hann[rp[LOGN-1:0]];
                    if (!s_axis_data_tvalid) rd_stage <= 1'b1;   
                end
                if (rd_stage && !s_axis_data_tvalid) begin
                    s_axis_data_tdata  <= {16'd0, wsamp};
                    s_axis_data_tvalid <= 1'b1;
                    s_axis_data_tlast  <= (rp == N-1);
                end
                if (s_axis_data_tvalid && s_axis_data_tready) begin
                    s_axis_data_tvalid <= 1'b0; rd_stage <= 1'b0;
                    if (rp == N-1) begin rp<=0; state<=S_DRAIN; end
                    else rp <= rp+1;
                end
            end

            S_DRAIN: if (m_axis_data_tvalid) begin
                        if (m_axis_data_tuser[LOGN-1:0] == bin_sel) begin
                            Xre[fch] <= m_axis_data_tdata[15:0];
                            Xim[fch] <= m_axis_data_tdata[31:16];
                        end
                        if (m_axis_data_tlast) begin
                            if (fch == 2'd3) state <= S_ROT;
                            else begin fch <= fch+1; rp<=0; rd_stage<=1'b0; state <= S_LOAD; end
                        end
                     end

        
            S_ROT: begin
                Xre[3] <= rr >>> 15;
                Xim[3] <= ri >>> 15;
                och<=2'd0; ce_ctr<=0; state<=S_OUT;
            end

           
            S_OUT: begin
                cov_clk_enable  <= 1'b1;
                cov_samp_re     <= Xre[och];
                cov_samp_im     <= Xim[och];
                cov_frame_start <= (och==2'd0) && (snap_ctr==16'd0);
                if (ce_ctr == CE_DIV-1) begin
                    ce_ctr <= 0;
                    if (och == 2'd3) begin
                        cov_clk_enable <= 1'b0; cov_frame_start <= 1'b0;
                        snap_ctr <= (snap_ctr == snapshots_k-1) ? 16'd0 : snap_ctr+1;
                        state <= S_IDLE;
                    end else och <= och+1;
                end else ce_ctr <= ce_ctr+1;
            end
            endcase
        end
    end
endmodule
