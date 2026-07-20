//   TDM :  TX1, TX2, TX1, TX2, ...        -> route each chirp straight
//          through to TX1 or TX2 on parity, no arithmetic.
//   BPM :  (TX1+TX2), (TX1-TX2), ...      -> sum/diff a chirp pair.
//          h1 = (y_k + y_k+1)/2 = TX1-only, h2 = (y_k - y_k+1)/2 = TX2-only.
//          Scale-by-2 is left unnormalized here (both TX channels see the
//          same factor, so it's a gain absorb it downstream).
//
// IWRL6432 RX channels are REAL, not I/Q (Sec 7.10: "real-only Rx
// channels"; complex sample rate row is blank in the part-family table).
// So this decode operates on real 12-bit ADC codes, ahead of any FFT.

module tdm_bpm_decode #(
    parameter ADC_W = 12                 // sample w (12b check ds)
)(
    input  wire                     clk,
    input  wire                     rst,

    input  wire                     bpm_mode,          
    input  wire [15:0]              samples_per_chirp, 
    input  wire                     stream_sync,       

    input  wire signed [ADC_W-1:0]  adc_sample,
    input  wire [1:0]               rx_idx,           
    input  wire                     adc_valid,

    // decoding virtual channels: 3 RX x {TX1,TX2} = 6 real sample streams
    output reg  signed [ADC_W:0]    tx1_rx0, output reg tx1_rx0_valid,
    output reg  signed [ADC_W:0]    tx1_rx1, output reg tx1_rx1_valid,
    output reg  signed [ADC_W:0]    tx1_rx2, output reg tx1_rx2_valid,
    output reg  signed [ADC_W:0]    tx2_rx0, output reg tx2_rx0_valid,
    output reg  signed [ADC_W:0]    tx2_rx1, output reg tx2_rx1_valid,
    output reg  signed [ADC_W:0]    tx2_rx2, output reg tx2_rx2_valid
);

    // chirp position tracking, free-running off the known chirp length
    reg [15:0] samp_ctr;   // sample index within the current chirp
    reg        chirp_b;    // 0 = TDM-TX1 / BPM chirp-A, 1 = TDM-TX2 / BPM chirp-B

    wire round_done = adc_valid && (rx_idx == 2'd2);         // all 3 RX seen for this sample instant
    wire chirp_done = round_done && (samp_ctr == samples_per_chirp - 16'd1);

    always @(posedge clk) begin
        if (rst || stream_sync) begin
            samp_ctr <= 16'd0;
            chirp_b  <= 1'b0;
        end else if (round_done) begin
            samp_ctr <= chirp_done ? 16'd0 : samp_ctr + 16'd1;
            if (chirp_done) chirp_b <= ~chirp_b;
        end
    end

    reg signed [ADC_W-1:0] chirpA_buf [0:2];


    function signed [ADC_W:0] sext;
        input signed [ADC_W-1:0] v;
        sext = {v[ADC_W-1], v};
    endfunction

    always @(posedge clk) begin
        {tx1_rx0_valid, tx1_rx1_valid, tx1_rx2_valid,
         tx2_rx0_valid, tx2_rx1_valid, tx2_rx2_valid} <= 6'b0;

        if (adc_valid) begin
            if (!bpm_mode) begin
                
                case ({chirp_b, rx_idx})
                    3'b0_00: begin tx1_rx0 <= sext(adc_sample); tx1_rx0_valid <= 1'b1; end
                    3'b0_01: begin tx1_rx1 <= sext(adc_sample); tx1_rx1_valid <= 1'b1; end
                    3'b0_10: begin tx1_rx2 <= sext(adc_sample); tx1_rx2_valid <= 1'b1; end
                    3'b1_00: begin tx2_rx0 <= sext(adc_sample); tx2_rx0_valid <= 1'b1; end
                    3'b1_01: begin tx2_rx1 <= sext(adc_sample); tx2_rx1_valid <= 1'b1; end
                    3'b1_10: begin tx2_rx2 <= sext(adc_sample); tx2_rx2_valid <= 1'b1; end
                    default: ; 
                endcase
            end else begin
                
                if (!chirp_b) begin
                    chirpA_buf[rx_idx] <= adc_sample;
                end else begin
                    case (rx_idx)
                        2'd0: begin
                            tx1_rx0 <= sext(chirpA_buf[0]) + sext(adc_sample); tx1_rx0_valid <= 1'b1;
                            tx2_rx0 <= sext(chirpA_buf[0]) - sext(adc_sample); tx2_rx0_valid <= 1'b1;
                        end
                        2'd1: begin
                            tx1_rx1 <= sext(chirpA_buf[1]) + sext(adc_sample); tx1_rx1_valid <= 1'b1;
                            tx2_rx1 <= sext(chirpA_buf[1]) - sext(adc_sample); tx2_rx1_valid <= 1'b1;
                        end
                        2'd2: begin
                            tx1_rx2 <= sext(chirpA_buf[2]) + sext(adc_sample); tx1_rx2_valid <= 1'b1;
                            tx2_rx2 <= sext(chirpA_buf[2]) - sext(adc_sample); tx2_rx2_valid <= 1'b1;
                        end
                        default: ;
                    endcase
                end
            end
        end
    end

endmodule


// Picks 4 of the 6 decoded virtual channels for the 4-channel covariance
// core. Geometry CONFIRMED from the IWRL6432BOOST EVM user guide
// (SWRU596, Sec 6.1.1, lambda at 62 GHz, D = lambda/2):
//   RX spacing = D azimuth; TX2 offset from TX1 = 2D azimuth + D elevation.
// Virtual array (azimuth in units of D):
//   TX1 row (elev 0):  V0=T1R0@0  V1=T1R1@1  V2=T1R2@2
//   TX2 row (elev D):  V3=T2R0@2  V4=T2R1@3  V5=T2R2@4
//
// DEFAULT SELECTION {0,1,2,4} = {T1R0,T1R1,T1R2,T2R1} = azimuth 0,1,2,3:
// a uniform lambda/2 ULA, matching covariance_core_fixptp44 + Root-MUSIC
// assumptions. Caveats (both correctable via the existing cal_phase AXI
// register applied as a rotation on ch3): (a) ch3 sits D higher in
// elevation; (b) in TDM mode ch3 carries a Doppler phase of
// 2*pi*fD*Tchirp from the one-chirp TX2 delay. The redundant pair V2/V3
// (same azimuth, different row) measures both effects for calibration.
//
// sel0..sel3: 0..5 mapping to {tx1_rx0,tx1_rx1,tx1_rx2,tx2_rx0,tx2_rx1,tx2_rx2}
module vch_select4 #(
    parameter ADC_W = 12
)(
    input  wire clk,
    input  wire [2:0] sel0, sel1, sel2, sel3,  

    input  wire signed [ADC_W:0] tx1_rx0, tx1_rx1, tx1_rx2,
    input  wire signed [ADC_W:0] tx2_rx0, tx2_rx1, tx2_rx2,
    input  wire tx1_rx0_valid, tx1_rx1_valid, tx1_rx2_valid,
    input  wire tx2_rx0_valid, tx2_rx1_valid, tx2_rx2_valid,

    output reg signed [ADC_W:0] ch0, ch1, ch2, ch3,
    output reg                  ch0_v, ch1_v, ch2_v, ch3_v
);
    function signed [ADC_W:0] pick_s;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: pick_s = tx1_rx0; 3'd1: pick_s = tx1_rx1; 3'd2: pick_s = tx1_rx2;
                3'd3: pick_s = tx2_rx0; 3'd4: pick_s = tx2_rx1; default: pick_s = tx2_rx2;
            endcase
        end
    endfunction
    function pick_v;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: pick_v = tx1_rx0_valid; 3'd1: pick_v = tx1_rx1_valid; 3'd2: pick_v = tx1_rx2_valid;
                3'd3: pick_v = tx2_rx0_valid; 3'd4: pick_v = tx2_rx1_valid; default: pick_v = tx2_rx2_valid;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        ch0 <= pick_s(sel0); ch0_v <= pick_v(sel0);
        ch1 <= pick_s(sel1); ch1_v <= pick_v(sel1);
        ch2 <= pick_s(sel2); ch2_v <= pick_v(sel2);
        ch3 <= pick_s(sel3); ch3_v <= pick_v(sel3);
    end
endmodule