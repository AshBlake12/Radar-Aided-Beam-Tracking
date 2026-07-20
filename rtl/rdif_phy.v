module rdif_phy (
    // Check RDIF section in ds
    input  wire        RDIF_CLK,
    input  wire [3:0]  RDIF_D,        // D0,D1,D2,D3
    input  wire        RDIF_FRM_CLK,

    // clk_fpga_0 domain
    input  wire         clk_fpga_0,
    input  wire         rst_fpga_0,
    input  wire         stream_sync,   
    output wire signed [11:0] adc_sample,
    
    output wire [1:0]   rx_idx,
    output wire         adc_valid,
    output wire         rdif_clk_out   // buffered RDIF clk
);

    //clock buffer for the source syncronous bit clock
        wire rdif_clk_bufg;
    BUFG bufg_rdif_clk (.I(RDIF_CLK), .O(rdif_clk_bufg));
    assign rdif_clk_out = rdif_clk_bufg;     

    wire [3:0] d_rise, d_fall;
    wire       frm_rise, frm_fall;

    genvar gi;
    generate
        for (gi = 0; gi < 4; gi = gi + 1) begin : ddr_lane
            IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED")) u_iddr (
                .C(rdif_clk_bufg), .CE(1'b1), .S(1'b0), .R(1'b0),
                .D(RDIF_D[gi]), .Q1(d_rise[gi]), .Q2(d_fall[gi]));
        end
    endgenerate
    IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED")) u_iddr_frm (
        .C(rdif_clk_bufg), .CE(1'b1), .S(1'b0), .R(1'b0),
        .D(RDIF_FRM_CLK), .Q1(frm_rise), .Q2(frm_fall));

    wire frm_active = frm_rise | frm_fall;    

    reg        frm_active_d;
    reg [1:0]  bitpos;        //crosscheck dis

    wire frame_start = frm_active & ~frm_active_d;   // rising edge 

    
    reg [11:0] acc;
    reg        word_rdy;
    reg [1:0]  word_rx;
    reg [11:0] word_out;

    //rotating rx channel and missalignment
    reg [1:0]  rx_rot;

    //collect bits in 012 edges
    always @(posedge rdif_clk_bufg) begin
        frm_active_d <= frm_active;
        word_rdy     <= 1'b0;

        if (rst_fpga_0 || stream_sync) begin
            bitpos <= 2'd0; rx_rot <= 2'd0;
        end else begin
            if (frame_start) bitpos <= 2'd0;  

            if (frm_active) begin  //fixed this 
                case (bitpos)
                    2'd0: begin
                        
                        acc <= {d_rise[3:0], d_fall[3:0], 4'b0};
                        bitpos <= 2'd2;
                    end
                    2'd2: begin
                        
                        word_out <= {acc[11:8], acc[7:4], d_rise[3:0]};
                        acc      <= {d_fall[3:0], 8'b0};
                        word_rdy <= 1'b1; word_rx <= rx_rot;
                        rx_rot   <= (rx_rot == 2'd2) ? 2'd0 : rx_rot + 2'd1;
                        bitpos   <= 2'd1;
                    end
                    2'd1: begin
                       
                        word_out <= {acc[11:8], d_rise[3:0], d_fall[3:0]};
                        word_rdy <= 1'b1; word_rx <= rx_rot;
                        rx_rot   <= (rx_rot == 2'd2) ? 2'd0 : rx_rot + 2'd1;
                        bitpos   <= 2'd0;
                    end
                endcase
            end
        end
    end

    wire        word_valid_pre = word_rdy;   // every slot is real data -- no padding to drop

    // CDC into clk_fpga_0 via async FIFO 
    wire [13:0] fifo_din  = {word_rx, word_out};
    wire [13:0] fifo_dout;
    wire        fifo_empty, fifo_rd_en;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("distributed"),
        .FIFO_WRITE_DEPTH(32),
        .WRITE_DATA_WIDTH(14),
        .READ_DATA_WIDTH(14),
        .READ_MODE("fwft"),
        .CDC_SYNC_STAGES(3)
    ) u_cdc_fifo (
        .wr_clk(rdif_clk_bufg), .rst(rst_fpga_0),
        .din(fifo_din), .wr_en(word_valid_pre), .full(),
        .rd_clk(clk_fpga_0), .rd_en(fifo_rd_en),
        .dout(fifo_dout), .empty(fifo_empty),
        .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    assign fifo_rd_en = ~fifo_empty;
    assign adc_valid  = ~fifo_empty;
    assign rx_idx      = fifo_dout[13:12];
    assign adc_sample = fifo_dout[11:0];

endmodule