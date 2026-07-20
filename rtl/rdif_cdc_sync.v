module rdif_cdc_sync (
    input  wire clk_fpga_0,
    input  wire rdif_clk,          // common BUFG out
    input  wire rst_fpga_0,        
    input  wire stream_sync_req,   

    output wire rst_rdif,          
    output wire stream_sync_rdif   // single cycle pulse in rdif_clk domain
);
    (* ASYNC_REG = "true" *) reg [1:0] rst_ff;
    always @(posedge rdif_clk or posedge rst_fpga_0) begin
        if (rst_fpga_0) rst_ff <= 2'b11;
        else            rst_ff <= {rst_ff[0], 1'b0};
    end
    assign rst_rdif = rst_ff[1];
    
    reg tgl_src;
    always @(posedge clk_fpga_0) begin
        if (rst_fpga_0)          tgl_src <= 1'b0;
        else if (stream_sync_req) tgl_src <= ~tgl_src;
    end
    (* ASYNC_REG = "true" *) reg [2:0] tgl_ff;
    always @(posedge rdif_clk) tgl_ff <= {tgl_ff[1:0], tgl_src};
    assign stream_sync_rdif = tgl_ff[2] ^ tgl_ff[1];
endmodule
