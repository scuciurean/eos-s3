module sdma #(
    parameter SRC_DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst,

    input wire [SRC_DATA_WIDTH-1:0] data,
    input wire valid,

    output reg sdma_req,
    output wire sdma_irq,
    input wire sdma_done,
    input wire sdma_active
);
    // RAM_16K_BLK #(
    //     .addr_int(9),
    //     .data_depth_int(512),
    //     .data_width_int(SRC_DATA_WIDTH),
    //     .wr_enable_int(4),
    //     .reg_rd_int(0)
    // ) RAM_INST (
    //     .WA(9'h0),
    //     .RA(9'h0),
    //     .WD(data),
    //     .WClk(clk),
    //     .RClk(clk),
    //     .WClk_En(1'b1),
    //     .RClk_En(1'b0),
    //     .WEN({ram_wen,ram_wen,ram_wen,ram_wen}),
    //     .RD()
    // );

    reg ram_wen;
    reg ram_written;
    reg[7:0] delay;

    initial begin
        ram_wen = 0;
        ram_written = 0;
        delay = 0;
    end

    always @(posedge clk) begin
        if (valid) begin
            delay = delay + 1;
            if (~ram_written) begin
                sdma_req =1;
                ram_written = 1;
            end else if (delay == 3) begin
                sdma_req = 0;
            end
        end else begin
            ram_written = 0;
            sdma_req = 0;
            delay = 0;
        end
    end

    // always @(posedge clk) begin
    //     if (valid) begin
    //         if (~ram_written) begin
    //             ram_wen = 1;
    //             ram_written = 1;
    //         end else begin
    //             sdma_req = 1;
    //             ram_wen = 0;
    //         end
    //     end else begin
    //         ram_wen = 0;
    //         ram_written = 0;
    //         if (sdma_done) begin
    //             sdma_req = 0;
    //         end
    //     end
    // end
endmodule