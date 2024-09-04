module wishbone_ram #(
    parameter MUX_ADDR_WIDTH = 9,
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 32
)(
    input  wire                  WBS_CYC,
    input  wire                  WBS_WE,
    input  wire                  WBS_RD,
    input  wire [DATA_WIDTH-1:0] WBS_WR_DAT,
    input  wire [3:0]            WBS_BYTE_STB,
    input  wire                  WBS_STB,
    input  wire [ADDR_WIDTH - MUX_ADDR_WIDTH-1:0] WBS_ADR,
    input  wire                  WB_CLK,
    input  wire                  WB_RST,
    output reg  [DATA_WIDTH-1:0] WBS_RD_DAT,
    output reg                   WBS_ACK
);
    r512x32_512x32 RAM3_INST (
			.WA({1'b0,WBS_ADR}),
            .RA({1'b0,WBS_ADR}),
			.WD(WBS_WR_DAT),
			.RD(WBS_RD_DAT),
			.WClk(WB_CLK),
			.RClk(WB_CLK),
			.WClk_En(1'b1),
			.RClk_En(1'b1),
			.WEN({WBS_WE,WBS_WE,WBS_WE,WBS_WE})
			);

    always @(posedge WB_CLK or posedge WB_RST) begin
        if (WB_RST) begin
            WBS_ACK <= 0;
        end else begin
            if (WBS_CYC && WBS_STB && ~WBS_ACK) begin  // Write operation
                WBS_ACK <= 1;
            end else begin
                WBS_ACK <= 0;
            end
        end
    end
endmodule


module wishbone_read_only_ram  #(
    parameter MUX_ADDR_WIDTH = 9,
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 32
)(
    // Wishbone bus
    input  wire                  WBS_CYC,
    input  wire                  WBS_WE,
    input  wire                  WBS_RD,
    input  wire [DATA_WIDTH-1:0] WBS_WR_DAT,
    input  wire [3:0]            WBS_BYTE_STB,
    input  wire                  WBS_STB,
    input  wire [ADDR_WIDTH - MUX_ADDR_WIDTH-1:0] WBS_ADR,
    input  wire                  WB_CLK,
    input  wire                  WB_RST,
    output reg  [DATA_WIDTH-1:0] WBS_RD_DAT,
    output reg                   WBS_ACK,
    // BRAM Write bus
    input wire [8:0] WR_ADDR,
    input wire [31:0] WR_DATA,
    input wire [3:0] WR_EN,
    input wire WR_CLK,
    input wire WR_CLK_EN
);

    r512x32_512x32 RAM3_INST (
        .RA({1'b0,WBS_ADR}),
        .RD(WBS_RD_DAT),
        .RClk(WB_CLK),
        .RClk_En(1'b1),
        .WA(WR_ADDR),
        .WD(WR_DATA),
        .WClk(WR_CLK),
        .WClk_En(WR_CLK_EN),
        .WEN(WR_EN)
    );

    always @(posedge WB_CLK or posedge WB_RST) begin
        if (WB_RST) begin
            WBS_ACK <= 0;
        end else begin
            if (WBS_CYC && WBS_STB && ~WBS_ACK) begin  // Write operation
                WBS_ACK <= 1;
            end else begin
                WBS_ACK <= 0;
            end
        end
    end
endmodule