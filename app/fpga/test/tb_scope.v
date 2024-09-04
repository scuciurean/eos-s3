`timescale 10ns /10ps

module tb_system;
    reg clk, rst;

    reg    [16:0]  WB_ADR         ;
    reg            WB_CYC         ;
    reg     [3:0]  WB_BYTE_STB    ;
    reg            WB_WE          ;
    reg            WB_RD          ;
    reg            WB_STB         ;
    reg    [31:0]  WB_WR_DAT      ;
    wire            WB_CLK         ;
    wire            WB_RST         ;
    wire    [31:0]  WB_RD_DAT     ;
    wire            WB_ACK        ;

    assign WB_CLK = clk;
    assign WB_RST = rst;

    integer i;

    top DUT (
        .clk(clk), .rst(rst),

        .WB_ADR(WB_ADR),
        .WB_CYC(WB_CYC),
        .WB_BYTE_STB(WB_BYTE_STB),
        .WB_WE(WB_WE),
        .WB_RD(WB_RD),
        .WB_STB(WB_STB),
        .WB_WR_DAT(WB_WR_DAT),
        .WB_CLK(WB_CLK),
        .WB_RST(WB_RST),
        .WB_RD_DAT(WB_RD_DAT),
        .WB_ACK(WB_ACK)
    );

    always
        #1 clk = !clk;

    initial begin
        WB_ADR = 0;
        WB_WR_DAT = 0;
        WB_WE = 0;
        clk = 0;
        rst = 1;
        #2
        rst = 0;
    end

    initial begin
        $dumpfile("tb_system.vcd");
        $dumpvars(0,tb_system);
        $display("\t\ttime,\tclk,\trst");
        $monitor("%d,\t%b,\t%b",$time, clk,rst);
    end

    task wishbone_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            // Set the signals for a write transaction
            WB_ADR <= addr;
            WB_WR_DAT <= data;
            WB_CYC  <= 1'b1;
            WB_STB  <= 1'b1;
            WB_BYTE_STB  <= 4'b1111;
            WB_WE   <= 1'b1;

            // Wait for acknowledge
            @(posedge WB_CLK);
            while (!WB_ACK) begin
                @(posedge WB_CLK);
            end

            // Clear the signals after acknowledgment
            WB_CYC  <= 1'b0;
            WB_STB  <= 1'b0;
            WB_BYTE_STB  <= 4'b0;
            WB_WE   <= 1'b0;
        end
    endtask


    initial begin
        @(negedge rst);
        #5
        wishbone_write(32'h0000200C, 32'h4);
        wishbone_write(32'h00002000, 32'h1);
    end


    initial
    #2000 $finish;


endmodule

module top (
    input wire clk,
    input wire rst,

    input wire    [16:0]  WB_ADR         ,
    input wire            WB_CYC         ,
    input wire     [3:0]  WB_BYTE_STB    ,
    input wire            WB_WE          ,
    input wire            WB_RD          ,
    input wire            WB_STB         ,
    input wire    [31:0]  WB_WR_DAT      ,
    input wire            WB_CLK         ,
    input wire            WB_RST         ,
    output wire   [31:0]  WB_RD_DAT      ,
    output wire           WB_ACK
);
    localparam NUM_SLAVES = 2;
    localparam DATA_WIDTH = 32;
    localparam WB_ADDR_WIDTH = 17;
    localparam WB_MUX_ADDR_WIDTH = 9;
    localparam [NUM_SLAVES*WB_ADDR_WIDTH-1:0] SLAVE_BASE_ADDR = {
        17'h02000,  // scope
        17'h01000   // scope_ram
    };

    wire [NUM_SLAVES-1:0] WBS_CYC;
    wire [NUM_SLAVES-1:0] WBS_ACK;
    wire [NUM_SLAVES*DATA_WIDTH-1:0] WBS_RD_DAT;
    wire [NUM_SLAVES-1:0] WBS_WE;
    wire [NUM_SLAVES-1:0] WBS_RD;
    wire [DATA_WIDTH-1:0] WBS_WR_DAT;
    wire [WB_ADDR_WIDTH-WB_MUX_ADDR_WIDTH-1:0] WBS_ADR;
    wire [NUM_SLAVES-1:0] WBS_STB;
    wire [3:0]            WBS_BYTE_STB;


    wire [8:0] DEBUG_RAM_WR_ADDR;
    wire [31:0] DEBUG_RAM_WR_DATA;
    wire [3:0]DEBUG_RAM_WR_EN;
    wire DEBUG_RAM_WR_CLK;
    wire DEBUG_RAM_WR_CLK_EN;

    wishbone_interconnect #(
        .NUM_SLAVES(NUM_SLAVES),
        .ADDR_WIDTH(WB_ADDR_WIDTH),
        .MUX_ADDR_WIDTH(WB_MUX_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) interconnect (
        .WB_ADR(WB_ADR),
        .WB_CYC(WB_CYC),
        .WB_WE(WB_WE),
        .WB_RD(WB_RD),
        .WB_WR_DAT(WB_WR_DAT),
        .WB_STB(WB_STB),
        .WB_BYTE_STB(WB_BYTE_STB),
        .WB_CLK(WB_CLK),
        .WB_RST(WB_RST),
        .WB_RD_DAT(WB_RD_DAT),
        .WB_ACK(WB_ACK),
        .WBS_CYC(WBS_CYC),
        .WBS_ACK(WBS_ACK),
        .WBS_RD_DAT(WBS_RD_DAT),
        .WBS_WE(WBS_WE),
        .WBS_RD(WBS_RD),
        .WBS_WR_DAT(WBS_WR_DAT),
        .WBS_STB(WBS_STB),
        .WBS_ADR(WBS_ADR),
        .WBS_BYTE_STB(WBS_BYTE_STB),

        .SLAVE_BASE_ADDRESSES(SLAVE_BASE_ADDR)
    );

    wishbone_scope #(
        .ADDR_WIDTH(WB_ADDR_WIDTH),
        .MUX_ADDR_WIDTH(WB_MUX_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) DEBUG_SCOPE (
        .CLK(WB_CLK),
        .RST(WB_RST_FPGA),

        .WB_CLK(WB_CLK),
        .WB_RST(WB_RST_FPGA),
        .WBS_CYC(WBS_CYC[1]),
        .WBS_WE(WBS_WE[1]),
        .WBS_RD(WBS_RD[1]),
        .WBS_WR_DAT(WBS_WR_DAT),
        .WBS_STB(WBS_STB[1]),
        .WBS_BYTE_STB(WBS_BYTE_STB),
        .WBS_ADR(WBS_ADR),
        .WBS_RD_DAT(WBS_RD_DAT[2*DATA_WIDTH-1:(2-1)*DATA_WIDTH]),
        .WBS_ACK(WBS_ACK[1]),

        .TRIGGER(1'b0),
        .IN_PORTS(1'b0),

        .RAM_WR_ADDR(DEBUG_RAM_WR_ADDR),
        .RAM_WR_DATA(DEBUG_RAM_WR_DATA),
        .RAM_WR_EN(DEBUG_RAM_WR_EN),
        .RAM_WR_CLK_EN(DEBUG_RAM_WR_CLK_EN)
    );

//     wishbone_read_only_ram #(
//         .ADDR_WIDTH(WB_ADDR_WIDTH),
//         .MUX_ADDR_WIDTH(WB_MUX_ADDR_WIDTH),
//         .DATA_WIDTH(DATA_WIDTH)
//     ) RO_RAM (
//         .WB_CLK(WB_CLK),
//         .WB_RST(WB_RST_FPGA),

//         .WBS_CYC(WBS_CYC[0]),
//         .WBS_WE(WBS_WE[0]),
//         .WBS_RD(WBS_RD[0]),
//         .WBS_WR_DAT(WBS_WR_DAT),
//         .WBS_STB(WBS_STB[0]),
//         .WBS_BYTE_STB(WBS_BYTE_STB),
//         .WBS_ADR(WBS_ADR),
//         .WBS_RD_DAT(WBS_RD_DAT[1*DATA_WIDTH-1:(911)*DATA_WIDTH]),
//         .WBS_ACK(WBS_ACK[0]),

//         .WR_ADDR(DEBUG_RAM_WR_ADDR),
//         .WR_DATA(DEBUG_RAM_WR_DATA),
//         .WR_EN(DEBUG_RAM_WR_EN),
//         .WR_CLK(WB_CLK),
//         .WR_CLK_EN(DEBUG_RAM_WR_CLK_EN)
//     );
endmodule