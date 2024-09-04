`timescale 10ns /10ps
// `include "wb_pwm.v"
// `include "wb_interconnect.v"
// `include "ad7984.v"
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

    wire SCK;
    wire CNV;
    reg SDO;
    wire ADC_DATA_VALID;
    wire [15:0] ADC_DATA;

    wire            SDMA0_Active   ;
    wire            SDMA0_Done     ;
    wire            SDMA0_Req      ;
    wire            SDMA0_INT      ;

    reg [15:0] sample_data;
    reg [15:0] samples [0:5];

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
        .WB_ACK(WB_ACK),

        .SCK(SCK),
        .CNV(CNV),
        .SDO(SDO),
        .ADC_DATA_VALID(ADC_DATA_VALID),
        .ADC_DATA(ADC_DATA),

        .SDMA0_Active(SDMA0_Active),
        .SDMA0_Done(SDMA0_Done),
        .SDMA0_Req(SDMA0_Req),
        .SDMA0_INT(SDMA0_INT)
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
        //Configure the pwm sample rate
        wishbone_write(32'h00001004, 32'h24);
        wishbone_write(32'h00001008, 32'h40);
        wishbone_write(32'h00001000, 32'h1);
        //Enable the ADC to read 3 samples
        wishbone_write(32'h00002008, 32'h3);
        wishbone_write(32'h00002000, 32'h3);
        //Continuos sampling
        #800
        wishbone_write(32'h00002000, 32'h1);
        #300
        wishbone_write(32'h00002000, 32'h0);
    end

    initial begin
        samples[0] = 16'hABCD;
        samples[1] = 16'h1234;
        samples[2] = 16'hAA55;
        samples[3] = 16'h3C41;
        samples[4] = 16'h0000;

        i = 0;
        sample_data = 16'hABCD;
        SDO = sample_data[0];
    end

    always @(posedge SCK  && ~rst or posedge CNV) begin
        if (CNV) begin
            sample_data = samples[i%5];
            i = i + 1;
        end else begin
            SDO = sample_data[15];
            sample_data = {sample_data[14:0],1'b0};
        end
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
    output wire           WB_ACK         ,

    output wire CNV,
    output wire SCK,
    input wire SDO,

    output wire [15:0] ADC_DATA,
    output wire ADC_DATA_VALID,


    output wire SDMA0_Active   ,
    output wire SDMA0_Done     ,
    output wire SDMA0_Req      ,
    output wire SDMA0_INT
);
    localparam NUM_SLAVES = 2;
    localparam DATA_WIDTH = 32;
    localparam WB_ADDR_WIDTH = 17;
    localparam WB_MUX_ADDR_WIDTH = 9;
    localparam [NUM_SLAVES*WB_ADDR_WIDTH-1:0] SLAVE_BASE_ADDR = {
        17'h02000,  // AD7984
        17'h01000   // PWM controller
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

    wishbone_pwm #(
        .ADDR_WIDTH(WB_ADDR_WIDTH),
        .MUX_ADDR_WIDTH(WB_MUX_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) pwm_pmod_cnv (
        .CLK(WB_CLK),
        .RST(WB_RST),

        .WB_CLK(WB_CLK),
        .WB_RST(WB_RST),
        .WBS_CYC(WBS_CYC[0]),
        .WBS_WE(WBS_WE[0]),
        .WBS_RD(WBS_RD[0]),
        .WBS_WR_DAT(WBS_WR_DAT),
        .WBS_STB(WBS_STB[0]),
        .WBS_BYTE_STB(WBS_BYTE_STB),
        .WBS_ADR(WBS_ADR),
        .WBS_RD_DAT(WBS_RD_DAT[1*DATA_WIDTH-1:(1-1)*DATA_WIDTH]),
        .WBS_ACK(WBS_ACK[0]),

        .PWM_OUT(pmod_ad7984_cnv)
    );

    ad7984 #(
        .ADDR_WIDTH(WB_ADDR_WIDTH),
        .MUX_ADDR_WIDTH(WB_MUX_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    )  pmod_ad7984 (
        .CLK(WB_CLK),
        .RST(WB_RST),

        .WB_CLK(WB_CLK),
        .WB_RST(WB_RST),
        .WBS_CYC(WBS_CYC[1]),
        .WBS_WE(WBS_WE[1]),
        .WBS_RD(WBS_RD[1]),
        .WBS_WR_DAT(WBS_WR_DAT),
        .WBS_STB(WBS_STB[1]),
        .WBS_BYTE_STB(WBS_BYTE_STB),
        .WBS_ADR(WBS_ADR),
        .WBS_RD_DAT(WBS_RD_DAT[2*DATA_WIDTH-1:(2-1)*DATA_WIDTH]),
        .WBS_ACK(WBS_ACK[1]),

        .SIG_CNV(pmod_ad7984_cnv),
        .SIG_SCK(WB_CLK),

        .CNV(CNV),
        .SCK(SCK),
        .SDO(SDO),

        .ADC_DATA(ADC_DATA),
        .ADC_DATA_VALID(ADC_DATA_VALID)
    );

    sdma #(
        .SRC_DATA_WIDTH(32)
    ) sdma_ad7894 (
        .clk(WB_CLK),
        .rst(WB_RST),

        .valid(ADC_DATA_VALID),
        .data({16'h0, ADC_DATA}),
        .sdma_req(SDMA0_Req),
        .sdma_irq(SDMA0_INT),
        .sdma_done(SDMA0_Done),
        .sdma_active(SDMA0_Active)
    );
endmodule