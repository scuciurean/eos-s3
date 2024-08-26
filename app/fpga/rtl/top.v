`timescale 1ns / 10ps

`define RED_LED     rgb_led[0]
`define GREEN_LED   rgb_led[1]
`define BLUE_LED    rgb_led[2]

module top(
   output wire [2:0] rgb_led,
   output wire PMOD_CNV,
   output wire PMOD_SCK,
   input wire PMOD_SDO,

   output wire PMOD_DEBUG_CNV,
   output wire PMOD_DEBUG_SCK,
   output wire PMOD_DEBUG_SDO
);

assign PMOD_DEBUG_CNV = PMOD_CNV;
assign PMOD_DEBUG_SCK = PMOD_SCK;
assign PMOD_DEBUG_SDO = PMOD_SDO;

localparam NUM_SLAVES = 6;
localparam DATA_WIDTH = 32;
localparam WB_ADDR_WIDTH = 17;
localparam WB_MUX_ADDR_WIDHT = 9;

// IMPORTANT: Define baseaddress from top to bottom
localparam [NUM_SLAVES*WB_ADDR_WIDTH-1:0] SLAVE_BASE_ADDR = {
    17'h06000,  // AD7984
    17'h05000,  // PWM PMOD SCK
    17'h04000,  // PWM PMOD CNMVV
    17'h03000,  // PWM controller tied to RED led
    17'h02000,  // GPIO controller 1 tied to GREEN led
    17'h01000   // GPIO controller 0 tied to BLUE led
};

// FB clocks
wire            Clk_C16        ;
wire            Clk_C16_Rst    ;
wire            Clk_C21        ;
wire            Clk_C21_Rst    ;

wire    [2:0]   SDMA_Done_Extra  ;
wire    [2:0]   SDMA_Active_Extra;

wire            SDMA0_Active   ;
wire            SDMA0_Done     ;
wire            SDMA0_Req      ;
wire            SDMA0_INT      ;

// Wishbone Bus Signals
wire    [16:0]  WB_ADR         ; // Wishbone Address Bus
wire            WB_CYC         ; // Wishbone Client Cycle Strobe (i.e. Chip Select)
wire     [3:0]  WB_BYTE_STB    ; // Wishbone Byte Enables
wire            WB_WE          ; // Wishbone Write Enable Strobe
wire            WB_RD          ; // Wishbone Read Enable Strobe
wire            WB_STB         ; // Wishbone Transfer Strobe
wire    [31:0]  WB_WR_DAT      ; // Wishbone Write Data Bus
wire            WB_CLK         ; // Wishbone Clock
wire            WB_RST         ; // Wishbone FPGA Reset
wire    [31:0]  WB_RD_DAT      ; // Wishbone Read Data Bus
wire            WB_ACK         ; // Wishbone Client Acknowledge
wire            WB_RST_FPGA    ; // Wishbone FPGA Reset [to FPGA_IP]

// Interconnect to Slave Signals
wire [NUM_SLAVES-1:0] WBS_CYC;
wire [NUM_SLAVES-1:0] WBS_ACK;
wire [NUM_SLAVES*DATA_WIDTH-1:0] WBS_RD_DAT;
wire [NUM_SLAVES-1:0] WBS_WE;
wire [NUM_SLAVES-1:0] WBS_RD;
wire [DATA_WIDTH-1:0] WBS_WR_DAT;
wire [WB_ADDR_WIDTH-WB_MUX_ADDR_WIDHT-1:0] WBS_ADR;
wire [NUM_SLAVES-1:0] WBS_STB;
wire [3:0]            WBS_BYTE_STB;

// assign wire [WB_MUX_ADDR_WIDHT-1:0] WBS_IP_ADDR = WBS_ADR[WB_ADDR_WIDTH-1:WB_MUX_ADDR_WIDHT-1]
// wire [WB_ADDR_WIDTH-WB_MUX_ADDR_WIDHT-1:0] WBS_IP_REGISTER_ADDR;
// assign WBS_IP_REGISTER_ADDR = WBS_ADR[WB_ADDR_WIDTH-WB_MUX_ADDR_WIDHT-1:0];

// GPIO controllers io ports
wire [31:0] gpio_controller0_io;
wire [31:0] gpio_controller1_io;

assign `BLUE_LED = gpio_controller0_io[0];
assign `GREEN_LED = gpio_controller1_io[0];

wire pmod_ad7984_cnv;
wire pmod_ad7984_sck;
wire pmod_ad7984_sdo;

wire [15:0] ADC_DATA;
wire ADC_DATA_VALID;

// Fabric clock reset buffer
gclkbuff buff_reset (
    .A(Clk_C16_Rst | WB_RST),
    .Z(WB_RST_FPGA)
);

// Fabric clock buffer
gclkbuff buff_clock (
    .A(Clk_C16),
    .Z(WB_CLK)
);

// Wishbone Interconnect
wishbone_interconnect #(
    .NUM_SLAVES(NUM_SLAVES),
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
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
    .WB_RST(WB_RST_FPGA),
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

// GPIO controller 0
wishbone_gpio #(
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
    .DATA_WIDTH(DATA_WIDTH)
) gpio_blue_led (
    .WB_CLK(WB_CLK),
    .WB_RST(WB_RST_FPGA),
    .WBS_CYC(WBS_CYC[0]),
    .WBS_WE(WBS_WE[0]),
    .WBS_RD(WBS_RD[0]),
    .WBS_WR_DAT(WBS_WR_DAT),
    .WBS_STB(WBS_STB[0]),
    .WBS_BYTE_STB(WBS_BYTE_STB),
    .WBS_ADR(WBS_ADR),
    .WBS_RD_DAT(WBS_RD_DAT[1*DATA_WIDTH-1:(1-1)*DATA_WIDTH]),
    .WBS_ACK(WBS_ACK[0]),

    .GPIO_IO(gpio_controller0_io)
);

// GPIO controller 1
wishbone_gpio #(
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
    .DATA_WIDTH(DATA_WIDTH)
) gpio_green_led (
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

    .GPIO_IO(gpio_controller1_io)
);

wishbone_pwm #(
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
    .DATA_WIDTH(DATA_WIDTH)
) pwm_red_led (
    .CLK(WB_CLK),
    .RST(WB_RST_FPGA),

    .WB_CLK(WB_CLK),
    .WB_RST(WB_RST_FPGA),
    .WBS_CYC(WBS_CYC[2]),
    .WBS_WE(WBS_WE[2]),
    .WBS_RD(WBS_RD[2]),
    .WBS_WR_DAT(WBS_WR_DAT),
    .WBS_STB(WBS_STB[2]),
    .WBS_BYTE_STB(WBS_BYTE_STB),
    .WBS_ADR(WBS_ADR),
    .WBS_RD_DAT(WBS_RD_DAT[3*DATA_WIDTH-1:(3-1)*DATA_WIDTH]),
    .WBS_ACK(WBS_ACK[2]),

    .PWM_OUT(`RED_LED)
);

wishbone_pwm #(
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
    .DATA_WIDTH(DATA_WIDTH)
) pwm_pmod_cnv (
    .CLK(WB_CLK),
    .RST(WB_RST_FPGA),

    .WB_CLK(WB_CLK),
    .WB_RST(WB_RST_FPGA),
    .WBS_CYC(WBS_CYC[3]),
    .WBS_WE(WBS_WE[3]),
    .WBS_RD(WBS_RD[3]),
    .WBS_WR_DAT(WBS_WR_DAT),
    .WBS_STB(WBS_STB[3]),
    .WBS_BYTE_STB(WBS_BYTE_STB),
    .WBS_ADR(WBS_ADR),
    .WBS_RD_DAT(WBS_RD_DAT[4*DATA_WIDTH-1:(4-1)*DATA_WIDTH]),
    .WBS_ACK(WBS_ACK[3]),

    .PWM_OUT(pmod_ad7984_cnv)
);

wishbone_pwm #(
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
    .DATA_WIDTH(DATA_WIDTH)
) pwm_pmod_sck (
    .CLK(WB_CLK),
    .RST(WB_RST_FPGA),

    .WB_CLK(WB_CLK),
    .WB_RST(WB_RST_FPGA),
    .WBS_CYC(WBS_CYC[4]),
    .WBS_WE(WBS_WE[4]),
    .WBS_RD(WBS_RD[4]),
    .WBS_WR_DAT(WBS_WR_DAT),
    .WBS_STB(WBS_STB[4]),
    .WBS_BYTE_STB(WBS_BYTE_STB),
    .WBS_ADR(WBS_ADR),
    .WBS_RD_DAT(WBS_RD_DAT[5*DATA_WIDTH-1:(5-1)*DATA_WIDTH]),
    .WBS_ACK(WBS_ACK[4]),

    .PWM_OUT(pmod_ad7984_sck)
);

ad7984 #(
    .ADDR_WIDTH(WB_ADDR_WIDTH),
    .MUX_ADDR_WIDHT(WB_MUX_ADDR_WIDHT),
    .DATA_WIDTH(DATA_WIDTH)
)  pmod_ad7984 (
    .CLK(WB_CLK),
    .RST(WB_RST_FPGA),

    .WB_CLK(WB_CLK),
    .WB_RST(WB_RST_FPGA),
    .WBS_CYC(WBS_CYC[5]),
    .WBS_WE(WBS_WE[5]),
    .WBS_RD(WBS_RD[5]),
    .WBS_WR_DAT(WBS_WR_DAT),
    .WBS_STB(WBS_STB[5]),
    .WBS_BYTE_STB(WBS_BYTE_STB),
    .WBS_ADR(WBS_ADR),
    .WBS_RD_DAT(WBS_RD_DAT[6*DATA_WIDTH-1:(6-1)*DATA_WIDTH]),
    .WBS_ACK(WBS_ACK[5]),

    .SIG_CNV(pmod_ad7984_cnv),
    .SIG_SCK(pmod_ad7984_sck),

    .CNV(PMOD_CNV),
    .SCK(PMOD_SCK),
    .SDO(PMOD_SDO),

    .ADC_DATA(ADC_DATA),
    .ADC_DATA_VALID(ADC_DATA_VALID)
);

sdma #(
    .SRC_DATA_WIDTH(32)
) sdma_ad7894 (
    .clk(WB_CLK),
    .rst(WB_RST),

    .data({16'h0, ADC_DATA}),
    .valid(ADC_DATA_VALID),
    .sdma_req(SDMA0_Req),
    .sdma_irq(SDMA0_INT),
    .sdma_done(SDMA0_Done),
    .sdma_active(SDMA0_Active)
);


// MCU
// Verilog model of QLAL4S3B
qlal4s3b_cell_macro
    u_qlal4s3b_cell_macro
    (
        // AHB-To-FPGA Bridge
        .WBs_ADR                   ( WB_ADR                        ), // output [16:0] | Address Bus                   to   FPGA
        .WBs_CYC                   ( WB_CYC                        ), // output        | Cycle Chip Select             to   FPGA
        .WBs_BYTE_STB              ( WB_BYTE_STB                   ), // output  [3:0] | Byte Select                   to   FPGA
        .WBs_WE                    ( WB_WE                         ), // output        | Write Enable                  to   FPGA
        .WBs_RD                    ( WB_RD                         ), // output        | Read  Enable                  to   FPGA
        .WBs_STB                   ( WB_STB                        ), // output        | Strobe Signal                 to   FPGA
        .WBs_WR_DAT                ( WB_WR_DAT                     ), // output [31:0] | Write Data Bus                to   FPGA
        .WB_CLK                    ( WB_CLK                        ), // input         | FPGA Clock                    from FPGA
        .WB_RST                    ( WB_RST                        ), // output        | FPGA Reset                    to   FPGA
        .WBs_RD_DAT                ( WB_RD_DAT                     ), // input  [31:0] | Read Data Bus                 from FPGA
        .WBs_ACK                   ( WB_ACK                        ), // input         | Transfer Cycle Acknowledge    from FPGA

        // SDMA Signals
        .SDMA_Req                  ({ 3'b000, SDMA0_Req        }), // input   [3:0]
        .SDMA_Sreq                 (  4'b0000                   ), // input   [3:0]
        .SDMA_Done                 ({ SDMA_Done_Extra, SDMA0_Done }), // output  [3:0]
        .SDMA_Active               ({ SDMA_Active_Extra, SDMA0_Active }), // output  [3:0]

        // FB Interrupts
        .FB_msg_out                ({1'b0, 1'b0, 1'b0, SDMA0_INT}), // input   [3:0]
        .FB_Int_Clr                (  8'h0                       ), // input   [7:0]
        .FB_Start                  (                             ), // output
        .FB_Busy                   (  1'b0                       ), // input

        // FB Clocks
        .Clk_C16                   ( Clk_C16                        ), // output
        .Clk_C16_Rst               ( Clk_C16_Rst                    ), // output
        .Clk_C21                   ( Clk_C21                        ), // output
        .Clk_C21_Rst               ( Clk_C21_Rst                    ), // output

        // Packet FIFO
        .Sys_PKfb_Clk              (  1'b0                          ), // input
        .Sys_PKfb_Rst              (                                ), // output
        .FB_PKfbData               ( 32'h0                          ), // input  [31:0]
        .FB_PKfbPush               (  4'h0                          ), // input   [3:0]
        .FB_PKfbSOF                (  1'b0                          ), // input
        .FB_PKfbEOF                (  1'b0                          ), // input
        .FB_PKfbOverflow           (                                ), // output

        // Sensor Interface
        .Sensor_Int                (                                ), // output  [7:0]
        .TimeStamp                 (                                ), // output [23:0]

        // SPI Master APB Bus
        .Sys_Pclk                  (                                ), // output
        .Sys_Pclk_Rst              (                                ), // output      <-- Fixed to add "_Rst"
        .Sys_PSel                  (  1'b0                          ), // input
        .SPIm_Paddr                ( 16'h0                          ), // input  [15:0]
        .SPIm_PEnable              (  1'b0                          ), // input
        .SPIm_PWrite               (  1'b0                          ), // input
        .SPIm_PWdata               ( 32'h0                          ), // input  [31:0]
        .SPIm_Prdata               (                                ), // output [31:0]
        .SPIm_PReady               (                                ), // output
        .SPIm_PSlvErr              (                                ), // output

        // Misc
        .Device_ID                 ( 16'h0001                       ), // input  [15:0]

        // FBIO Signals
        .FBIO_In                   (                                ), // output [13:0] <-- Do Not make any connections; Use Constraint manager in SpDE to sFBIO
        .FBIO_In_En                (                                ), // input  [13:0] <-- Do Not make any connections; Use Constraint manager in SpDE to sFBIO
        .FBIO_Out                  (                                ), // input  [13:0] <-- Do Not make any connections; Use Constraint manager in SpDE to sFBIO
        .FBIO_Out_En               (                                ), // input  [13:0] <-- Do Not make any connections; Use Constraint manager in SpDE to sFBIO

        // ???
        .SFBIO                     (                                ), // inout  [13:0]
        .Device_ID_6S              ( 1'b0                           ), // input
        .Device_ID_4S              ( 1'b0                           ), // input
        .SPIm_PWdata_26S           ( 1'b0                           ), // input
        .SPIm_PWdata_24S           ( 1'b0                           ), // input
        .SPIm_PWdata_14S           ( 1'b0                           ), // input
        .SPIm_PWdata_11S           ( 1'b0                           ), // input
        .SPIm_PWdata_0S            ( 1'b0                           ), // input
        .SPIm_Paddr_8S             ( 1'b0                           ), // input
        .SPIm_Paddr_6S             ( 1'b0                           ), // input
        .FB_PKfbPush_1S            ( 1'b0                           ), // input
        .FB_PKfbData_31S           ( 1'b0                           ), // input
        .FB_PKfbData_21S           ( 1'b0                           ), // input
        .FB_PKfbData_19S           ( 1'b0                           ), // input
        .FB_PKfbData_9S            ( 1'b0                           ), // input
        .FB_PKfbData_6S            ( 1'b0                           ), // input
        .Sys_PKfb_ClkS             ( 1'b0                           ), // input
        .FB_BusyS                  ( 1'b0                           ), // input
        .WB_CLKS                   ( 1'b0                           )  // input

    );

endmodule