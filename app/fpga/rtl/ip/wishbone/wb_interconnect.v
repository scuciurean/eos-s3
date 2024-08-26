
`timescale 1ns / 10ps

module wishbone_interconnect #(
    parameter NUM_SLAVES = 1,
    parameter MUX_ADDR_WIDHT = 9,
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 32
)(
    input  wire [ADDR_WIDTH-1:0] WB_ADR,                   // Wishbone Address Bus
    input  wire                  WB_CYC,                   // Wishbone Client Cycle Strobe
    input  wire                  WB_WE,                    // Wishbone Write Enable Strobe
    input  wire                  WB_RD,
    input  wire [DATA_WIDTH-1:0] WB_WR_DAT,                // Wishbone Write Data Bus
    input  wire                  WB_STB,                   // Wishbone Strobe signal
    input  wire [3:0]            WB_BYTE_STB,              // Wishbone Byte Enable Strobe
    input  wire                  WB_CLK,                   // Wishbone Clock
    input  wire                  WB_RST,                   // Wishbone Reset
    output reg  [DATA_WIDTH-1:0] WB_RD_DAT,                // Wishbone Read Data Bus
    output reg                   WB_ACK,                   // Wishbone Acknowledge

    // Slave Interface Signals
    output wire [NUM_SLAVES-1:0] WBS_CYC,                  // Cycle signals for slaves
    output wire [NUM_SLAVES-1:0] WBS_WE,                   // Write enable signals for slaves
    output wire [NUM_SLAVES-1:0] WBS_RD,
    output wire [DATA_WIDTH-1:0] WBS_WR_DAT,               // Write data bus for slaves
    output wire [ADDR_WIDTH-MUX_ADDR_WIDHT-1:0] WBS_ADR,                  // Address bus for slaves
    output wire [NUM_SLAVES-1:0] WBS_STB,
    output wire [3:0]            WBS_BYTE_STB,             // Byte strobe for slaves
    input  wire [NUM_SLAVES*DATA_WIDTH-1:0] WBS_RD_DAT, // Flattened data buses from slaves
    input  wire [NUM_SLAVES-1:0] WBS_ACK,                  // Acknowledge signals from slaves
    input  wire [NUM_SLAVES*ADDR_WIDTH-1:0] SLAVE_BASE_ADDRESSES // Flattened base addresses
);
    reg [NUM_SLAVES-1:0] active_slave;

    // Manually reconstruct the array elements
    wire [DATA_WIDTH-1:0] WBS_RD_DAT_EXPANDED [NUM_SLAVES-1:0];
    wire [ADDR_WIDTH-1:0] SLAVE_BASE_ADDRESSES_EXPANDED [NUM_SLAVES-1:0];

   // Generate block to expand the flattened arrays into 2D arrays
    genvar idx;
    generate
        for (idx = 0; idx < NUM_SLAVES; idx = idx + 1) begin : EXPAND_ARRAYS
            assign WBS_RD_DAT_EXPANDED[idx] = WBS_RD_DAT[(idx+1)*DATA_WIDTH-1:idx*DATA_WIDTH];
            assign SLAVE_BASE_ADDRESSES_EXPANDED[idx] = SLAVE_BASE_ADDRESSES[(idx+1)*ADDR_WIDTH-1:idx*ADDR_WIDTH];
        end
    endgenerate

    // Address decoding and CYC signal assignment
    integer i;  // Declare integer for procedural loop
    always @(*) begin
        active_slave = 0;
        // WB_RD_DAT = 32'hDEAD_DEAD; // Default read data for invalid accesses
        WB_ACK = 0; // Default to no acknowledgment

        // Iterate over all slaves to check address match
        for (i = 0; i < NUM_SLAVES; i = i + 1) begin
            if (WB_ADR[ADDR_WIDTH-1:MUX_ADDR_WIDHT-1] == SLAVE_BASE_ADDRESSES_EXPANDED[i][ADDR_WIDTH-1:MUX_ADDR_WIDHT-1]) begin
                active_slave[i] = 1;
                WB_RD_DAT = WBS_RD_DAT_EXPANDED[i]; // Use valid read data from active slave
                WB_ACK = WBS_ACK[i];                // Acknowledge from the active slave
            end
        end
    end

    assign WBS_CYC = active_slave & {NUM_SLAVES{WB_CYC}};
    assign WBS_WE = active_slave & {NUM_SLAVES{WB_WE}};
    assign WBS_RD = active_slave & {NUM_SLAVES{WB_RD}};
    assign WBS_STB = active_slave & {NUM_SLAVES{WB_STB}};
    assign WBS_WR_DAT = WB_WR_DAT;
    assign WBS_ADR = WB_ADR[ADDR_WIDTH-MUX_ADDR_WIDHT-1:0];
    assign WBS_BYTE_STB = WB_BYTE_STB;


endmodule

