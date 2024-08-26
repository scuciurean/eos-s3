`define AD7984_REG_CFG 8'h0
`define AD7984_REG_DATA 8'h4
`define AD7984_REG_BUFF_NSAMPLES 8'h8

module ad7984 #(
    parameter NUM_BITS = 16,
    parameter MUX_ADDR_WIDHT = 9,
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 32
)(
    input wire CLK,
    input wire RST,

    input  wire                  WBS_CYC,                   // Wishbone Cycle signal
    input  wire                  WBS_WE,                    // Wishbone Write Enable signal
    input  wire                  WBS_RD,
    input  wire [DATA_WIDTH-1:0] WBS_WR_DAT,                // Wishbone Write Data
    input  wire [3:0]            WBS_BYTE_STB,              // Wishbone Byte Enable Strobe
    input  wire                  WBS_STB,
    input  wire [ADDR_WIDTH - MUX_ADDR_WIDHT-1:0] WBS_ADR,  // Wishbone Address : 4 registers
    input  wire                  WB_CLK,                    // Wishbone Clock
    input  wire                  WB_RST,                    // Wishbone Reset
    output reg  [DATA_WIDTH-1:0] WBS_RD_DAT,                // Wishbone Read Data
    output reg                   WBS_ACK,                   // Wishbone Acknowledge

    input wire SIG_CNV,
    input wire SIG_SCK,

    output reg [NUM_BITS-1:0] ADC_DATA,
    output wire ADC_DATA_VALID,

    output wire CNV,
    output wire SCK,
    input wire SDO
);
    localparam  CONVERSION  = 1'b0,
                ACQUISITION = 1'b1;

    reg [31:0] reg_control;
    reg [NUM_BITS-1:0] sdo_data;
    reg [31:0] reg_num_samples;
    reg [31:0] sample_count;

    wire sig_enabled;
    reg enabled;
    wire buffer_enable;
    wire [1:0] state;
    reg [7:0] sck_count;

    assign sig_enabled = reg_control[0];
    assign buffer_enable = reg_control[1];
    assign CNV = SIG_CNV && enabled;
    // sck_count > 1 for tEN
    assign SCK = SIG_SCK && ~SIG_CNV && 1 < sck_count && sck_count <= (NUM_BITS + 1) && enabled;
    assign state = CNV ? CONVERSION : ACQUISITION;
    assign ADC_DATA_VALID = CNV && sample_count > 0 && sck_count == 0;

    initial begin
        sample_count = 0;
        enabled = 0;
        sck_count = 0;
        sdo_data = 0;
        ADC_DATA = 0;
    end

    always @(posedge CLK) begin
        if (sig_enabled) begin
            if (sck_count == 0 && SIG_CNV) begin
                enabled = 1;
            end
        end else begin
            // ~SIG_CNV so the CNV pulse won't be cut after first clock
            if (sck_count == 0 && ~SIG_CNV) begin
                enabled = 0;
            end
        end
    end

    // always @(posedge SIG_SCK) begin
    always @(negedge SIG_SCK) begin
        if (~SIG_CNV && 1 < sck_count && sck_count <= (NUM_BITS + 1) && enabled) begin
        // if (~SIG_CNV && 0 < sck_count && sck_count <= (NUM_BITS + 1) && enabled) begin
            sdo_data <= {sdo_data[14:0], SDO};
            // sdo_data <= {SDO, sdo_data[15:1]};
        end
    end

    // always @(posedge SCK) begin
    //     sdo_data <= {sdo_data[14:0], SDO};
    // end

    always @(posedge SIG_CNV) begin
        if (enabled) begin
            ADC_DATA[15:0] = sdo_data[15:0];
            if (sample_count == reg_num_samples) begin
                sample_count <= 0;
            end else begin
                sample_count <= sample_count + 1;
            end
        end else begin
            sample_count <= 0;
        end
    end

    always @(posedge SIG_SCK) begin
        if (enabled && ~SIG_CNV) begin
            sck_count <= sck_count + 1;
        end else begin
            sck_count <= 0;
        end
    end

    // Handle Reset and Transactions
    always @(posedge WB_CLK or posedge WB_RST) begin
        if (WB_RST) begin
            WBS_ACK <= 0;
            WBS_RD_DAT <= 32'h0;
            reg_control <= 32'h0;
            reg_num_samples <= 32'h0;
        end else begin
            if (enabled && buffer_enable) begin
                if (sample_count == reg_num_samples) begin
                    reg_control[1] <= 0;
                    reg_control[0] <= 0;
                end
            end
            if (WBS_CYC && WBS_STB && ~WBS_ACK) begin  // Write operation
                if (WBS_WE) begin
                    case (WBS_ADR)
                        `AD7984_REG_CFG: begin
                            if (WBS_BYTE_STB[0]) reg_control[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) reg_control[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) reg_control[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) reg_control[31:23] <= WBS_WR_DAT[31:23];
                        end
                        `AD7984_REG_BUFF_NSAMPLES: begin
                            if (WBS_BYTE_STB[0]) reg_num_samples[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) reg_num_samples[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) reg_num_samples[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) reg_num_samples[31:23] <= WBS_WR_DAT[31:23];
                        end
                    endcase
                end
                if (~WBS_WE && WBS_RD) begin
                    case (WBS_ADR)
                        `AD7984_REG_CFG: begin
                            WBS_RD_DAT <= reg_control;
                        end
                        `AD7984_REG_DATA: begin
                            WBS_RD_DAT <= {16'h0, ADC_DATA};
                        end
                        `AD7984_REG_BUFF_NSAMPLES: begin
                            WBS_RD_DAT <= reg_num_samples;
                        end

                        default: begin
                            WBS_RD_DAT <= 32'hDEAD_DEAD;
                        end
                    endcase
                end
                WBS_ACK <= 1;
            end else begin
                WBS_ACK <= 0;
            end
        end
    end

endmodule