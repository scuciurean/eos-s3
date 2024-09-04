`define SCOPE_REG_CONFIG  8'h00
`define SCOPE_REG_CHANNELS  8'h04
`define SCOPE_REG_BUFFER  8'h08
`define SCOPE_REG_NUM_SAMPLES 8'h0C

module wishbone_scope #(
    parameter MUX_ADDR_WIDTH = 9,
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 32,
    parameter NUM_PROBES = 3,
    parameter PROBE_DATA_WIDTH = 1
)(
    input wire CLK,
    input wire RST,

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

    input wire TRIGGER,
    input wire [NUM_PROBES-1:0] IN_PORTS,

    output reg [8:0] RAM_WR_ADDR,
    output reg [31:0] RAM_WR_DATA,
    output reg [3:0] RAM_WR_EN,
    output reg RAM_WR_CLK_EN
);

    reg [31:0] sample_index;

    reg [31:0] reg_config;
    reg [31:0] reg_channels;
    reg [31:0] reg_buffer;
    reg [31:0] reg_num_samples;
    wire signal_enabled;
    reg enabled;

    reg [NUM_PROBES-1:0] probes_enabled;

    assign signal_enabled = reg_config[0];

    initial begin
        sample_index = 32'h0;
        reg_config = 32'h0;
        reg_channels = 32'h0;
        reg_buffer = 32'h0;
        reg_num_samples = 32'h0;
        enabled = 0;
        RAM_WR_CLK_EN = 0;
        RAM_WR_EN = 4'b0000;
    end

    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            sample_index = 0;
            enabled = 0;
            RAM_WR_CLK_EN = 0;
            RAM_WR_EN = 4'b0000;
        end else if (signal_enabled) begin
            RAM_WR_CLK_EN = 1;
            if (sample_index < reg_num_samples) begin        // Reset the index after all samples are written
                enabled = 1;
                RAM_WR_DATA[31:NUM_PROBES] <= 0;
                RAM_WR_DATA[NUM_PROBES-1:0] <= IN_PORTS;
                RAM_WR_ADDR = sample_index[8:0] * 4;
                RAM_WR_EN = 4'b1111;           // Enable write for all byte lanes
                sample_index = sample_index + 1;
            end else begin
                enabled = 0;
                RAM_WR_EN = 4'b0000;           // Disable writing after sampling is done
                RAM_WR_CLK_EN = 0;             // Optionally disable the write clock enable once done
            end
        end else begin
            sample_index = 0;
        end
    end

    always @(posedge WB_CLK or posedge WB_RST) begin
        if (WB_RST) begin
            WBS_ACK <= 0;
            WBS_RD_DAT <= 32'h0;
            reg_config <= 32'h0;
            reg_channels <= 32'h0;
            reg_buffer <= 32'h0;
        end else begin
            // Reg control
            if (!enabled && sample_index == reg_num_samples) begin
                reg_config[0] = 0;
            end
            // WB transfer
            if (WBS_CYC && WBS_STB && ~WBS_ACK) begin
                if (WBS_WE) begin
                    case (WBS_ADR)
                        `SCOPE_REG_CONFIG: begin
                            if (WBS_BYTE_STB[0]) reg_config[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) reg_config[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) reg_config[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) reg_config[31:23] <= WBS_WR_DAT[31:23];
                        end
                        `SCOPE_REG_CHANNELS: begin
                            if (WBS_BYTE_STB[0]) reg_channels[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) reg_channels[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) reg_channels[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) reg_channels[31:23] <= WBS_WR_DAT[31:23];
                        end
                        `SCOPE_REG_BUFFER: begin
                            if (WBS_BYTE_STB[0]) reg_buffer[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) reg_buffer[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) reg_buffer[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) reg_buffer[31:23] <= WBS_WR_DAT[31:23];
                        end
                        `SCOPE_REG_NUM_SAMPLES: begin
                            if (WBS_BYTE_STB[0]) reg_num_samples[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) reg_num_samples[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) reg_num_samples[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) reg_num_samples[31:23] <= WBS_WR_DAT[31:23];
                        end
                    endcase
                end
                if (~WBS_WE && WBS_RD) begin
                    case (WBS_ADR)
                        `SCOPE_REG_CONFIG: begin
                            WBS_RD_DAT <= reg_config;
                        end
                        `SCOPE_REG_CHANNELS: begin
                            WBS_RD_DAT <= reg_channels;
                        end
                        `SCOPE_REG_BUFFER: begin
                            WBS_RD_DAT <= reg_buffer;
                        end
                        `SCOPE_REG_NUM_SAMPLES: begin
                            WBS_RD_DAT <= reg_num_samples;
                        end
                        default: begin
                            WBS_RD_DAT <= 32'h0;
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