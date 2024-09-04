`define GPIO_REG0 8'h000  // Address for Register 0 (used)
`define GPIO_REG1 8'h004  // Address for Register 1 (used)
`define GPIO_REG2 8'h008  // Address for Register 2 (ignored)
`define GPIO_REG3 8'h010  // Address for Register 3 (ignored)

module wishbone_gpio #(
    parameter MUX_ADDR_WIDTH = 9,
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 32,
)(
    input  wire                  WBS_CYC,                   // Wishbone Cycle signal
    input  wire                  WBS_WE,                    // Wishbone Write Enable signal
    input  wire                  WBS_RD,
    input  wire [DATA_WIDTH-1:0] WBS_WR_DAT,                // Wishbone Write Data
    input  wire [3:0]            WBS_BYTE_STB,              // Wishbone Byte Enable Strobe
    input  wire                  WBS_STB,
    input  wire [ADDR_WIDTH - MUX_ADDR_WIDTH-1:0] WBS_ADR,  // Wishbone Address : 4 registers
    input  wire                  WB_CLK,                    // Wishbone Clock
    input  wire                  WB_RST,                    // Wishbone Reset
    output reg  [DATA_WIDTH-1:0] WBS_RD_DAT,                // Wishbone Read Data
    output reg                  WBS_ACK,                   // Wishbone Acknowledge

    inout  wire [31:0]            GPIO_IO                    // 8-bit GPIO Interface
);

    // Internal Register for GPIO states
    reg [31:0] gpio_data_reg;

    // Assign output to the GPIO pins
    assign GPIO_IO = gpio_data_reg;

    // Handle Reset and Transactions
    always @(posedge WB_CLK or posedge WB_RST) begin
        if (WB_RST) begin
            WBS_ACK <= 0;
            WBS_RD_DAT <= 32'h0;
            gpio_data_reg <= 32'b0;   // Reset the GPIO register
        end else begin
            if (WBS_CYC && WBS_STB && ~WBS_ACK) begin  // Write operation
                if (WBS_WE) begin
                    case (WBS_ADR)
                        `GPIO_REG0: begin
                            if (WBS_BYTE_STB[0]) gpio_data_reg[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) gpio_data_reg[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) gpio_data_reg[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) gpio_data_reg[31:24] <= WBS_WR_DAT[31:24];
                        end
                        `GPIO_REG1: begin
                            if (WBS_BYTE_STB[0]) gpio_data_reg[7:0] <= ~WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) gpio_data_reg[15:8] <= ~WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) gpio_data_reg[23:16] <= ~WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) gpio_data_reg[31:23] <= ~WBS_WR_DAT[31:23];
                        end
                    endcase
                end
                if (~WBS_WE && WBS_RD) begin
                    case (WBS_ADR)
                        `GPIO_REG0: begin
                            WBS_RD_DAT <= gpio_data_reg;  // Return the current state of the GPIO register
                        end
                        `GPIO_REG1: begin
                            WBS_RD_DAT <= ~gpio_data_reg;
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
