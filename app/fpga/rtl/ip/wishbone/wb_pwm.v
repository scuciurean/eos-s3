`define PWM_REG_CONFIG  8'h00
`define PWM_REG_DUTY    8'h04
`define PWM_REG_PERIOD  8'h08


module wishbone_pwm #(
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
    output reg                  WBS_ACK,                   // Wishbone Acknowledge

    output reg                 PWM_OUT
);
    reg [31:0] counter; // Counter for the PWM
    reg [31:0] period;
    reg [31:0] duty_cycle;
    reg [31:0] cfg;


    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            counter <= 32'h0;
            PWM_OUT <= 1'b0;
        end else begin
            if (cfg[0]) begin  // Bit 0: PWM Enable

                if (counter < period - 1) begin
                    counter <= counter + 1;
                end else begin
                    counter <= 32'b0;
                end

                if (counter < duty_cycle) begin
                    PWM_OUT <= 1'b1;  // Set PWM high if counter is less than duty cycle
                end else begin
                    PWM_OUT <= 1'b0;  // Set PWM low otherwise
                end
            end else begin
                counter <= 32'b0;
                PWM_OUT <= 1'b0;
            end
        end
    end

    // Invert the output if Bit 1 of config is set
    // always @(posedge CLK or negedge RST) begin
    //     if (!RST) begin
    //         PWM_OUT <= 1'b0;
    //     end else begin
    //         if (cfg[1]) begin  // Bit 1: Invert Output
    //             PWM_OUT <= ~PWM_OUT;
    //         end
    //     end
    // end

    // Handle Reset and Transactions
    always @(posedge WB_CLK or posedge WB_RST) begin
        if (WB_RST) begin
            WBS_ACK <= 0;
            WBS_RD_DAT <= 32'h0;
            period <= 32'h0;
            duty_cycle <= 32'h0;
            cfg <= 0;
        end else begin
            if (WBS_CYC && WBS_STB && ~WBS_ACK) begin  // Write operation
                if (WBS_WE) begin
                    case (WBS_ADR)
                        `PWM_REG_CONFIG: begin
                            if (WBS_BYTE_STB[0]) cfg[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) cfg[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) cfg[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) cfg[31:23] <= WBS_WR_DAT[31:23];
                        end
                        `PWM_REG_DUTY: begin
                            if (WBS_BYTE_STB[0]) duty_cycle[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) duty_cycle[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) duty_cycle[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) duty_cycle[31:23] <= WBS_WR_DAT[31:23];
                        end
                        `PWM_REG_PERIOD: begin
                            if (WBS_BYTE_STB[0]) period[7:0] <= WBS_WR_DAT[7:0];
                            if (WBS_BYTE_STB[1]) period[15:8] <= WBS_WR_DAT[15:8];
                            if (WBS_BYTE_STB[2]) period[23:16] <= WBS_WR_DAT[23:16];
                            if (WBS_BYTE_STB[3]) period[31:23] <= WBS_WR_DAT[31:23];
                        end
                    endcase
                end
                if (~WBS_WE && WBS_RD) begin
                    case (WBS_ADR)
                        `PWM_REG_CONFIG: begin
                            WBS_RD_DAT <= cfg;  // Return the current state of the GPIO register
                        end
                        `PWM_REG_DUTY: begin
                            WBS_RD_DAT <= duty_cycle;  // Return the current state of the GPIO register
                        end
                        `PWM_REG_PERIOD: begin
                            WBS_RD_DAT <= period;  // Return the current state of the GPIO register
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