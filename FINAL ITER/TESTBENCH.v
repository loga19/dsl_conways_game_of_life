`timescale 1ns / 1ps

module tb_lfsr();
    reg sysclk;
    reg [1:0] SW;
    reg [1:0] btn;
    wire [5:0] HEX;
    wire [6:0] SEG;
    wire [1:0] led;
    wire tx;

    // Instantiate your TRNG module
    lfsr_nl_modified_von_1_3M_V3 uut (
        .sysclk(sysclk), .SW(SW), .btn(btn),
        .HEX(HEX), .SEG(SEG), .led(led), .tx(tx)
    );

    // Generate 12MHz Clock
    initial sysclk = 0;
    always #41.66 sysclk = ~sysclk;

    initial begin
        // 1. Start in Reset
        SW = 2'b10; btn = 2'b00;
        #200;
        
        // 2. Release Reset & Enable
        SW = 2'b01; 
        #5000; // Run for a bit to see the LFSR shift
        
        $finish;
    end
endmodule
