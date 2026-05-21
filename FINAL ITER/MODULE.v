module lfsr_nl_modified_von_1_3M_V3(
    input         sysclk,   // 12MHz system clock input
    input  [1:0]  SW,       // SW[1]=reset(active high), SW[0]=enable
    input  [1:0]  btn,      // btn[0]=freeze display
    output [5:0]  HEX,      // 7-seg digit select (active low)
    output        DP,       // decimal point (unused, tied low)
    output [6:0]  SEG,      // 7-seg segment data (active low)
    output [1:0]  led,      // led[0]=enable indicator, led[1]=heartbeat
    output        tx        // UART transmit line (115200 baud)
);

    wire rstn   = ~SW[1];   // active-low reset: SW[1]=1 means reset asserted
    wire en     =  SW[0];   // enable signal: SW[0]=1 means RNG is enabled
    wire freeze =  btn[0];  // freeze display: btn[0]=1 holds current display value

    // --------------------------------------------------------
    // 1. XADC — internal temperature sensor entropy source
    // --------------------------------------------------------
    // XADC is Xilinx's built-in ADC — reads internal chip temperature
    // Temperature sensor produces thermally-noisy LSBs = physical entropy source
    wire [15:0] xadc_do;    // 16-bit ADC output data register
    wire        xadc_drdy;  // data ready strobe — pulses high when new sample available
    wire        xadc_eoc;   // end of conversion — pulses high when ADC conversion done

    // Pre-declare these here so LFSR block can reference update_tick
    // before it is defined later in section 5
    reg [23:0] update_ctr;  // 24-bit counter for output update rate timing
    reg        update_tick; // single-cycle pulse every 1.3M clocks (~10Hz at 12MHz)
    reg [15:0] display_word;// latched output word shown on 7-seg and sent via UART

    XADC #(
        // INIT_40: acquisition settings — continuous sampling mode
        .INIT_40(16'h9000),
        // INIT_41: alarm thresholds disabled, single channel mode
        .INIT_41(16'hff00),
        // INIT_42: DCLK divisor — sets ADC sample rate
        .INIT_42(16'h0400),
        // INIT_48: sequence register — enables temperature channel only
        .INIT_48(16'h0800),
        // INIT_49: auxiliary channel sequencer — all aux channels disabled
        .INIT_49(16'h0000)
    ) xadc_inst (
        .DCLK   (sysclk),   // drive ADC with system clock
        .DRDY   (xadc_drdy),// data ready output — goes high when DO is valid
        .DO     (xadc_do),  // 16-bit ADC result output
        .EOC    (xadc_eoc), // end of conversion output pulse
        .DEN    (xadc_eoc), // drive DEN with EOC — auto-read on each conversion
        .DADDR  (7'h00),    // address 0x00 = temperature sensor register
        .RESET  (~rstn),    // XADC reset is active high, so invert rstn
        // unused analog inputs tied off
        .VP(1'b0), .VN(1'b0), .VAUXP(16'h0), .VAUXN(16'h0),
        // unused conversion trigger inputs tied off
        .CONVST(1'b0), .CONVSTCLK(1'b0), .DI(16'h0), .DWE(1'b0)
    );

    // --------------------------------------------------------
    // 2. Von Neumann Corrector — removes bias from raw entropy bits
    // --------------------------------------------------------
    // Raw XADC bits may be biased (more 0s than 1s or vice versa)
    // Von Neumann correction: take pairs of bits
    //   pair = 01 → output 0  (unbiased)
    //   pair = 10 → output 1  (unbiased)
    //   pair = 00 → discard   (biased, skip)
    //   pair = 11 → discard   (biased, skip)
    // Result: clean_bit is guaranteed unbiased

    reg vn_state;   // FSM state: 0=waiting for first bit, 1=waiting for second bit
    reg vn_bit_a;   // stores first bit of the pair
    reg vn_valid;   // pulses high for 1 cycle when a valid clean_bit is ready
    reg clean_bit;  // the bias-corrected output bit from Von Neumann

    // XOR reduction of bottom 4 ADC bits — uses more thermal noise than just bit 0
    // ^xadc_do[3:0] = xadc_do[3] ^ xadc_do[2] ^ xadc_do[1] ^ xadc_do[0]
    wire raw_entropy_bit = ^xadc_do[3:0];

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) begin
            vn_state <= 1'b0;   // reset FSM to idle state
            vn_valid <= 1'b0;   // no valid output on reset
        end else if (xadc_drdy) begin
            // new ADC sample is ready — process it
            if (!vn_state) begin
                // STATE 0: capture first bit of the pair
                vn_bit_a <= raw_entropy_bit;  // store first bit
                vn_state <= 1'b1;             // move to state 1
                vn_valid <= 1'b0;             // no output yet
            end else begin
                // STATE 1: compare second bit against first
                vn_state <= 1'b0;             // return to idle regardless
                if (vn_bit_a != raw_entropy_bit) begin
                    // bits differ (01 or 10) — valid unbiased pair
                    clean_bit <= vn_bit_a;    // output first bit as clean entropy
                    vn_valid  <= 1'b1;        // signal that clean_bit is valid
                end else begin
                    // bits same (00 or 11) — biased pair, discard
                    vn_valid <= 1'b0;         // no output this cycle
                end
            end
        end else begin
            vn_valid <= 1'b0;  // no new ADC data — no valid output
        end
    end


    // --------------------------------------------------------
    // 3. 16-bit LFSR — pseudo-random generator with live entropy injection
    // --------------------------------------------------------
    // LFSR = Linear Feedback Shift Register
    // Generates a maximal-length pseudo-random sequence (period = 2^16 - 1 = 65535)
    // Tap positions {15,14,12,3} chosen for maximal-length primitive polynomial
    // Real entropy from Von Neumann injected continuously via improved_feedback

    reg [15:0] lfsr;  // 16-bit shift register — main RNG state

    // Pure LFSR feedback using tap positions only (no entropy yet)
    // These tap positions give maximal length sequence for 16-bit LFSR
    wire lfsr_feedback = lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3];

    // Improved feedback: XOR clean_bit into feedback ONLY when vn_valid is high
    // vn_valid=0 → uses pure LFSR taps (clean_bit not injected, avoids stale bit)
    // vn_valid=1 → XORs fresh clean_bit into feedback (live entropy injection)
    wire improved_feedback = lfsr_feedback ^ (vn_valid ? clean_bit : 1'b0);

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) begin
            lfsr <= 16'hACE1;  // non-zero seed (LFSR must never be all zeros)
        end else begin
            if (update_tick)
                // shift left by 1, insert improved_feedback at LSB
                // LFSR advances once per output tick (~10Hz)
                // entropy drips in continuously via improved_feedback
                lfsr <= {lfsr[14:0], improved_feedback};
        end
    end

    // --------------------------------------------------------
    // 3b. Free-running Galois LFSR — secondary entropy source
    // --------------------------------------------------------
    // Runs every clock cycle (much faster than main LFSR)
    // Galois form: feedback XOR applied to multiple tap positions simultaneously
    // tap mask 0xB400 = bits {15,14,12} — same polynomial as main LFSR
    // Replaces original free_ctr which was arithmetic (+31957) — detectable pattern
    // Galois LFSR has no arithmetic carries → no detectable structure

    reg [15:0] free_lfsr;       // 16-bit free-running Galois LFSR state
    wire free_fb = free_lfsr[0];// LSB is the feedback bit for Galois form

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) free_lfsr <= 16'hFFFF;  // seed must be non-zero
        else
            // Galois LFSR: shift right, conditionally XOR tap mask
            // free_fb=0 → plain right shift
            // free_fb=1 → right shift XOR 0xB400 (tap positions)
            free_lfsr <= {free_fb, free_lfsr[15:1]} 
                       ^ (free_fb ? 16'hB400 : 16'h0000);
    end

    // --------------------------------------------------------
    // 4. Whitening Functions — improve output uniformity
    // --------------------------------------------------------
    // xor_mix: multiplicative hash function (like Murmur/xxHash style)
    // Spreads bit patterns across all 16 bits — removes LFSR structural bias
    // Uses multiply + XOR-shift sequence for avalanche effect
    // (changing 1 input bit flips ~half the output bits)

    function [15:0] xor_mix;
        input [15:0] x;
        reg   [15:0] h;
        begin
            h = x ^ (x >> 7);       // XOR with right-shifted self
            h = h * 16'h9E37;       // multiply by prime constant — avalanche
            h = h ^ (h >> 9);       // XOR with right-shifted self again
            h = h * 16'h8445;       // second multiply by different prime
            xor_mix = h ^ (h >> 13);// final XOR-shift — output
        end
    endfunction

    // sbox: 4-bit substitution box — non-linear transformation
    // Breaks linear relationships in LFSR output
    // Each 4-bit nibble mapped to different 4-bit value (bijective/reversible)
    function [3:0] sbox;
        input [3:0] x;
        case (x)
            4'h0: sbox = 4'hE; 4'h1: sbox = 4'h4; // 0→14, 1→4
            4'h2: sbox = 4'hD; 4'h3: sbox = 4'h1; // 2→13, 3→1
            4'h4: sbox = 4'h2; 4'h5: sbox = 4'hF; // 4→2,  5→15
            4'h6: sbox = 4'hB; 4'h7: sbox = 4'h8; // 6→11, 7→8
            4'h8: sbox = 4'h3; 4'h9: sbox = 4'hA; // 8→3,  9→10
            4'hA: sbox = 4'h6; 4'hB: sbox = 4'hC; // 10→6, 11→12
            4'hC: sbox = 4'h5; 4'hD: sbox = 4'h9; // 12→5, 13→9
            4'hE: sbox = 4'h0; 4'hF: sbox = 4'h7; // 14→0, 15→7
        endcase
    endfunction

    // sbox_16: applies 4-bit sbox to all four nibbles of a 16-bit value
    // Each nibble processed independently through substitution
    function [15:0] sbox_16;
        input [15:0] x;
        sbox_16 = {sbox(x[15:12]),  // top nibble
                   sbox(x[11:8]),   // upper-mid nibble
                   sbox(x[7:4]),    // lower-mid nibble
                   sbox(x[3:0])};   // bottom nibble
    endfunction

    // --------------------------------------------------------
    // 5. Display & Update Logic — controls output rate
    // --------------------------------------------------------
    // update_ctr counts up to 1,299,999 then fires update_tick
    // At 12MHz: 1,300,000 cycles = ~9.23Hz update rate
    // update_tick is a single-cycle pulse used to advance LFSR and latch display

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) begin
            update_ctr  <= 24'd0;   // reset counter to zero
            update_tick <= 1'b0;    // no tick on reset
        end else if (update_ctr >= 24'd1299999) begin
            update_ctr  <= 24'd0;   // reset counter cleanly (no bias)
            update_tick <= 1'b1;    // fire single-cycle tick
        end else begin
            update_tick <= 1'b0;    // tick is only high for one cycle
            update_ctr  <= update_ctr + 1; // increment counter
        end
    end

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) display_word <= 16'hACE1; // reset display to known value
        else if (update_tick & ~freeze)
            // combine sbox-substituted LFSR and free_lfsr, then whiten
            // sbox_16 applies non-linear substitution to both LFSRs
            // addition mixes the two sources together
            // xor_mix applies final avalanche whitening
            display_word <= xor_mix(sbox_16(lfsr) + sbox_16(free_lfsr));
    end

    // Split display_word into 4 hex nibbles for 7-seg display
    wire [3:0] digit3 = display_word[15:12]; // most significant nibble
    wire [3:0] digit2 = display_word[11:8];  // second nibble
    wire [3:0] digit1 = display_word[7:4];   // third nibble
    wire [3:0] digit0 = display_word[3:0];   // least significant nibble

    // --------------------------------------------------------
    // 6. 7-Segment Display Multiplexer (~1kHz refresh rate)
    // --------------------------------------------------------
    // Only one digit active at a time — cycle through all 4 rapidly
    // At 12MHz: 12000 cycles per digit = 1kHz per digit = 250Hz full refresh
    // Fast enough that human eye sees all 4 digits simultaneously

    reg [13:0] mux_ctr;    // counter for digit refresh timing
    reg [1:0]  digit_sel;  // selects which digit is currently active (0-3)

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) begin
            mux_ctr   <= 14'd0;  // reset refresh counter
            digit_sel <= 2'd0;   // start at digit 0
        end else if (mux_ctr == 14'd11999) begin
            mux_ctr   <= 14'd0;          // reset counter
            digit_sel <= digit_sel + 1;  // advance to next digit (wraps 3→0)
        end else begin
            mux_ctr <= mux_ctr + 1;      // increment refresh counter
        end
    end

    // hex2seg: converts 4-bit hex value to 7-segment encoding
    // Output is active-high internally, inverted on output assign below
    function [6:0] hex2seg;
        input [3:0] d;
        case (d)
            4'h0: hex2seg = 7'b1000000; // 0
            4'h1: hex2seg = 7'b1111001; // 1
            4'h2: hex2seg = 7'b0100100; // 2
            4'h3: hex2seg = 7'b0110000; // 3
            4'h4: hex2seg = 7'b0011001; // 4
            4'h5: hex2seg = 7'b0010010; // 5
            4'h6: hex2seg = 7'b0000010; // 6
            4'h7: hex2seg = 7'b1111000; // 7
            4'h8: hex2seg = 7'b0000000; // 8
            4'h9: hex2seg = 7'b0010000; // 9
            4'hA: hex2seg = 7'b0001000; // A
            4'hB: hex2seg = 7'b0000011; // B
            4'hC: hex2seg = 7'b1000110; // C
            4'hD: hex2seg = 7'b0100001; // D
            4'hE: hex2seg = 7'b0000110; // E
            4'hF: hex2seg = 7'b0001110; // F
        endcase
    endfunction

    // Select which nibble to display based on current digit_sel
    reg [3:0] cur_nibble;
    always @(*) begin
        case (digit_sel)
            2'd0: cur_nibble = digit3; // leftmost digit = most significant nibble
            2'd1: cur_nibble = digit2;
            2'd2: cur_nibble = digit1;
            2'd3: cur_nibble = digit0; // rightmost digit = least significant nibble
        endcase
    end

    // Drive 7-seg outputs — all active low so invert
    assign SEG    = ~hex2seg(cur_nibble); // segment data for current digit
    assign HEX[3] = ~(digit_sel == 2'd0); // enable digit 3 (leftmost)
    assign HEX[2] = ~(digit_sel == 2'd1); // enable digit 2
    assign HEX[1] = ~(digit_sel == 2'd2); // enable digit 1
    assign HEX[0] = ~(digit_sel == 2'd3); // enable digit 0 (rightmost)
    assign HEX[5:4] = 2'b11;              // upper 2 digits disabled (unused)
    assign DP = 1'b0;                      // decimal point always off

    // --------------------------------------------------------
    // 7. LED Indicators
    // --------------------------------------------------------
    reg [23:0] hb_ctr;      // 24-bit heartbeat counter
    reg        led0_reg;    // register for led[0]
    reg        led1_reg;    // register for led[1]

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) begin
            hb_ctr   <= 24'd0;  // reset heartbeat counter
            led0_reg <= 1'b0;   // led0 off on reset
            led1_reg <= 1'b0;   // led1 off on reset
        end else begin
            hb_ctr   <= hb_ctr + 1;  // increment every clock cycle
            led0_reg <= en;           // led[0] mirrors SW[0] enable switch
            led1_reg <= hb_ctr[23];   // led[1] toggles at 12MHz/2^24 = ~0.7Hz heartbeat
                                      // solid blink = clock running normally
        end
    end
    assign led[0] = led0_reg; // shows enable state
    assign led[1] = led1_reg; // shows clock heartbeat

    // --------------------------------------------------------
    // 8. UART Transmitter — 115200 baud, 8N1
    // --------------------------------------------------------
    // Sends display_word as 4 ASCII hex chars + CR + LF every update_tick
    // Format: "XXXX\r\n" where XXXX is the 16-bit hex value
    // At 115200 baud, each bit = 1/115200 = 8.68us
    // CLKS_PER_BIT = 12MHz / 115200 = 104.16 ≈ 104 cycles per bit

    localparam CLKS_PER_BIT = 104; // clock cycles per UART bit period

    // nibble2ascii: converts 4-bit hex digit to ASCII character
    // 0-9 → ASCII 48-57 ('0'-'9')
    // A-F → ASCII 65-70 ('A'-'F')
    function [7:0] nibble2ascii;
        input [3:0] n;
        nibble2ascii = (n < 4'd10) ? (8'd48 + n) : (8'd55 + n);
    endfunction

    reg [7:0]  tx_buf [0:5];    // 6-byte transmit buffer: 4 hex chars + CR + LF
    reg [2:0]  tx_byte_idx;     // index of current byte being transmitted (0-5)
    reg [3:0]  tx_bit_idx;      // index of current bit being transmitted (0-9)
                                 // 0=start bit, 1-8=data bits, 9=stop bit
    reg [6:0]  tx_clk_ctr;      // clock cycle counter within current bit period
    reg        tx_busy;          // high while transmission is in progress
    reg        tx_reg;           // UART TX line register

    assign tx = tx_reg;          // connect register to output pin

    always @(posedge sysclk or negedge rstn) begin
        if (!rstn) begin
            tx_busy     <= 1'b0;  // not transmitting on reset
            tx_reg      <= 1'b1;  // UART idle state is high
            tx_byte_idx <= 3'd0;
            tx_bit_idx  <= 4'd0;
            tx_clk_ctr  <= 7'd0;
        end else begin

            // Load new data when update_tick fires and TX is not busy
            if (update_tick && !tx_busy) begin
                // convert each nibble to ASCII and load into buffer
                tx_buf[0] <= nibble2ascii(display_word[15:12]); // MSN → ASCII
                tx_buf[1] <= nibble2ascii(display_word[11:8]);
                tx_buf[2] <= nibble2ascii(display_word[7:4]);
                tx_buf[3] <= nibble2ascii(display_word[3:0]);   // LSN → ASCII
                tx_buf[4] <= 8'h0D; // carriage return (CR)
                tx_buf[5] <= 8'h0A; // line feed (LF)
                tx_busy     <= 1'b1; // mark transmitter as busy
                tx_byte_idx <= 3'd0; // start from first byte
                tx_bit_idx  <= 4'd0; // start from first bit (start bit)
                tx_clk_ctr  <= 7'd0; // reset bit timer
            end

            // Transmit state machine — runs while busy
            if (tx_busy) begin
                if (tx_clk_ctr < CLKS_PER_BIT - 1) begin
                    tx_clk_ctr <= tx_clk_ctr + 1; // wait for full bit period
                end else begin
                    tx_clk_ctr <= 7'd0;  // reset bit timer for next bit
                    case (tx_bit_idx)
                        4'd0: tx_reg <= 1'b0; // start bit — always 0
                        // data bits 0-7 sent LSB first (UART standard)
                        4'd1: tx_reg <= tx_buf[tx_byte_idx][0]; // bit 0 (LSB)
                        4'd2: tx_reg <= tx_buf[tx_byte_idx][1]; // bit 1
                        4'd3: tx_reg <= tx_buf[tx_byte_idx][2]; // bit 2
                        4'd4: tx_reg <= tx_buf[tx_byte_idx][3]; // bit 3
                        4'd5: tx_reg <= tx_buf[tx_byte_idx][4]; // bit 4
                        4'd6: tx_reg <= tx_buf[tx_byte_idx][5]; // bit 5
                        4'd7: tx_reg <= tx_buf[tx_byte_idx][6]; // bit 6
                        4'd8: tx_reg <= tx_buf[tx_byte_idx][7]; // bit 7 (MSB)
                        4'd9: begin
                            tx_reg <= 1'b1; // stop bit — always 1
                            if (tx_byte_idx == 3'd5) begin
                                tx_busy <= 1'b0; // all 6 bytes sent — done
                            end else begin
                                // move to next byte
                                tx_byte_idx <= tx_byte_idx + 1;
                                tx_bit_idx  <= 4'd0; // restart from start bit
                            end
                        end
                    endcase
                    // advance bit index unless we just sent stop bit
                    if (tx_bit_idx < 4'd9) tx_bit_idx <= tx_bit_idx + 1;
                end
            end

        end
    end

endmodule
