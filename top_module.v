//   Basic Wire Problem

module top_module( input in, output out );
assign out = in;
endmodule

module top_module( 
    input a,b,c,
    output w,x,y,z );
    assign w = a;
    assign  z = c;
    assign x = b;
    assign y = b;

endmodule

module top_module( input in, output out );
assign out = !in;
endmodule





module top_module( 
    input a, 
    input b, 
    output out );
    assign out = !((a*!(b))+(b*!(a)));
    

endmodule

default_nettype none
module top_module(
    input a,
    input b,
    input c,
    input d,
    output out,
    output out_n   ); 
    wire and1_in;
    wire and2_in;
    assign and1_in = a&b;
    assign and2_in = c&d;
    assign out = and1_in | and2_in;
    assign out_n = ~out;
    
endmodule

module top_module ( 
    input p1a, p1b, p1c, p1d, p1e, p1f,
    output p1y,
    input p2a, p2b, p2c, p2d,
    output p2y );
    assign p1y = (p1a&p1b&p1c)|(p1d&p1e&p1f);
    assign p2y = (p2a&p2b)|(p2c&p2d);


endmodule

// VECTORS Problem

module top_module ( 
    input wire [2:0] vec,
    output wire [2:0] outv,
    output wire o2,
    output wire o1,
    output wire o0  ); // Module body starts after module declaration
 assign outv = vec;
    assign o0 = vec[0];
    assign o1 = vec[1];
    assign o2 = vec[2];
    
endmodule

default_nettype none     // Disable implicit nets. Reduces some types of bugs.
module top_module( 
    input wire [15:0] in,
    output wire [7:0] out_hi,
    output wire [7:0] out_lo );
    assign out_lo = in[7:0];
    assign out_hi = in[15:8];

endmodule

module top_module( 
    input [31:0] in,
    output [31:0] out );//
    assign out[31:24] = in[7:0];
    assign out[23:16] = in[15:8];
    assign out[15:8] = in[23:16];
    assign out[7:0] = in[31:24];


endmodule

module top_module( 
    input [2:0] a,
    input [2:0] b,
    output [2:0] out_or_bitwise,
    output out_or_logical,
    output [5:0] out_not
);
    assign out_or_bitwise =  a | b;
     assign out_or_logical = a || b;
    assign out_not =    {~b,~a}; 

endmodule

module top_module( 
    input [3:0] in,
    output out_and,
    output out_or,
    output out_xor
);
    assign out_and = &in;
     assign out_or = |in;
     assign out_xor = ^in;

endmodule

module top_module (
    input [4:0] a, b, c, d, e, f,
    output [7:0] w, x, y, z );//

    wire [31:0] concat_bits;
    assign concat_bits = {a,b,c,d,e,f,2'b11};
    assign w = concat_bits[31:24];
    assign x = concat_bits[23:16];
    assign y = concat_bits[15:8];
    assign z = concat_bits[7:0];

endmodule

module top_module( 
    input [7:0] in,
    output [7:0] out
);
    assign out = {in[0],in[1],in[2],in[3],in[4],in[5],in[6],in[7]};
endmodule

module top_module (
    input [7:0] in,
    output [31:0] out );//

    assign out = { {24{in[7]}},in};

endmodule

module top_module (
    input a, b, c, d, e,
    output [24:0] out );//

    // The output is XNOR of two vectors created by 
    // concatenating and replicating the five inputs.
    assign out = ~{ {5{a}}, {5{b}}, {5{c}}, {5{d}}, {5{e}} } ^ {5{a,b,c,d,e}};


endmodule

//MODULES 
module top_module ( input a, input b, output out );
    mod_a instace1( a,b,out);

endmodule


module top_module ( 
    input a, 
    input b, 
    input c,
    input d,
    output out1,
    output out2
);
    mod_a instance1(out1,out2,a,b,c,d);
endmodule

module top_module ( 
    input a, 
    input b, 
    input c,
    input d,
    output out1,
    output out2
);
    mod_a instance1(.out1(out1),.out2(out2),.in1(a),.in2(b),.in3(c),.in4(d));
endmodule

module top_module ( input clk, input d, output q );
     wire q1, q2;

    my_dff d1 (clk, d, q1);
    my_dff d2 (clk, q1, q2);
    my_dff d3 (clk, q2, q);

endmodule

module top_module ( 
    input clk, 
    input [7:0] d, 
    input [1:0] sel, 
    output [7:0] q 
);
    wire [7:0] q1, q2, q3;

    my_dff8 dff1 (clk, d, q1);
    my_dff8 dff2 (clk, q1, q2);
    my_dff8 dff3 (clk, q2, q3);

    always @(*) begin
        case (sel)
            2'b00: q = d;
            2'b01: q = q1;
            2'b10: q = q2;
            2'b11: q = q3;
        endcase
    end

endmodule

// ADDER Problem

module top_module(
    input [31:0] a,
    input [31:0] b,
    output [31:0] sum
);
    wire cout;

    // First 16-bit adder: lower bits
    add16 add_low (
        .a(a[15:0]),
        .b(b[15:0]),
        .cin(1'b0),
        .sum(sum[15:0]),
        .cout(cout)
    );

    // Second 16-bit adder: upper bits
    add16 add_high (
        .a(a[31:16]),
        .b(b[31:16]),
        .cin(cout),
        .sum(sum[31:16]),
        .cout()
    );

endmodule

module top_module (
    input  [31:0] a,
    input  [31:0] b,
    output [31:0] sum
);
    wire cout;

    add16 add_low (
        .a    (a[15:0]),
        .b    (b[15:0]),
        .cin  (1'b0),
        .sum  (sum[15:0]),
        .cout (cout)
    );

    add16 add_high (
        .a    (a[31:16]),
        .b    (b[31:16]),
        .cin  (cout),
        .sum  (sum[31:16]),
        .cout ()
    );

endmodule

module add1 (
    input  a,
    input  b,
    input  cin,
    output sum,
    output cout
);
    assign sum  = a ^ b ^ cin;
    assign cout = (a & b) | (b & cin) | (a & cin);
endmodule

module top_module(
    input [31:0] a,
    input [31:0] b,
    output [31:0] sum
);
    wire cout;
    wire [15:0] sum0,sum1;
    add16 add_low(
        .a(a[15:0]),
        .b(b[15:0]),
        .cin(1'b0),
        .sum(sum[15:0]),
        .cout(cout)
    );
    add16 add_high_low_cin(
        .a(a[31:16]),
        .b(b[31:16]),
        .cin(1'b0),
        .sum(sum0),
        .cout()
    );
    add16 add_high_high_cin(
        .a(a[31:16]),
        .b(b[31:16]),
        .cin(1'b1),
        .sum(sum1),
        .cout()
    );
    assign sum[31:16] = cout ? sum1:sum0;

endmodule
    

module top_module(
    input [31:0] a,
    input [31:0] b,
    input sub,
    output [31:0] sum
);
    wire cout;
    add16 add_low(
        .a(a[15:0]),
        .b(b[15:0]^{16{sub}}),
        .cin(sub),
        .sum(sum[15:0]),
        .cout(cout)
    );
    add16 add_high(
        .a(a[31:16]),
        .b(b[31:16]^{16{sub}}),
        .cin(cout),
        .sum(sum[31:16]),
        .cout()
    );
    

endmodule

//  Always Block  If Else and case statement and Latches

module top_module(
    input a, 
    input b,
    output wire out_assign,
    output reg out_alwaysblock
);
    // assign statement
    assign out_assign = a & b;

    // combinational always block
    always @(*) begin
        out_alwaysblock = a & b;
    end

endmodule

// synthesis verilog_input_version verilog_2001
module top_module(
    input clk,
    input a,
    input b,
    output wire out_assign,
    output reg out_always_comb,
    output reg out_always_ff );

    assign out_assign = a ^ b; // continuous assignment to wire

    always @(*) begin
        out_always_comb = a ^ b; // blocking assignment in combinational always block
    end

    always @(posedge clk) begin
        out_always_ff <= a ^ b; // non-blocking assignment in clocked always block
    end

endmodule

// synthesis verilog_input_version verilog_2001
module top_module(
    input a,
    input b,
    input sel_b1,
    input sel_b2,
    output wire out_assign,
    output reg out_always );

    // Part 1: Using assign and ternary operator
    assign out_assign = (sel_b1 & sel_b2) ? b : a;

    // Part 2: Using procedural if
    always @(*) begin
        if (sel_b1 & sel_b2)
            out_always = b;
        else
            out_always = a;
    end

endmodule


// synthesis verilog_input_version verilog_2001
module top_module (
    input      cpu_overheated,
    output reg shut_off_computer,
    input      arrived,
    input      gas_tank_empty,
    output reg keep_driving  ); //

    always @(*) begin
        if (cpu_overheated)
           shut_off_computer = 1;
        else 
            shut_off_computer = 0;
    end

    always @(*) begin
        if (~arrived)
           keep_driving = ~gas_tank_empty;
        else 
            keep_driving = 0;
    end

endmodule

// synthesis verilog_input_version verilog_2001
module top_module ( 
    input [2:0] sel, 
    input [3:0] data0,
    input [3:0] data1,
    input [3:0] data2,
    input [3:0] data3,
    input [3:0] data4,
    input [3:0] data5,
    output reg [3:0] out   );//

    always@(*) begin  // This is a combinational circuit
        
        case (sel)
      4'b0000: begin 
               out = data0;  // begin-end if >1 statement
            end
      4'b0001: begin 
               out = data1;  // begin-end if >1 statement
            end
                4'b0010: begin 
               out = data2;  // begin-end if >1 statement
            end
                4'b0011: begin 
               out = data3;  // begin-end if >1 statement
            end
                4'b0100: begin 
               out = data4;  // begin-end if >1 statement
            end
                 4'b0101: out = data5;
                
      default: out = 0;
    endcase
    end

endmodule

module top_module (
    input [7:0] in,
    output reg [2:0] pos
);
    always @(*) begin
        casez (in)
            8'b???????1: pos = 3'd0;
            8'b??????10: pos = 3'd1;
            8'b?????100: pos = 3'd2;
            8'b????1000: pos = 3'd3;
            8'b???10000: pos = 3'd4;
            8'b??100000: pos = 3'd5;
            8'b?1000000: pos = 3'd6;
            8'b10000000: pos = 3'd7;
            default: pos = 3'd0;
        endcase
    end
endmodule

module top_module (
    input [15:0] scancode,
    output reg left,
    output reg down,
    output reg right,
    output reg up
);
    always @(*) begin
        // Default assignments: all outputs are 0 unless overridden
        left = 1'b0;
        down = 1'b0;
        right = 1'b0;
        up = 1'b0;
        
        // Check for each scancode and set the corresponding output to 1 if matched
        case (scancode)
            16'he06b: left = 1'b1;   // Left arrow
            16'he072: down = 1'b1;   // Down arrow
            16'he074: right = 1'b1;  // Right arrow
            16'he075: up = 1'b1;     // Up arrow
            // No default needed since all outputs are already 0
        endcase
    end
endmodule

//Conditional ternary operator
module top_module (
    input [7:0] a, b, c, d,
    output [7:0] min);//

    wire [7:0] min1,min2;
    assign min1 = (a < b) ? a : b;
    assign min2 = (c < d) ? c : d;
    assign min = (min1 < min2) ? min1 : min2 ;
  

endmodule

module top_module (
    input [7:0] in,
    output parity);
    assign parity = ^in;

endmodule

module top_module( 
    input [99:0] in,
    output out_and,
    output out_or,
    output out_xor 
);
    assign out_and = &in;
    assign out_or  = |in;
    assign out_xor = ^in;


endmodule

module top_module(
    input  [99:0] in,
    output [99:0] out
);
    genvar i;
    generate
        for (i = 0; i < 100; i = i + 1) begin : reverse_bits
            assign out[i] = in[99 - i];
        end
    endgenerate
endmodule

module top_module(
    input  [254:0] in,
    output [7:0] out
);
    integer i;
    reg [7:0] count;

    always @(*) begin
        count = 0;
        for (i = 0; i < 255; i = i + 1)
            count = count + in[i];
    end

    assign out = count;
endmodule

module top_module(
    input  [399:0] a, b,
    input  cin,
    output cout,
    output [399:0] sum
);

    wire [100:0] carry;         // 101 carry signals: carry[0] = cin, carry[100] = cout
    assign carry[0] = cin;      // Initial carry-in

    genvar i;
    generate
        for (i = 0; i < 100; i = i + 1) begin : bcd_adders
            bcd_fadd adder (
                .a(a[i*4 +: 4]),     // 4-bit slice of a
                .b(b[i*4 +: 4]),     // 4-bit slice of b
                .cin(carry[i]),      // carry-in from previous stage
                .sum(sum[i*4 +: 4]), // 4-bit sum slice
                .cout(carry[i+1])    // carry-out to next stage
            );
        end
    endgenerate

    assign cout = carry[100];   // Final carry-out

endmodule

// Multiplexers
// 2 to 1 MUX
module top_module( 
    input a, b, sel,
    output out ); 
    assign out = (sel == 0) ? a : b ;

endmodule

module top_module( 
    input [99:0] a, b,
    input sel,
    output [99:0] out );
    assign out = (sel == 0) ? a : b;

endmodule

// 9 to 1 MUX
module top_module( 
    input [15:0] a, b, c, d, e, f, g, h, i,
    input [3:0] sel,
    output  [15:0] out );

    always @(*) begin
        case (sel)
            4'd0: out = a;
            4'd1: out = b;
            4'd2: out = c;
            4'd3: out = d;
            4'd4: out = e;
            4'd5: out = f;
            4'd6: out = g;
            4'd7: out = h;
            4'd8: out = i;
            default: out = 16'hFFFF;
        endcase
    end

endmodule

// 256 to 1 MUX
module top_module( 
    input [255:0] in,
    input [7:0] sel,
    output out );
    assign out = in[sel];

endmodule


module top_module( 
    input [1023:0] in,
    input [7:0] sel,
    output [3:0] out );
     assign out = in[sel*4 +: 4];

endmodule


// GATES
module top_module (
    input in1,
    input in2,
    output out);
    assign out = !(in1 || in2);

endmodule

module top_module( 
    input a, 
    input b, 
    output out );
    assign out = a&&b;

endmodule

module top_module( 
    input a, 
    input b, 
    output out );
    assign out = !(a||b);

endmodule

module top_module (
    input in1,
    input in2,
    output out);
    assign out = (in1 & !(in2));

endmodule

module top_module (
    input in1,
    input in2,
    input in3,
    output out);
    
    wire out1;
    assign out1 = in1^in2;
    assign out = (!(out1) ^ in3);

endmodule


module top_module ( 
    input p1a, p1b, p1c, p1d,
    output p1y,
    input p2a, p2b, p2c, p2d,
    output p2y );
    assign p1y = !(p1a & p1b & p1c & p1d);
    assign p2y = !(p2a & p2b & p2c & p2d);


endmodule

// Latches And FF
module top_module (
    input clk,
    input d,
    output reg q );

    // Triggered on the positive edge of the clock
    always @(posedge clk)
        q <= d;

endmodule

module top_module (
    input clk,
    input reset,            // Synchronous reset
    input [7:0] d,
    output [7:0] q
);
    always @(posedge clk)
        if(reset)
            q <= 0;
    else
        q<=d;

endmodule

module top_module (
    input d, 
    input ena,
    output q);
    always@(*)begin
        if(ena)
            q = d;
    end
    

endmodule

module top_module (
	input clk,
	input L,
	input r_in,
	input q_in,
	output reg Q);
    always @(posedge clk)begin
        if(L)
            Q = r_in;
        else
            Q = q_in;
    end

endmodule

module top_module (
    input clk,
    input [7:0] in,
    output reg [7:0] pedge
);

    reg [7:0] prev;

    always @(posedge clk) begin
        pedge <= ~prev & in;  // Detect 0->1 transition
        prev <= in;           // Store current input for next cycle
    end

endmodule

module top_module (
    input clk,
    input d,
    output q
);
    
    reg q_pos, q_neg;
    
    // Positive-edge triggered flip-flop
    always @(posedge clk) begin
        q_pos <= d;
    end
    
    // Negative-edge triggered flip-flop
    always @(negedge clk) begin
        q_neg <= d;
    end
    
    // Output selection based on current clock state
    assign q = clk ? q_pos : q_neg;
    
endmodule

// Counters 
module top_module (
    input clk,
    input reset,      // Synchronous active-high reset
    output [3:0] q);
    always@(posedge clk)begin
        if(reset)
            q <= 4'd0;
        else 
            q<= q+1;
    end

endmodule

module top_module (
    input clk,
    input reset,        // Synchronous active-high reset
    output [3:0] q);
    always @(posedge clk)begin
        if(reset)
            q<=4'd0;
        else if (q == 4'd9)
            q <= 4'd0;
        else
            q<=q+1;
    end
endmodule

module top_module (
    input clk,
    input slowena,
    input reset,
    output reg [3:0] q
);
    always @(posedge clk) begin
        if (reset)
            q <= 4'd0;
        else if (slowena) begin
            if (q == 4'd9)
                q <= 4'd0;
            else
                q <= q + 1;
        end
    end
endmodule
