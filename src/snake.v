module topModule(
    input CLK100MHZ,
    input [4:0]BTN,
    input pulse,
    output wire [4:0]LED,
    output [3:0]VGA_R,
    output [3:0]VGA_G,
    output [3:0]VGA_B,
    output VGA_HS,
    output VGA_VS
);
wire btnc,btnl,btnr,btnu,btnd;//center,left,right,down
wire CLK25MHZ;//25MHZ clock, for VGA
wire CLK2HZ;//0.5HZ clock, for GAME CONTROL
wire dead;
wire [8:0] xPos,yPos;
wire [8:0] apple_xPos,apple_yPos;
wire apple,snake,border,head;
wire [8:0] head_xPos, head_yPos;//snake's head position
wire intoBorder,intoBody;
wire appleRefresh;

assign LED[1]=intoBorder;
assign LED[2]=intoBody;
assign LED[3]=dead;
assign LED[4]=appleRefresh;
assign btnc=BTN[4];
assign btnl=BTN[1];        
assign btnu=BTN[0];
assign btnd=BTN[3];
assign btnr=BTN[2];


clockInitialize clockInitialize(
    .CLK100MHZ(CLK100MHZ),
    .CLK25MHZ(CLK25MHZ),
    .CLK2HZ(CLK2HZ));

snakeMove snakeMove(
    .btnc(btnc),
    .clockDirect(CLK25MHZ),
    .clockMove(CLK2HZ),
    .btnr(btnr),
    .btnl(btnl),
    .btnu(btnu),
    .btnd(btnd),
    .LED(LED[0]),
    .intoBorder(intoBorder),
    .intoBody(intoBody),
    .xPos(xPos),
    .yPos(yPos),
    .apple_xPos(apple_xPos),
    .apple_yPos(apple_yPos),
    .dead(dead),
    .snake(snake),
    .head(head),
    .pulse(pulse),
    .appleRefresh(appleRefresh),
    .head_xPos(head_xPos),
    .head_yPos(head_yPos));

gameControl gameControl(
    .intoBody(intoBody),
    .intoBorder(intoBorder),
    .pulse(pulse),
    .btnc(btnc),
    .CLK25MHZ(CLK25MHZ),
    .dead(dead));

appleControl appleControl(
    .clock(CLK25MHZ),
    .xPos(xPos),
    .yPos(yPos),
    .btnc(btnc),
    .dead(dead),
    .appleRefresh(appleRefresh),
    .apple_xPos(apple_xPos),
    .apple_yPos(apple_yPos),
    .apple(apple));

wallControl wallControl(
    .xPos(xPos),
    .yPos(yPos),
    .border(border));

VGA VGA(
    .R_clk_25M(CLK25MHZ),
    .snake(snake),
    .apple(apple),
    .border(border),
    .xpos(xPos),
    .ypos(yPos),
    .O_red(VGA_R),
    .O_green(VGA_G),
    .O_blue(VGA_B),
    .O_hs(VGA_HS),
    .O_vs(VGA_VS),
    .head(head)
);

endmodule


module clockInitialize(
    input CLK100MHZ,
    output reg LED,
    output reg CLK25MHZ,
    output reg CLK2HZ
);
reg [24:0]count2HZ;
reg count25MHZ;

always@(posedge CLK100MHZ)//0.5HZ
begin
    if (count2HZ == 25'b10_1111_1010_1111_0000_1000_00) begin
        CLK2HZ <= ~CLK2HZ;
        count2HZ <= 25'b0;
    end
    else begin
        count2HZ <= count2HZ + 1'b1;
    end
end

always @(posedge CLK100MHZ)//25MHZ
begin
    count25MHZ <= count25MHZ + 1'b1;
    if(count25MHZ == 1'b0)
        CLK25MHZ <= ~CLK25MHZ;
end

endmodule


module snakeMove(
    input clockDirect,//CLK25MHZ
    input clockMove,//CLKtwoHZ
    input btnr,
    input btnl,
    input btnu,
    input btnd,
    input btnc,
    input [8:0]xPos,
    input [8:0]yPos,
    input [8:0]apple_xPos,
    input [8:0]apple_yPos,
    input dead,
    input pulse,
    output reg LED,
    output reg intoBorder,intoBody,
    output wire snake,
    output head,
    output reg appleRefresh,
    output reg[8:0] head_xPos,
    output reg[8:0] head_yPos
);
reg [1:0]direction;
parameter up=2'b00,down=2'b01,left=2'b10,right=2'b11;
reg [8:0] body_xPos [0:99];//snake's body position X, MAX_LENGTH=100
reg [8:0] body_yPos [0:99];//snake's body position Y, MAX_LENGTH=100
reg [7:0] length;



always@(posedge clockDirect or posedge dead)///////direction control
begin
    if(dead)begin
        direction <= right;
    end
    else if (btnc) begin
        direction <= right;
    end
    else if(btnr)begin
        if(direction != left)
        direction <= right;
        else begin
            direction <= left;
        end
    end
    else if(btnl)begin
        if(direction != right)
        direction <= left;
        else begin
            direction <= right;
        end
    end
    else if(btnu)begin
        if(direction != down)
        direction <= up;
        else begin
            direction <= down;
        end
    end
    else if(btnd)begin
        if(direction != up)
        direction <= down;
        else begin
            direction <= up;
        end
    end
    else begin
        direction <= direction;
     end
end

always@(posedge clockDirect or posedge dead)///////update the length
begin
    if (pulse) begin
        length <= length;
        appleRefresh <= 0;
    end
    else if(btnc)begin
        appleRefresh <= 0;
        length <= 8'd1;
    end
    else if(dead) begin
        length <= 8'd1;
        appleRefresh <= 0;
    end
    else if(head_xPos==apple_xPos && head_yPos==apple_yPos)
    begin
        if(length < 8'd100)
        begin
            appleRefresh <= 1;
            length <= length+ 1  ;
        end
        else begin
            length <= length;
            appleRefresh <= 0;
        end
    end
    else begin 
        appleRefresh <= 0;
    end
end

assign snake = (body_xPos[0] == xPos && body_yPos[0] == yPos && length > 8'd1) ||(body_xPos[1] == xPos && body_yPos[1] == yPos && length > 8'd2) ||(body_xPos[2] == xPos && body_yPos[2] == yPos && length > 8'd3) ||(body_xPos[3] == xPos && body_yPos[3] == yPos && length > 8'd4) ||(body_xPos[4] == xPos && body_yPos[4] == yPos && length > 8'd5) ||(body_xPos[5] == xPos && body_yPos[5] == yPos && length > 8'd6) ||(body_xPos[6] == xPos && body_yPos[6] == yPos && length > 8'd7) ||(body_xPos[7] == xPos && body_yPos[7] == yPos && length > 8'd8) ||(body_xPos[8] == xPos && body_yPos[8] == yPos && length > 8'd9) ||(body_xPos[9] == xPos && body_yPos[9] == yPos && length > 8'd10) ||(body_xPos[10] == xPos && body_yPos[10] == yPos && length > 8'd11) ||(body_xPos[11] == xPos && body_yPos[11] == yPos && length > 8'd12) ||(body_xPos[12] == xPos && body_yPos[12] == yPos && length > 8'd13) ||(body_xPos[13] == xPos && body_yPos[13] == yPos && length > 8'd14) ||(body_xPos[14] == xPos && body_yPos[14] == yPos && length > 8'd15) ||(body_xPos[15] == xPos && body_yPos[15] == yPos && length > 8'd16) ||(body_xPos[16] == xPos && body_yPos[16] == yPos && length > 8'd17) ||(body_xPos[17] == xPos && body_yPos[17] == yPos && length > 8'd18) ||(body_xPos[18] == xPos && body_yPos[18] == yPos && length > 8'd19) ||(body_xPos[19] == xPos && body_yPos[19] == yPos && length > 8'd20) ||(body_xPos[20] == xPos && body_yPos[20] == yPos && length > 8'd21) ||(body_xPos[21] == xPos && body_yPos[21] == yPos && length > 8'd22) ||(body_xPos[22] == xPos && body_yPos[22] == yPos && length > 8'd23) ||(body_xPos[23] == xPos && body_yPos[23] == yPos && length > 8'd24) ||(body_xPos[24] == xPos && body_yPos[24] == yPos && length > 8'd25) ||(body_xPos[25] == xPos && body_yPos[25] == yPos && length > 8'd26) ||(body_xPos[26] == xPos && body_yPos[26] == yPos && length > 8'd27) ||(body_xPos[27] == xPos && body_yPos[27] == yPos && length > 8'd28) ||(body_xPos[28] == xPos && body_yPos[28] == yPos && length > 8'd29) ||(body_xPos[29] == xPos && body_yPos[29] == yPos && length > 8'd30) ||(body_xPos[30] == xPos && body_yPos[30] == yPos && length > 8'd31) ||(body_xPos[31] == xPos && body_yPos[31] == yPos && length > 8'd32) ||(body_xPos[32] == xPos && body_yPos[32] == yPos && length > 8'd33) ||(body_xPos[33] == xPos && body_yPos[33] == yPos && length > 8'd34) ||(body_xPos[34] == xPos && body_yPos[34] == yPos && length > 8'd35) ||(body_xPos[35] == xPos && body_yPos[35] == yPos && length > 8'd36) ||(body_xPos[36] == xPos && body_yPos[36] == yPos && length > 8'd37) ||(body_xPos[37] == xPos && body_yPos[37] == yPos && length > 8'd38) ||(body_xPos[38] == xPos && body_yPos[38] == yPos && length > 8'd39) ||(body_xPos[39] == xPos && body_yPos[39] == yPos && length > 8'd40) ||(body_xPos[40] == xPos && body_yPos[40] == yPos && length > 8'd41) ||(body_xPos[41] == xPos && body_yPos[41] == yPos && length > 8'd42) ||(body_xPos[42] == xPos && body_yPos[42] == yPos && length > 8'd43) ||(body_xPos[43] == xPos && body_yPos[43] == yPos && length > 8'd44) ||(body_xPos[44] == xPos && body_yPos[44] == yPos && length > 8'd45) ||(body_xPos[45] == xPos && body_yPos[45] == yPos && length > 8'd46) ||(body_xPos[46] == xPos && body_yPos[46] == yPos && length > 8'd47) ||(body_xPos[47] == xPos && body_yPos[47] == yPos && length > 8'd48) ||(body_xPos[48] == xPos && body_yPos[48] == yPos && length > 8'd49) ||(body_xPos[49] == xPos && body_yPos[49] == yPos && length > 8'd50) ||(body_xPos[50] == xPos && body_yPos[50] == yPos && length > 8'd51) ||(body_xPos[51] == xPos && body_yPos[51] == yPos && length > 8'd52) ||(body_xPos[52] == xPos && body_yPos[52] == yPos && length > 8'd53) ||(body_xPos[53] == xPos && body_yPos[53] == yPos && length > 8'd54) ||(body_xPos[54] == xPos && body_yPos[54] == yPos && length > 8'd55) ||(body_xPos[55] == xPos && body_yPos[55] == yPos && length > 8'd56) ||(body_xPos[56] == xPos && body_yPos[56] == yPos && length > 8'd57) ||(body_xPos[57] == xPos && body_yPos[57] == yPos && length > 8'd58) ||(body_xPos[58] == xPos && body_yPos[58] == yPos && length > 8'd59) ||(body_xPos[59] == xPos && body_yPos[59] == yPos && length > 8'd60) ||(body_xPos[60] == xPos && body_yPos[60] == yPos && length > 8'd61) ||(body_xPos[61] == xPos && body_yPos[61] == yPos && length > 8'd62) ||(body_xPos[62] == xPos && body_yPos[62] == yPos && length > 8'd63) ||(body_xPos[63] == xPos && body_yPos[63] == yPos && length > 8'd64) ||(body_xPos[64] == xPos && body_yPos[64] == yPos && length > 8'd65) ||(body_xPos[65] == xPos && body_yPos[65] == yPos && length > 8'd66) ||(body_xPos[66] == xPos && body_yPos[66] == yPos && length > 8'd67) ||(body_xPos[67] == xPos && body_yPos[67] == yPos && length > 8'd68) ||(body_xPos[68] == xPos && body_yPos[68] == yPos && length > 8'd69) ||(body_xPos[69] == xPos && body_yPos[69] == yPos && length > 8'd70) ||(body_xPos[70] == xPos && body_yPos[70] == yPos && length > 8'd71) ||(body_xPos[71] == xPos && body_yPos[71] == yPos && length > 8'd72) ||(body_xPos[72] == xPos && body_yPos[72] == yPos && length > 8'd73) ||(body_xPos[73] == xPos && body_yPos[73] == yPos && length > 8'd74) ||(body_xPos[74] == xPos && body_yPos[74] == yPos && length > 8'd75) ||(body_xPos[75] == xPos && body_yPos[75] == yPos && length > 8'd76) ||(body_xPos[76] == xPos && body_yPos[76] == yPos && length > 8'd77) ||(body_xPos[77] == xPos && body_yPos[77] == yPos && length > 8'd78) ||(body_xPos[78] == xPos && body_yPos[78] == yPos && length > 8'd79) ||(body_xPos[79] == xPos && body_yPos[79] == yPos && length > 8'd80) ||(body_xPos[80] == xPos && body_yPos[80] == yPos && length > 8'd81) ||(body_xPos[81] == xPos && body_yPos[81] == yPos && length > 8'd82) ||(body_xPos[82] == xPos && body_yPos[82] == yPos && length > 8'd83) ||(body_xPos[83] == xPos && body_yPos[83] == yPos && length > 8'd84) ||(body_xPos[84] == xPos && body_yPos[84] == yPos && length > 8'd85) ||(body_xPos[85] == xPos && body_yPos[85] == yPos && length > 8'd86) ||(body_xPos[86] == xPos && body_yPos[86] == yPos && length > 8'd87) ||(body_xPos[87] == xPos && body_yPos[87] == yPos && length > 8'd88) ||(body_xPos[88] == xPos && body_yPos[88] == yPos && length > 8'd89) ||(body_xPos[89] == xPos && body_yPos[89] == yPos && length > 8'd90) ||(body_xPos[90] == xPos && body_yPos[90] == yPos && length > 8'd91) ||(body_xPos[91] == xPos && body_yPos[91] == yPos && length > 8'd92) ||(body_xPos[92] == xPos && body_yPos[92] == yPos && length > 8'd93) ||(body_xPos[93] == xPos && body_yPos[93] == yPos && length > 8'd94) ||(body_xPos[94] == xPos && body_yPos[94] == yPos && length > 8'd95) ||(body_xPos[95] == xPos && body_yPos[95] == yPos && length > 8'd96) ||(body_xPos[96] == xPos && body_yPos[96] == yPos && length > 8'd97) ||(body_xPos[97] == xPos && body_yPos[97] == yPos && length > 8'd98) ||(body_xPos[98] == xPos && body_yPos[98] == yPos && length > 8'd99) ||(body_xPos[99] == xPos && body_yPos[99] == yPos && length > 8'd100);
assign head = (head_xPos == xPos && head_yPos == yPos);
/*
always@(posedge clockMove)begin
    if(btnc)begin end
    else if(pulse == 0)begin 
    LED <= ~LED;
    end
    else begin end

end
*/

always@(posedge clockMove or posedge dead)////////////////////////////////update snake's postion
begin
    if(btnc | dead)begin
    head_xPos <= 9'd20;
    head_yPos <= 9'd20; 
    body_xPos[99] <= 9'b0;
    body_yPos[99] <= 9'b0;
    body_xPos[98] <= 9'b0;
    body_yPos[98] <= 9'b0;
    body_xPos[97] <= 9'b0;
    body_yPos[97] <= 9'b0;
    body_xPos[96] <= 9'b0;
    body_yPos[96] <= 9'b0;
    body_xPos[95] <= 9'b0;
    body_yPos[95] <= 9'b0;
    body_xPos[94] <= 9'b0;
    body_yPos[94] <= 9'b0;
    body_xPos[93] <= 9'b0;
    body_yPos[93] <= 9'b0;
    body_xPos[92] <= 9'b0;
    body_yPos[92] <= 9'b0;
    body_xPos[91] <= 9'b0;
    body_yPos[91] <= 9'b0;
    body_xPos[90] <= 9'b0;
    body_yPos[90] <= 9'b0;
    body_xPos[89] <= 9'b0;
    body_yPos[89] <= 9'b0;
    body_xPos[88] <= 9'b0;
    body_yPos[88] <= 9'b0;
    body_xPos[87] <= 9'b0;
    body_yPos[87] <= 9'b0;
    body_xPos[86] <= 9'b0;
    body_yPos[86] <= 9'b0;
    body_xPos[85] <= 9'b0;
    body_yPos[85] <= 9'b0;
    body_xPos[84] <= 9'b0;
    body_yPos[84] <= 9'b0;
    body_xPos[83] <= 9'b0;
    body_yPos[83] <= 9'b0;
    body_xPos[82] <= 9'b0;
    body_yPos[82] <= 9'b0;
    body_xPos[81] <= 9'b0;
    body_yPos[81] <= 9'b0;
    body_xPos[80] <= 9'b0;
    body_yPos[80] <= 9'b0;
    body_xPos[79] <= 9'b0;
    body_yPos[79] <= 9'b0;
    body_xPos[78] <= 9'b0;
    body_yPos[78] <= 9'b0;
    body_xPos[77] <= 9'b0;
    body_yPos[77] <= 9'b0;
    body_xPos[76] <= 9'b0;
    body_yPos[76] <= 9'b0;
    body_xPos[75] <= 9'b0;
    body_yPos[75] <= 9'b0;
    body_xPos[74] <= 9'b0;
    body_yPos[74] <= 9'b0;
    body_xPos[73] <= 9'b0;
    body_yPos[73] <= 9'b0;
    body_xPos[72] <= 9'b0;
    body_yPos[72] <= 9'b0;
    body_xPos[71] <= 9'b0;
    body_yPos[71] <= 9'b0;
    body_xPos[70] <= 9'b0;
    body_yPos[70] <= 9'b0;
    body_xPos[69] <= 9'b0;
    body_yPos[69] <= 9'b0;
    body_xPos[68] <= 9'b0;
    body_yPos[68] <= 9'b0;
    body_xPos[67] <= 9'b0;
    body_yPos[67] <= 9'b0;
    body_xPos[66] <= 9'b0;
    body_yPos[66] <= 9'b0;
    body_xPos[65] <= 9'b0;
    body_yPos[65] <= 9'b0;
    body_xPos[64] <= 9'b0;
    body_yPos[64] <= 9'b0;
    body_xPos[63] <= 9'b0;
    body_yPos[63] <= 9'b0;
    body_xPos[62] <= 9'b0;
    body_yPos[62] <= 9'b0;
    body_xPos[61] <= 9'b0;
    body_yPos[61] <= 9'b0;
    body_xPos[60] <= 9'b0;
    body_yPos[60] <= 9'b0;
    body_xPos[59] <= 9'b0;
    body_yPos[59] <= 9'b0;
    body_xPos[58] <= 9'b0;
    body_yPos[58] <= 9'b0;
    body_xPos[57] <= 9'b0;
    body_yPos[57] <= 9'b0;
    body_xPos[56] <= 9'b0;
    body_yPos[56] <= 9'b0;
    body_xPos[55] <= 9'b0;
    body_yPos[55] <= 9'b0;
    body_xPos[54] <= 9'b0;
    body_yPos[54] <= 9'b0;
    body_xPos[53] <= 9'b0;
    body_yPos[53] <= 9'b0;
    body_xPos[52] <= 9'b0;
    body_yPos[52] <= 9'b0;
    body_xPos[51] <= 9'b0;
    body_yPos[51] <= 9'b0;
    body_xPos[50] <= 9'b0;
    body_yPos[50] <= 9'b0;
    body_xPos[49] <= 9'b0;
    body_yPos[49] <= 9'b0;
    body_xPos[48] <= 9'b0;
    body_yPos[48] <= 9'b0;
    body_xPos[47] <= 9'b0;
    body_yPos[47] <= 9'b0;
    body_xPos[46] <= 9'b0;
    body_yPos[46] <= 9'b0;
    body_xPos[45] <= 9'b0;
    body_yPos[45] <= 9'b0;
    body_xPos[44] <= 9'b0;
    body_yPos[44] <= 9'b0;
    body_xPos[43] <= 9'b0;
    body_yPos[43] <= 9'b0;
    body_xPos[42] <= 9'b0;
    body_yPos[42] <= 9'b0;
    body_xPos[41] <= 9'b0;
    body_yPos[41] <= 9'b0;
    body_xPos[40] <= 9'b0;
    body_yPos[40] <= 9'b0;
    body_xPos[39] <= 9'b0;
    body_yPos[39] <= 9'b0;
    body_xPos[38] <= 9'b0;
    body_yPos[38] <= 9'b0;
    body_xPos[37] <= 9'b0;
    body_yPos[37] <= 9'b0;
    body_xPos[36] <= 9'b0;
    body_yPos[36] <= 9'b0;
    body_xPos[35] <= 9'b0;
    body_yPos[35] <= 9'b0;
    body_xPos[34] <= 9'b0;
    body_yPos[34] <= 9'b0;
    body_xPos[33] <= 9'b0;
    body_yPos[33] <= 9'b0;
    body_xPos[32] <= 9'b0;
    body_yPos[32] <= 9'b0;
    body_xPos[31] <= 9'b0;
    body_yPos[31] <= 9'b0;
    body_xPos[30] <= 9'b0;
    body_yPos[30] <= 9'b0;
    body_xPos[29] <= 9'b0;
    body_yPos[29] <= 9'b0;
    body_xPos[28] <= 9'b0;
    body_yPos[28] <= 9'b0;
    body_xPos[27] <= 9'b0;
    body_yPos[27] <= 9'b0;
    body_xPos[26] <= 9'b0;
    body_yPos[26] <= 9'b0;
    body_xPos[25] <= 9'b0;
    body_yPos[25] <= 9'b0;
    body_xPos[24] <= 9'b0;
    body_yPos[24] <= 9'b0;
    body_xPos[23] <= 9'b0;
    body_yPos[23] <= 9'b0;
    body_xPos[22] <= 9'b0;
    body_yPos[22] <= 9'b0;
    body_xPos[21] <= 9'b0;
    body_yPos[21] <= 9'b0;
    body_xPos[20] <= 9'b0;
    body_yPos[20] <= 9'b0;
    body_xPos[19] <= 9'b0;
    body_yPos[19] <= 9'b0;
    body_xPos[18] <= 9'b0;
    body_yPos[18] <= 9'b0;
    body_xPos[17] <= 9'b0;
    body_yPos[17] <= 9'b0;
    body_xPos[16] <= 9'b0;
    body_yPos[16] <= 9'b0;
    body_xPos[15] <= 9'b0;
    body_yPos[15] <= 9'b0;
    body_xPos[14] <= 9'b0;
    body_yPos[14] <= 9'b0;
    body_xPos[13] <= 9'b0;
    body_yPos[13] <= 9'b0;
    body_xPos[12] <= 9'b0;
    body_yPos[12] <= 9'b0;
    body_xPos[11] <= 9'b0;
    body_yPos[11] <= 9'b0;
    body_xPos[10] <= 9'b0;
    body_yPos[10] <= 9'b0;
    body_xPos[9] <= 9'b0;
    body_yPos[9] <= 9'b0;
    body_xPos[8] <= 9'b0;
    body_yPos[8] <= 9'b0;
    body_xPos[7] <= 9'b0;
    body_yPos[7] <= 9'b0;
    body_xPos[6] <= 9'b0;
    body_yPos[6] <= 9'b0;
    body_xPos[5] <= 9'b0;
    body_yPos[5] <= 9'b0;
    body_xPos[4] <= 9'b0;
    body_yPos[4] <= 9'b0;
    body_xPos[3] <= 9'b0;
    body_yPos[3] <= 9'b0;
    body_xPos[2] <= 9'b0;
    body_yPos[2] <= 9'b0;
    body_xPos[1] <= 9'b0;
    body_yPos[1] <= 9'b0;
    body_xPos[0] <= 9'b0;
    body_yPos[0] <= 9'b0;
    end
    else if(pulse == 0)begin
        body_xPos[99][8:0] <= body_xPos[98][8:0];
        body_yPos[99][8:0] <= body_yPos[98][8:0];
        body_xPos[99][8:0] <= body_xPos[98][8:0];
        body_yPos[99][8:0] <= body_yPos[98][8:0];
        body_xPos[98][8:0] <= body_xPos[97][8:0];
        body_yPos[98][8:0] <= body_yPos[97][8:0];
        body_xPos[97][8:0] <= body_xPos[96][8:0];
        body_yPos[97][8:0] <= body_yPos[96][8:0];
        body_xPos[96][8:0] <= body_xPos[95][8:0];
        body_yPos[96][8:0] <= body_yPos[95][8:0];
        body_xPos[95][8:0] <= body_xPos[94][8:0];
        body_yPos[95][8:0] <= body_yPos[94][8:0];
        body_xPos[94][8:0] <= body_xPos[93][8:0];
        body_yPos[94][8:0] <= body_yPos[93][8:0];
        body_xPos[93][8:0] <= body_xPos[92][8:0];
        body_yPos[93][8:0] <= body_yPos[92][8:0];
        body_xPos[92][8:0] <= body_xPos[91][8:0];
        body_yPos[92][8:0] <= body_yPos[91][8:0];
        body_xPos[91][8:0] <= body_xPos[90][8:0];
        body_yPos[91][8:0] <= body_yPos[90][8:0];
        body_xPos[90][8:0] <= body_xPos[89][8:0];
        body_yPos[90][8:0] <= body_yPos[89][8:0];
        body_xPos[89][8:0] <= body_xPos[88][8:0];
        body_yPos[89][8:0] <= body_yPos[88][8:0];
        body_xPos[88][8:0] <= body_xPos[87][8:0];
        body_yPos[88][8:0] <= body_yPos[87][8:0];
        body_xPos[87][8:0] <= body_xPos[86][8:0];
        body_yPos[87][8:0] <= body_yPos[86][8:0];
        body_xPos[86][8:0] <= body_xPos[85][8:0];
        body_yPos[86][8:0] <= body_yPos[85][8:0];
        body_xPos[85][8:0] <= body_xPos[84][8:0];
        body_yPos[85][8:0] <= body_yPos[84][8:0];
        body_xPos[84][8:0] <= body_xPos[83][8:0];
        body_yPos[84][8:0] <= body_yPos[83][8:0];
        body_xPos[83][8:0] <= body_xPos[82][8:0];
        body_yPos[83][8:0] <= body_yPos[82][8:0];
        body_xPos[82][8:0] <= body_xPos[81][8:0];
        body_yPos[82][8:0] <= body_yPos[81][8:0];
        body_xPos[81][8:0] <= body_xPos[80][8:0];
        body_yPos[81][8:0] <= body_yPos[80][8:0];
        body_xPos[80][8:0] <= body_xPos[79][8:0];
        body_yPos[80][8:0] <= body_yPos[79][8:0];
        body_xPos[79][8:0] <= body_xPos[78][8:0];
        body_yPos[79][8:0] <= body_yPos[78][8:0];
        body_xPos[78][8:0] <= body_xPos[77][8:0];
        body_yPos[78][8:0] <= body_yPos[77][8:0];
        body_xPos[77][8:0] <= body_xPos[76][8:0];
        body_yPos[77][8:0] <= body_yPos[76][8:0];
        body_xPos[76][8:0] <= body_xPos[75][8:0];
        body_yPos[76][8:0] <= body_yPos[75][8:0];
        body_xPos[75][8:0] <= body_xPos[74][8:0];
        body_yPos[75][8:0] <= body_yPos[74][8:0];
        body_xPos[74][8:0] <= body_xPos[73][8:0];
        body_yPos[74][8:0] <= body_yPos[73][8:0];
        body_xPos[73][8:0] <= body_xPos[72][8:0];
        body_yPos[73][8:0] <= body_yPos[72][8:0];
        body_xPos[72][8:0] <= body_xPos[71][8:0];
        body_yPos[72][8:0] <= body_yPos[71][8:0];
        body_xPos[71][8:0] <= body_xPos[70][8:0];
        body_yPos[71][8:0] <= body_yPos[70][8:0];
        body_xPos[70][8:0] <= body_xPos[69][8:0];
        body_yPos[70][8:0] <= body_yPos[69][8:0];
        body_xPos[69][8:0] <= body_xPos[68][8:0];
        body_yPos[69][8:0] <= body_yPos[68][8:0];
        body_xPos[68][8:0] <= body_xPos[67][8:0];
        body_yPos[68][8:0] <= body_yPos[67][8:0];
        body_xPos[67][8:0] <= body_xPos[66][8:0];
        body_yPos[67][8:0] <= body_yPos[66][8:0];
        body_xPos[66][8:0] <= body_xPos[65][8:0];
        body_yPos[66][8:0] <= body_yPos[65][8:0];
        body_xPos[65][8:0] <= body_xPos[64][8:0];
        body_yPos[65][8:0] <= body_yPos[64][8:0];
        body_xPos[64][8:0] <= body_xPos[63][8:0];
        body_yPos[64][8:0] <= body_yPos[63][8:0];
        body_xPos[63][8:0] <= body_xPos[62][8:0];
        body_yPos[63][8:0] <= body_yPos[62][8:0];
        body_xPos[62][8:0] <= body_xPos[61][8:0];
        body_yPos[62][8:0] <= body_yPos[61][8:0];
        body_xPos[61][8:0] <= body_xPos[60][8:0];
        body_yPos[61][8:0] <= body_yPos[60][8:0];
        body_xPos[60][8:0] <= body_xPos[59][8:0];
        body_yPos[60][8:0] <= body_yPos[59][8:0];
        body_xPos[59][8:0] <= body_xPos[58][8:0];
        body_yPos[59][8:0] <= body_yPos[58][8:0];
        body_xPos[58][8:0] <= body_xPos[57][8:0];
        body_yPos[58][8:0] <= body_yPos[57][8:0];
        body_xPos[57][8:0] <= body_xPos[56][8:0];
        body_yPos[57][8:0] <= body_yPos[56][8:0];
        body_xPos[56][8:0] <= body_xPos[55][8:0];
        body_yPos[56][8:0] <= body_yPos[55][8:0];
        body_xPos[55][8:0] <= body_xPos[54][8:0];
        body_yPos[55][8:0] <= body_yPos[54][8:0];
        body_xPos[54][8:0] <= body_xPos[53][8:0];
        body_yPos[54][8:0] <= body_yPos[53][8:0];
        body_xPos[53][8:0] <= body_xPos[52][8:0];
        body_yPos[53][8:0] <= body_yPos[52][8:0];
        body_xPos[52][8:0] <= body_xPos[51][8:0];
        body_yPos[52][8:0] <= body_yPos[51][8:0];
        body_xPos[51][8:0] <= body_xPos[50][8:0];
        body_yPos[51][8:0] <= body_yPos[50][8:0];
        body_xPos[50][8:0] <= body_xPos[49][8:0];
        body_yPos[50][8:0] <= body_yPos[49][8:0];
        body_xPos[49][8:0] <= body_xPos[48][8:0];
        body_yPos[49][8:0] <= body_yPos[48][8:0];
        body_xPos[48][8:0] <= body_xPos[47][8:0];
        body_yPos[48][8:0] <= body_yPos[47][8:0];
        body_xPos[47][8:0] <= body_xPos[46][8:0];
        body_yPos[47][8:0] <= body_yPos[46][8:0];
        body_xPos[46][8:0] <= body_xPos[45][8:0];
        body_yPos[46][8:0] <= body_yPos[45][8:0];
        body_xPos[45][8:0] <= body_xPos[44][8:0];
        body_yPos[45][8:0] <= body_yPos[44][8:0];
        body_xPos[44][8:0] <= body_xPos[43][8:0];
        body_yPos[44][8:0] <= body_yPos[43][8:0];
        body_xPos[43][8:0] <= body_xPos[42][8:0];
        body_yPos[43][8:0] <= body_yPos[42][8:0];
        body_xPos[42][8:0] <= body_xPos[41][8:0];
        body_yPos[42][8:0] <= body_yPos[41][8:0];
        body_xPos[41][8:0] <= body_xPos[40][8:0];
        body_yPos[41][8:0] <= body_yPos[40][8:0];
        body_xPos[40][8:0] <= body_xPos[39][8:0];
        body_yPos[40][8:0] <= body_yPos[39][8:0];
        body_xPos[39][8:0] <= body_xPos[38][8:0];
        body_yPos[39][8:0] <= body_yPos[38][8:0];
        body_xPos[38][8:0] <= body_xPos[37][8:0];
        body_yPos[38][8:0] <= body_yPos[37][8:0];
        body_xPos[37][8:0] <= body_xPos[36][8:0];
        body_yPos[37][8:0] <= body_yPos[36][8:0];
        body_xPos[36][8:0] <= body_xPos[35][8:0];
        body_yPos[36][8:0] <= body_yPos[35][8:0];
        body_xPos[35][8:0] <= body_xPos[34][8:0];
        body_yPos[35][8:0] <= body_yPos[34][8:0];
        body_xPos[34][8:0] <= body_xPos[33][8:0];
        body_yPos[34][8:0] <= body_yPos[33][8:0];
        body_xPos[33][8:0] <= body_xPos[32][8:0];
        body_yPos[33][8:0] <= body_yPos[32][8:0];
        body_xPos[32][8:0] <= body_xPos[31][8:0];
        body_yPos[32][8:0] <= body_yPos[31][8:0];
        body_xPos[31][8:0] <= body_xPos[30][8:0];
        body_yPos[31][8:0] <= body_yPos[30][8:0];
        body_xPos[30][8:0] <= body_xPos[29][8:0];
        body_yPos[30][8:0] <= body_yPos[29][8:0];
        body_xPos[29][8:0] <= body_xPos[28][8:0];
        body_yPos[29][8:0] <= body_yPos[28][8:0];
        body_xPos[28][8:0] <= body_xPos[27][8:0];
        body_yPos[28][8:0] <= body_yPos[27][8:0];
        body_xPos[27][8:0] <= body_xPos[26][8:0];
        body_yPos[27][8:0] <= body_yPos[26][8:0];
        body_xPos[26][8:0] <= body_xPos[25][8:0];
        body_yPos[26][8:0] <= body_yPos[25][8:0];
        body_xPos[25][8:0] <= body_xPos[24][8:0];
        body_yPos[25][8:0] <= body_yPos[24][8:0];
        body_xPos[24][8:0] <= body_xPos[23][8:0];
        body_yPos[24][8:0] <= body_yPos[23][8:0];
        body_xPos[23][8:0] <= body_xPos[22][8:0];
        body_yPos[23][8:0] <= body_yPos[22][8:0];
        body_xPos[22][8:0] <= body_xPos[21][8:0];
        body_yPos[22][8:0] <= body_yPos[21][8:0];
        body_xPos[21][8:0] <= body_xPos[20][8:0];
        body_yPos[21][8:0] <= body_yPos[20][8:0];
        body_xPos[20][8:0] <= body_xPos[19][8:0];
        body_yPos[20][8:0] <= body_yPos[19][8:0];
        body_xPos[19][8:0] <= body_xPos[18][8:0];
        body_yPos[19][8:0] <= body_yPos[18][8:0];
        body_xPos[18][8:0] <= body_xPos[17][8:0];
        body_yPos[18][8:0] <= body_yPos[17][8:0];
        body_xPos[17][8:0] <= body_xPos[16][8:0];
        body_yPos[17][8:0] <= body_yPos[16][8:0];
        body_xPos[16][8:0] <= body_xPos[15][8:0];
        body_yPos[16][8:0] <= body_yPos[15][8:0];
        body_xPos[15][8:0] <= body_xPos[14][8:0];
        body_yPos[15][8:0] <= body_yPos[14][8:0];
        body_xPos[14][8:0] <= body_xPos[13][8:0];
        body_yPos[14][8:0] <= body_yPos[13][8:0];
        body_xPos[13][8:0] <= body_xPos[12][8:0];
        body_yPos[13][8:0] <= body_yPos[12][8:0];
        body_xPos[12][8:0] <= body_xPos[11][8:0];
        body_yPos[12][8:0] <= body_yPos[11][8:0];
        body_xPos[11][8:0] <= body_xPos[10][8:0];
        body_yPos[11][8:0] <= body_yPos[10][8:0];
        body_xPos[10][8:0] <= body_xPos[9][8:0];
        body_yPos[10][8:0] <= body_yPos[9][8:0];
        body_xPos[9][8:0] <= body_xPos[8][8:0];
        body_yPos[9][8:0] <= body_yPos[8][8:0];
        body_xPos[8][8:0] <= body_xPos[7][8:0];
        body_yPos[8][8:0] <= body_yPos[7][8:0];
        body_xPos[7][8:0] <= body_xPos[6][8:0];
        body_yPos[7][8:0] <= body_yPos[6][8:0];
        body_xPos[6][8:0] <= body_xPos[5][8:0];
        body_yPos[6][8:0] <= body_yPos[5][8:0];
        body_xPos[5][8:0] <= body_xPos[4][8:0];
        body_yPos[5][8:0] <= body_yPos[4][8:0];
        body_xPos[4][8:0] <= body_xPos[3][8:0];
        body_yPos[4][8:0] <= body_yPos[3][8:0];
        body_xPos[3][8:0] <= body_xPos[2][8:0];
        body_yPos[3][8:0] <= body_yPos[2][8:0];
        body_xPos[2][8:0] <= body_xPos[1][8:0];
        body_yPos[2][8:0] <= body_yPos[1][8:0];
        body_xPos[1][8:0] <= body_xPos[0][8:0];
        body_yPos[1][8:0] <= body_yPos[0][8:0];
        body_xPos[0][8:0] <= head_xPos[8:0];
        body_yPos[0][8:0] <= head_yPos[8:0];
        case (direction)
            up: head_yPos <= head_yPos - 9'd1;
            down: head_yPos <= head_yPos + 9'd1;
            right: head_xPos <= head_xPos + 9'd1;
            left: head_xPos <= head_xPos - 9'd1;
        endcase
    end
    else begin end
end

always@(posedge clockDirect)
begin
    if(btnc)begin
        intoBorder <= 0;
    end
    else if((head_xPos<9'd1) || (head_yPos<9'd1) || (head_xPos>9'd50) || (head_yPos>9'd50))begin
        intoBorder <= 1;
    end
    else begin
        intoBorder <= 0;
    end

    if (btnc) begin
        intoBody <= 0;
    end
    else if((head_xPos[8:0]==body_xPos[3][8:0] && head_yPos[8:0]==body_yPos[3][8:0] && length>4)||(head_xPos[8:0]==body_xPos[4][8:0] && head_yPos[8:0]==body_yPos[4][8:0] && length>5)||(head_xPos[8:0]==body_xPos[5][8:0] && head_yPos[8:0]==body_yPos[5][8:0] && length>6)||(head_xPos[8:0]==body_xPos[6][8:0] && head_yPos[8:0]==body_yPos[6][8:0] && length>7)||(head_xPos[8:0]==body_xPos[7][8:0] && head_yPos[8:0]==body_yPos[7][8:0] && length>8)||(head_xPos[8:0]==body_xPos[8][8:0] && head_yPos[8:0]==body_yPos[8][8:0] && length>9)||(head_xPos[8:0]==body_xPos[9][8:0] && head_yPos[8:0]==body_yPos[9][8:0] && length>10)||(head_xPos[8:0]==body_xPos[10][8:0] && head_yPos[8:0]==body_yPos[10][8:0] && length>11)||(head_xPos[8:0]==body_xPos[11][8:0] && head_yPos[8:0]==body_yPos[11][8:0] && length>12)||(head_xPos[8:0]==body_xPos[12][8:0] && head_yPos[8:0]==body_yPos[12][8:0] && length>13)||(head_xPos[8:0]==body_xPos[13][8:0] && head_yPos[8:0]==body_yPos[13][8:0] && length>14)||(head_xPos[8:0]==body_xPos[14][8:0] && head_yPos[8:0]==body_yPos[14][8:0] && length>15)||(head_xPos[8:0]==body_xPos[15][8:0] && head_yPos[8:0]==body_yPos[15][8:0] && length>16)||(head_xPos[8:0]==body_xPos[16][8:0] && head_yPos[8:0]==body_yPos[16][8:0] && length>17)||(head_xPos[8:0]==body_xPos[17][8:0] && head_yPos[8:0]==body_yPos[17][8:0] && length>18)||(head_xPos[8:0]==body_xPos[18][8:0] && head_yPos[8:0]==body_yPos[18][8:0] && length>19)||(head_xPos[8:0]==body_xPos[19][8:0] && head_yPos[8:0]==body_yPos[19][8:0] && length>20)||(head_xPos[8:0]==body_xPos[20][8:0] && head_yPos[8:0]==body_yPos[20][8:0] && length>21)||(head_xPos[8:0]==body_xPos[21][8:0] && head_yPos[8:0]==body_yPos[21][8:0] && length>22)||(head_xPos[8:0]==body_xPos[22][8:0] && head_yPos[8:0]==body_yPos[22][8:0] && length>23)||(head_xPos[8:0]==body_xPos[23][8:0] && head_yPos[8:0]==body_yPos[23][8:0] && length>24)||(head_xPos[8:0]==body_xPos[24][8:0] && head_yPos[8:0]==body_yPos[24][8:0] && length>25)||(head_xPos[8:0]==body_xPos[25][8:0] && head_yPos[8:0]==body_yPos[25][8:0] && length>26)||(head_xPos[8:0]==body_xPos[26][8:0] && head_yPos[8:0]==body_yPos[26][8:0] && length>27)||(head_xPos[8:0]==body_xPos[27][8:0] && head_yPos[8:0]==body_yPos[27][8:0] && length>28)||(head_xPos[8:0]==body_xPos[28][8:0] && head_yPos[8:0]==body_yPos[28][8:0] && length>29)||(head_xPos[8:0]==body_xPos[29][8:0] && head_yPos[8:0]==body_yPos[29][8:0] && length>30)||(head_xPos[8:0]==body_xPos[30][8:0] && head_yPos[8:0]==body_yPos[30][8:0] && length>31)||(head_xPos[8:0]==body_xPos[31][8:0] && head_yPos[8:0]==body_yPos[31][8:0] && length>32)||(head_xPos[8:0]==body_xPos[32][8:0] && head_yPos[8:0]==body_yPos[32][8:0] && length>33)||(head_xPos[8:0]==body_xPos[33][8:0] && head_yPos[8:0]==body_yPos[33][8:0] && length>34)||(head_xPos[8:0]==body_xPos[34][8:0] && head_yPos[8:0]==body_yPos[34][8:0] && length>35)||(head_xPos[8:0]==body_xPos[35][8:0] && head_yPos[8:0]==body_yPos[35][8:0] && length>36)||(head_xPos[8:0]==body_xPos[36][8:0] && head_yPos[8:0]==body_yPos[36][8:0] && length>37)||(head_xPos[8:0]==body_xPos[37][8:0] && head_yPos[8:0]==body_yPos[37][8:0] && length>38)||(head_xPos[8:0]==body_xPos[38][8:0] && head_yPos[8:0]==body_yPos[38][8:0] && length>39)||(head_xPos[8:0]==body_xPos[39][8:0] && head_yPos[8:0]==body_yPos[39][8:0] && length>40)||(head_xPos[8:0]==body_xPos[40][8:0] && head_yPos[8:0]==body_yPos[40][8:0] && length>41)||(head_xPos[8:0]==body_xPos[41][8:0] && head_yPos[8:0]==body_yPos[41][8:0] && length>42)||(head_xPos[8:0]==body_xPos[42][8:0] && head_yPos[8:0]==body_yPos[42][8:0] && length>43)||(head_xPos[8:0]==body_xPos[43][8:0] && head_yPos[8:0]==body_yPos[43][8:0] && length>44)||(head_xPos[8:0]==body_xPos[44][8:0] && head_yPos[8:0]==body_yPos[44][8:0] && length>45)||(head_xPos[8:0]==body_xPos[45][8:0] && head_yPos[8:0]==body_yPos[45][8:0] && length>46)||(head_xPos[8:0]==body_xPos[46][8:0] && head_yPos[8:0]==body_yPos[46][8:0] && length>47)||(head_xPos[8:0]==body_xPos[47][8:0] && head_yPos[8:0]==body_yPos[47][8:0] && length>48)||(head_xPos[8:0]==body_xPos[48][8:0] && head_yPos[8:0]==body_yPos[48][8:0] && length>49)||(head_xPos[8:0]==body_xPos[49][8:0] && head_yPos[8:0]==body_yPos[49][8:0] && length>50)||(head_xPos[8:0]==body_xPos[50][8:0] && head_yPos[8:0]==body_yPos[50][8:0] && length>51)||(head_xPos[8:0]==body_xPos[51][8:0] && head_yPos[8:0]==body_yPos[51][8:0] && length>52)||(head_xPos[8:0]==body_xPos[52][8:0] && head_yPos[8:0]==body_yPos[52][8:0] && length>53)||(head_xPos[8:0]==body_xPos[53][8:0] && head_yPos[8:0]==body_yPos[53][8:0] && length>54)||(head_xPos[8:0]==body_xPos[54][8:0] && head_yPos[8:0]==body_yPos[54][8:0] && length>55)||(head_xPos[8:0]==body_xPos[55][8:0] && head_yPos[8:0]==body_yPos[55][8:0] && length>56)||(head_xPos[8:0]==body_xPos[56][8:0] && head_yPos[8:0]==body_yPos[56][8:0] && length>57)||(head_xPos[8:0]==body_xPos[57][8:0] && head_yPos[8:0]==body_yPos[57][8:0] && length>58)||(head_xPos[8:0]==body_xPos[58][8:0] && head_yPos[8:0]==body_yPos[58][8:0] && length>59)||(head_xPos[8:0]==body_xPos[59][8:0] && head_yPos[8:0]==body_yPos[59][8:0] && length>60)||(head_xPos[8:0]==body_xPos[60][8:0] && head_yPos[8:0]==body_yPos[60][8:0] && length>61)||(head_xPos[8:0]==body_xPos[61][8:0] && head_yPos[8:0]==body_yPos[61][8:0] && length>62)||(head_xPos[8:0]==body_xPos[62][8:0] && head_yPos[8:0]==body_yPos[62][8:0] && length>63)||(head_xPos[8:0]==body_xPos[63][8:0] && head_yPos[8:0]==body_yPos[63][8:0] && length>64)||(head_xPos[8:0]==body_xPos[64][8:0] && head_yPos[8:0]==body_yPos[64][8:0] && length>65)||(head_xPos[8:0]==body_xPos[65][8:0] && head_yPos[8:0]==body_yPos[65][8:0] && length>66)||(head_xPos[8:0]==body_xPos[66][8:0] && head_yPos[8:0]==body_yPos[66][8:0] && length>67)||(head_xPos[8:0]==body_xPos[67][8:0] && head_yPos[8:0]==body_yPos[67][8:0] && length>68)||(head_xPos[8:0]==body_xPos[68][8:0] && head_yPos[8:0]==body_yPos[68][8:0] && length>69)||(head_xPos[8:0]==body_xPos[69][8:0] && head_yPos[8:0]==body_yPos[69][8:0] && length>70)||(head_xPos[8:0]==body_xPos[70][8:0] && head_yPos[8:0]==body_yPos[70][8:0] && length>71)||(head_xPos[8:0]==body_xPos[71][8:0] && head_yPos[8:0]==body_yPos[71][8:0] && length>72)||(head_xPos[8:0]==body_xPos[72][8:0] && head_yPos[8:0]==body_yPos[72][8:0] && length>73)||(head_xPos[8:0]==body_xPos[73][8:0] && head_yPos[8:0]==body_yPos[73][8:0] && length>74)||(head_xPos[8:0]==body_xPos[74][8:0] && head_yPos[8:0]==body_yPos[74][8:0] && length>75)||(head_xPos[8:0]==body_xPos[75][8:0] && head_yPos[8:0]==body_yPos[75][8:0] && length>76)||(head_xPos[8:0]==body_xPos[76][8:0] && head_yPos[8:0]==body_yPos[76][8:0] && length>77)||(head_xPos[8:0]==body_xPos[77][8:0] && head_yPos[8:0]==body_yPos[77][8:0] && length>78)||(head_xPos[8:0]==body_xPos[78][8:0] && head_yPos[8:0]==body_yPos[78][8:0] && length>79)||(head_xPos[8:0]==body_xPos[79][8:0] && head_yPos[8:0]==body_yPos[79][8:0] && length>80)||(head_xPos[8:0]==body_xPos[80][8:0] && head_yPos[8:0]==body_yPos[80][8:0] && length>81)||(head_xPos[8:0]==body_xPos[81][8:0] && head_yPos[8:0]==body_yPos[81][8:0] && length>82)||(head_xPos[8:0]==body_xPos[82][8:0] && head_yPos[8:0]==body_yPos[82][8:0] && length>83)||(head_xPos[8:0]==body_xPos[83][8:0] && head_yPos[8:0]==body_yPos[83][8:0] && length>84)||(head_xPos[8:0]==body_xPos[84][8:0] && head_yPos[8:0]==body_yPos[84][8:0] && length>85)||(head_xPos[8:0]==body_xPos[85][8:0] && head_yPos[8:0]==body_yPos[85][8:0] && length>86)||(head_xPos[8:0]==body_xPos[86][8:0] && head_yPos[8:0]==body_yPos[86][8:0] && length>87)||(head_xPos[8:0]==body_xPos[87][8:0] && head_yPos[8:0]==body_yPos[87][8:0] && length>88)||(head_xPos[8:0]==body_xPos[88][8:0] && head_yPos[8:0]==body_yPos[88][8:0] && length>89)||(head_xPos[8:0]==body_xPos[89][8:0] && head_yPos[8:0]==body_yPos[89][8:0] && length>90)||(head_xPos[8:0]==body_xPos[90][8:0] && head_yPos[8:0]==body_yPos[90][8:0] && length>91)||(head_xPos[8:0]==body_xPos[91][8:0] && head_yPos[8:0]==body_yPos[91][8:0] && length>92)||(head_xPos[8:0]==body_xPos[92][8:0] && head_yPos[8:0]==body_yPos[92][8:0] && length>93)||(head_xPos[8:0]==body_xPos[93][8:0] && head_yPos[8:0]==body_yPos[93][8:0] && length>94)||(head_xPos[8:0]==body_xPos[94][8:0] && head_yPos[8:0]==body_yPos[94][8:0] && length>95)||(head_xPos[8:0]==body_xPos[95][8:0] && head_yPos[8:0]==body_yPos[95][8:0] && length>96)||(head_xPos[8:0]==body_xPos[96][8:0] && head_yPos[8:0]==body_yPos[96][8:0] && length>97)||(head_xPos[8:0]==body_xPos[97][8:0] && head_yPos[8:0]==body_yPos[97][8:0] && length>98)||(head_xPos[8:0]==body_xPos[98][8:0] && head_yPos[8:0]==body_yPos[98][8:0] && length>99)||(head_xPos[8:0]==body_xPos[99][8:0] && head_yPos[8:0]==body_yPos[99][8:0] && length>100))
    begin
        intoBody <= 1;
    end
    else begin
        intoBody <= 0;
    end
end

endmodule



module gameControl(
    input intoBody,
    input intoBorder,
    input pulse,
    input CLK25MHZ,
    input btnc,
    output reg dead
);
always@(posedge CLK25MHZ)
begin
    if((pulse == 0) && (dead == 0))begin
        if(intoBody)
        begin
            dead <= 1;
        end
        else if (intoBorder) begin
            dead <= 1;
        end
    end
    else if(btnc) begin
      dead <= 0;
    end
end
endmodule


module appleControl(
    input clock,//for random number initializing, 25MHZ
    input [8:0]xPos,
    input [8:0]yPos,
    input dead,
    input appleRefresh,
    input btnc,
    output reg [8:0] apple_xPos,
    output reg [8:0] apple_yPos,
    output wire apple
);

reg [18:0]rand_num;
always@(posedge clock or posedge btnc)
begin
    if(btnc)
        rand_num <= 19'b000_0110_1100_0000_1101;   //seed
    else
        begin
            rand_num[0] <= 1;
            rand_num[1] <= rand_num[0];
            rand_num[2] <= rand_num[1];
            rand_num[3] <= rand_num[2];
            rand_num[4] <= rand_num[3] ^~rand_num[14];
            rand_num[5] <= 0;
            rand_num[6] <= 0;
            rand_num[7] <= 0;
			rand_num[8] <= 0;
			rand_num[9] <= 0;
			rand_num[10] <= 1;
			rand_num[11] <= rand_num[10];
			rand_num[12] <= rand_num[11]^~rand_num[14];
			rand_num[13] <= rand_num[12]^~rand_num[14];
			rand_num[14] <= rand_num[13]^~rand_num[12];
			rand_num[15] <= 0;
            rand_num[16] <= 0;
			rand_num[17] <= 0;
			rand_num[18] <= 0;
        end
            
end

always@(posedge appleRefresh or posedge btnc)
begin
    if(appleRefresh)begin
        apple_yPos[8:0] <= rand_num[8:0];
        apple_xPos[8:0] <= rand_num[18:10];
    end
    else if (btnc) begin
        apple_yPos[8:0] <= rand_num[8:0];
        apple_xPos[8:0] <= rand_num[18:10];
    end
    else begin
        apple_xPos[8:0] <= apple_xPos[8:0];
        apple_yPos[8:0] <= apple_yPos[8:0];
    end
end

assign apple = (apple_xPos[8:0]==xPos[8:0]) && (apple_yPos[8:0]==yPos[8:0]) && (dead==0);

endmodule




module wallControl(
    input [8:0]xPos,
    input [8:0]yPos,
    output wire border
);

assign border = (xPos==9'd0 && yPos<9'd52) || (xPos==9'd51 && yPos<9'd52) || (yPos==9'd0 && xPos<9'd52) || (yPos==9'd51 && xPos<9'd52);
endmodule


module VGA(
    input                   R_clk_25M   , // clock with 25MHz
    input                   snake,
    input                   apple,
    input                   border,
    input                   head,
    output   wire  [8:0]    xpos,//current scanning position X
    output   wire  [8:0]    ypos,//current scanning position Y
    output   reg   [3:0]    O_red   , // VGA red
    output   reg   [3:0]    O_green , // VGA green
    output   reg   [3:0]    O_blue  , // VGA blue
    output                  O_hs    , // VGA行同步信号
    output                  O_vs      // VGA场同步信号
);
parameter I_rst_n = 0;
// 分辨率为640*480时行时序各个参数定义
parameter       C_H_SYNC_PULSE      =   96  , 
                C_H_BACK_PORCH      =   48  ,
                C_H_ACTIVE_TIME     =   640 ,
                C_H_FRONT_PORCH     =   16  ,
                C_H_LINE_PERIOD     =   800 ;

// 分辨率为640*480时场时序各个参数定义               
parameter       C_V_SYNC_PULSE      =   2   , 
                C_V_BACK_PORCH      =   33  ,
                C_V_ACTIVE_TIME     =   480 ,
                C_V_FRONT_PORCH     =   10  ,
                C_V_FRAME_PERIOD    =   525 ;
                
parameter       C_COLOR_BAR_WIDTH   =   C_H_ACTIVE_TIME / 8  ;  

reg [11:0]      R_h_cnt         ; // 行时序计数器
reg [11:0]      R_v_cnt         ; // 列时序计数器
wire [11:0]     R_h_cnt_pos     ;
wire [11:0]     R_v_cnt_pos     ;

wire            W_active_flag   ; // 激活标志，当这个信号为1时RGB的数据可以显示在屏幕上

//////////////////////////////////////////////////////////////////
// 功能：产生行时序
//////////////////////////////////////////////////////////////////
always @(posedge R_clk_25M )
begin
    if(I_rst_n)
        R_h_cnt <=  12'd0   ;
    else if(R_h_cnt == C_H_LINE_PERIOD - 1'b1)
        R_h_cnt <=  12'd0   ;
    else
        R_h_cnt <=  R_h_cnt + 1'b1  ;                
end                

assign O_hs =   (R_h_cnt < C_H_SYNC_PULSE) ? 1'b0 : 1'b1    ; 
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// 功能：产生场时序
//////////////////////////////////////////////////////////////////
always @(posedge R_clk_25M )
begin
    if(I_rst_n)
        R_v_cnt <=  12'd0   ;
    else if(R_v_cnt == C_V_FRAME_PERIOD - 1'b1)
        R_v_cnt <=  12'd0   ;
    else if(R_h_cnt == C_H_LINE_PERIOD - 1'b1)
        R_v_cnt <=  R_v_cnt + 1'b1  ;
    else
        R_v_cnt <=  R_v_cnt ;                        
end                

assign O_vs =   (R_v_cnt < C_V_SYNC_PULSE) ? 1'b0 : 1'b1    ; 

assign W_active_flag =  (R_h_cnt >= (C_H_SYNC_PULSE + C_H_BACK_PORCH                  ))  &&
                        (R_h_cnt <= (C_H_SYNC_PULSE + C_H_BACK_PORCH + C_H_ACTIVE_TIME))  && 
                        (R_v_cnt >= (C_V_SYNC_PULSE + C_V_BACK_PORCH                  ))  &&
                        (R_v_cnt <= (C_V_SYNC_PULSE + C_V_BACK_PORCH + C_V_ACTIVE_TIME))  ;                     

assign R_h_cnt_pos = (R_h_cnt - C_H_SYNC_PULSE - C_H_BACK_PORCH);
assign R_v_cnt_pos = (R_v_cnt - C_V_SYNC_PULSE - C_V_BACK_PORCH);
assign xpos[8:0] = R_h_cnt_pos[11:3];
assign ypos[8:0] = R_v_cnt_pos[11:3];

always @(posedge R_clk_25M)
begin
    if(I_rst_n) 
        begin
            O_red   <=  4'b0000    ;
            O_green <=  4'b0000   ;
            O_blue  <=  4'b0000    ; 
        end
    else if(W_active_flag)     
        begin
            if(apple)//red apple
              begin
                O_red   <=  4'b1111    ; 
                O_green <=  4'b0000   ;
                O_blue  <=  4'b0000    ;
              end
            else if(snake)//green snake
              begin
                O_red   <=  4'b0000    ;
                O_green <=  4'b1100   ;
                O_blue  <=  4'b0010    ;
              end
            else if(head)
              begin
                O_red   <=  4'b0110    ;
                O_green <=  4'b1011   ;
                O_blue  <=  4'b1110    ;
              end
            else if(border)//white border
              begin
                O_red   <=  4'b1111    ; 
                O_green <=  4'b1111   ; 
                O_blue  <=  4'b1111    ;
              end
            else begin//black background
                O_red   <=  4'b0000    ; 
                O_green <=  4'b0000   ;
                O_blue  <=  4'b0000    ;
            end          
        end
    else
        begin
            O_red   <=  4'b0000    ;
            O_green <=  4'b0000    ;
            O_blue  <=  4'b0000    ; 
        end           
end
endmodule
