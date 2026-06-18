module hdmi_bram_display(
    input clk,
    input rst,
    output hs,
    output vs,
    output de,
    output [7:0] rgb_r,
    output [7:0] rgb_g,
    output [7:0] rgb_b,
    output reg bram_en,
    output [3:0] bram_we,
    output [31:0] bram_addr,
    output [31:0] bram_din,
    input [31:0] bram_dout
);

// ============================================================================
// HDMI 时序参数 (1280×720 @ 60Hz)
// ============================================================================
parameter H_ACTIVE = 16'd1280;
parameter H_FP     = 16'd110;
parameter H_SYNC   = 16'd40;
parameter H_BP     = 16'd220;
parameter V_ACTIVE = 16'd720;
parameter V_FP     = 16'd5;
parameter V_SYNC   = 16'd5;
parameter V_BP     = 16'd20;

// ============================================================================
// 图像参数
// ============================================================================
parameter IMG_WIDTH  = 128;
parameter IMG_HEIGHT = 72;

// ============================================================================
// 缩放与定位 — 核心: 坐标映射
//   SCALE_X / SCALE_Y : 缩放倍数 (1~10)
//   POSITION_MODE     : 00=左上, 01=居中, 10=自定义
//   OFFSET_X / OFFSET_Y : 自定义偏移 (仅 POSITION_MODE=2'b10 时有效)
//
//   注意: 默认 SCALE=1 使 128×72 图像以原始分辨率显示,
//   定位效果立即可见。调大 SCALE 可放大图像。
// ============================================================================
parameter [3:0] SCALE_X = 4'd1;          // X 方向放大倍数 (默认 1: 原始 128 像素宽)
parameter [3:0] SCALE_Y = 4'd1;          // Y 方向放大倍数 (默认 1: 原始  72 像素高)
parameter [1:0] POSITION_MODE = 2'b01;   // 定位: 00=左上, 01=居中, 10=自定义
parameter [11:0] OFFSET_X = 12'd0;       // 自定义 X 偏移 (仅 MODE=2'b10)
parameter [11:0] OFFSET_Y = 12'd0;       // 自定义 Y 偏移 (仅 MODE=2'b10)

// ============================================================================
// 边框参数
// ============================================================================
parameter BORDER_ENABLE    = 1'b0;
parameter [4:0] BORDER_WIDTH = 5'd4;
parameter [23:0] BORDER_COLOR = 24'hFF0000;    // 红

// ============================================================================
// 背景颜色
// ============================================================================
parameter [23:0] BG_COLOR = 24'h000000;        // 黑

// ============================================================================
// 内部常量 — 综合时全部折叠为常数
// ============================================================================
localparam H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP;   // 1650
localparam V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP;   //  750
localparam [11:0] H_START = H_FP + H_SYNC + H_BP;       //  370
localparam [11:0] V_START = V_FP + V_SYNC + V_BP;       //   30

// 缩放后图像显示尺寸
localparam [11:0] IMG_DISP_W = IMG_WIDTH  * SCALE_X;
localparam [11:0] IMG_DISP_H = IMG_HEIGHT * SCALE_Y;

// ---------- 图像显示区域 [IMG_X0, IMG_X1) × [IMG_Y0, IMG_Y1) ----------
localparam [11:0] IMG_X0 = (POSITION_MODE == 2'b01) ? ((H_ACTIVE - IMG_DISP_W) / 2) :
                           (POSITION_MODE == 2'b10) ? OFFSET_X : 12'd0;
localparam [11:0] IMG_Y0 = (POSITION_MODE == 2'b01) ? ((V_ACTIVE - IMG_DISP_H) / 2) :
                           (POSITION_MODE == 2'b10) ? OFFSET_Y : 12'd0;
localparam [11:0] IMG_X1 = IMG_X0 + IMG_DISP_W;
localparam [11:0] IMG_Y1 = IMG_Y0 + IMG_DISP_H;

// ---------- 边框区域 (向外扩展 BORDER_WIDTH, 裁剪到有效区) ----------
localparam [11:0] BDR_X0 = ((IMG_X0 > BORDER_WIDTH) ? (IMG_X0 - BORDER_WIDTH) : 12'd0);
localparam [11:0] BDR_Y0 = ((IMG_Y0 > BORDER_WIDTH) ? (IMG_Y0 - BORDER_WIDTH) : 12'd0);
localparam [11:0] BDR_X1 = ((IMG_X1 + BORDER_WIDTH <= H_ACTIVE) ? (IMG_X1 + BORDER_WIDTH) : H_ACTIVE[11:0]);
localparam [11:0] BDR_Y1 = ((IMG_Y1 + BORDER_WIDTH <= V_ACTIVE) ? (IMG_Y1 + BORDER_WIDTH) : V_ACTIVE[11:0]);

// ============================================================================
// 寄存器 — 统一 3 级流水线 (所有控制信号对齐)
//   阶段 0 (组合) : h_cnt/v_cnt → ax/ay → img_valid / bdr_valid / img_addr
//   阶段 1 (  r) : 捕获全部组合信号 + 发出 BRAM 地址
//   阶段 2 ( _d) : BRAM 数据返回, 所有控制信号延迟一拍
//   阶段 3 (_d1) : 输出级, de / img_vld / bdr_vld / pixel 严格同步
// ============================================================================
reg [11:0] h_cnt, v_cnt;
reg hs_r, vs_r, de_r,        img_vld_r,  bdr_vld_r;
reg hs_d, vs_d, de_d,        img_vld_d,  bdr_vld_d;
reg hs_d1, vs_d1, de_d1,     img_vld_d1, bdr_vld_d1;
reg [31:0] bram_addr_r;
reg [23:0] pixel_d;
reg [23:0] pixel_d1;

// ============================================================================
// 组合逻辑
// ============================================================================

// 视频时序
wire h_act  = (h_cnt >= H_START) && (h_cnt < (H_START + H_ACTIVE));
wire v_act  = (v_cnt >= V_START) && (v_cnt < (V_START + V_ACTIVE));
wire vid_act = h_act && v_act;

wire h_sync = (h_cnt >= H_FP[11:0]) && (h_cnt < (H_FP + H_SYNC));
wire v_sync = (v_cnt >= V_FP[11:0]) && (v_cnt < (V_FP + V_SYNC));

// 有效区内坐标
wire [11:0] ax = h_cnt - H_START;
wire [11:0] ay = v_cnt - V_START;

// 图像有效: 当前扫描点落在图像显示区域内
wire img_valid = (ax >= IMG_X0) && (ax < IMG_X1) &&
                 (ay >= IMG_Y0) && (ay < IMG_Y1);

// 边框有效: 在扩展区域内但不在图像区域内 (BORDER_ENABLE 关闭时恒为 0)
wire in_bdr = (ax >= BDR_X0) && (ax < BDR_X1) &&
              (ay >= BDR_Y0) && (ay < BDR_Y1);
wire bdr_valid = BORDER_ENABLE && in_bdr && !img_valid;

// 图像内部坐标 → 原始图像像素坐标 (最近邻)
//   img_x = (ax - IMG_X0) / SCALE_X   →  0 … 127
//   img_y = (ay - IMG_Y0) / SCALE_Y   →  0 …  71
//   BRAM 字地址 = img_y * IMG_WIDTH + img_x
wire [6:0] img_x = (ax - IMG_X0) / {1'b0, SCALE_X};
wire [6:0] img_y = (ay - IMG_Y0) / {1'b0, SCALE_Y};
wire [13:0] img_word_addr = {img_y, 7'b0} + {7'd0, img_x};

// ============================================================================
// 输出
// ============================================================================
assign hs = hs_d1;
assign vs = vs_d1;
assign de = de_d1;

// 优先级: 图像 > 边框 > 背景 > 消隐期黑
// de_d1 / img_vld_d1 / bdr_vld_d1 / pixel_d1 全部对齐 (统一 3 级流水)
assign rgb_r = de_d1 ? (img_vld_d1 ? pixel_d1[23:16] :
                        bdr_vld_d1 ? BORDER_COLOR[23:16] : BG_COLOR[23:16]) : 8'h00;
assign rgb_g = de_d1 ? (img_vld_d1 ? pixel_d1[15:8]  :
                        bdr_vld_d1 ? BORDER_COLOR[15:8]  : BG_COLOR[15:8])  : 8'h00;
assign rgb_b = de_d1 ? (img_vld_d1 ? pixel_d1[7:0]   :
                        bdr_vld_d1 ? BORDER_COLOR[7:0]   : BG_COLOR[7:0])   : 8'h00;

assign bram_we   = 4'b0000;
assign bram_din  = 32'd0;
assign bram_addr = bram_addr_r;

// ============================================================================
// 时序: 行列计数器
// ============================================================================
always @(posedge clk) begin
    if (rst) begin
        h_cnt <= 12'd0;
    end else if (h_cnt == H_TOTAL - 1) begin
        h_cnt <= 12'd0;
    end else begin
        h_cnt <= h_cnt + 12'd1;
    end
end

always @(posedge clk) begin
    if (rst) begin
        v_cnt <= 12'd0;
    end else if (h_cnt == H_TOTAL - 1) begin
        v_cnt <= (v_cnt == V_TOTAL - 1) ? 12'd0 : v_cnt + 12'd1;
    end
end

// ============================================================================
// 主流水线 — 所有控制信号统一 3 级, pixel 额外一级 (BRAM 1 clk 延迟)
//
//   时钟沿 N (h_cnt 刚更新):
//     组合: vid_act / img_valid / bdr_valid / img_word_addr (基于 h_cnt)
//
//   时钟沿 N+1:
//     阶段 1: 全部组合信号 → _r 寄存器 + bram_addr_r
//
//   时钟沿 N+2:
//     阶段 2: _r → _d; BRAM 数据返回 → pixel_d
//
//   时钟沿 N+3:
//     阶段 3: _d → _d1; pixel_d → pixel_d1
//     输出: de_d1 / img_vld_d1 / bdr_vld_d1 / pixel_d1 全部对齐
// ============================================================================
always @(posedge clk) begin
    if (rst) begin
        hs_r   <= 1'b0; vs_r   <= 1'b0; de_r   <= 1'b0;
        hs_d   <= 1'b0; vs_d   <= 1'b0; de_d   <= 1'b0;
        hs_d1  <= 1'b0; vs_d1  <= 1'b0; de_d1  <= 1'b0;
        img_vld_r  <= 1'b0; img_vld_d  <= 1'b0; img_vld_d1  <= 1'b0;
        bdr_vld_r  <= 1'b0; bdr_vld_d  <= 1'b0; bdr_vld_d1  <= 1'b0;
        bram_en     <= 1'b0;
        bram_addr_r <= 32'd0;
        pixel_d     <= 24'd0;
        pixel_d1    <= 24'd0;
    end else begin
        // ---- 阶段 0→1: 捕获组合信号 ----
        hs_r  <= h_sync;
        vs_r  <= v_sync;
        de_r  <= vid_act;
        img_vld_r <= img_valid;
        bdr_vld_r <= bdr_valid;

        // ---- 阶段 1→2: 延迟一拍, BRAM 数据返回 ----
        hs_d  <= hs_r;
        vs_d  <= vs_r;
        de_d  <= de_r;
        img_vld_d <= img_vld_r;
        bdr_vld_d <= bdr_vld_r;
        pixel_d   <= bram_dout[23:0];   // 与 img_vld_d 对齐

        // ---- 阶段 2→3: 输出级, 所有信号再对齐 ----
        hs_d1  <= hs_d;
        vs_d1  <= vs_d;
        de_d1  <= de_d;
        img_vld_d1 <= img_vld_d;
        bdr_vld_d1 <= bdr_vld_d;
        pixel_d1   <= pixel_d;          // 与 de_d1 对齐

        // BRAM 读 (与 _r 同一拍发出地址)
        bram_en <= vid_act;
        bram_addr_r <= (vid_act && img_valid) ? {16'd0, img_word_addr, 2'b00} : 32'd0;
    end
end

endmodule
