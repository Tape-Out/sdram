package Sdram;

// SDR SDRAM 控制器：上电序列、分散式自动刷新、一笔访存一次激活。片子是什么样（地址怎么拆、
// 命令怎么编、各段时序几拍）在 SdramChip（BH）里，这里只管按拍推进。
// 总线上一个 32 位字对应片子上同一行相邻两列的两个 16 位字，字节选通变成两个字各自的 DQM。
// 每一笔都要等片子，所以控制口只有会停顿的 RegTarget。

import RegIf::*;
import SdramChip::*;

typedef struct {
  Bit#(0) none;
} SdramCfg;

interface SdramPins;
  (* always_ready, result = "sdr_cke" *)   method Bit#(1)  cke;
  (* always_ready, result = "sdr_cs_n" *)  method Bit#(1)  cs_n;
  (* always_ready, result = "sdr_ras_n" *) method Bit#(1)  ras_n;
  (* always_ready, result = "sdr_cas_n" *) method Bit#(1)  cas_n;
  (* always_ready, result = "sdr_we_n" *)  method Bit#(1)  we_n;
  (* always_ready, result = "sdr_addr" *)  method Bit#(13) addr;
  (* always_ready, result = "sdr_ba" *)    method Bit#(2)  ba;
  (* always_ready, result = "sdr_dq_o" *)  method Bit#(16) dq_o;
  (* always_ready, result = "sdr_dq_oe" *) method Bit#(1)  dq_oe;
  (* always_ready, result = "sdr_dqm" *)   method Bit#(2)  dqm;
  (* always_ready, always_enabled, prefix = "" *)
  method Action dq_in((* port = "sdr_dq_i" *) Bit#(16) v);
endinterface

interface SdramIfc#(numeric type aw, numeric type dw, numeric type rowBits,
                    numeric type colBits, numeric type cl, numeric type mhz);
  interface RegTarget#(aw, dw) mem;
  interface SdramPins          pins;
endinterface

typedef enum { Pause, InitRef, InitMode, Idle, Rw0, Rw1, Cap0, Cap1, Pre } St
  deriving (Bits, Eq);

module mkSdram#(SdramCfg cfg)(SdramIfc#(aw, dw, rowBits, colBits, cl, mhz))
    provisos (Add#(0, dw, 32), Add#(0, TDiv#(dw, 8), 4), Add#(16, _h, dw),
              Add#(_a, TAdd#(1, TAdd#(rowBits, colBits)), aw),
              Add#(_r, rowBits, 13), Add#(_c, colBits, 13));

  Integer mhzI   = valueOf(mhz);
  Integer clI    = valueOf(cl);
  Integer pause  = pauseCycles(mhzI);
  Integer rcd    = rcdCycles(mhzI);
  Integer rp     = rpCycles(mhzI);
  Integer rc     = rcCycles(mhzI);
  Integer budget = refreshBudget(mhzI, 2 ** valueOf(rowBits), clI);
  Bit#(13) mode  = modeReg(clI);
  // 片子容量（字节）以外的地址回错
  Bit#(aw) outside = fromInteger(2 ** valueOf(aw) - 2 ** (3 + valueOf(rowBits) + valueOf(colBits)));

  function Loc#(rowBits, colBits) locOf(RegReq#(aw, dw) r);
    Bit#(TAdd#(1, TAdd#(rowBits, colBits))) w = truncate(r.addr >> 2);
    return unpack({w, 1'b0});
  endfunction

  Reg#(St)        st    <- mkReg(Pause);
  Reg#(UInt#(16)) quiet <- mkReg(0);     // 还要空几拍才能发下一条命令
  Reg#(UInt#(16)) t     <- mkReg(0);     // 停顿时数拍，初始化时数自动刷新
  Reg#(UInt#(16)) since <- mkReg(0);     // 离上一次自动刷新几拍
  Reg#(Bool)      up    <- mkReg(False);
  Reg#(Maybe#(RegReq#(aw, dw))) pend <- mkReg(tagged Invalid);
  Reg#(Bool)        ans  <- mkReg(False);
  Reg#(RegRsp#(dw)) rspR <- mkReg(RegRsp { rdata: 0, err: False });
  Reg#(Bit#(16))    lo   <- mkReg(0);

  // 引脚都从寄存器出去
  Reg#(Bit#(4))  cmdR  <- mkReg(encode(Nop));
  Reg#(Bit#(13)) addrR <- mkReg(0);
  Reg#(Bit#(2))  baR   <- mkReg(0);
  Reg#(Bit#(16)) dqoR  <- mkReg(0);
  Reg#(Bit#(1))  oeR   <- mkReg(0);
  Reg#(Bit#(2))  dqmR  <- mkReg(2'b11);   // 停顿期间 DQM 拉高（Winbond 7.1）

  Wire#(Bit#(16))        dqIn  <- mkBypassWire;
  Wire#(Bool)            takeV <- mkDWire(False);
  Wire#(RegReq#(aw, dw)) takeR <- mkDWire(unpack(0));

  Bool canTake = up && !isValid(pend) && !ans;

  // 状态只有这一条规则写，方法只发线：ready 与 rspValid 都只读寄存器，发起方在同一条规则里
  // 读两者、调 req，次序就只有一种
  rule step;
    Bit#(4)   nc     = encode(Nop);
    Bit#(13)  na     = addrR;
    Bit#(2)   nb     = baR;
    Bit#(16)  ndq    = dqoR;
    Bit#(1)   noe    = 0;
    Bit#(2)   ndqm   = up ? 0 : 2'b11;
    St        ns     = st;
    UInt#(16) nq     = quiet == 0 ? 0 : quiet - 1;
    UInt#(16) nt     = t;
    UInt#(16) nsince = since == maxBound ? since : since + 1;
    Maybe#(RegReq#(aw, dw)) np = pend;
    Bool        nans = False;
    RegRsp#(dw) nrsp = rspR;
    Bool        nup  = up;
    Bit#(16)    nlo  = lo;

    if (takeV) np = tagged Valid takeR;

    let r = fromMaybe(unpack(0), pend);
    Loc#(rowBits, colBits) l = locOf(r);

    if (quiet == 0) begin
      case (st)
        Pause: begin
          if (t == fromInteger(pause)) begin
            nc = encode(Precharge); na = 13'h0400;   // A10 高：全部 bank
            ns = InitRef; nt = 0; nq = fromInteger(rp - 1);
          end else nt = t + 1;
        end
        InitRef: begin
          nc = encode(AutoRefresh); nsince = 0; nq = fromInteger(rc - 1);
          if (t + 1 == fromInteger(initRefreshes)) ns = InitMode; else nt = t + 1;
        end
        InitMode: begin
          nc = encode(LoadMode); na = mode; nb = 0; nq = fromInteger(mrdCycles - 1);
          ns = Idle; nup = True;
        end
        Idle: begin
          if (since >= fromInteger(budget)) begin
            nc = encode(AutoRefresh); nsince = 0; nq = fromInteger(rc - 1);
          end else if (isValid(pend)) begin
            if ((r.addr & outside) != 0) begin
              np = tagged Invalid; nans = True; nrsp = RegRsp { rdata: 0, err: True };
            end else begin
              nc = encode(Active); nb = l.bank; na = zeroExtend(l.row);
              nq = fromInteger(rcd - 1); ns = Rw0;
            end
          end
        end
        Rw0: begin
          na = zeroExtend(l.col);
          if (r.write) begin
            nc = encode(Write); ndq = r.wdata[15:0]; noe = 1; ndqm = ~r.wstrb[1:0];
          end else nc = encode(Read);
          ns = Rw1;
        end
        Rw1: begin
          na = zeroExtend(l.col) | 1;
          if (r.write) begin
            nc = encode(Write); ndq = r.wdata[31:16]; noe = 1; ndqm = ~r.wstrb[3:2];
            nq = fromInteger(wrCycles - 1); ns = Pre;
          end else begin
            nc = encode(Read); nq = fromInteger(clI - 1); ns = Cap0;
          end
        end
        // 第一条 READ 在引脚上那一拍之后第 CL 拍数据有效，第二条晚一拍
        Cap0: begin
          nlo = dqIn; ns = Cap1;
        end
        Cap1: begin
          nrsp = RegRsp { rdata: {dqIn, lo}, err: False }; nans = True; np = tagged Invalid;
          nc = encode(Precharge); na = 0; nq = fromInteger(rp - 1); ns = Idle;
        end
        Pre: begin
          nrsp = RegRsp { rdata: 0, err: False }; nans = True; np = tagged Invalid;
          nc = encode(Precharge); na = 0; nq = fromInteger(rp - 1); ns = Idle;
        end
      endcase
    end

    st <= ns; quiet <= nq; t <= nt; since <= nsince; up <= nup;
    pend <= np; ans <= nans; rspR <= nrsp; lo <= nlo;
    cmdR <= nc; addrR <= na; baR <= nb; dqoR <= ndq; oeR <= noe; dqmR <= ndqm;
  endrule

  interface RegTarget mem;
    method Action req(Bool valid, RegReq#(aw, dw) q);
      if (valid && canTake) begin
        takeV <= True;
        takeR <= q;
      end
    endmethod
    method Bool ready = canTake;
    method Bool rspValid = ans;
    method RegRsp#(dw) rsp = rspR;
  endinterface

  interface SdramPins pins;
    method Bit#(1)  cke   = 1;
    method Bit#(1)  cs_n  = cmdR[3];
    method Bit#(1)  ras_n = cmdR[2];
    method Bit#(1)  cas_n = cmdR[1];
    method Bit#(1)  we_n  = cmdR[0];
    method Bit#(13) addr  = addrR;
    method Bit#(2)  ba    = baR;
    method Bit#(16) dq_o  = dqoR;
    method Bit#(1)  dq_oe = oeR;
    method Bit#(2)  dqm   = dqmR;
    method Action dq_in(Bit#(16) v);
      dqIn._write(v);
    endmethod
  endinterface
endmodule

endpackage
