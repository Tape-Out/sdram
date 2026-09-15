"""sdram 的行为测试台：一颗片子的模型，自己查时序、自己存数。

期望值与时序界限都从三家手册（Winbond W9825G6KH、Micron 256Mb、Alliance AS4C4M16SA，
本地 ~/download/refs/sdram/）的原始数字在这里重算，不用被测件的 SdramChip：
  · 上电停顿 200 µs，之前只许 NOP；PRECHARGE ALL 之后才收 AUTO REFRESH 与 LOAD MODE REGISTER；
    8 次 AUTO REFRESH 与模式寄存器都有了才收 ACTIVE；LOAD MODE REGISTER 之后 2 拍只许 NOP
  · ACTIVE 到 READ/WRITE 不少于 tRCD（21 ns）· PRECHARGE 到下一条不少于 tRP（20 ns）·
    ACTIVE 或 AUTO REFRESH 之间不少于 tRC（66 ns）· 最后一条 WRITE 到 PRECHARGE 不少于 tWR（2 拍）
  · 初始化之后两次 AUTO REFRESH 之间不超过 64 ms ÷ 行数
  · 模型按写进模式寄存器的 CAS 延迟出数；WRITE 时 DQM 高的那个字节不写
序列：每个 bank 的首尾两个字写读 · 半边选通 · 连续访存 160 轮（刷新要插得进去）·
空闲三个刷新间隔（刷新要自己走）· 片子容量以外的地址回错且不发命令。

一拍一个时钟周期；时钟频率是矩阵里的 `mhz`。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
knobs = cfg.get("knobs", {})
rb = int(knobs.get("rowBits", 13))
cb = int(knobs.get("colBits", 9))
cl = int(knobs.get("cl", 3))
mhz = int(knobs.get("mhz", 100))


def at_least(ns):
    return -(-ns * mhz // 1000)


def at_most(ns):
    return ns * mhz // 1000


ROWS, COLS = 1 << rb, 1 << cb
PAUSE = at_least(200_000)
RCD = at_least(21)
RP = at_least(20)
RC = at_least(66)
WR = 2
MRD = 2
REF_LIMIT = at_most(64_000_000 // ROWS)
MODE = cl << 4
CAP_BYTES = 1 << (3 + rb + cb)
LOOPS = 160


def first(bank):
    return 0xA5000001 + bank


def last(bank):
    return 0x5A00FFFE - bank


banks = []
for b in range(4):
    banks.append(f"""    xfer(busAddr({b}, 0, 0), True, 32'h{first(b):08X}, 4'hF);
    xfer(busAddr({b}, {ROWS - 1}, {COLS - 2}), True, 32'h{last(b):08X}, 4'hF);""")
for b in range(4):
    banks.append(f"""    xfer(busAddr({b}, 0, 0), False, 0, 4'hF);
    action if (rsp.rdata != 32'h{first(b):08X} || rsp.err) begin $display("FAIL bank {b} first word reads %08h (err %0d), want {first(b):08x}", rsp.rdata, rsp.err); bad <= True; end endaction
    xfer(busAddr({b}, {ROWS - 1}, {COLS - 2}), False, 0, 4'hF);
    action if (rsp.rdata != 32'h{last(b):08X} || rsp.err) begin $display("FAIL bank {b} last word reads %08h (err %0d), want {last(b):08x}", rsp.rdata, rsp.err); bad <= True; end endaction""")

verdict = (f"power-up order and every datasheet interval hold, the first and last words of all four banks read back, "
           f"a half-strobed write keeps the other bytes, refresh keeps within {REF_LIMIT} cycles through {LOOPS} busy "
           f"rounds and an idle stretch, and an address past the chip gives an error without a command")

TEMPLATE = r'''package Sdram@L@Tb;

// 由 tb/mksdramtb.py 生成，勿手改。这一点：rowBits=@RB@ colBits=@CB@ cl=@CL@ mhz=@MHZ@

import Vector::*;
import StmtFSM::*;
import RegIf::*;
import Sdram::*;

(* synthesize *)
module mkSdram@L@Tb(Empty);
  SdramIfc#(32, 32, @RB@, @CB@, @CL@, @MHZ@) d <- mkSdram(SdramCfg { none: ? });

  Reg#(UInt#(32)) cyc <- mkReg(0);

  // ---- 片子的模型 ----
  Reg#(Maybe#(UInt#(32))) tRA   <- mkReg(tagged Invalid);   // 最近一条 ACTIVE 或 AUTO REFRESH
  Reg#(Maybe#(UInt#(32))) tAct  <- mkReg(tagged Invalid);
  Reg#(Maybe#(UInt#(32))) tPre  <- mkReg(tagged Invalid);
  Reg#(Maybe#(UInt#(32))) tRef  <- mkReg(tagged Invalid);
  Reg#(Maybe#(UInt#(32))) tMode <- mkReg(tagged Invalid);
  Reg#(Maybe#(UInt#(32))) tWr   <- mkReg(tagged Invalid);
  Reg#(Bool)              preAll   <- mkReg(False);
  Reg#(UInt#(32))         initRefs <- mkReg(0);
  Reg#(UInt#(32))         postRefs <- mkReg(0);
  Reg#(UInt#(32))         acts     <- mkReg(0);
  Reg#(Maybe#(Bit#(13)))  modeV    <- mkReg(tagged Invalid);
  Reg#(Vector#(4, Bool))     opened <- mkReg(replicate(False));
  Reg#(Vector#(4, Bit#(13))) orow <- mkReg(replicate(0));
  Reg#(Maybe#(Bit#(16))) rd0 <- mkReg(tagged Invalid);
  Reg#(Maybe#(Bit#(16))) rd1 <- mkReg(tagged Invalid);
  Reg#(Maybe#(Bit#(16))) rd2 <- mkReg(tagged Invalid);
  Vector#(64, Reg#(Maybe#(Tuple2#(Bit#(24), Bit#(16))))) store <- replicateM(mkReg(tagged Invalid));
  Reg#(UInt#(8))           ncell <- mkReg(0);
  Vector#(16, Reg#(Bool))  said  <- replicateM(mkReg(False));
  Reg#(Bool)               modelBad <- mkReg(False);

  function UInt#(32) ago(Maybe#(UInt#(32)) x) = cyc - fromMaybe(0, x);
  function Bool tooSoon(Maybe#(UInt#(32)) x, Integer g) = isValid(x) && ago(x) < fromInteger(g);

  function Maybe#(UInt#(8)) findCell(Bit#(24) k);
    Maybe#(UInt#(8)) f = tagged Invalid;
    for (Integer i = 0; i < 64; i = i + 1)
      if (store[i] matches tagged Valid {.a, .v} &&& a == k) f = tagged Valid fromInteger(i);
    return f;
  endfunction

  function Bit#(16) valueAt(Maybe#(UInt#(8)) ix);
    Bit#(16) v = 0;
    if (ix matches tagged Valid .i) v = tpl_2(fromMaybe(?, store[i]));
    return v;
  endfunction

  Bit#(13) modeNow = fromMaybe(13'h30, modeV);
  Maybe#(Bit#(16)) outStage = modeNow[6:4] == 2 ? rd1 : rd2;
  Bit#(16) dqLine = d.pins.dq_oe == 1 ? d.pins.dq_o : fromMaybe(16'h0, outStage);

  rule bus;
    d.pins.dq_in(dqLine);
  endrule

  rule model;
    Bit#(4)  c = {d.pins.cs_n, d.pins.ras_n, d.pins.cas_n, d.pins.we_n};
    Bit#(13) a = d.pins.addr;
    Bit#(2)  b = d.pins.ba;
    Bool fail = False;
    Maybe#(Bit#(16)) nrd0 = tagged Invalid;
    Vector#(4, Bool)     nopened = opened;
    Vector#(4, Bit#(13)) norow = orow;

    if (c[3] == 0 && c != 4'b0111) begin
      if (cyc < @PAUSE@ && !said[0]) begin
        $display("FAIL a command came %0d cycles after power-up, before the @PAUSE@-cycle pause", cyc);
        said[0] <= True; fail = True;
      end
      if (tooSoon(tMode, @MRD@) && !said[4]) begin
        $display("FAIL a command %0d cycles after LOAD MODE REGISTER, want at least @MRD@", ago(tMode));
        said[4] <= True; fail = True;
      end
    end

    case (c)
      4'b0010: begin   // PRECHARGE
        if (tooSoon(tWr, @WR@) && !said[9]) begin
          $display("FAIL PRECHARGE %0d cycles after the last WRITE, want at least @WR@ (tWR)", ago(tWr));
          said[9] <= True; fail = True;
        end
        if (a[10] == 1) begin preAll <= True; nopened = replicate(False); end
        else nopened[b] = False;
        tPre <= tagged Valid cyc; tWr <= tagged Invalid;
      end
      4'b0001: begin   // AUTO REFRESH
        if (!preAll && !said[1]) begin
          $display("FAIL AUTO REFRESH before PRECHARGE ALL"); said[1] <= True; fail = True;
        end
        if (tooSoon(tPre, @RP@) && !said[7]) begin
          $display("FAIL a command %0d cycles after PRECHARGE, want at least @RP@ (tRP)", ago(tPre));
          said[7] <= True; fail = True;
        end
        if (tooSoon(tRA, @RC@) && !said[8]) begin
          $display("FAIL AUTO REFRESH %0d cycles after ACTIVE or AUTO REFRESH, want at least @RC@ (tRC)", ago(tRA));
          said[8] <= True; fail = True;
        end
        if (!isValid(modeV)) initRefs <= initRefs + 1; else postRefs <= postRefs + 1;
        tRef <= tagged Valid cyc; tRA <= tagged Valid cyc;
      end
      4'b0000: begin   // LOAD MODE REGISTER
        if (!preAll && !said[1]) begin
          $display("FAIL LOAD MODE REGISTER before PRECHARGE ALL"); said[1] <= True; fail = True;
        end
        if (tooSoon(tPre, @RP@) && !said[7]) begin
          $display("FAIL a command %0d cycles after PRECHARGE, want at least @RP@ (tRP)", ago(tPre));
          said[7] <= True; fail = True;
        end
        if (a != 13'h@MODE@ && !said[5]) begin
          $display("FAIL the mode register was loaded with %03h, want @MODE@", a); said[5] <= True; fail = True;
        end
        modeV <= tagged Valid a; tMode <= tagged Valid cyc;
      end
      4'b0011: begin   // ACTIVE
        if (initRefs < 8 && !said[2]) begin
          $display("FAIL ACTIVE after %0d AUTO REFRESH cycles, want 8 before the first access", initRefs);
          said[2] <= True; fail = True;
        end
        if (!isValid(modeV) && !said[3]) begin
          $display("FAIL ACTIVE before the mode register was loaded"); said[3] <= True; fail = True;
        end
        if (tooSoon(tPre, @RP@) && !said[7]) begin
          $display("FAIL a command %0d cycles after PRECHARGE, want at least @RP@ (tRP)", ago(tPre));
          said[7] <= True; fail = True;
        end
        if (tooSoon(tRA, @RC@) && !said[8]) begin
          $display("FAIL ACTIVE %0d cycles after ACTIVE or AUTO REFRESH, want at least @RC@ (tRC)", ago(tRA));
          said[8] <= True; fail = True;
        end
        nopened[b] = True; norow[b] = a;
        tAct <= tagged Valid cyc; tRA <= tagged Valid cyc; acts <= acts + 1;
      end
      4'b0101: begin   // READ
        if (!opened[b] && !said[10]) begin
          $display("FAIL READ on bank %0d with no active row", b); said[10] <= True; fail = True;
        end
        if (tooSoon(tAct, @RCD@) && !said[6]) begin
          $display("FAIL READ %0d cycles after ACTIVE, want at least @RCD@ (tRCD)", ago(tAct));
          said[6] <= True; fail = True;
        end
        nrd0 = tagged Valid valueAt(findCell({b, orow[b], a[8:0]}));
      end
      4'b0100: begin   // WRITE
        if (!opened[b] && !said[10]) begin
          $display("FAIL WRITE on bank %0d with no active row", b); said[10] <= True; fail = True;
        end
        if (tooSoon(tAct, @RCD@) && !said[6]) begin
          $display("FAIL WRITE %0d cycles after ACTIVE, want at least @RCD@ (tRCD)", ago(tAct));
          said[6] <= True; fail = True;
        end
        if (d.pins.dq_oe == 0 && !said[11]) begin
          $display("FAIL WRITE with the data bus not driven"); said[11] <= True; fail = True;
        end
        Bit#(24) key = {b, orow[b], a[8:0]};
        Maybe#(UInt#(8)) ix = findCell(key);
        Bit#(16) old = valueAt(ix);
        Bit#(2)  m = d.pins.dqm;
        Bit#(16) w = d.pins.dq_o;
        Bit#(16) nv = {m[1] == 1 ? old[15:8] : w[15:8], m[0] == 1 ? old[7:0] : w[7:0]};
        store[fromMaybe(ncell, ix)] <= tagged Valid tuple2(key, nv);
        if (!isValid(ix)) ncell <= ncell + 1;
        tWr <= tagged Valid cyc;
      end
    endcase

    if (isValid(modeV) && isValid(tRef) && ago(tRef) > @REF@ && !said[12]) begin
      $display("FAIL %0d cycles without AUTO REFRESH, want at most @REF@", ago(tRef));
      said[12] <= True; fail = True;
    end

    rd0 <= nrd0; rd1 <= rd0; rd2 <= rd1;
    opened <= nopened; orow <= norow;
    if (fail) modelBad <= True;
  endrule

  // ---- 发起方：顶着请求直到答复，答复那一拍就放下 ----
  Reg#(Bool)            hold[2] <- mkCReg(2, False);
  Reg#(Bool)            got[2]  <- mkCReg(2, False);
  Reg#(RegReq#(32, 32)) q       <- mkReg(unpack(0));
  Reg#(RegRsp#(32))     rsp     <- mkReg(unpack(0));
  Reg#(Bool)            bad     <- mkReg(False);
  Reg#(Bit#(16))        i       <- mkReg(0);
  Reg#(UInt#(32))       mark    <- mkReg(0);

  rule drive;
    d.mem.req(hold[0] && !d.mem.rspValid, q);
  endrule

  rule take (hold[0] && d.mem.rspValid);
    rsp <= d.mem.rsp;
    hold[0] <= False;
    got[0] <= True;
  endrule

  function Bit#(32) busAddr(Bit#(2) bank, Bit#(16) row, Bit#(16) col) =
    ((zeroExtend(bank) << @RBCB@) | (zeroExtend(row) << @CB@) | zeroExtend(col)) << 1;

  function Stmt xfer(Bit#(32) addr, Bool w, Bit#(32) v, Bit#(4) s) = seq
    action
      q <= RegReq { addr: addr, write: w, wdata: v, wstrb: s };
      hold[1] <= True;
      got[1] <= False;
    endaction
    await(got[1]);
  endseq;

  Stmt test = seq
    await(d.mem.ready || cyc > @PAUSE@ + 5000);
    action if (!d.mem.ready) begin $display("FAIL the controller is still not ready %0d cycles after power-up", cyc); bad <= True; end endaction

@BANKS@

    xfer(busAddr(0, 1, 2), True, 32'h11223344, 4'hF);
    xfer(busAddr(0, 1, 2), True, 32'hAABBCCDD, 4'b0101);
    xfer(busAddr(0, 1, 2), False, 0, 4'hF);
    action if (rsp.rdata != 32'h11BB33DD || rsp.err) begin $display("FAIL a write strobing bytes 0 and 2 reads back %08h, want 11bb33dd", rsp.rdata); bad <= True; end endaction

    i <= 0;
    // 行号在 8 行里轮转：这一段要的是刷新插得进去，不是地址覆盖；模型只存 64 个格子
    while (i < @LOOPS@) seq
      xfer(busAddr(1, (i & 7) + 2, 4), True, 32'hC0DE0000 | zeroExtend(i), 4'hF);
      xfer(busAddr(1, (i & 7) + 2, 4), False, 0, 4'hF);
      action if (rsp.rdata != (32'hC0DE0000 | zeroExtend(i)) || rsp.err) begin $display("FAIL busy round %0d reads %08h", i, rsp.rdata); bad <= True; end endaction
      i <= i + 1;
    endseq

    mark <= postRefs;
    delay(@IDLE@);
    action if (postRefs < mark + 2) begin $display("FAIL only %0d AUTO REFRESH in an idle stretch of @IDLE@ cycles, want at least 2", postRefs - mark); bad <= True; end endaction

    mark <= acts;
    xfer(32'h@CAP@, False, 0, 4'hF);
    action
      Bool wrong = False;
      if (!rsp.err) begin $display("FAIL an address past the chip reads without an error"); wrong = True; end
      if (acts != mark) begin $display("FAIL an address past the chip put %0d ACTIVE commands on the bus", acts - mark); wrong = True; end
      if (wrong) bad <= True;
    endaction
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule count;
    cyc <= cyc + 1;
    if (cyc > @LIMIT@) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad || modelBad) $display("FAILED");
    else $display("PASS sdram: @VERDICT@");
    $finish((bad || modelBad) ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label)
       .replace("@RBCB@", str(rb + cb)).replace("@RB@", str(rb)).replace("@CB@", str(cb))
       .replace("@CL@", str(cl)).replace("@MHZ@", str(mhz))
       .replace("@PAUSE@", str(PAUSE)).replace("@MRD@", str(MRD)).replace("@WR@", str(WR))
       .replace("@RP@", str(RP)).replace("@RC@", str(RC)).replace("@RCD@", str(RCD))
       .replace("@REF@", str(REF_LIMIT)).replace("@MODE@", f"{MODE:03X}")
       .replace("@BANKS@", "\n".join(banks)).replace("@LOOPS@", str(LOOPS))
       .replace("@IDLE@", str(3 * REF_LIMIT)).replace("@CAP@", f"{CAP_BYTES:08X}")
       .replace("@LIMIT@", str(PAUSE + 400_000))
       .replace("@VERDICT@", verdict))

(out / f"Sdram{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  sdram 行为测试台就位：rowBits={rb} colBits={cb} cl={cl} mhz={mhz}")
