#!/usr/bin/env python3
"""Static worst-case cycle bound for one tick of a Cortex-M timing image (gap register G9).

Runs the timing ELF under qemu -icount with `-d in_asm,exec,nochain` and the
memory_region_ops_read trace event, cuts the execution trace into ticks at each
SysTick CVR read (0xE000E018, the same reads ci/run_mcu_timing.sh differences),
and gets the exact dynamic instruction mix of every tick. Each instruction is
then charged its worst-case cycle cost from a per-core table (below, with
sources) and the script prints the worst tick's bound and the lowest clock that
guarantees the tick budget.

The bound is analytic: published (M3/M4) or stated (M7) worst-case costs times
the exact executed instruction mix. It is not a measurement on silicon.

Usage:
  python tools/mcu_cycle_bound.py --elf targets/mcu_drone_loop_timing_x500_832_inference_dense_m7 \
      --machine mps2-an500 --models m7
  python tools/mcu_cycle_bound.py --elf targets/mcu_drone_loop_timing_x500_832_inference_dense_m3 --machine mps2-an385 --models m3,m4
Options --wf / --wd add wait states per instruction-fetch word / per data word
(0 = TCM / zero-wait SRAM, the stated requirement). --log FILE re-uses or keeps
the QEMU log instead of a temporary one.
"""
import argparse, collections, os, re, shutil, subprocess, sys, tempfile, time

COND = {"eq", "ne", "cs", "hs", "cc", "lo", "mi", "pl", "vs", "vc", "hi", "ls", "ge", "lt", "gt", "le", "al"}

# Instruction classes. Every executed mnemonic must map to one, or the script stops.
ALU = set("""mov mvn neg addw subw add adc sub sbc rsb and orr orn eor bic cmp cmn tst teq lsl lsr asr ror rrx
clz rbit rev rev16 revsh sxtb sxth uxtb uxth sxtab sxtah uxtab uxtah ubfx sbfx bfc bfi movw movt adr
ssat usat ssat16 usat16 qadd qsub qdadd qdsub sel nop pkhbt pkhtb uxtb16 sxtb16 uxtab16 sxtab16""".split())
MUL32 = set("""mul smulbb smulbt smultb smultt smulwb smulwt smlabb smlabt smlatb smlatt smlawb smlawt
smlad smladx smlsd smlsdx smuad smuadx smusd smusdx smmul smmulr smmla smmlar smmls smmlsr""".split())  # DSP: DDI 0439B Table 3-2, 1 cycle
MAC32 = {"mla", "mls"}
MULL = {"umull", "smull"}
MLAL = {"umlal", "smlal", "umaal", "smlalbb", "smlalbt", "smlaltb", "smlaltt", "smlald", "smlaldx", "smlsld", "smlsldx"}
DIV = {"sdiv", "udiv"}
LD1 = {"ldr", "ldrb", "ldrh", "ldrsb", "ldrsh", "ldrex", "ldrexb", "ldrexh", "ldrt", "ldrbt", "ldrht"}
ST1 = {"str", "strb", "strh", "strex", "strexb", "strexh", "strt", "strbt", "strht"}
LDD = {"ldrd"}
STD = {"strd"}
LDM = {"ldm", "ldmia", "ldmdb", "pop"}
STM = {"stm", "stmia", "stmdb", "push"}
BR = {"b", "bl", "bx", "blx", "cbz", "cbnz"}
TB = {"tbb", "tbh"}
IT = {"it", "itt", "ite", "ittt", "itte", "itet", "itee", "itttt", "ittte", "ittet", "ittee",
      "itett", "itete", "iteet", "iteee"}
SYS = {"cpsid", "cpsie", "mrs", "msr", "dmb", "dsb", "isb", "wfi", "wfe", "sev"}
KNOWN = ALU | MUL32 | MAC32 | MULL | MLAL | DIV | LD1 | ST1 | LDD | STD | LDM | STM | BR | TB | IT | SYS


def base_mnemonic(m):
    m = m.lower().split(".")[0]
    if m in KNOWN:
        return m
    if m[:1] == "b" and m[1:] in COND:          # b<cc> before bl/bx readings (bls, blt ...)
        return "b"
    for k in sorted(KNOWN, key=len, reverse=True):
        if m.startswith(k):
            rest = m[len(k):]
            if rest in ("", "s") or rest in COND or (rest[:1] == "s" and rest[1:] in COND):
                return k
    raise SystemExit("unclassified mnemonic: " + m)


def reglist_n(ops):
    r = re.search(r"\{([^}]*)\}", ops)
    n = 0
    for part in r.group(1).split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            n += int(b.strip()[1:]) - int(a.strip()[1:]) + 1
        elif part:
            n += 1
    return n


def writes_pc(base, ops):
    o = ops.replace(" ", "").lower()
    if base in LDM:
        return "pc" in re.search(r"\{([^}]*)\}", o).group(1).split(",")
    return o.startswith("pc,")


# Cost tables: worst case per executed instruction, zero-wait-state memory.
# P = pipeline refill on a PC write; every branch is charged as taken (path-
# direction independent); every load/store pays +1 for the address-generation
# interlock (DDI 0439B 3.3.2: an address dependent on the preceding data-
# processing result stalls one cycle, no forwarding path) and gets no credit
# for load/store pipelining; LDR [PC] pays +1 for fetch contention (3.3.2);
# IT is charged 1 (no folding credit).
M4 = dict(name="Cortex-M4", P=3, alu=1, mul32=1, mac32=2, mull=1, mlal=1, div=12,
          ld=2, st=2, ldd=3, std=3, mbase=1, agu=1, litpool=1, ldpc_min=5, tbb_min=6,
          it=1, sys=2, br_nt=None)
# Cortex-M3 (DDI 0337, ch. 18 instruction timing): same pipeline costs; long
# multiplies 3-5 (UMULL/SMULL) and 4-7 (UMLAL/SMLAL) cycles, early terminating.
M3 = dict(M4, name="Cortex-M3", mull=5, mlal=7)
# Cortex-M7: Arm publishes no per-instruction cycle table (DDI 0489). This is a
# stated conservative envelope, not vendor data: no dual issue; every load
# 1 issue + 2 load-use + 1 AGU = 4; every branch and PC write charged a full
# mispredict of 8 (pipeline is 6 stages; LLVM's M7 model uses 4, forward 6);
# multiplies 2, long multiplies 2, divide 12.
M7 = dict(name="Cortex-M7 (stated envelope)", P=8, alu=1, mul32=2, mac32=2, mull=2, mlal=2,
          div=12, ld=4, st=2, ldd=5, std=3, mbase=2, agu=0, litpool=1, ldpc_min=0, tbb_min=0,
          it=1, sys=2, br_nt=None)
MODELS = {"m3": M3, "m4": M4, "m7": M7}


def insn_cost(t, base, ops):
    """(cycles, data_words) for one executed instruction under table t."""
    P = t["P"]
    o = ops.replace(" ", "").lower()
    if base in ALU:
        return (1 + P if writes_pc(base, ops) else t["alu"]), 0
    if base in MUL32: return t["mul32"], 0
    if base in MAC32: return t["mac32"], 0
    if base in MULL: return t["mull"], 0
    if base in MLAL: return t["mlal"], 0
    if base in DIV: return t["div"], 0
    if base in LD1:
        c = t["ld"] + t["agu"] + (t["litpool"] if "[pc" in o else 0)
        if writes_pc(base, ops):
            c = max(c + P, t["ldpc_min"] + t["agu"])
        return c, 1
    if base in ST1: return t["st"] + t["agu"], 1
    if base in LDD: return t["ldd"] + t["agu"], 2
    if base in STD: return t["std"] + t["agu"], 2
    if base in LDM or base in STM:
        n = reglist_n(ops)
        c = t["mbase"] + n + t["agu"] + (P if base in LDM and writes_pc(base, ops) else 0)
        return c, n
    if base in BR: return 1 + P, 0
    if base in TB: return max(2 + P, t["tbb_min"]) + t["agu"], 1
    if base in IT: return t["it"], 0
    if base in SYS: return t["sys"], 0
    raise SystemExit("no cost for " + base)


RE_TRACE = re.compile(r"^Trace \d+: ([0-9a-f]+) \[[0-9a-f]+/([0-9a-f]+)/[0-9a-f]+/([0-9a-f]+)\]")
RE_INSN = re.compile(r"^0x([0-9a-f]+):\s+((?:[0-9a-f]{4} ?){1,2})\s+(\S+)\s*(.*)$")
RE_REWIND = re.compile(r"rewound execution of TB to ([0-9a-f]+)")
RE_STOPPED = re.compile(r"Stopped execution of TB chain before ([0-9a-f]+)")   # entered, nothing executed
SYST_CVR = "addr 0xe000e018"


def follow(path, proc):
    """Yield complete lines of a file that QEMU is still writing."""
    with open(path, "r", errors="replace") as f:
        buf = ""
        while True:
            chunk = f.read(1 << 20)
            if chunk:
                buf += chunk
                lines = buf.split("\n")
                buf = lines.pop()
                for ln in lines:
                    yield ln
            elif proc is None or proc.poll() is not None:
                if buf:
                    yield buf
                return
            else:
                time.sleep(0.05)


def trace_ticks(lines, stop):
    """Return (tbs, ticks, systicks): tbs[ptr] = [(addr, size, base, ops)], ticks = list of
    Counters of (ptr, cut), systicks = the firmware's own per-tick SysTick deltas x 40 (instructions)."""
    tbs, insn_at, cvr = {}, {}, []
    pending, cur_in = None, None
    ticks, cur = [], None
    prev_key, prev_tick = None, None
    last_pc, same = None, 0
    for ln in lines:
        if ln.startswith("IN:"):
            cur_in = []
            continue
        if cur_in is not None:
            m = RE_INSN.match(ln)
            if m:
                a = int(m.group(1), 16)
                ins = (a, len(m.group(2).replace(" ", "")) // 2, base_mnemonic(m.group(3)), m.group(4))
                cur_in.append(ins)
                insn_at[a] = ins
                continue
            if cur_in:
                pending, cur_in = cur_in, None
            else:
                cur_in = None
        if ln.startswith("Trace"):
            m = RE_TRACE.match(ln)
            ptr, pc, cflags = m.group(1), int(m.group(2), 16), int(m.group(3), 16)
            if pending is not None and pending[0][0] == pc:
                tbs[ptr] = pending
            elif ptr not in tbs or tbs[ptr][0][0] != pc:
                n, a, blk = cflags & 0x1FF, pc, []      # io-recompiled TB: not printed by in_asm
                for _ in range(n):
                    blk.append(insn_at[a]); a += insn_at[a][1]
                if not blk:
                    raise SystemExit("unbound TB at 0x%x" % pc)
                tbs[ptr] = blk
            pending = None
            if prev_key is not None and prev_tick is not None:
                prev_tick[prev_key] += 1
            prev_key, prev_tick = (ptr, None), cur
            if cur is not None:
                same = same + 1 if pc == last_pc else 0
                if same > 20000:                          # main's for (;;) after the run
                    stop(); break
            last_pc = pc
            continue
        m = RE_STOPPED.match(ln)
        if m and prev_key is not None and prev_key[0] == m.group(1):
            prev_key = None
            continue
        m = RE_REWIND.search(ln)
        if m and prev_key is not None:
            prev_key = (prev_key[0], int(m.group(1), 16))
            continue
        if SYST_CVR in ln and "memory_region_ops_read" in ln:
            if prev_key is not None and prev_tick is not None:
                prev_tick[prev_key] += 1                # the TB doing the read ends the old tick
                prev_key = None
            cur = collections.Counter()
            ticks.append(cur)
            cvr.append(int(re.search(r"value 0x([0-9a-f]+)", ln).group(1), 16))
    # ticks[k] runs from read k to read k+1; the last entry runs past the final read
    systicks = [((cvr[i] - cvr[i + 1]) & 0xFFFFFF) * 40 for i in range(len(cvr) - 1)]
    return tbs, ticks[:-1], systicks


def tick_cost(tbs, counter, t, wf, wd):
    cyc = instr = branches = dwords = 0
    mix = collections.Counter()
    for (ptr, cut), n in counter.items():
        for a, size, base, ops in tbs[ptr]:
            if cut is not None and a >= cut:
                break
            c, d = insn_cost(t, base, ops)
            cyc += n * c; dwords += n * d; instr += n
            mix[base] += n
            if base in BR or base in TB or writes_pc(base, ops):
                branches += n
    fetch_words = instr + branches          # <= 1 new fetch word per instruction, +1 per refill
    return dict(instr=instr, cycles=cyc + wf * fetch_words + wd * dwords, base_cycles=cyc,
                branches=branches, dwords=dwords, fetch_words=fetch_words, mix=mix)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--elf", required=True)
    ap.add_argument("--machine", required=True, help="mps2-an500 (M7 build) or mps2-an385 (M3 build)")
    ap.add_argument("--models", default="m4", help="comma list of m3,m4,m7")
    ap.add_argument("--qemu", default=shutil.which("qemu-system-arm") or r"C:\Program Files\qemu\qemu-system-arm.exe")
    ap.add_argument("--log", help="QEMU log path; re-used without running QEMU if it exists")
    ap.add_argument("--wf", type=int, default=0, help="wait states per instruction-fetch word")
    ap.add_argument("--wd", type=int, default=0, help="wait states per data word")
    ap.add_argument("--budget-us", type=float, default=1000.0)
    ap.add_argument("--stride", type=int, default=1,
                    help="keep every STRIDE-th segment between CVR reads, from --offset (2 / 1 for the "
                         "worst-case plasticity twins, whose arming between ticks is cut out by a second read)")
    ap.add_argument("--offset", type=int, default=0)
    a = ap.parse_args()

    tmpdir = None
    log = a.log
    proc = None
    if not log:
        tmpdir = tempfile.mkdtemp(prefix="arc_cyc_")
        log = os.path.join(tmpdir, "qemu.log")
    if not os.path.exists(log):
        open(log, "w").close()
        proc = subprocess.Popen([a.qemu, "-machine", a.machine, "-nographic", "-serial", "none",
                                 "-monitor", "none", "-icount", "shift=0,align=off", "-kernel", a.elf,
                                 "-d", "in_asm,exec,nochain,trace:memory_region_ops_read", "-D", log])

    def stop():
        if proc is not None and proc.poll() is None:
            proc.kill(); proc.wait()
    try:
        tbs, ticks, systicks = trace_ticks(follow(log, proc), stop)
    finally:
        stop()
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)
    if not ticks:
        raise SystemExit("no ticks found (no SysTick CVR reads in the trace)")
    if a.stride > 1:
        ticks, systicks = ticks[a.offset::a.stride], systicks[a.offset::a.stride]

    # Cross-check: the trace's per-tick instruction count must agree with the firmware's SysTick
    # delta (25 MHz, 40 instructions per count at shift=0) to within one count.
    counts = [tick_cost(tbs, c, M4, 0, 0)["instr"] for c in ticks]
    dev = max(abs(n - s) for n, s in zip(counts, systicks))
    print("CYCLE_BOUND elf=%s ticks=%d wf=%d wd=%d systick_check max|trace-systick|=%d instr (%s, limit 40) "
          "max instr/tick trace %d systick %d" % (os.path.basename(a.elf), len(ticks), a.wf, a.wd, dev,
          "ok" if dev < 40 else "FAIL", max(counts), max(systicks)))
    if dev >= 40:
        raise SystemExit("trace does not match the firmware's SysTick counts")
    for key in a.models.split(","):
        t = MODELS[key]
        res = [tick_cost(tbs, c, t, a.wf, a.wd) for c in ticks]
        w = max(range(len(res)), key=lambda i: res[i]["cycles"])
        wi = max(r["instr"] for r in res)
        r = res[w]
        mhz = r["cycles"] / a.budget_us
        print("CYCLE_BOUND %s: worst tick %d: %d instr (max instr any tick %d), bound %d cycles "
              "(%.2f cycles/instr), branches %d, data words %d -> lowest clock for %.0f us: %.1f MHz"
              % (t["name"], w, r["instr"], wi, r["cycles"], r["cycles"] / r["instr"], r["branches"],
                 r["dwords"], a.budget_us, mhz))
        groups = collections.Counter()
        for b, n in r["mix"].items():
            g = ("alu" if b in ALU else "mul" if b in MUL32 | MAC32 | MULL | MLAL else "div" if b in DIV
                 else "load" if b in LD1 | LDD | LDM | TB else "store" if b in ST1 | STD | STM
                 else "branch" if b in BR else "it" if b in IT else "sys")
            groups[g] += n
        print("CYCLE_BOUND %s mix: %s" % (t["name"], " ".join("%s=%d" % kv for kv in sorted(groups.items()))))
        print("CYCLE_BOUND %s top: %s" % (t["name"], " ".join("%s=%d" % kv for kv in r["mix"].most_common(12))))


if __name__ == "__main__":
    main()
