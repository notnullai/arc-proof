#!/usr/bin/env python3
"""G9 board bench: read the bench firmware's report, check it, give the verdict (docs/g9_bench.md).

The firmware (tests/mcu/g9_bench.cpp) runs the tick sequences of the QEMU timing images and prints,
per workload, the loop hash and every tick's DWT_CYCCNT delta. This script checks that the run is
valid (hashes equal the host / QEMU twins, code in ITCM and data in DTCM per the linker map and per
the firmware's own read-back, interrupts masked, no fault, boot header sane), then compares each
worst tick with the G9 cycle-bound rows and reports the measured cycles per instruction.

Before the run, the hypotheses and what each predicts (printed with every verdict):
  H1  The stated Cortex-M7 envelope bounds the tick: worst tick <= 135,284 cycles (flight default),
      <= 195,408 (learning demo, amortised, dense stress) and <= 211,745 (learning demo, amortised,
      worst case: every synapse of the slice flips, ceiling walk, G8). One worst tick above its row refutes the
      envelope and reopens G9.
  H2  "1 cycle per instruction" (the clock floors in gap register G8): worst cycles <= QEMU
      instructions, i.e. CPI <= 1.00. CPI above 1 refutes it; G9 can still close on H1, but the G8
      floors must then be restated with the measured CPI.

Sources (stdlib + pyserial; without pyserial, --port falls back to the OS serial device):
  python tools/g9_bench.py --port COM5                      board run (Teensy 4.0, LPUART6 pin 1)
  python tools/g9_bench.py --qemu build-mcu7/g9_bench_mps2  QEMU stand-in (instructions, not cycles)
  python tools/g9_bench.py --log capture.txt                a saved capture
Board runs check the image's linker map and hex: targets/g9_bench_teensy40.map / .hex when shipped
(public repo), else build-mcu7/g9_bench_teensy.map / .hex (--map / --hex to change).
--out FILE writes the JSON record (file it under docs/).
"""
import argparse, datetime, hashlib, json, os, re, shutil, subprocess, sys, tempfile, threading, time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The committed hashes (tests/mcu/g9_wl_*.cpp, checked on the host by test_g9_workloads and equal to
# the timing images' g_drone_loop_hash in docs/px4_sitl.md D5). Kept here too, so a firmware with an
# altered expectation cannot pass itself.
EXPECTED = {
    "golden_twin": 0xac29f2cfa8126d1b,
    "flight_plain": 0xac29f2cfa8126d1b,
    "flight_dense": 0x628fde62eca6d3f1,
    "learn_sliced_dense": 0x83e4d89dc5e02f94,
    "learn_dense": 0x06d80bb3fbefac89,
    "learn_sliced_worst": 0x0e1dadd0c83fe861,
}
WORST_FLIPS = 920        # every synapse of a 10-row slice: 10 x (96 - 4)
TICKS = 1999

# G9 rows: docs/gap_register.md G9 and docs/px4_sitl.md D5 (static worst-case cycle bound,
# 2026-10-25 after the G8 fast plasticity pass, M7 build, mps2-an500). instr = exact executed instructions of the timing image's worst
# tick from tools/mcu_cycle_bound.py; bench_bound / bench_instr = the same tool on this bench's own
# stand-in image (g9_bench_mps2_<row>), 2026-10-25: its M7-envelope bound and its exact maximum
# instructions in any tick (the bench's tick hook is lighter than the timing image's, so both are a
# little lower). CPI is measured against bench_instr, the instruction count of the code that ran.
ROWS = {
    "flight_dense": dict(label="flight default (candidate v5) 832/96 inference, dense stress",
                         bound=135284, instr=73604, bench_bound=135186, bench_instr=73612,
                         image="mcu_drone_loop_timing_x500_832_inference_dense"),
    "learn_sliced_dense": dict(label="learning demo 480/96, plasticity amortised x10, dense stress",
                               bound=195408, instr=97459, bench_bound=195306, bench_instr=97432,
                               image="mcu_drone_loop_timing_x500_480_sliced_dense"),
    # G8 2026-10-25 (docs/px4_sitl.md D5 "Worst-case plasticity bound", fast pass): every synapse of the
    # slice flips; the worst arming is the ceiling walk (moves onto and off the prior alternate).
    "learn_sliced_worst": dict(label="learning demo 480/96, amortised x10, WORST CASE (every synapse flips)",
                               bound=211745, instr=105945, bench_bound=211745, bench_instr=105945,
                               image="mcu_drone_loop_timing_x500_480_sliced_worst_walk"),
}
CORE_HZ = 600000000

FLASH = (0x60000000, 0x60000000 + 1984 * 1024)
DTCM0 = 0x20000000
FLASH_INPUTS = (".flashconfig", ".ivt", ".bootdata", ".startup", ".vectors")


# ---------------------------------------------------------------- reading the report

def parse_kv(line):
    out = {}
    for tok in line.split()[2:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def num(v):
    return int(v, 16) if v.startswith("0x") else int(v)


def parse_block(lines):
    """Last complete BEGIN..END block -> dict; FAULT lines are returned separately."""
    blocks, cur, faults = [], None, []
    for ln in lines:
        ln = ln.strip()
        if ln.startswith("G9 FAULT"):
            faults.append(parse_kv(ln))
        elif ln.startswith("G9 BEGIN"):
            cur = {"begin": parse_kv(ln), "sys": {}, "place": {}, "wl": [], "hist": {}, "end": None}
        elif cur is None:
            continue
        elif ln.startswith("G9 SYS"):
            cur["sys"] = parse_kv(ln)
        elif ln.startswith("G9 PLACE"):
            cur["place"] = parse_kv(ln)
        elif ln.startswith("G9 WL"):
            cur["wl"].append(parse_kv(ln))
        elif ln.startswith("G9 HIST"):
            kv = parse_kv(ln)
            h = cur["hist"].setdefault(kv["name"], {})
            h[int(kv["i"])] = [int(x, 16) for x in kv["v"].split(",") if x]
        elif ln.startswith("G9 END"):
            cur["end"] = parse_kv(ln)
            blocks.append(cur)
            cur = None
    return (blocks[-1] if blocks else None), faults


def read_serial(port, baud, timeout):
    """Lines from the board until one full report block (or a fault) has arrived."""
    lines, deadline = [], time.time() + timeout
    def done():
        return any(l.startswith("G9 END") for l in lines) and any(l.startswith("G9 BEGIN") for l in lines) \
            or sum(l.startswith("G9 FAULT") for l in lines) >= 2
    try:
        import serial  # pyserial
        with serial.Serial(port, baud, timeout=1) as s:
            buf = b""
            while time.time() < deadline and not done():
                buf += s.read(4096)
                while b"\n" in buf:
                    ln, buf = buf.split(b"\n", 1)
                    t = ln.decode("ascii", "replace").strip()
                    if t:
                        lines.append(t); echo(t)
        return lines
    except ImportError:
        pass
    # No pyserial: configure the port with the OS tool and read the device file.
    if os.name == "nt":
        subprocess.run(["mode", port + ":", "BAUD=%d" % baud, "PARITY=N", "DATA=8", "STOP=1"],
                       shell=True, check=False, stdout=subprocess.DEVNULL)
        dev = r"\\.\%s" % port
    else:
        subprocess.run(["stty", "-F", port, str(baud), "raw", "-echo"], check=False)
        dev = port
    f = open(dev, "rb", buffering=0)
    def pump():
        buf = b""
        while True:
            b = f.read(1)
            if not b:
                return
            buf += b
            if b == b"\n":
                t = buf.decode("ascii", "replace").strip(); buf = b""
                if t:
                    lines.append(t); echo(t)
    threading.Thread(target=pump, daemon=True).start()
    while time.time() < deadline and not done():
        time.sleep(0.2)
    return list(lines)


def run_qemu(elf, qemu, timeout):
    tmp = tempfile.mkdtemp(prefix="g9_qemu_")
    log = os.path.join(tmp, "serial.log")
    p = subprocess.Popen([qemu, "-machine", "mps2-an500", "-nographic", "-monitor", "none",
                          "-serial", "file:" + log, "-icount", "shift=0,align=off", "-kernel", elf])
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(1)
            if os.path.exists(log):
                with open(log, errors="replace") as f:
                    txt = f.read()
                if "G9 END" in txt:
                    return txt.splitlines()
            if p.poll() is not None:
                break
        with open(log, errors="replace") as f:
            return f.read().splitlines()
    finally:
        if p.poll() is None:
            p.kill(); p.wait()
        shutil.rmtree(tmp, ignore_errors=True)


QUIET = [False]
def echo(t):
    if not QUIET[0] and not t.startswith("G9 HIST"):
        print("  | " + t)


# ---------------------------------------------------------------- image checks

def rel(path):
    try:
        return os.path.relpath(path, ROOT).replace(os.sep, "/")
    except ValueError:
        return os.path.abspath(path)


def parse_map(path):
    """Input sections (out, name, addr, size, obj) and symbol values from a GNU ld map."""
    secs, syms, out, pending = [], {}, None, None
    started = False
    for raw in open(path, errors="replace"):
        ln = raw.rstrip("\n")
        if ln.startswith("Linker script and memory map"):
            started = True; continue
        if not started:
            continue
        m = re.match(r"^\s+0x([0-9a-f]+)\s+(\S+) = ", ln)
        if m:
            syms[m.group(2)] = int(m.group(1), 16); continue
        m = re.match(r"^\s+0x([0-9a-f]+)\s+([A-Za-z_][\w$.]*)$", ln)
        if m:
            syms.setdefault(m.group(2), int(m.group(1), 16)); continue
        m = re.match(r"^(\.\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)", ln)
        if m:
            out = m.group(1); continue
        m = re.match(r"^(\.\S+)$", ln)
        if m:
            out = m.group(1); continue
        m = re.match(r"^ (\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s*(.*)$", ln)
        if m and not m.group(1).startswith("*"):
            secs.append((out, m.group(1), int(m.group(2), 16), int(m.group(3), 16), m.group(4))); pending = None
            continue
        m = re.match(r"^ (\.\S+)$", ln)
        if m:
            pending = m.group(1); continue
        m = re.match(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s*(.*)$", ln)
        if m and pending:
            secs.append((out, pending, int(m.group(1), 16), int(m.group(2), 16), m.group(3))); pending = None
    return secs, syms


def check_map(path):
    secs, syms = parse_map(path)
    res = {"map": rel(path), "ok": True, "problems": []}
    try:
        itcm_lim, dtcm_lim = syms["__itcm_limit"], syms["__dtcm_limit"]
    except KeyError:
        res["ok"] = False; res["problems"].append("no __itcm_limit / __dtcm_limit: not a teensy4_tcm.ld map")
        return res, syms
    n = {"itcm": 0, "dtcm": 0, "flash_boot": 0}
    for out, name, addr, size, obj in secs:
        if size == 0 or out is None or out.startswith((".debug", ".comment", ".ARM.attributes")):
            continue
        code = name.startswith((".text", ".glue", ".vfp11", ".v4_bx", ".startup", ".vectors")) or "stub" in obj
        data = name.startswith((".rodata", ".data", ".bss", "COMMON", ".ARM.exidx", ".sdata", ".sbss"))
        if FLASH[0] <= addr < FLASH[1]:
            if out in (".text.headers", ".text.code") and name.startswith(FLASH_INPUTS):
                n["flash_boot"] += size; continue
            res["problems"].append("%s %s at 0x%08x in flash (%s)" % (out, name, addr, obj))
        elif code and 0 <= addr and addr + size <= itcm_lim:
            n["itcm"] += size
        elif data and DTCM0 <= addr and addr + size <= dtcm_lim:
            n["dtcm"] += size
        else:
            res["problems"].append("%s %s at 0x%08x size %d outside ITCM [0,0x%x) / DTCM [0x%08x,0x%08x) (%s)"
                                   % (out, name, addr, size, itcm_lim, DTCM0, dtcm_lim, obj))
    stack_top = syms.get("__g9_stack_top", 0)
    bss_end = syms.get("__bss_end__", 0)
    if not (bss_end + 16384 <= stack_top <= dtcm_lim):
        res["problems"].append("stack: bss end 0x%08x + 16 KB > stack top 0x%08x" % (bss_end, stack_top))
    res.update(itcm_code_bytes=n["itcm"], dtcm_data_bytes=n["dtcm"], flash_boot_bytes=n["flash_boot"],
               itcm_limit=itcm_lim, dtcm_limit=dtcm_lim, stack_top=stack_top,
               bank_config="0x%08x" % (syms.get("_flexram_bank_config", 0) & 0xFFFFFFFF))
    res["ok"] = not res["problems"]
    return res, syms


def read_hex(path):
    mem, base = {}, 0
    for ln in open(path):
        ln = ln.strip()
        if not ln.startswith(":"):
            continue
        b = bytes.fromhex(ln[1:])
        if (sum(b) & 0xFF) != 0:
            raise SystemExit("bad checksum in %s" % path)
        cnt, addr, typ, dat = b[0], (b[1] << 8) | b[2], b[3], b[4:4 + b[0]]
        if typ == 0:
            for i, x in enumerate(dat):
                mem[base + addr + i] = x
        elif typ == 4:
            base = ((dat[0] << 8) | dat[1]) << 16
        elif typ == 2:
            base = ((dat[0] << 8) | dat[1]) << 4
        elif typ == 1:
            break
    return mem


def check_hex(path, syms):
    mem = read_hex(path)
    w = lambda a: sum(mem.get(a + i, 0) << (8 * i) for i in range(4))
    p = []
    if w(0x60000000) != 0x42464346:
        p.append("no FlexSPI config tag 'FCFB' at 0x60000000")
    hdr = w(0x60001000)
    if (hdr & 0xFF) != 0xD1 or ((hdr >> 8) & 0xFFFF) != 0x2000 or not (0x40 <= (hdr >> 24) <= 0x45):
        p.append("IVT header 0x%08x is not tag 0xD1, length 0x20" % hdr)
    entry, bootdata, self_ = w(0x60001004), w(0x60001010), w(0x60001014)
    if not (entry & 1) or not (FLASH[0] <= entry < FLASH[1]):
        p.append("IVT entry 0x%08x is not a Thumb address in flash" % entry)
    if "g9_reset" in syms and (entry & ~1) != syms["g9_reset"]:
        p.append("IVT entry 0x%08x is not g9_reset 0x%08x" % (entry, syms["g9_reset"]))
    if self_ != 0x60001000 or bootdata != 0x60001020:
        p.append("IVT self / boot data pointers 0x%08x / 0x%08x" % (self_, bootdata))
    start, length = w(0x60001020), w(0x60001024)
    top = max(mem) + 1 if mem else 0
    if start != 0x60000000 or start + length < top:
        p.append("boot data start 0x%08x length %d does not cover the image (top 0x%08x)" % (start, length, top))
    sha = hashlib.sha256(open(path, "rb").read()).hexdigest()
    return {"hex": rel(path), "sha256": sha, "ok": not p, "problems": p,
            "image_bytes": top - 0x60000000, "ivt_entry": "0x%08x" % entry}


# ---------------------------------------------------------------- verdict

def pct(sorted_v, n):
    return sorted_v[n * 50 // 100], sorted_v[n * 90 // 100 - 1]


def evaluate(block, faults, mapres, syms, hexres, ref, standin_expected):
    problems, notes = [], []
    if faults:
        problems.append("firmware fault: " + " ".join("%s=%s" % kv for kv in faults[-1].items()))
    if block is None:
        problems.append("no complete G9 BEGIN..END report received")
        return None, problems, notes
    b = block["begin"]
    scale = int(b["scale"])
    standin = block["sys"].get("standin") == "1"
    if standin != standin_expected:
        problems.append("report says standin=%s" % block["sys"].get("standin"))
    wls = {}
    for w in block["wl"]:
        name = w["name"]
        h = block["hist"].get(name, {})
        vals = [v for i in sorted(h) for v in h[i]]
        rec = dict(name=name, image=w["image"], bits=int(w["bits"]), hidden=int(w["hidden"]),
                   plasticity=int(w["plasticity"]), slices=int(w["slices"]), dense=int(w["dense"]),
                   ticks=int(w["ticks"]), hash="0x" + w["hash"], expect="0x" + w["expect"])
        if name not in EXPECTED:
            problems.append("%s: unknown workload" % name); continue
        if int(w["hash"], 16) != EXPECTED[name] or int(w["expect"], 16) != EXPECTED[name] or w["hash_ok"] != "1":
            problems.append("%s: hash 0x%s, committed 0x%016x: the run is invalid" % (name, w["hash"], EXPECTED[name]))
        if "worst_armed" in w:
            rec.update(worst_armed=int(w["worst_armed"]), worst_missed=int(w["worst_missed"]),
                       worst_flips_max=int(w["worst_flips_max"]), worst_flips_total=int(w["worst_flips_total"]))
            if rec["worst_armed"] == 0 or rec["worst_missed"] != 0 or rec["worst_flips_max"] != WORST_FLIPS:
                problems.append("%s: worst case not reached on this run (armed %d, missed %d, most flips %d, want %d): "
                                "the timed figure is not the worst case" % (name, rec["worst_armed"], rec["worst_missed"],
                                                                          rec["worst_flips_max"], WORST_FLIPS))
        elif name == "learn_sliced_worst":
            problems.append("%s: no arming counters in the report" % name)
        if rec["ticks"] != TICKS or len(vals) != TICKS:
            problems.append("%s: %s ticks, %d values received (want %d)" % (name, w["ticks"], len(vals), TICKS))
        if vals:
            sv = sorted(vals)
            p50, p90 = pct(sv, len(sv))
            if (p50, p90, max(vals), sum(vals)) != (int(w["p50"]), int(w["p90"]), int(w["max"]), int(w["total"])):
                problems.append("%s: per-tick values disagree with the summary (serial corruption?)" % name)
            rec.update(p50=p50 * scale, p90=p90 * scale, worst=max(vals) * scale, worst_at=vals.index(max(vals)),
                       mean=sum(vals) * scale / len(vals), total=sum(vals) * scale, per_tick=[v * scale for v in vals])
        wls[name] = rec
    for name in EXPECTED:
        if name not in wls:
            problems.append("%s: missing from the report" % name)
    if not standin:
        s, pl = block["sys"], block["place"]
        if num(s.get("primask", "0")) & 1 != 1 or num(s.get("nvic_iser_or", "1")) != 0 or num(s.get("faultmask", "0")):
            problems.append("interrupts not masked for the run (primask=%s nvic=%s)" % (s.get("primask"), s.get("nvic_iser_or")))
        if s.get("unalign_trp") != "ticks":
            problems.append("unaligned-access trap not armed for the ticks")
        if pl.get("tcm") != "1" or pl.get("place_ok") != "1":
            problems.append("firmware read-back: code / data not in ITCM / DTCM (%s)" % " ".join("%s=%s" % kv for kv in pl.items()))
        if mapres is None:
            problems.append("no linker map checked (--map)")
        elif not mapres["ok"]:
            problems += ["map: " + x for x in mapres["problems"]]
        else:
            for key, sym in (("text", "__itcm_text_start"), ("text_end", "__itcm_text_end"), ("arc_run", "arc_run"),
                             ("data", "__data_start__"), ("bss_end", "__bss_end__"), ("stack_top", "__g9_stack_top")):
                if key in pl and sym in syms and num(pl[key]) != syms[sym]:
                    problems.append("flashed image is not the checked map: %s 0x%08x vs map 0x%08x" % (key, num(pl[key]), syms[sym]))
        if hexres is None:
            notes.append("no hex checked (--hex)")
        elif not hexres["ok"]:
            problems += ["hex: " + x for x in hexres["problems"]]
        hz = int(b.get("core_hz", "0"))
        if not hz:
            notes.append("core clock not measured (PIT); cycles do not depend on it")
        elif abs(hz - CORE_HZ) > CORE_HZ // 100:
            notes.append("core clock %.1f MHz, not 600 MHz: cycle counts stand, the microsecond margins use the measured clock" % (hz / 1e6))
    return wls, problems, notes


def rows_and_hypotheses(wls, ref, standin, core_hz):
    rows, h1, h2 = [], [], []
    hz = core_hz or CORE_HZ
    for name, row in ROWS.items():
        w = wls.get(name)
        if not w or "worst" not in w:
            continue
        r = dict(workload=name, label=row["label"], timing_image=row["image"], bound_cycles=row["bound"],
                 qemu_worst_instr=row["instr"], measured_worst=w["worst"], worst_at=w["worst_at"],
                 unit="instructions" if standin else "cycles")
        r["margin_vs_bound"] = round(row["bound"] / w["worst"], 3)
        r["cpi_worst"] = round(w["worst"] / (row["bench_instr"] or row["instr"]), 3)
        if row.get("bench_bound"):
            r["bench_image_bound_cycles"], r["bench_image_worst_instr"] = row["bench_bound"], row["bench_instr"]
        rr = ref.get(name) if ref else None
        if rr and len(rr) == len(w["per_tick"]):
            cpis = [c / i for c, i in zip(w["per_tick"], rr) if i >= 1000]
            r["cpi_mean"] = round(sum(w["per_tick"]) / sum(rr), 3)
            r["cpi_per_tick_min"], r["cpi_per_tick_max"] = round(min(cpis), 3), round(max(cpis), 3)
        r["worst_us_at_core_clock"] = round(w["worst"] / hz * 1e6, 1)
        r["pass"] = w["worst"] <= row["bound"]
        rows.append(r)
        h1.append(r["pass"]); h2.append(r["cpi_worst"] <= 1.0)
    for name, w in wls.items():                         # CPI for the workloads without a bound row
        if name in ROWS or "worst" not in w or not ref or name not in ref:
            continue
        rr = ref[name]
        if len(rr) == len(w["per_tick"]):
            w["cpi_worst"] = round(w["worst"] / max(rr), 3)
            w["cpi_mean"] = round(sum(w["per_tick"]) / sum(rr), 3)
    def judge(h):
        if standin:
            return "not tested (stand-in counts instructions)"
        return "supported" if h and all(h) else "refuted" if h else "not tested"
    hyp = {
        "H1_m7_envelope_bounds_tick": {"prediction": "worst tick <= 135,284 (flight), <= 195,408 (learning, dense) and <= 211,745 (learning, worst case) cycles",
                                       "result": judge(h1)},
        "H2_one_cycle_per_instruction": {"prediction": "worst-tick cycles <= QEMU instructions (CPI <= 1.00)",
                                         "result": judge(h2)},
    }
    return rows, hyp


def load_ref(path):
    if not path or not os.path.exists(path):
        return None
    d = json.load(open(path))
    return {w["name"]: w["per_tick"] for w in d.get("workloads", []) if "per_tick" in w}


def first_existing(*rel_paths):
    """The shipped image (public repo: targets/) or the local build (private repo: build-mcu7/)."""
    for r in rel_paths:
        if os.path.exists(os.path.join(ROOT, r)):
            return os.path.join(ROOT, r)
    return os.path.join(ROOT, rel_paths[-1])


def git_head():
    try:
        return subprocess.run(["git", "-C", ROOT, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the USB-serial adapter on Teensy pin 1 (COM5, /dev/ttyUSB0)")
    src.add_argument("--qemu", metavar="ELF", help="run the QEMU stand-in image (build-mcu7/g9_bench_mps2)")
    src.add_argument("--log", help="parse a saved capture")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--qemu-bin", default=shutil.which("qemu-system-arm") or r"C:\Program Files\qemu\qemu-system-arm.exe")
    ap.add_argument("--map", default=first_existing("targets/g9_bench_teensy40.map", "build-mcu7/g9_bench_teensy.map"))
    ap.add_argument("--hex", default=first_existing("targets/g9_bench_teensy40.hex", "build-mcu7/g9_bench_teensy.hex"))
    ap.add_argument("--ref", default=os.path.join(ROOT, "docs", "g9_bench_standin.json"),
                    help="stand-in record with the per-tick instruction counts (per-tick CPI)")
    ap.add_argument("--standin", action="store_true", help="the --log capture is from the QEMU stand-in")
    ap.add_argument("--out", help="write the JSON record here")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    QUIET[0] = a.quiet

    standin = bool(a.qemu) or a.standin
    if a.port:
        print("G9 bench: reading %s at %d baud (press the Teensy's program button only to re-flash)" % (a.port, a.baud))
        lines = read_serial(a.port, a.baud, a.timeout)
    elif a.qemu:
        print("G9 bench: QEMU stand-in %s" % a.qemu)
        lines = run_qemu(a.qemu, a.qemu_bin, a.timeout)
    else:
        lines = open(a.log, errors="replace").read().splitlines()
    block, faults = parse_block(lines)

    mapres = syms = hexres = None
    if not standin:
        if os.path.exists(a.map):
            mapres, syms = check_map(a.map)
        if os.path.exists(a.hex):
            hexres = check_hex(a.hex, syms or {})
    ref = None if standin else load_ref(a.ref)
    wls, problems, notes = evaluate(block, faults, mapres, syms or {}, hexres, ref, standin)
    core_hz = int(block["begin"].get("core_hz", "0")) if block else 0
    rows, hyp = rows_and_hypotheses(wls or {}, ref, standin, core_hz) if wls else ([], {})
    valid = not problems and len(rows) == len(ROWS)

    print()
    print("G9 bench %s, %s" % ("QEMU STAND-IN (instructions, not cycles: does not close G9)" if standin else "board run",
                               block["begin"].get("board") if block else "-"))
    for w in (wls or {}).values():
        if "worst" in w:
            extra = (" cpi worst %.3f mean %.3f" % (w["cpi_worst"], w["cpi_mean"])) if "cpi_mean" in w else ""
            print("  %-18s hash %s %s  p50 %7d  p90 %7d  worst %7d (tick %4d)%s" % (
                w["name"], w["hash"], "ok " if int(w["hash"], 16) == EXPECTED.get(w["name"]) else "BAD",
                w["p50"], w["p90"], w["worst"], w["worst_at"], extra))
    for r in rows:
        cpi = " cpi mean %.3f, per tick %.3f-%.3f" % (r["cpi_mean"], r["cpi_per_tick_min"], r["cpi_per_tick_max"]) \
            if "cpi_mean" in r else ""
        print("  ROW %-62s worst %7d vs bound %7d  %s  (bound/worst %.2fx; cpi worst %.3f vs %d QEMU instr%s; %.1f us)" % (
            r["label"], r["measured_worst"], r["bound_cycles"], "PASS" if r["pass"] else "FAIL",
            r["margin_vs_bound"], r["cpi_worst"], ROWS[r["workload"]]["bench_instr"] or r["qemu_worst_instr"], cpi, r["worst_us_at_core_clock"]))
    for k, v in hyp.items():
        print("  %s: predicted %s -> %s" % (k, v["prediction"], v["result"].upper()))
    if mapres:
        print("  map %s: %s; %d B code in ITCM (< 0x%x), %d B data in DTCM (< 0x%08x), %d B boot code in flash, banks %s" % (
            mapres["map"], "ok" if mapres["ok"] else "FAIL", mapres.get("itcm_code_bytes", 0), mapres.get("itcm_limit", 0),
            mapres.get("dtcm_data_bytes", 0), mapres.get("dtcm_limit", 0), mapres.get("flash_boot_bytes", 0),
            mapres.get("bank_config")))
    if hexres:
        print("  hex %s: %s; FCFB + IVT entry %s, %d B image, sha256 %s" % (
            hexres["hex"], "ok" if hexres["ok"] else "FAIL", hexres["ivt_entry"], hexres["image_bytes"], hexres["sha256"][:16]))
    for n in notes:
        print("  note: " + n)
    for p in problems:
        print("  INVALID: " + p)
    if standin:
        verdict = "STAND-IN %s: report, parser and sequences check out; G9 stays open until the board run" % (
            "OK" if valid else "FAILED")
    elif not valid:
        verdict = "INVALID RUN: fix the listed problem and run again; G9 neither closes nor reopens"
    elif all(r["pass"] for r in rows):
        verdict = "G9 CLOSES: every worst tick is inside its Cortex-M7 cycle bound on the board"
        if hyp["H2_one_cycle_per_instruction"]["result"] == "refuted":
            verdict += " (CPI > 1: restate the G8 '1 cycle/instr' clock floors with the measured CPI)"
    else:
        verdict = "G9 REOPENS: a worst tick exceeds its cycle bound, the M7 envelope is wrong (restate it from this run)"
    print("VERDICT: " + verdict)

    if a.out:
        rec = dict(schema="arc.g9_bench.v1", date=datetime.date.today().isoformat(), git_head=git_head(),
                   source="qemu-standin" if standin else "board", board=block["begin"].get("board") if block else None,
                   unit="instructions" if standin else "cycles", core_hz=core_hz,
                   sys=block["sys"] if block else {}, place=block["place"] if block else {},
                   map_check=mapres, hex_check=hexres, workloads=list((wls or {}).values()), rows=rows,
                   hypotheses=hyp, problems=problems, notes=notes, faults=faults, verdict=verdict)
        txt = json.dumps(rec, indent=1)
        txt = re.sub(r"\[\s*(-?\d[\d.,\s-]*?)\s*\]", lambda m: "[" + re.sub(r"\s+", "", m.group(1)) + "]", txt)
        with open(a.out, "w") as f:
            f.write(txt + "\n")
        print("record: " + a.out)
    sys.exit(0 if valid and (standin or all(r["pass"] for r in rows)) else 1)


if __name__ == "__main__":
    main()
