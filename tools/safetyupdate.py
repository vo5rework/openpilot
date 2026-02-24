import os
import re
from pathlib import Path

NEED_ENTRIES = [
  # bus 0 (party/main)
  r"{0x45,  0, 8",  # Unity-style formatting
  r"{0x45, 0, 8",   # alt spacing
  # bus 2 (AP-side) - Unity also allows it
  r"{0x45,  2, 8",
  r"{0x45, 2, 8",
]

def patch_unity_style_tx_msgs(text: str) -> tuple[str, bool]:
  """
  Patch panda/board/safety/safety_tesla*.h style arrays:
    static const CanMsg ...TX_MSGS[] = { ... };
  Ensures 0x45 entries exist for bus 0 and 2.
  """
  changed = False
  if any(e in text for e in NEED_ENTRIES):
    return text, False

  # Find a TX allowlist block and inject after long control entries if possible.
  m = re.search(r"(static\s+const\s+CanMsg\s+.*TX_MSGS\[\]\s*=\s*\{\s*\n)(.*?)(\n\s*\};)", text, re.S)
  if not m:
    return text, False

  hdr, body, tail = m.group(1), m.group(2), m.group(3)

  # Insert near other control messages. Fallback: append near end.
  insert = "    {0x45,  0, 8},  // STW_ACTN_RQ - ACC Control\n" \
           "    {0x45,  2, 8},  // STW_ACTN_RQ - ACC Control\n"

  if "DAS_longControl" in body or "DAS_control" in body:
    # Put right after the last long/control line
    lines = body.splitlines(True)
    idx = 0
    for i, ln in enumerate(lines):
      if "DAS_longControl" in ln or "DAS_control" in ln:
        idx = i + 1
    lines.insert(idx, insert)
    body2 = "".join(lines)
  else:
    body2 = body.rstrip() + "\n" + insert

  changed = (body2 != body)
  if not changed:
    return text, False

  out = text[:m.start()] + hdr + body2 + tail + text[m.end():]
  return out, True

def patch_opendbc_style_tesla_legacy(text: str) -> tuple[str, bool]:
  """
  Patch opendbc_repo/opendbc/safety/modes/tesla_legacy.h style arrays:
    static const CanMsg TESLA_LEGACY_TX_MSGS_LATERAL[] = { ... };
  Ensures 0x45 is present on bus0 (and bus2 optional).
  """
  if "TESLA_LEGACY_TX_MSGS" not in text:
    return text, False
  if "0x45" in text:
    return text, False

  changed = False

  def inject_into_array(arr_name: str, bus: int) -> None:
    nonlocal text, changed
    pat = rf"(static\s+const\s+CanMsg\s+{re.escape(arr_name)}\[\]\s*=\s*\{{\s*\n)(.*?)(\n\s*\}};)"
    m = re.search(pat, text, re.S)
    if not m:
      return
    hdr, body, tail = m.group(1), m.group(2), m.group(3)
    ins = f"    {{0x45, {bus}, 8, .check_relay = false}},  // STW_ACTN_RQ\n"
    body2 = body + ins
    text2 = text[:m.start()] + hdr + body2 + tail + text[m.end():]
    if text2 != text:
      text = text2
      changed = True

  # Add to both lateral & long lists if present
  inject_into_array("TESLA_LEGACY_TX_MSGS_LATERAL", 0)
  inject_into_array("TESLA_LEGACY_TX_MSGS_LONG", 0)
  # Optional AP-side bus 2 (safe)
  inject_into_array("TESLA_LEGACY_TX_MSGS_LATERAL", 2)
  inject_into_array("TESLA_LEGACY_TX_MSGS_LONG", 2)

  return text, changed

def main():
  roots = [
    Path("/data/openpilot"),
  ]
  candidates = []

  for r in roots:
    if not r.exists():
      continue
    # Panda safety sources (most likely)
    candidates += list(r.glob("panda/board/safety/safety_tesla*.h"))
    # opendbc safety (sometimes used in XNOR forks)
    candidates += list(r.glob("opendbc_repo/opendbc/safety/modes/tesla*.h"))
    candidates += list(r.glob("opendbc/safety/modes/tesla*.h"))

  # Deduplicate
  seen = set()
  files = []
  for p in candidates:
    if p.exists() and p.is_file():
      rp = str(p.resolve())
      if rp not in seen:
        seen.add(rp)
        files.append(p)

  if not files:
    print("ERROR: no candidate tesla safety files found under /data/openpilot")
    return

  patched_any = False
  print("=== scanning candidates ===")
  for p in files:
    t = p.read_text(errors="ignore")
    before_has_45 = ("0x45" in t)
    t2, ch1 = patch_unity_style_tx_msgs(t)
    t3, ch2 = patch_opendbc_style_tesla_legacy(t2)

    if ch1 or ch2:
      bak = p.with_suffix(p.suffix + ".bak")
      if not bak.exists():
        bak.write_text(t, encoding="utf-8")
      p.write_text(t3, encoding="utf-8")
      patched_any = True
      print(f"PATCHED: {p} (backup: {bak})")
    else:
      print(f"SKIP   : {p} (has_0x45={before_has_45})")

  print("\n=== next steps ===")
  if patched_any:
    panda_board = Path("/data/openpilot/panda/board")
    if panda_board.exists():
      print("Panda sources found. Rebuild+flash:")
      print("  cd /data/openpilot/panda/board && ./build.sh && ./flash.sh")
      print("Then reboot.")
    else:
      print("No /data/openpilot/panda/board found.")
      print("If your build uses opendbc safety compiled into libpandasafety, rebuild your native components and reboot.")
  else:
    print("No changes made. Either 0x45 is already allowed, or the active safety source is elsewhere.")
    print("In that case, paste: `find /data/openpilot -maxdepth 4 -type f -name 'safety_tesla*.h' -o -name 'tesla_legacy.h'`")
  print("DONE")

if __name__ == "__main__":
  main()
