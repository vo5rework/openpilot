# /data/openpilot/tools/tesla_acc_gate_probe.py (inline)
import time
import cereal.messaging as messaging
from openpilot.common.params import Params

PARAM_KEYS = [
  "TinklaAutopilotDisabled",
  "TinklaAdjustAccWithSpeedLimit",
  "TinklaSpeedLimitUseRelative",
  "TinklaSpeedLimitOffset",
  "TinklaEnableACC",
]

def safe_get(obj, name, default=None):
  try:
    return getattr(obj, name)
  except Exception:
    return default

def safe_get_path(obj, path, default=None):
  cur = obj
  for p in path.split("."):
    cur = safe_get(cur, p, default=None)
    if cur is None:
      return default
  return cur

def field_names(capnp_obj):
  try:
    # pycapnp: schema.fields is a dict-like mapping
    return list(capnp_obj.schema.fields.keys())
  except Exception:
    return []

def dump_params() -> None:
  p = Params()
  print("=== PARAMS (gate keys) ===")
  for k in PARAM_KEYS:
    raw = None
    b = None
    try:
      raw = p.get(k, encoding="utf-8")
    except Exception:
      pass
    try:
      b = p.get_bool(k)
    except Exception:
      pass
    print(f"{k:28s} raw={raw!r} bool={b!r}")
  print("DONE.\n")

def main(duration_s: float = 12.0) -> None:
  dump_params()

  sm = messaging.SubMaster(["carState", "controlsState", "logMessage"])
  end = time.monotonic() + float(duration_s)
  next_print = 0.0
  printed_schema = False

  print("=== LIVE (controlsState + carState + XNOR_CRUISE_SYNC logs) ===")
  while time.monotonic() < end:
    sm.update(100)

    if not printed_schema and sm.valid.get("carState", False) and sm.valid.get("controlsState", False):
      printed_schema = True
      print("carState fields:", field_names(sm["carState"]))
      print("controlsState fields:", field_names(sm["controlsState"]))
      print("logMessage fields:", field_names(sm["logMessage"]))
      print("---")

    # logMessage: on most builds it's root.LogMessage with .msg
    if sm.updated.get("logMessage", False):
      lm = sm["logMessage"]
      msg = safe_get(lm, "msg", "") or ""
      if "XNOR_CRUISE_SYNC" in msg:
        print(f"[log] {msg}")

    now = time.monotonic()
    if now >= next_print and sm.valid.get("carState", False) and sm.valid.get("controlsState", False):
      next_print = now + 0.5

      cs = sm["carState"]          # root is CarState
      ctrl = sm["controlsState"]   # root is ControlsState

      enabled = bool(safe_get(ctrl, "enabled", False))
      lat_active = bool(safe_get(ctrl, "latActive", False))
      long_active = bool(safe_get(ctrl, "longActive", False))

      cruise_enabled = bool(safe_get_path(cs, "cruiseState.enabled", False))
      cruise_speed = float(safe_get_path(cs, "cruiseState.speed", 0.0) or 0.0)

      sl_valid = bool(safe_get(cs, "speedLimitValid", False))
      sl = float(safe_get(cs, "speedLimit", 0.0) or 0.0)
      sl_off = float(safe_get(cs, "speedLimitOffset", 0.0) or 0.0)

      v = float(safe_get(cs, "vEgo", 0.0) or 0.0)

      # Helpful raw enums if present
      cruise_state = safe_get_path(cs, "cruiseState.mode", None)
      cruise_available = safe_get_path(cs, "cruiseState.available", None)

      print(
        f"enabled={enabled} latActive={lat_active} longActive={long_active} "
        f"cruise.enabled={cruise_enabled} cruise.speed_ms={cruise_speed:.2f} "
        f"speedLimitValid={sl_valid} speedLimit_ms={sl:.2f} speedLimitOffset_ms={sl_off:.2f} "
        f"vEgo_ms={v:.2f} "
        f"cruise.mode={cruise_state} cruise.available={cruise_available}"
      )

  print("DONE.")

if __name__ == "__main__":
  main(12.0)
